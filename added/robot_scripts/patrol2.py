#!/usr/bin/env python3
import os, math, time, json, base64, urllib.request
import rclpy, cv2, yaml, numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition
import tf2_ros
from PIL import Image as PImage
from scipy import ndimage

OUT="/home/ubuntu/patrol_out"; os.makedirs(OUT, exist_ok=True)
MODEL="qwen/qwen3-vl-4b"; URL="http://192.168.0.148:1234/v1/chat/completions"
meta=yaml.safe_load(open("/home/ubuntu/ros2_ws/src/slam/maps/map_01.yaml"))
resm=meta["resolution"]; ox,oy=meta["origin"][0],meta["origin"][1]
mimg=np.array(PImage.open("/home/ubuntu/ros2_ws/src/slam/maps/"+meta["image"]))
if mimg.ndim==3: mimg=mimg[:,:,0]
H,W=mimg.shape; occ=mimg<90; free=mimg>220
dist_m=ndimage.distance_transform_edt(~occ)*resm
def clr(x,y):
    c=int((x-ox)/resm); r=int(H-1-(y-oy)/resm)
    if r<0 or r>=H or c<0 or c>=W or not free[r,c]: return 0.0
    return float(dist_m[r,c])
def snap(x,y,need=0.35,rad=0.7):
    if clr(x,y)>=need: return x,y
    best=None;bd=1e9
    for dx in np.arange(-rad,rad+0.01,0.1):
        for dy in np.arange(-rad,rad+0.01,0.1):
            nx,ny=x+dx,y+dy; d=dx*dx+dy*dy
            if clr(nx,ny)>=need and d<bd: bd=d;best=(round(nx,2),round(ny,2))
    return best

ROUTE=[(1.5,0.0,0),(2.7,-0.2,0),(3.8,-0.4,0),(4.8,-0.6,0),(5.2,-0.86,0)]

rclpy.init(); n=Node("patrol2"); br=CvBridge()
servo=n.create_publisher(ServosPosition,"/ros_robot_controller/bus_servo/set_position",1)
ac=ActionClient(n,NavigateToPose,"navigate_to_pose")
buf=tf2_ros.Buffer(); tf2_ros.TransformListener(buf,n)
frame={}
n.create_subscription(Image,"/depth_cam/rgb0/image_raw",lambda m: frame.__setitem__("m",m),10)
time.sleep(1.0)
def log(*a):
    s=" ".join(str(x) for x in a); print(s,flush=True); open(OUT+"/log2.txt","a").write(s+"\n")
def aim(pan):
    msg=ServosPosition(); msg.duration=1.0
    for sid,pos in ((1,max(0,min(1000,pan))),(2,701),(3,120),(4,300),(5,500)):
        sp=ServoPosition(); sp.id=sid; sp.position=pos; msg.position.append(sp)
    for _ in range(3): servo.publish(msg); time.sleep(0.1)
    time.sleep(1.8)
def cap(path):
    frame.pop("m",None); t0=time.time()
    while "m" not in frame and time.time()-t0<5: rclpy.spin_once(n,timeout_sec=0.2)
    for _ in range(6): rclpy.spin_once(n,timeout_sec=0.1)
    img=br.imgmsg_to_cv2(frame["m"],desired_encoding="bgr8"); cv2.imwrite(path,img)
    return base64.b64encode(cv2.imencode(".jpg",img)[1]).decode()
def vision(b):
    q="You are a robot's eyes scanning a room. List notable objects, furniture and points of interest. Concise short bullet lines. If just floor/blank wall, say so."
    p={"model":MODEL,"max_tokens":220,"temperature":0.2,"messages":[{"role":"user","content":[{"type":"text","text":q},{"type":"image_url","image_url":{"url":"data:image/jpeg;base64,"+b}}]}]}
    req=urllib.request.Request(URL,data=json.dumps(p).encode(),headers={"Content-Type":"application/json"})
    try: return json.load(urllib.request.urlopen(req,timeout=90))["choices"][0]["message"]["content"].strip()
    except Exception as ex: return "[vision error] %r"%ex
def pose():
    try:
        t=buf.lookup_transform("map","base_footprint",rclpy.time.Time())
        q=t.transform.rotation; yaw=math.degrees(math.atan2(2*(q.w*q.z),1-2*q.z*q.z))
        return round(t.transform.translation.x,2),round(t.transform.translation.y,2),round(yaw,0)
    except Exception: return None
def drive(x,y,yd):
    ac.wait_for_server()
    g=NavigateToPose.Goal(); g.pose.header.frame_id="map"; g.pose.header.stamp=n.get_clock().now().to_msg()
    g.pose.pose.position.x=float(x); g.pose.pose.position.y=float(y)
    yr=math.radians(yd); g.pose.pose.orientation.z=math.sin(yr/2); g.pose.pose.orientation.w=math.cos(yr/2)
    sh=ac.send_goal_async(g); rclpy.spin_until_future_complete(n,sh); gh=sh.result()
    if not gh.accepted: return "REJECTED"
    rf=gh.get_result_async(); t0=time.time()
    while not rf.done() and time.time()-t0<90: rclpy.spin_once(n,timeout_sec=0.3)
    if not rf.done(): gh.cancel_goal_async(); rclpy.spin_once(n,timeout_sec=1.0); return "TIMEOUT"
    return {4:"SUCCEEDED",5:"CANCELED",6:"ABORTED"}.get(rf.result().status,"?")

results=[]
log("=== TRAVERSE START",time.strftime("%H:%M:%S"),"===")
for i,(x,y,yd) in enumerate(ROUTE):
    s=snap(x,y)
    if s is None: log("WP%d (%.1f,%.1f) no safe cell, skip"%(i,x,y)); continue
    sx,sy=s
    log("\n-- WP%d target=(%.1f,%.1f)->snap(%.2f,%.2f) clr=%.2f"%(i,x,y,sx,sy,clr(sx,sy)))
    st=drive(sx,sy,yd); pp=pose(); log("   drive:",st," pose:",pp)
    for pan,lab in ((160,"right"),(500,"center"),(840,"left")):
        aim(pan); path="%s/t%d_%s.jpg"%(OUT,i,lab); b=cap(path); d=vision(b)
        log("   [%s]"%lab, d.replace("\n"," | "))
        results.append({"wp":i,"snap":[sx,sy],"drive":st,"pose":pp,"pan":lab,"img":os.path.basename(path),"desc":d})
    json.dump(results,open(OUT+"/report2.json","w"),indent=1)
aim(500)
log("\n=== TRAVERSE DONE",time.strftime("%H:%M:%S"),"===")
json.dump(results,open(OUT+"/report2.json","w"),indent=1)
n.destroy_node(); rclpy.shutdown()
