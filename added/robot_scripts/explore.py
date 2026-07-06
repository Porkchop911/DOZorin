#!/usr/bin/env python3
# Autonomous frontier explorer: slam_toolbox builds /map; we spin-scan, pick nearest
# frontier, drive there via /controller/cmd_vel with lidar safety. Vision at each stop.
import os, math, time, json, base64, urllib.request, subprocess, threading
import numpy as np, rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition
import tf2_ros

OUT="/home/ubuntu/scout_out"; os.makedirs(OUT, exist_ok=True)
STOP=OUT+"/STOP"; MODEL="qwen/qwen3-vl-4b"; URL="http://192.168.0.148:1234/v1/chat/completions"
LIN=0.16; ANG=0.6; FRONT_STOP=0.45; MAX_RUN=1000  # 15 min cap

rclpy.init(); n=Node("explore"); br=CvBridge()
cmd=n.create_publisher(Twist,"/controller/cmd_vel",10)
servo=n.create_publisher(ServosPosition,"/ros_robot_controller/bus_servo/set_position",1)
buf=tf2_ros.Buffer(); tf2_ros.TransformListener(buf,n)
S={}; M={}; F={}
n.create_subscription(LaserScan,"/scan",lambda m:S.__setitem__("m",m),10)
n.create_subscription(OccupancyGrid,"/map",lambda m:M.__setitem__("m",m),1)
n.create_subscription(Image,"/depth_cam/rgb0/image_raw",lambda m:F.__setitem__("m",m),10)
time.sleep(1.0)
def log(*a):
    s=" ".join(str(x) for x in a); print(s,flush=True); open(OUT+"/log.txt","a").write(s+"\n")
def spin(): rclpy.spin_once(n,timeout_sec=0.05)
def drive(lx,az):
    t=Twist(); t.linear.x=float(lx); t.angular.z=float(az); cmd.publish(t)
def stop_move():
    for _ in range(3): drive(0,0); time.sleep(0.05)

def aim(pan=500):
    msg=ServosPosition(); msg.duration=1.0
    for sid,pos in ((1,max(0,min(1000,pan))),(2,701),(3,120),(4,300),(5,500)):
        sp=ServoPosition(); sp.id=sid; sp.position=pos; msg.position.append(sp)
    for _ in range(3): servo.publish(msg); time.sleep(0.1)

def pose():
    try:
        t=buf.lookup_transform("map","base_footprint",rclpy.time.Time())
        q=t.transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return t.transform.translation.x,t.transform.translation.y,yaw
    except Exception: return None

def front_min():
    m=S.get("m")
    if not m: return 9.9
    a=m.angle_min; best=9.9; i=0
    for r in m.ranges:
        ang=a+i*m.angle_increment; i+=1
        if abs(math.atan2(math.sin(ang),math.cos(ang)))<math.radians(35):
            if math.isfinite(r) and m.range_min<r<best: best=r
    return best

VIS=[]
def vision(tag,pan):
    m=F.get("m")
    if not m: return
    img=br.imgmsg_to_cv2(m,desired_encoding="bgr8")
    path="%s/%s.jpg"%(OUT,tag); cv2.imwrite(path,img)
    b=base64.b64encode(cv2.imencode(".jpg",img)[1]).decode()
    q="Robot eyes scanning a NEW room. List notable objects, furniture, points of interest. Short bullets. If blank wall/floor, say so."
    p={"model":MODEL,"max_tokens":200,"temperature":0.2,"messages":[{"role":"user","content":[{"type":"text","text":q},{"type":"image_url","image_url":{"url":"data:image/jpeg;base64,"+b}}]}]}
    try:
        req=urllib.request.Request(URL,data=json.dumps(p).encode(),headers={"Content-Type":"application/json"})
        d=json.load(urllib.request.urlopen(req,timeout=60))["choices"][0]["message"]["content"].strip()
    except Exception as ex: d="[vision err] %r"%ex
    pp=pose(); VIS.append({"tag":tag,"pan":pan,"pose":[round(pp[0],2),round(pp[1],2)] if pp else None,"img":os.path.basename(path),"desc":d})
    log("   VIS[%s]"%tag, d.replace("\n"," | "))
    json.dump(VIS,open(OUT+"/scout.json","w"),indent=1)

def spin_scan(cycle):
    # rotate ~360 in place, capture vision at 3 headings
    log(" spin-scan cycle",cycle)
    steps=[("a",160),("b",500),("c",840)]
    dur=2*math.pi/ANG; t0=time.time(); shot=0; nshots=3
    while time.time()-t0<dur:
        drive(0,ANG); spin()
        if shot<nshots and (time.time()-t0) > dur*(shot+0.5)/nshots:
            stop_move(); tag,pan=steps[shot]; aim(pan); time.sleep(1.5)
            vision("c%d_%s"%(cycle,tag),pan); shot+=1
        time.sleep(0.02)
    stop_move()

def frontiers():
    m=M.get("m")
    if not m: return []
    w=m.info.width; h=m.info.height; res=m.info.resolution
    ox=m.info.origin.position.x; oy=m.info.origin.position.y
    g=np.array(m.data,dtype=np.int16).reshape(h,w)
    free=(g>=0)&(g<50); unknown=(g<0); occ=g>=50
    fr=np.zeros((h,w),bool)
    fr[1:-1,1:-1]=free[1:-1,1:-1]&(unknown[:-2,1:-1]|unknown[2:,1:-1]|unknown[1:-1,:-2]|unknown[1:-1,2:])
    ys,xs=np.where(fr)
    pts=[(ox+(x+0.5)*res, oy+(y+0.5)*res) for x,y in zip(xs,ys)]
    # cluster crudely: bucket by 0.4m grid, centroid each, require >=3 pts
    buck={}
    for x,y in pts:
        k=(round(x/0.4),round(y/0.4)); buck.setdefault(k,[]).append((x,y))
    cents=[]
    for k,v in buck.items():
        if len(v)>=3:
            cx=sum(p[0] for p in v)/len(v); cy=sum(p[1] for p in v)/len(v); cents.append((cx,cy,len(v)))
    return cents

def turn_to(bearing):
    t0=time.time()
    while time.time()-t0<8:
        pp=pose()
        if not pp: spin(); continue
        err=math.atan2(math.sin(bearing-pp[2]),math.cos(bearing-pp[2]))
        if abs(err)<0.12: break
        drive(0, ANG if err>0 else -ANG); spin(); time.sleep(0.03)
    stop_move()

def advance(tx,ty,maxt=45):
    t0=time.time()
    while time.time()-t0<maxt:
        if os.path.exists(STOP): return "stop"
        pp=pose()
        if not pp: spin(); continue
        d=math.hypot(tx-pp[0],ty-pp[1])
        if d<0.5: return "reached"
        fm=front_min()
        if fm<FRONT_STOP: stop_move(); return "blocked"
        bearing=math.atan2(ty-pp[1],tx-pp[0])
        err=math.atan2(math.sin(bearing-pp[2]),math.cos(bearing-pp[2]))
        az=max(-ANG,min(ANG,1.2*err))
        lx=LIN if abs(err)<0.4 and fm>FRONT_STOP+0.15 else 0.0
        drive(lx,az); spin(); time.sleep(0.05)
    return "timeout"

def save_map(name):
    try:
        subprocess.run(["bash","-lc","source ~/.zshrc; ros2 run nav2_map_server map_saver_cli -f /home/ubuntu/ros2_ws/src/slam/maps/%s --ros-args -p save_map_timeout:=8.0"%name],timeout=40)
        log(" saved map:",name)
    except Exception as ex: log(" map save err",repr(ex))

log("=== SCOUT START",time.strftime("%H:%M:%S"),"===")
aim(500); time.sleep(1.0)
t_start=time.time(); cycle=0; visited=[]
while time.time()-t_start<MAX_RUN and not os.path.exists(STOP):
    spin_scan(cycle)
    save_map("map_02")
    cents=frontiers(); pp=None
    for _ in range(20):
        pp=pose()
        if pp: break
        spin(); time.sleep(0.25)
    if not pp: log(" no pose after retries, abort"); break
    # drop frontiers we've been near already
    cand=[]
    for cx,cy,sz in cents:
        if all(math.hypot(cx-vx,cy-vy)>0.7 for vx,vy in visited):
            cand.append((math.hypot(cx-pp[0],cy-pp[1]),cx,cy,sz))
    cand.sort()
    log(" frontiers:",len(cents)," unvisited-candidates:",len(cand))
    if not cand: log(" EXPLORATION COMPLETE - no new frontiers"); break
    dist,cx,cy,sz=cand[0]
    log(" -> frontier (%.2f,%.2f) size=%d dist=%.2f"%(cx,cy,sz,dist))
    turn_to(math.atan2(cy-pp[1],cx-pp[0]))
    res=advance(cx,cy)
    log("    advance:",res)
    visited.append((cx,cy))
    if res=="blocked":
        # nudge: rotate a bit to escape
        drive(0,ANG); time.sleep(1.0); stop_move()
    cycle+=1
stop_move(); aim(500)
save_map("map_02")
log("=== SCOUT DONE",time.strftime("%H:%M:%S"),"cycles=%d"%cycle,"===")
json.dump(VIS,open(OUT+"/scout.json","w"),indent=1)
n.destroy_node(); rclpy.shutdown()
