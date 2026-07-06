#!/usr/bin/env python3
# aim arm-camera, grab frame from ROS topic, ask vision model. Low-level bus servo path.
# usage: vlook.py [pan 0-1000] [outjpg] [question...]
import sys, time, base64, json, urllib.request, rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition

MODEL="qwen/qwen3-vl-4b"
URL="http://192.168.0.148:1234/v1/chat/completions"
pan=int(sys.argv[1]) if len(sys.argv)>1 else 500
outjpg=sys.argv[2] if len(sys.argv)>2 else "/tmp/vlook.jpg"
q=" ".join(sys.argv[3:]) if len(sys.argv)>3 else "List the notable objects, furniture and points of interest you see. Be brief, one line each."

def clamp(v): return max(0,min(1000,int(v)))
rclpy.init(); n=Node("vlook"); br=CvBridge()
pub=n.create_publisher(ServosPosition,"/ros_robot_controller/bus_servo/set_position",1)
time.sleep(0.6)
def aim(p):
    msg=ServosPosition(); msg.duration=1.2
    for sid,pos in ((1,clamp(p)),(2,701),(3,120),(4,300),(5,500)):
        sp=ServoPosition(); sp.id=sid; sp.position=pos; msg.position.append(sp)
    for _ in range(3):
        pub.publish(msg); time.sleep(0.1)
aim(pan); time.sleep(2.0)
got={}
n.create_subscription(Image,"/depth_cam/rgb0/image_raw",lambda m: got.__setitem__("m",m),10)
t0=time.time()
while "m" not in got and time.time()-t0<5: rclpy.spin_once(n,timeout_sec=0.2)
for _ in range(8): rclpy.spin_once(n,timeout_sec=0.1)
m=got["m"]; img=br.imgmsg_to_cv2(m,desired_encoding="bgr8"); cv2.imwrite(outjpg,img)
b=base64.b64encode(cv2.imencode(".jpg",img)[1]).decode()
p={"model":MODEL,"max_tokens":220,"temperature":0.2,"messages":[{"role":"user","content":[{"type":"text","text":q},{"type":"image_url","image_url":{"url":"data:image/jpeg;base64,"+b}}]}]}
req=urllib.request.Request(URL,data=json.dumps(p).encode(),headers={"Content-Type":"application/json"})
try: r=json.load(urllib.request.urlopen(req,timeout=90))["choices"][0]["message"]["content"]
except Exception as ex: r="[vision error] %r"%ex
print("PAN=%d saved=%s"%(pan,outjpg)); print("VISION:",r)
n.destroy_node(); rclpy.shutdown()
