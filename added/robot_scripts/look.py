#!/usr/bin/env python3
# Vision+arm: aim arm-mounted camera, grab frame, ask qwen3.5-9b (multimodal).
# Usage: look.py [left|right|center|around|<0-1000>] [question...]
#   pan servo1: 900=LEFT, 500=center, 100=RIGHT  | tilt servo4=300 (forward)
import sys, time, base64, json, urllib.request, subprocess
import rclpy
from servo_controller_msgs.msg import ServosPosition
from servo_controller.bus_servo_control import set_servo_position
MODEL="qwen3.5-9b"
DIRS={"left":900,"right":100,"center":500,"forward":500,"middle":500,"ahead":500}
def clamp(v): return max(0,min(1000,int(v)))
args=sys.argv[1:]; pan=500; mode=None; q="What do you see? List the main objects briefly."
if args:
    a=args[0].lower()
    if a=="around": mode="around"; args=args[1:]
    elif a in DIRS: pan=DIRS[a]; args=args[1:]
    elif a.lstrip("-+").isdigit(): pan=clamp(args[0]); args=args[1:]
if args: q=" ".join(args)
rclpy.init(); n=rclpy.create_node('look'); pub=n.create_publisher(ServosPosition,'servo_controller',1); time.sleep(0.5)
def aim(p): set_servo_position(pub,1.2,((1,clamp(p)),(2,701),(3,120),(4,300),(5,500),(10,500))); time.sleep(1.8)
def grab():
    subprocess.run(['bash','-c','curl -s --max-time 10 "http://127.0.0.1:8080/snapshot?topic=/depth_cam/rgb0/image_raw" -o /tmp/look.jpg'])
    return base64.b64encode(open('/tmp/look.jpg','rb').read()).decode()
def ask(img,question):
    p={"model":MODEL,"max_tokens":160,"temperature":0.2,"messages":[{"role":"user","content":[{"type":"text","text":question},{"type":"image_url","image_url":{"url":"data:image/jpeg;base64,"+img}}]}]}
    req=urllib.request.Request("http://192.168.0.148:1234/v1/chat/completions",data=json.dumps(p).encode(),headers={"Content-Type":"application/json"})
    return json.load(urllib.request.urlopen(req,timeout=60))["choices"][0]["message"]["content"]
if mode=="around":
    for p,name in [(100,"RIGHT"),(500,"AHEAD"),(900,"LEFT")]:
        aim(p); print("\n=== %s (servo1=%d) ==="%(name,p), flush=True); print(ask(grab(),"In one sentence, what do you see?"), flush=True)
    aim(500)
else:
    aim(pan); print("PAN=%d  Q: %s"%(pan,q), flush=True); print("SEE:", ask(grab(),q))
n.destroy_node(); rclpy.shutdown()
