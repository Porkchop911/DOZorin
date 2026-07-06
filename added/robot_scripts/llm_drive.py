#!/usr/bin/env python3
import sys, json, re, urllib.request, time
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
LM_URL="http://192.168.0.148:1234/v1/chat/completions"; MODEL="qwen3.5-9b"
PROMPT='''Convert the user's command into mecanum robot motion as JSON. Each action is [x, y, z, t]:
  x = forward(+)/back(-)   m/s
  y = strafe LEFT(+)/RIGHT(-)   m/s   (sideways slide; strafe uses ONLY y)
  z = turn LEFT/CCW(+)/RIGHT/CW(-)   rad/s   (turn in place uses ONLY z)
  t = seconds
RULES:
- Pick ONLY the axis matching the verb. forward/back -> x. strafe/slide -> y. turn/rotate/spin -> z. Never put forward speed in a strafe, never put a turn in y.
- RIGHT is NEGATIVE, LEFT is POSITIVE, for BOTH strafe (y) and turn (z).
- Turning D degrees: use |z|=1.0 and t = D*0.01745 (90deg -> t=1.57, 180 -> 3.14).
- Default speeds: x,y = 0.2 m/s, z = 1.0 rad/s. A "stop/pause Ns" = [0,0,0,N].
- If the input is not a movement command, return {"action": [], "response":"not a motion command"}.
- ALWAYS end a non-empty action list with [0.0,0.0,0.0,0.0].
Output ONLY json: {"action": [[x,y,z,t], ..., [0.0,0.0,0.0,0.0]], "response":"short msg"}
EXAMPLES:
"strafe right 1s" -> {"action":[[0.0,-0.2,0.0,1.0],[0.0,0.0,0.0,0.0]],"response":"Sliding right"}
"turn right 90 degrees" -> {"action":[[0.0,0.0,-1.0,1.57],[0.0,0.0,0.0,0.0]],"response":"Turning right"}
"forward 1s then strafe left 2s" -> {"action":[[0.2,0.0,0.0,1.0],[0.0,0.2,0.0,2.0],[0.0,0.0,0.0,0.0]],"response":"Ahead then left"}'''
cmd=" ".join(sys.argv[1:]) or "move forward 1 second"
print("YOU: %s"%cmd, flush=True)
p={"model":MODEL,"temperature":0.2,"max_tokens":400,"messages":[{"role":"system","content":PROMPT},{"role":"user","content":cmd}]}
req=urllib.request.Request(LM_URL,data=json.dumps(p).encode(),headers={"Content-Type":"application/json"})
try:
    content=json.load(urllib.request.urlopen(req,timeout=30))["choices"][0]["message"]["content"]
    m=re.search(r'\{.*\}',content,re.S)
    data=json.loads(m.group(0)) if m else None
except Exception as ex:
    print("(brain unreachable or bad reply: %s)"%str(ex)[:90], flush=True); sys.exit(0)
if not data or not data.get("action"):
    print("(no motion understood: %s)"%(data.get("response","") if data else ""), flush=True); sys.exit(0)
print("qwen: %s"%data.get("response",""), flush=True)
print("plan: %s"%data.get("action"), flush=True)
rclpy.init(); node=rclpy.create_node('llm_drive')
pub=node.create_publisher(Twist,'/controller/cmd_vel',10)
gz=[]; node.create_subscription(Imu,'/ros_robot_controller/imu_raw', lambda mm: gz.append(mm.angular_velocity.z), 10)
def hold(x,y,z,dur):
    t=Twist(); t.linear.x=float(x); t.linear.y=float(y); t.angular.z=float(z)
    end=time.time()+dur; n=0
    while time.time()<end:
        pub.publish(t); rclpy.spin_once(node,timeout_sec=0.0); time.sleep(0.05); n+=1
    return n
try:
    for a in data.get("action",[]):
        v=(list(a)+[0,0,0,0])[:4]; x,y,z,t=[float(i) for i in v]
        if t>0 and (x or y or z):
            n=hold(x,y,z,t); print("  -> streamed %d msgs: x=%s y=%s z=%s for %ss"%(n,x,y,z,t), flush=True)
finally:
    z0=Twist()
    for _ in range(20): pub.publish(z0); time.sleep(0.02)
    if gz: print("IMU gyro.z: %d samples, min %+.3f, max %+.3f rad/s"%(len(gz),min(gz),max(gz)), flush=True)
    node.destroy_node(); rclpy.shutdown()
print("done (stopped)", flush=True)
