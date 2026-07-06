#!/usr/bin/env python3
# Continuous reactive frontier explorer. Drives toward most-open direction, biased to
# nearest unexplored frontier. Lidar is mounted 180deg from base -> base heading th maps
# to lidar beam angle th+pi. slam_toolbox builds /map. Vision at periodic spin-scans.
import os, math, time, json, base64, urllib.request, subprocess
import numpy as np, rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition
import tf2_ros

OUT="/home/ubuntu/scout_out"; os.makedirs(OUT,exist_ok=True)
STOP=OUT+"/STOP"; MODEL="qwen/qwen3-vl-4b"; URL="http://192.168.0.148:1234/v1/chat/completions"
LIN=0.10; ANG=0.30; HARD=0.30; MOVE_CLR=0.55; MAX_RUN=1000; SPIN_EVERY=60.0
DEPTH_HARD=0.34; DEPTH_SLOW=0.55; DEPTH_MOVE_CLR=0.62; DEPTH_STALE=0.8
CHAIR_SIDE=0.70; CHAIR_CENTER=0.50; REAR_CLR=0.48

rclpy.init(); n=Node("explore2"); br=CvBridge()
cmd=n.create_publisher(Twist,"/controller/cmd_vel",10)
servo=n.create_publisher(ServosPosition,"/ros_robot_controller/bus_servo/set_position",1)
buf=tf2_ros.Buffer(); tf2_ros.TransformListener(buf,n)
S={}; M={}; F={}; D={}
n.create_subscription(LaserScan,"/scan",lambda m:S.__setitem__("m",m),10)
n.create_subscription(OccupancyGrid,"/map",lambda m:M.__setitem__("m",m),1)
n.create_subscription(Image,"/depth_cam/rgb0/image_raw",lambda m:F.__setitem__("m",m),10)
n.create_subscription(Image,"/depth_cam/depth0/image_raw",lambda m:(D.__setitem__("m",m),D.__setitem__("t",time.time())),5)
time.sleep(1.0)
def log(*a):
    s=" ".join(str(x) for x in a); print(s,flush=True); open(OUT+"/log.txt","a").write(s+"\n")
def spin(): rclpy.spin_once(n,timeout_sec=0.03)
def drive(lx,az):
    t=Twist(); t.linear.x=float(lx); t.angular.z=float(az); cmd.publish(t)
def stop_move():
    for _ in range(3): drive(0,0); time.sleep(0.04)
def aim(pan=500):
    m=ServosPosition(); m.duration=1.0
    for sid,pos in ((1,max(0,min(1000,pan))),(2,701),(3,120),(4,300),(5,500)):
        sp=ServoPosition(); sp.id=sid; sp.position=pos; m.position.append(sp)
    for _ in range(3): servo.publish(m); time.sleep(0.1)
def _yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def pose():
    for _ in range(6):
        try:
            mo=buf.lookup_transform("map","odom",rclpy.time.Time())
            ob=buf.lookup_transform("odom","base_footprint",rclpy.time.Time())
            moy=_yaw(mo.transform.rotation); mx=mo.transform.translation.x; my=mo.transform.translation.y
            bx=ob.transform.translation.x; by=ob.transform.translation.y
            x=mx+bx*math.cos(moy)-by*math.sin(moy); y=my+bx*math.sin(moy)+by*math.cos(moy)
            return x,y,moy+_yaw(ob.transform.rotation)
        except Exception: spin(); time.sleep(0.1)
    return None
def scan_arr():
    m=S.get("m")
    if not m: return None,None
    r=np.array(m.ranges,dtype=float); a=m.angle_min+np.arange(len(r))*m.angle_increment
    v=np.isfinite(r)&(r>m.range_min)&(r<m.range_max)
    return a[v],r[v]
def clear_at(a,r,base_th,half=0.30):
    if a is None or len(a)==0: return 0.0
    tgt=base_th+math.pi
    d=np.arctan2(np.sin(a-tgt),np.cos(a-tgt))
    mask=np.abs(d)<half
    return float(np.min(r[mask])) if mask.any() else 9.9
def depth_image():
    m=D.get("m")
    if not m or time.time()-D.get("t",0)>DEPTH_STALE: return None
    try:
        img=br.imgmsg_to_cv2(m,desired_encoding="passthrough")
    except Exception:
        return None
    dep=np.asarray(img)
    if dep.ndim==3: dep=dep[:,:,0]
    dep=dep.astype(np.float32)
    enc=m.encoding.lower()
    if dep.size and (dep.dtype==np.uint16 or enc in ("16uc1","mono16") or np.nanmax(dep)>20.0):
        dep*=0.001
    return dep
def depth_near(dep,x0,x1,y0,y1):
    h,w=dep.shape[:2]
    roi=dep[int(y0*h):int(y1*h):3, int(x0*w):int(x1*w):3]
    v=roi[np.isfinite(roi)&(roi>0.18)&(roi<3.0)]
    if len(v)<40: return None,0.0
    near=float(np.percentile(v,2.0))
    frac=float(np.count_nonzero(v<CHAIR_SIDE))/float(len(v))
    return near,frac
def depth_bumper():
    dep=depth_image()
    if dep is None: return None
    h,w=dep.shape[:2]
    c,cf=depth_near(dep,0.32,0.68,0.32,0.78)
    l,lf=depth_near(dep,0.14,0.40,0.32,0.82)
    rr,rf=depth_near(dep,0.60,0.86,0.32,0.82)
    low,lowf=depth_near(dep,0.25,0.75,0.58,0.92)
    vals=[v for v in (c,l,rr,low) if v is not None]
    if not vals: return None
    side_cage=(l is not None and rr is not None and l<CHAIR_SIDE and rr<CHAIR_SIDE and (c is None or c<0.95))
    center_bar=(c is not None and c<CHAIR_CENTER and max(l or 9.9,rr or 9.9)<0.95)
    low_rail=(low is not None and low<0.42 and lowf>0.015)
    return {"center":c,"left":l,"right":rr,"low":low,"frac":max(cf,lf,rf,lowf),"cage":side_cage or center_bar or low_rail}
def fused_front(lidar_front,bumper):
    d=bumper.get("center") if bumper else None
    if d is None: return lidar_front,None
    return min(lidar_front,d),d
def guarded_reverse_turn(a,r,turn_sign,dur=0.55):
    # Rear is BLIND (lidar ~219deg FOV; base rear uncovered) -> never translate backward.
    # Rotate in place to find an opening within the covered front arc.
    t0=time.time()
    while time.time()-t0<dur and not os.path.exists(STOP):
        drive(0.0,turn_sign*ANG); spin(); time.sleep(0.04)

VIS=[]
def vision(tag,pan):
    m=F.get("m")
    if not m: return
    img=br.imgmsg_to_cv2(m,desired_encoding="bgr8"); path="%s/%s.jpg"%(OUT,tag); cv2.imwrite(path,img)
    b=base64.b64encode(cv2.imencode(".jpg",img)[1]).decode()
    q="Robot eyes scanning a room. List notable objects, furniture, points of interest. Short bullets. If blank wall/floor, say so."
    p={"model":MODEL,"max_tokens":200,"temperature":0.2,"messages":[{"role":"user","content":[{"type":"text","text":q},{"type":"image_url","image_url":{"url":"data:image/jpeg;base64,"+b}}]}]}
    try:
        req=urllib.request.Request(URL,data=json.dumps(p).encode(),headers={"Content-Type":"application/json"})
        d=json.load(urllib.request.urlopen(req,timeout=60))["choices"][0]["message"]["content"].strip()
    except Exception as ex: d="[vision err] %r"%ex
    pp=pose(); VIS.append({"tag":tag,"pan":pan,"pose":[round(pp[0],2),round(pp[1],2)] if pp else None,"img":os.path.basename(path),"desc":d})
    log("  VIS[%s]"%tag, d.replace("\n"," | ")); json.dump(VIS,open(OUT+"/scout.json","w"),indent=1)
def spin_scan(c):
    log(" spin-scan",c); dur=2*math.pi/ANG; t0=time.time(); shot=0; steps=[("a",160),("b",500),("c",840)]
    while time.time()-t0<dur and not os.path.exists(STOP):
        drive(0,ANG); spin()
        if shot<3 and (time.time()-t0)>dur*(shot+0.5)/3:
            stop_move(); tag,pan=steps[shot]; aim(pan); time.sleep(1.4); vision("c%d_%s"%(c,tag),pan); shot+=1
        time.sleep(0.02)
    stop_move()
def frontiers():
    m=M.get("m")
    if not m: return []
    w=m.info.width; h=m.info.height; res=m.info.resolution; ox=m.info.origin.position.x; oy=m.info.origin.position.y
    g=np.array(m.data,dtype=np.int16).reshape(h,w)
    free=(g>=0)&(g<50); unk=(g<0)
    fr=np.zeros((h,w),bool)
    fr[1:-1,1:-1]=free[1:-1,1:-1]&(unk[:-2,1:-1]|unk[2:,1:-1]|unk[1:-1,:-2]|unk[1:-1,2:])
    ys,xs=np.where(fr); buck={}
    for x,y in zip(xs,ys):
        k=(x//8,y//8); buck.setdefault(k,[]).append((ox+(x+.5)*res, oy+(y+.5)*res))
    return [(sum(p[0] for p in v)/len(v),sum(p[1] for p in v)/len(v),len(v)) for v in buck.values() if len(v)>=3]

visited=[]
def nearest_frontier(pp):
    cs=frontiers(); best=None; bd=1e9
    for cx,cy,sz in cs:
        if any(math.hypot(cx-vx,cy-vy)<0.8 for vx,vy in visited): continue
        d=math.hypot(cx-pp[0],cy-pp[1])
        if d<bd: bd=d; best=(cx,cy)
    return best, len([1 for cx,cy,sz in cs if not any(math.hypot(cx-vx,cy-vy)<0.8 for vx,vy in visited)])
def best_heading(a,r,rel):
    best=None; bs=-1e9
    for hd in range(-100,101,20):        # covered front arc only (lidar ~219deg, rear blind)
        th=math.radians(hd); c=clear_at(a,r,th,0.26)
        if c>=9.8: continue              # no lidar data in this sector -> unknown, not free
        if c<MOVE_CLR: continue
        s=min(c,3.0)-0.2*abs(th)
        if rel is not None: s+=2.2*math.cos(th-rel)
        if s>bs: bs=s; best=th
    return best
def save_map(name):
    try: subprocess.run(["zsh","-lc","source ~/.zshrc; ros2 run nav2_map_server map_saver_cli -f /home/ubuntu/ros2_ws/src/slam/maps/%s --ros-args -p save_map_timeout:=8.0"%name],timeout=40)
    except Exception: pass

log("=== SCOUT2 START",time.strftime("%H:%M:%S"),"===")
aim(500); time.sleep(0.8)
t0=time.time(); cyc=0; last_spin=0; last_pp=None; last_move_t=time.time(); last_dbg=0; done=0
while time.time()-t0<MAX_RUN and not os.path.exists(STOP):
    if time.time()-last_spin>SPIN_EVERY:
        stop_move(); spin_scan(cyc); aim(500); time.sleep(0.8); save_map("map_02"); pp=pose()
        if pp: visited.append((pp[0],pp[1]))
        _,nf=nearest_frontier(pp) if pp else (None,0)
        log("  unvisited frontiers:",nf)
        done = (done+1) if (pp and nf==0) else 0
        if done>=2: log("  EXPLORATION COMPLETE"); break
        last_spin=time.time(); cyc+=1; last_move_t=time.time(); last_pp=pp; continue
    a,r=scan_arr(); pp=pose()
    if pp is None or a is None: spin(); continue
    goal,_=nearest_frontier(pp)
    rel=None
    if goal: rel=math.atan2(math.sin(math.atan2(goal[1]-pp[1],goal[0]-pp[0])-pp[2]),1)
    lidar_front=clear_at(a,r,0.0,0.32)
    bumper=depth_bumper()
    front,depth= fused_front(lidar_front,bumper)
    if depth is not None and depth<DEPTH_MOVE_CLR and time.time()-D.get("lastlog",0)>2.0:
        log("  depth front %.2fm lidar %.2fm"%(depth,lidar_front)); D["lastlog"]=time.time()
    if bumper and bumper.get("cage"):
        lc=clear_at(a,r,math.radians(70),0.35); rc=clear_at(a,r,math.radians(-70),0.35)
        turn=1 if lc>=rc else -1  # steer to the LIDAR-open side (depth FOV too narrow to see the opening)
        if time.time()-D.get("cagelog",0)>2.0:
            log("  CHAIR cage c=%s l=%s r=%s low=%s rear %.2f -> escape"%(
                "%.2f"%bumper["center"] if bumper.get("center") is not None else "none",
                "%.2f"%bumper["left"] if bumper.get("left") is not None else "none",
                "%.2f"%bumper["right"] if bumper.get("right") is not None else "none",
                "%.2f"%bumper["low"] if bumper.get("low") is not None else "none",
                clear_at(a,r,math.pi,0.35)))
            D["cagelog"]=time.time()
        guarded_reverse_turn(a,r,turn,0.8); continue
    if depth is not None and depth<DEPTH_HARD:
        if time.time()-D.get("hardlog",0)>2.0:
            log("  depth HARD %.2fm lidar %.2fm -> turn"%(depth,lidar_front)); D["hardlog"]=time.time()
        lc=clear_at(a,r,math.radians(60),0.4); rc=clear_at(a,r,math.radians(-60),0.4)
        guarded_reverse_turn(a,r,1 if lc>=rc else -1,0.35); continue
    if front<max(HARD,DEPTH_HARD if depth is not None else HARD):
        # back off slightly + rotate toward more open side
        lc=clear_at(a,r,math.radians(60),0.4); rc=clear_at(a,r,math.radians(-60),0.4)
        guarded_reverse_turn(a,r,1 if lc>=rc else -1,0.35); continue
    th=best_heading(a,r,rel)
    if th is None: drive(0,ANG); spin(); time.sleep(0.05); continue
    az=max(-ANG,min(ANG,1.3*th))
    move_clr=max(MOVE_CLR,DEPTH_MOVE_CLR if depth is not None else MOVE_CLR)
    creep_floor=max(0.38,DEPTH_HARD if depth is not None else 0.38)
    creep=0.045 if (depth is not None and depth<DEPTH_SLOW) else 0.07
    lx=LIN if (abs(th)<0.45 and front>move_clr) else (creep if front>creep_floor else 0.0)
    if time.time()-last_dbg>3.0:
        log("  drive lx %.2f az %.2f th %.2f front %.2f lidar %.2f depth %s"%(lx,az,th,front,lidar_front,"%.2f"%depth if depth is not None else "none"))
        last_dbg=time.time()
    drive(lx,az); spin(); time.sleep(0.05)
    # stuck escape
    if last_pp and math.hypot(pp[0]-last_pp[0],pp[1]-last_pp[1])>0.15: last_move_t=time.time(); last_pp=pp
    elif last_pp is None: last_pp=pp
    if time.time()-last_move_t>14:
        lc=clear_at(a,r,math.radians(60),0.4); rc=clear_at(a,r,math.radians(-60),0.4)
        log("  stuck -> guarded reverse escape rear %.2f"%(clear_at(a,r,math.pi,0.35)))
        guarded_reverse_turn(a,r,1 if lc>=rc else -1,1.4)
        t1=time.time()
        while time.time()-t1<0.8 and not os.path.exists(STOP): drive(0,ANG if lc>=rc else -ANG); spin(); time.sleep(0.03)
        last_move_t=time.time(); last_pp=pose()
stop_move(); aim(500); save_map("map_02")
log("=== SCOUT2 DONE",time.strftime("%H:%M:%S"),"cycles=%d"%cyc,"===")
json.dump(VIS,open(OUT+"/scout.json","w"),indent=1)
n.destroy_node(); rclpy.shutdown()
