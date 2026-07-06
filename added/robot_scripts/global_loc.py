import math, yaml, numpy as np, rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
from PIL import Image
from scipy import ndimage

import os; MAPY = os.environ.get("MAPY", "/home/ubuntu/ros2_ws/src/slam/maps/map_04.yaml")
with open(MAPY) as f: meta = yaml.safe_load(f)
res = meta["resolution"]; ox, oy = meta["origin"][0], meta["origin"][1]
img = np.array(Image.open("/home/ubuntu/ros2_ws/src/slam/maps/"+meta["image"]))
if img.ndim==3: img=img[:,:,0]
H,W = img.shape
occ = img < 90
free = img > 220
dist_px = ndimage.distance_transform_edt(~occ)   # px distance to nearest wall

rclpy.init(); n=Node("global_loc")
buf=tf2_ros.Buffer(); tf2_ros.TransformListener(buf,n)
scan={}
n.create_subscription(LaserScan, "/scan", lambda m: scan.__setitem__("m",m), 10)
t0=time.time()
while "m" not in scan and time.time()-t0<5: rclpy.spin_once(n,timeout_sec=0.2)
m=scan["m"]; frame=m.header.frame_id

# static base_footprint -> lidar_frame
bl=None; t0=time.time()
while time.time()-t0<5:
    try: bl=buf.lookup_transform("base_footprint", frame, rclpy.time.Time()); break
    except Exception: rclpy.spin_once(n,timeout_sec=0.2)
blx=bl.transform.translation.x; bly=bl.transform.translation.y
q=bl.transform.rotation
bl_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

# local scan points in lidar frame
ang=m.angle_min; pts=[]
for r in m.ranges:
    if math.isfinite(r) and m.range_min<r<min(r+1e9,m.range_max) and r<8.0:
        pts.append((r*math.cos(ang), r*math.sin(ang)))
    ang+=m.angle_increment
pts=np.array(pts).T  # 2xN
N=pts.shape[1]
print("scan pts",N,"base->lidar dx=%.3f dy=%.3f yaw=%.1f"%(blx,bly,math.degrees(bl_yaw)))

def score_pose(bx,by,byaw):
    # lidar pose in map
    lx = bx + blx*math.cos(byaw)-bly*math.sin(byaw)
    ly = by + blx*math.sin(byaw)+bly*math.cos(byaw)
    lyaw = byaw+bl_yaw
    c,s=math.cos(lyaw),math.sin(lyaw)
    wx = lx + c*pts[0]-s*pts[1]
    wy = ly + s*pts[0]+c*pts[1]
    col=((wx-ox)/res).astype(int)
    row=(H-1-(wy-oy)/res).astype(int)
    ok=(col>=0)&(col<W)&(row>=0)&(row<H)
    if ok.sum()<10: return -1
    d=dist_px[row[ok],col[ok]]
    return np.exp(-(d*res)**2/(2*0.1**2)).sum()  # gaussian likelihood, sigma 10cm

# candidate base positions: free cells subsampled
fr,fc = np.where(free)
sub = np.arange(0,len(fr),3)
cand_x = ox + fc[sub]*res
cand_y = oy + (H-1-fr[sub])*res
print("candidate positions:",len(sub))

best=(-1,0,0,0)
for yaw_deg in range(0,360,12):
    byaw=math.radians(yaw_deg)
    for bx,by in zip(cand_x,cand_y):
        sc=score_pose(bx,by,byaw)
        if sc>best[0]: best=(sc,bx,by,byaw)
print("coarse best score=%.1f x=%.2f y=%.2f yaw=%.0f"%(best[0],best[1],best[2],math.degrees(best[3])))

# refine
bx0,by0,byaw0=best[1],best[2],best[3]
for _ in range(2):
    improved=False
    for dyaw in np.radians(np.arange(-12,13,2)):
        for dx in np.arange(-0.15,0.16,0.03):
            for dy in np.arange(-0.15,0.16,0.03):
                sc=score_pose(bx0+dx,by0+dy,byaw0+dyaw)
                if sc>best[0]:
                    best=(sc,bx0+dx,by0+dy,byaw0+dyaw); improved=True
    bx0,by0,byaw0=best[1],best[2],best[3]
    if not improved: break
sc,bx,by,byaw=best
print("REFINED score=%.1f x=%.3f y=%.3f yaw=%.1fdeg (max possible=%d)"%(sc,bx,by,math.degrees(byaw),N))

# publish to /initialpose
pub=n.create_publisher(PoseWithCovarianceStamped,"/initialpose",10)
msg=PoseWithCovarianceStamped()
for i in range(6):
    msg.header.frame_id="map"; msg.header.stamp=n.get_clock().now().to_msg()
    msg.pose.pose.position.x=float(bx); msg.pose.pose.position.y=float(by)
    msg.pose.pose.orientation.z=math.sin(byaw/2); msg.pose.pose.orientation.w=math.cos(byaw/2)
    cov=[0.0]*36; cov[0]=0.1; cov[7]=0.1; cov[35]=0.03; msg.pose.covariance=cov
    pub.publish(msg); rclpy.spin_once(n,timeout_sec=0.2); time.sleep(0.15)
print("published best pose to /initialpose")
n.destroy_node(); rclpy.shutdown()
