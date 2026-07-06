#!/usr/bin/env python3
# find_places.py [--drive] — locate named targets (desk/door/drums/window) by
# arm-cam pan sweep + VLM + depth range, propose map-frame standoff poses,
# optionally drive to each (goto.py), photo-verify, and write ~/places.yaml.
import os, sys, math, time, json, base64, subprocess, urllib.request
import numpy as np, yaml, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition
import tf2_ros

TARGETS = ["big plant", "kitchen", "tv", "shoes"]
URL = "http://192.168.0.148:1234/v1/chat/completions"; MODEL = "qwen/qwen3-vl-4b"
MAPY = os.path.expanduser("~/ros2_ws/src/slam/maps/map_rtab.yaml")
PANS = [140, 320, 500, 680, 860]           # right .. left
RAD_PER_PULSE = 0.00419                     # bus servo ~240deg/1000
HFOV = 1.15                                 # rgb horizontal fov ~66deg
STANDOFF = 0.70; CLEAR = 0.30
OUT = os.path.expanduser("~/scout_out"); os.makedirs(OUT, exist_ok=True)

rclpy.init(); n = Node("find_places"); br = CvBridge()
servo = n.create_publisher(ServosPosition, "/ros_robot_controller/bus_servo/set_position", 1)
F = {}; D = {}
n.create_subscription(Image, "/depth_cam/rgb0/image_raw", lambda m: F.__setitem__("m", m), 5)
n.create_subscription(Image, "/depth_cam/depth0/image_raw", lambda m: D.__setitem__("m", m), 5)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
def spin(t=0.05): rclpy.spin_once(n, timeout_sec=t)
for _ in range(40): spin()

def aim(pan):
    m = ServosPosition(); m.duration = 1.2
    for sid, pos in ((1, pan), (2, 701), (3, 120), (4, 300), (5, 500)):
        sp = ServoPosition(); sp.id = sid; sp.position = pos; m.position.append(sp)
    for _ in range(3): servo.publish(m); time.sleep(0.1)
    t0 = time.time()
    while time.time() - t0 < 2.4: spin()

def _yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def pose():
    for _ in range(8):
        try:
            mo = buf.lookup_transform("map", "odom", rclpy.time.Time())
            ob = buf.lookup_transform("odom", "base_footprint", rclpy.time.Time())
            my = _yaw(mo.transform.rotation)
            bx, by = ob.transform.translation.x, ob.transform.translation.y
            x = mo.transform.translation.x + bx*math.cos(my) - by*math.sin(my)
            y = mo.transform.translation.y + bx*math.sin(my) + by*math.cos(my)
            return x, y, my + _yaw(ob.transform.rotation)
        except Exception: spin(); time.sleep(0.1)
    return None

def frames():
    F.pop("m", None); D.pop("m", None)
    t0 = time.time()
    while ("m" not in F or "m" not in D) and time.time() - t0 < 12: spin()
    assert "m" in F and "m" in D, "no camera frames within 12s"
    rgb = br.imgmsg_to_cv2(F["m"], "bgr8")
    dep = br.imgmsg_to_cv2(D["m"], desired_encoding="passthrough").astype(np.float32)
    if np.nanmax(dep) > 20.0: dep *= 0.001
    return rgb, dep

def ask_vlm(img, q):
    b = base64.b64encode(cv2.imencode(".jpg", img)[1]).decode()
    p = {"model": MODEL, "max_tokens": 300, "temperature": 0.1, "messages": [{"role": "user",
        "content": [{"type": "text", "text": q},
                    {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + b}}]}]}
    req = urllib.request.Request(URL, data=json.dumps(p).encode(), headers={"Content-Type": "application/json"})
    return json.load(urllib.request.urlopen(req, timeout=90))["choices"][0]["message"]["content"]

Q = ('You are a robot scanning a room. Which of these are CLEARLY visible: big plant, kitchen, tv, shoes? '
     'Reply ONLY JSON like {"big plant":{"visible":true,"x":0.4},"kitchen":{"visible":false}} — '
     'x = horizontal center of the object in the image, 0.0=left edge, 1.0=right edge. '
     'big plant = a large potted houseplant. kitchen = kitchen counter, island or cabinets. '
     'tv = a television screen. shoes = shoes on the floor by a wall or door. The dark shapes at the very bottom edge of the image are the robot own body - NEVER report those as anything. '
     'Include all four keys exactly: "big plant","kitchen","tv","shoes".')

def depth_at(dep, xfrac):
    h, w = dep.shape[:2]
    c = int(xfrac * w); x0, x1 = max(0, c-30), min(w, c+30)
    roi = dep[int(h*0.25):int(h*0.85), x0:x1]
    v = roi[np.isfinite(roi) & (roi > 0.25) & (roi < 6.0)]
    return float(np.median(v)) if len(v) > 50 else None

# map for free-cell check
meta = yaml.safe_load(open(MAPY))
grid = cv2.imread(os.path.join(os.path.dirname(MAPY), meta["image"]), cv2.IMREAD_GRAYSCALE)
res = meta["resolution"]; ox, oy = meta["origin"][0], meta["origin"][1]
free = (grid > 220).astype(np.uint8)  # trinary: 254 free
k = int(CLEAR/res)*2+1
free_clear = cv2.erode(free, np.ones((k, k), np.uint8))
def is_free(x, y):
    c = int((x-ox)/res); r = grid.shape[0]-1-int((y-oy)/res)
    return 0 <= r < grid.shape[0] and 0 <= c < grid.shape[1] and free_clear[r, c] > 0

pp = None
t0 = time.time()
while pp is None and time.time() - t0 < 15:
    spin(); pp = pose()
assert pp, "no map pose after 15s"
print("robot at x=%.2f y=%.2f yaw=%.1fdeg" % (pp[0], pp[1], math.degrees(pp[2])))
sightings = {}
for pan in PANS:
    aim(pan)
    rgb, dep = frames()
    cv2.imwrite("%s/fp_pan%d.jpg" % (OUT, pan), rgb)
    try:
        r = ask_vlm(rgb, Q)
        j = json.loads(r[r.index("{"): r.rindex("}")+1])
    except Exception as e:
        print("pan %d VLM fail: %r" % (pan, e)); continue
    for t in TARGETS:
        o = j.get(t) or {}
        if not o.get("visible"): continue
        xf = float(o.get("x", 0.5))
        d = depth_at(dep, xf)
        bearing = pp[2] + (pan-500)*RAD_PER_PULSE + (0.5-xf)*HFOV
        cen = abs(xf-0.5)
        print("  pan %3d: %-6s x=%.2f d=%s bearing=%.0fdeg" % (pan, t, xf, ("%.2fm" % d) if d else "?", math.degrees(bearing)))
        if t not in sightings or cen < sightings[t]["cen"]:
            sightings[t] = {"bearing": bearing, "d": d, "cen": cen, "pan": pan}
aim(500)

places = {}
for t, s in sightings.items():
    d = s["d"] or 1.5
    for back in np.arange(0, d, 0.15):
        g = max(0.35, d - STANDOFF - back)
        gx = pp[0] + g*math.cos(s["bearing"]); gy = pp[1] + g*math.sin(s["bearing"])
        if is_free(gx, gy):
            places[t] = {"x": round(gx, 2), "y": round(gy, 2), "yaw_deg": round(math.degrees(s["bearing"])) % 360}
            break
missing = [t for t in TARGETS if t not in places]
print("\nPROPOSED:", json.dumps(places, indent=1))
if missing: print("NOT FOUND:", missing)

if "--drive" in sys.argv and places:
    recorded = {}
    for t, g in places.items():
        print("\n=== driving to %s (%.2f,%.2f,%d) ===" % (t, g["x"], g["y"], g["yaw_deg"]))
        rc = subprocess.call(["python3", os.path.expanduser("~/goto.py"), str(g["x"]), str(g["y"]), str(g["yaw_deg"])])
        time.sleep(1.0)
        cur = pose()
        rgb, _ = frames(); cv2.imwrite("%s/fp_arrived_%s.jpg" % (OUT, t), rgb)
        if rc == 0 and cur:
            recorded[t] = {"x": round(cur[0], 2), "y": round(cur[1], 2), "yaw_deg": round(math.degrees(cur[2])) % 360}
            print("RECORDED %s at (%.2f,%.2f)" % (t, cur[0], cur[1]))
        else:
            print("FAILED %s (goto rc=%s)" % (t, rc))
    if recorded:
        hdr = "# Named waypoints for MAP_04 (recorded by find_places.py %s)\n" % time.strftime("%Y-%m-%d %H:%M")
        open(os.path.expanduser("~/places.yaml"), "w").write(hdr + yaml.safe_dump(recorded))
        print("\nplaces.yaml written:", list(recorded))
n.destroy_node(); rclpy.shutdown()
