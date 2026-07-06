import sys, rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
out = sys.argv[1] if len(sys.argv)>1 else "/tmp/cap.jpg"
rclpy.init(); n=Node("capframe"); br=CvBridge(); got={}
def cb(m): got["m"]=m
n.create_subscription(Image,"/depth_cam/rgb0/image_raw",cb,10)
import time; t0=time.time()
while "m" not in got and time.time()-t0<5: rclpy.spin_once(n,timeout_sec=0.2)
m=got.get("m")
if m is None: print("NO FRAME"); raise SystemExit(1)
img=br.imgmsg_to_cv2(m,desired_encoding="bgr8")
cv2.imwrite(out,img)
print("saved",out,img.shape,"enc",m.encoding)
n.destroy_node(); rclpy.shutdown()
