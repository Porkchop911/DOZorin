import sys, time, rclpy
from servo_controller_msgs.msg import ServosPosition
from servo_controller.bus_servo_control import set_servo_position
a=[float(x) for x in sys.argv[1:]]
dur=a[0]; pairs=tuple((int(a[i]),int(a[i+1])) for i in range(1,len(a),2))
rclpy.init(); n=rclpy.create_node('arm_pose')
pub=n.create_publisher(ServosPosition,'servo_controller',1); time.sleep(0.6)
set_servo_position(pub,dur,pairs); time.sleep(dur+0.6)
print("arm sent:",pairs); n.destroy_node(); rclpy.shutdown()
