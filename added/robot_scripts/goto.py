import sys, math, time, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

gx=float(sys.argv[1]); gy=float(sys.argv[2]); gyaw=math.radians(float(sys.argv[3]) if len(sys.argv)>3 else 0.0)

rclpy.init(); n=Node("goto")
ac=ActionClient(n,NavigateToPose,"navigate_to_pose")
print("waiting for nav2 action server...")
ac.wait_for_server()
goal=NavigateToPose.Goal()
goal.pose.header.frame_id="map"
goal.pose.header.stamp=n.get_clock().now().to_msg()
goal.pose.pose.position.x=gx; goal.pose.pose.position.y=gy
goal.pose.pose.orientation.z=math.sin(gyaw/2); goal.pose.pose.orientation.w=math.cos(gyaw/2)

last=[time.time()]
def fb(f):
    d=f.feedback.distance_remaining
    if time.time()-last[0]>1.5:
        p=f.feedback.current_pose.pose.position
        print("  dist_remaining=%.2f  pose=(%.2f,%.2f)  recoveries=%d"%(d,p.x,p.y,f.feedback.number_of_recoveries))
        last[0]=time.time()

sh=ac.send_goal_async(goal,feedback_callback=fb)
rclpy.spin_until_future_complete(n,sh)
gh=sh.result()
if not gh.accepted:
    print("GOAL REJECTED"); rclpy.shutdown(); raise SystemExit
print("goal accepted, navigating...")
rf=gh.get_result_async()
t0=time.time()
while not rf.done() and time.time()-t0<90:
    rclpy.spin_once(n,timeout_sec=0.3)
if rf.done():
    status=rf.result().status
    names={4:"SUCCEEDED",5:"CANCELED",6:"ABORTED"}
    print("RESULT:",names.get(status,status))
else:
    print("TIMEOUT after 90s, canceling"); gh.cancel_goal_async(); rclpy.spin_once(n,timeout_sec=1.0)
n.destroy_node(); rclpy.shutdown()
