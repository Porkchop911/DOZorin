# Wire-level wheel speeds in rev/s, order m1..m4 = FL, RL, FR, RR,
# signs exactly as sent to the controller (right side already inverted
# by the kinematics layer). Stamp used for staleness checks.
std_msgs/Header header
float32[4] rps
