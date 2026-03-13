# Vehicle velocity report from odometry or speed sensor.
# Published at ~50 Hz by the vehicle interface.

std_msgs/Header header
float32 longitudinal_velocity  # [m/s]  forward-positive
float32 lateral_velocity       # [m/s]  left-positive
float32 heading_rate           # [rad/s] CCW-positive (yaw rate)
