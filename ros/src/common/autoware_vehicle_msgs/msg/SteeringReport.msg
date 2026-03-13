# Steering tire angle report from the vehicle interface.
# Published at ~50 Hz.
#
# Note: This message uses a bare timestamp (not a full Header)
# to match Autoware universe conventions.

builtin_interfaces/Time stamp
float32 steering_tire_angle  # [rad] positive = left turn
