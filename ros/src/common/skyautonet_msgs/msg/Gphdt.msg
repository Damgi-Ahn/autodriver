# GNSS heading message derived from NMEA GPHDT sentence.
#
# heading: Bearing angle in degrees, CW+ from East (non-standard convention).
#   To convert to standard ENU yaw (CCW+ from East):
#   yaw_rad = -(heading_deg * pi / 180)
#
# status: GNSS fix quality, same convention as sensor_msgs/NavSatStatus:
#   -1 : STATUS_NO_FIX  — no fix
#    0 : STATUS_FIX     — RTK Fixed
#    1 : STATUS_SBAS_FIX — RTK Float / SBAS
#    2 : STATUS_GBAS_FIX — other

std_msgs/Header header
float64 heading                  # [deg] CW+ from East
sensor_msgs/NavSatStatus status
