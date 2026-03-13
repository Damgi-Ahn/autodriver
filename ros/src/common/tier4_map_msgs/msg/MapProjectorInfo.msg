# Map projection metadata used to convert GNSS (WGS84) coordinates
# to the local map frame (ENU or MGRS/UTM).
#
# projector_type constants:
#   LOCAL — simple flat-earth ENU approximation around map_origin
#   MGRS  — Military Grid Reference System
#   UTM   — Universal Transverse Mercator

string LOCAL="local"
string MGRS="MGRS"
string UTM="UTM"

string projector_type   # one of the constants above
string vertical_datum   # e.g. "WGS84", "EGM2008"

geographic_msgs/GeoPoint map_origin  # WGS84 reference origin (lat/lon/alt)
