# float32 fodlatitude
# float32 fodlongitude

# #fod size (cm)
# float32 fodsize

# #fod hazard level (ex 1,2,3)
# uint8 hazardlevel

# #fodtype 
# uint8 fodtype

# #fodimage
# sensor_msgs/CompressedImage fodimage

uint8 INIT = 0x00
uint8 WAIT = 0x01
uint8 FAIL = 0x02
uint8 CLEAN = 0x03

uint8 id
geometry_msgs/Pose pose # point 대신 pose로 만들고 현재차량 odom에서 orientation 넣어서 pose 완성시키고 fod DB에 저장하기
geometry_msgs/Point fod_local_point # 카메라 Detection Object Info

float64 y_offset

uint8 ego_lanelet_id
uint8 fod_lanelet_id

uint8 classification
uint8 count
bool is_clean
uint8 state

