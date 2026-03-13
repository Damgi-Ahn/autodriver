# constants for driving status
# Commn Drving Status
uint8 INIT          = 0x00  
uint8 MANUAL        = 0x01 
uint8 AUTONOMOUS    = 0x02
uint8 ROUTE_CREATE  = 0x03
uint8 LOCAL         = 0x04
uint8 REMOTE        = 0x05
uint8 EMERGENCY     = 0x06
# FODRO Status
uint8 DAS_CONTROL   = 0x07

# Planning mode
# Common Planning mode 
uint8 PLANNING_INIT        = 0x00
uint8 AUTO_REROUTE         = 0x01
# Maxen Planning Mode
uint8 LANE_CHANGE          = 0x02
# FOD Planning mode
uint8 FOD_PLANNING         = 0x03
uint8 RETURN_TO_BASE       = 0x04
uint8 SERVER_ASSISTED      = 0x05

# Lanechange direction
uint8 NONE=1
uint8 LEFT=2
uint8 RIGHT=3

#variables
uint8 status
uint8 dir
uint8 planning_mode
string[] list
string[] error_list
