uint8 PARKING_POS = 0
uint8 REVERSE_POS = 1
uint8 NEUTRAL_POS = 2
uint8 DRIVING_POS = 3

std_msgs/Header header
float64 speed
float64 float_speed
float64 steering_wheel_angle
float64 target_steering_wheel_angle
float64 steering_angle
float64 steering_torque
uint16 steering_speed
uint16 steering_status
uint16 gear_pos

uint16 engine_speed

uint8 accel_pedal
uint8 accel_pedal_switch

uint8 brake_pedal

uint8 turn_signal

bool controller_status

float64 mileage

bool power
bool emergency_button
bool start_button

uint8 SPEED_UP = 0x01
uint8 SPEED_DOWN = 0x02
uint8 EMERGENCY = 0x04
uint8 speed_button