Header header

uint8 CHARGING_NOT_CHARGING = 0
uint8 CHARGING_CHARGING_RECOVERY = 1
uint8 CHARGING_CHARGING = 2
uint8 CHARGING_TRICKLE_CHARGING = 3
uint8 CHARGING_WAITING = 4 
uint8 CHARGING_CHARGING_ERROR = 5

uint8 bumps_wheeldrops
bool wall
bool cliff_left
bool cliff_front_left
bool cliff_front_right
bool cliff_right
bool virtual_wall
uint8 motor_overcurrents
uint8 dirt_detector_left  #roomba_only
uint8 dirt_detector_right #roomba_only
uint8 remote_opcode
uint8 buttons
int16 distance  # mm
int16 angle # degrees
uint8 charging_state
uint16 voltage  # mV
int16 current  # mA
int8 temperature  # C
uint16 charge  # mAh
uint16 capacity  # mAh
