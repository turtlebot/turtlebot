uint8 OI_MODE_OFF = 0
uint8 OI_MODE_PASSIVE = 1
uint8 OI_MODE_SAFE = 2
uint8 OI_MODE_FULL = 3

uint8 REMOTE_LEFT = 129
uint8 REMOTE_FORWARD = 130 
uint8 REMOTE_RIGHT = 131 
uint8 REMOTE_SPOT = 132 
uint8 REMOTE_MAX = 133 
uint8 REMOTE_SMALL = 134 
uint8 REMOTE_MEDIUM = 135 
uint8 REMOTE_LARGE = 136 
uint8 REMOTE_CLEAN = 136 
uint8 REMOTE_PAUSE = 137 
uint8 REMOTE_POWER = 138 
uint8 REMOTE_ARC_LEFT = 139 
uint8 REMOTE_ARC_RIGHT = 140 
uint8 REMOTE_DRIVE_STOP = 141 
# Scheduling remote
uint8 REMOTE_SEND_ALL = 142 
uint8 REMOTE_SEEK_DOCK = 143 
# Home base
uint8 REMOTE_RESERVED = 240 
uint8 REMOTE_FORCE_FIELD = 242 
uint8 REMOTE_GREEN_BUOY = 244 
uint8 REMOTE_GREEN_BUOY_AND_FORCE_FIELD = 246 
uint8 REMOTE_RED_BUOY = 248 
uint8 REMOTE_RED_BUOY_AND_FORCE_FIELD = 250 
uint8 REMOTE_RED_BUOY_AND_GREEN_BUOY = 252 
uint8 REMOTE_RED_BUOY_AND_GREEN_BUOY_AND_FORCE_FIELD = 254 
uint8 REMOTE_NONE = 255

uint8 CHARGING_NOT_CHARGING = 0
uint8 CHARGING_CHARGING_RECOVERY = 1
uint8 CHARGING_CHARGING = 2
uint8 CHARGING_TRICKLE_CHARGING = 3
uint8 CHARGING_WAITING = 4 
uint8 CHARGING_CHARGING_ERROR = 5

Header header

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

uint16 wall_signal
uint16 cliff_left_signal
uint16 cliff_front_left_signal
uint16 cliff_front_right_signal
uint16 cliff_right_signal
uint8 user_digital_inputs
uint16 user_analog_input
uint8 charging_sources_available
uint8 oi_mode
uint8 song_number
bool song_playing

uint8 number_of_stream_packets
int16 requested_velocity  # mm/s
int16 requested_radius  # mm
int16 requested_right_velocity  # mm/s
int16 requested_left_velocity  # mm/s

