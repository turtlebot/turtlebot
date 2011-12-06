uint8 DISCHARGING = 0
uint8 CHARGING    = 1
uint8 CHARGED     = 2

Header  header
float32 voltage          # Voltage in Volts
float32 rate             # Negative when discharging (A)
float32 charge           # Current charge in Ah
float32 capacity         # Capacity in Ah (last full capacity)
float32 design_capacity  # Capacity in Ah (design capacity)
int32   percentage       # Charge percentage
uint8   charge_state     # Enum 
bool    present          # Should be an error if battery is not present