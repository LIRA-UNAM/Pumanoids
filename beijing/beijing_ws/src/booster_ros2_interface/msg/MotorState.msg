int8 mode
float32 q
float32 dq
float32 ddq
float32 tau_est
# Motor temperature in degrees Celsius. Expected range: -100 to 150.
int8 temperature
# Motor packet-loss count. Expected range: 0 to 9,999,999,999.
uint32 lost
# Motor status array: index 0 is the error bitmask (0-255); index 1 is the current communication rate (0-800 Hz).
uint32[2] reserve
