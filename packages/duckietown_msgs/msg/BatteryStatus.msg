Header header

float32 charge   # Between 0 and 1. 0 -> Dead battery, 1 -> full battery
float32 voltage  # Raw voltage of the battery cell. Generally between 3.2V and 4.2V
float32 current     # Current, in amps. Negative value means the battery is discharging. Positive value means the battery is charging.
int32 time_to_full  # Estimated time to full, in minutes, if charging. A value of 65535 means invalid.
int32 time_to_empty # Estimated time to empty, in minutes, if discharging. A value of 65535 means invalid.
int32 temperature   # Temperature of the battery, in celsius
