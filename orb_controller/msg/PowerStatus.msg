# Power status information from EZ-PD BCR chip
std_msgs/Header header

# Voltage in millivolts
uint16 voltage_mv

# Current in milliamps
uint16 current_ma

# Power in milliwatts (calculated)
uint32 power_mw

# Whether PD is connected and active
bool pd_active

# PD profile number (1=5V, 2=9V, 3=12V, 4=15V, 5=20V)
uint8 pd_profile