# USB-C Power Delivery status
std_msgs/Header header

# Raw PD status byte
uint8 pd_status_raw

# Port status byte
uint8 port_status_raw

# Type-C connection status
bool type_c_connected

# PD contract active
bool pd_contract_active

# Current PD voltage profile (1-5)
uint8 current_profile

# Negotiated voltage in millivolts
uint16 negotiated_voltage_mv

# Negotiated current in milliamps
uint16 negotiated_current_ma