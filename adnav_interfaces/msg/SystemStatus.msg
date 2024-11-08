# System Status bits of ANPP packet 20
#
# Specified using the Advanced Navigation Packet Protocol

# This field contains 16 bits that indicate problems with the system. 
# These are boolean fields with a zero indicating false and one indicating true.
bool system_failure 0
bool accelerometer_sensor_failure 0
bool gyroscope_sensor_failure 0 
bool magnetometer_sensor_failure 0
bool pressure_sensor_failure 0
bool gnss_failure 0
bool accelerometer_over_range 0
bool gyroscope_over_range 0
bool magnetometer_over_range 0
bool pressure_over_range 0
bool minimum_temperature_alarm 0
bool maximum_temperature_alarm 0
bool internal_data_logging_error 0
bool high_voltage_alarm 0
bool gnss_antenna_fault 0
bool serial_port_overflow_alarm 0