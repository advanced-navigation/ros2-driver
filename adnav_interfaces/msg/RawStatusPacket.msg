# This contains the raw data structure of ANPP Packet 20. The System Status Packet. 
#
# Specified using the Advanced Navigation Packet Protocol

# header.stamp specifies the ROS time for this measurement (the
#        time of the system is recorded in the Unix Time Section
#
# header.frame_id is the frame of reference of the device.
std_msgs/Header header

# ANPP Header
adnav_interfaces/ANPPHeader anpp_header

# System Status 16 Bit boolean array 
adnav_interfaces/SystemStatus system_status

# Filter Status 16 Bit boolean array
adnav_interfaces/FilterStatus filter_status

# This field provides UTC time in seconds since January 1, 1970
# including leap seconds. 
uint32 unix_time_seconds

# This field provides the sub-seconds component of time. It is 
# represented as microseconds since the last second.
# minimum value = 0, max value = 999999
uint32 microseconds


float64 latitude    # rad  
float64 longitude   # rad
float64 height      # m

# Velocity in North, East, Down (m/s)
adnav_interfaces/NED velocity 

# Body acceleration in XYZ (m/s/s)
geometry_msgs/Point body_acceleration

# g_force
float32 g_force

# Orientation of the device in Roll Pitch Heading (rad)
adnav_interfaces/RPH orientation

# Angular velocity of the device in XYZ (rad/s)
geometry_msgs/Point angular_velocity

# Latitude, Longitude, Height Standard Deviation (m) 
adnav_interfaces/LLH standard_deviation
