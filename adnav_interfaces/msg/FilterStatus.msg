# Filter Status bits of ANPP packet 20
#
# Specified using the Advanced Navigation Packet Protocol

# This field contains 16 bits that indicate the status of the filters. 
# These are boolean fields with a zero indicating false and one indicating true.

bool orientation_filter_initialised 0
bool ins_filter_initialised 0
bool heading_initialised 0
bool utc_time_initialised 0

# GNSS Fix Status information. 
adnav_interfaces/GNSSFixStatus gnss_fix_status

bool event1_flag 0
bool event2_flag 0
bool internal_gnss_enabled 0
bool dual_antenna_heading_active 0
bool velocity_heading_enabled 0
bool atmospheric_altitude_enabled 0
bool external_position_active 0
bool external_velocity_active 0
bool external_heading_active 0