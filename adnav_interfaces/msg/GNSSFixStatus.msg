# GNSS Fix Status of Filter Status 
#
# Specified using the Advanced Navigation Packet Protocol

uint8 GNSS_FIX_NONE = 0 #
uint8 GNSS_FIX_2D = 1 #
uint8 GNSS_FIX_3D = 2 #
uint8 GNSS_FIX_SBAS = 3 #
uint8 GNSS_FIX_DIFFERENTIAL = 4 #
uint8 GNSS_FIX_OMNISTAR = 5 #
uint8 GNSS_FIX_RTK_FLOAT = 6 #
uint8 GNSS_FIX_RTK_FIXED = 7 #

# three bit integer (0-7)
uint8 gnss_fix_type 0 