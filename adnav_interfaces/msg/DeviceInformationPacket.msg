# This contains the raw data structure of ANPP Packet 3. The Device Information Packet
#
# Specified using the Advanced Navigation Packet Protocol

# What version of software is currently on the device
uint32 software_version

# ID of the device
uint32 device_id

uint32 SPATIAL_ID = 1#
uint32 SPATIAL_FOG_ID = 4#
uint32 SPATIAL_DUAL_ID = 5#
uint32 ORIENTUS_ID = 11#
uint32 AIR_DATA_UNIT_ID = 13#
uint32 SUBSONUS_ID = 14#
uint32 SPATIAL_FOG_DUAL_ID = 16#
uint32 MOTUS_ID = 17#
uint32 GNSS_COMPASS_ID = 19#
uint32 SUBSONUS_TAG_ID = 21#
uint32 POSEIDON_ID = 22#
uint32 CERTUS_ID = 26#
uint32 ARIES_ID = 27#
uint32 BOREAS_D90_ID = 28#
uint32 BOREAS_D70_ID = 41#

uint32 hardware_revision

uint32 serial_number_part_1
uint32 serial_number_part_2
uint32 serial_number_part_3


