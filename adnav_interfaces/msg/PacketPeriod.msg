# This contains the data structure of for requesting a packet using ANPP Packet 181.
# The Packet Period Packet
#
# Specified using the Advanced Navigation Packet Protocol

# ID of the requested Packet
uint8 packet_id

# Period for the requested Packet (this is in the units defined using ANPP Packet 180
# the Packet Timer Period Packet)
# This indicates the period in units of the packet timer period. The packet rate can be 
# calculated as follows. Packet Rate = 1000000/(Packet Period x Packet Timer Period) Hz
# For example if the packet timer period is set to 1000 (1 ms). Setting packet ID 20 
# with a packet period of 50 will give the following.
# Packet 20 Rate = 1000000/(50 x 1000)
# Packet 20 Rate = 20 Hz

uint32 packet_period
