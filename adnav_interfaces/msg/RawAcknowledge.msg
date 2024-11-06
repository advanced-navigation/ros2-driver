# This contains the raw data structure of ANPP Packet 0. The Acknowledge Packet
#
# Specified using the Advanced Navigation Packet Protocol

# Packet ID being acknowledged
uint8 id

# CRC of the packet being acknowledged
uint16 crc

# The Acknowledgement Result
uint8 result

uint8 ACKNOWLEDGE_SUCCESS = 0
uint8 ACKNOWLEDGE_FAILURE_CRC_ERROR = 1
uint8 ACKNOWLEDGE_FAILURE_PACKET_SIZE_ERROR = 2
uint8 ACKNOWLEDGE_FAILURE_OUTSIDE_VALID_RANGE = 3
uint8 ACKNOWLEDGE_FAILURE_SYSTEM_FLASH_FAILURE = 4
uint8 ACKNOWLEDGE_FAILURE_SYSTEM_NOT_READY = 5
uint8 ACKNOWLEDGE_FAILURE_UNKNOWN_PACKET = 6