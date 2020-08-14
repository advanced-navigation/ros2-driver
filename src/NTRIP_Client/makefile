# $Id: makefile,v 1.8 2010/03/24 13:17:06 stoecker Exp $
# probably works not with all compilers. Thought this should be easy
# fixable. There is nothing special at this source.

ifdef windir
CC   = gcc
OPTS = -Wall -W -O3 -D_WIN32
LIBS = -lwsock32
else
OPTS = -Wall -W -O3 
endif

ntrip_example: ../rs232/rs232.c ../rs232/rs232.h ../an_packet_protocol.c ../an_packet_protocol.h ../spatial_packets.c ../spatial_packets.h NTRIP/ntripclient.h NTRIP/ntripclient.c ntrip_example.c
	$(CC) $(OPTS)  NTRIP/ntripclient.c ../rs232/rs232.c ../spatial_packets.c ../an_packet_protocol.c ntrip_example.c -o $@ $(LIBS) 

clean:
	$(RM) ntrip_example core*
#include ""
