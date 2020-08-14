/*
  NTRIP client for POSIX.
  $Id: ntripclient.c,v 1.51 2009/09/11 09:49:19 stoecker Exp $
  Copyright (C) 2003-2008 by Dirk Stöcker <soft@dstoecker.de>

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
  or read http://www.gnu.org/licenses/gpl.txt
*/


#ifndef NTRIPCLIENT_H_
#define NTRIPCLIENT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <ctype.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
  #include <winsock.h>
  typedef SOCKET sockettype;
  typedef u_long in_addr_t;
  typedef size_t socklen_t;
#else
  typedef int sockettype;
  #include <signal.h>
  #include <fcntl.h>
  #include <unistd.h>
  #include <arpa/inet.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <netdb.h>


  #define closesocket(sock)       close(sock)
  #define ALARMTIME   (2*60)
  #define myperror perror
#endif


#ifndef COMPILEDATE
#define COMPILEDATE " built " __DATE__
#endif

#define AGENTSTRING "NTRIP NtripClientPOSIX"
#define TIME_RESOLUTION 125
#define MAXDATASIZE 1000 
#define ARGOPT "-d:m:bhp:r:s:u:n:S:R:M:IP:D:B:T:C:Y:A:l:"

enum MODE { HTTP = 1, RTSP = 2, NTRIP1 = 3, AUTO = 4, UDP = 5, END };

struct Args
{
  const char *server;
  const char *port;
  const char *user;
  const char *proxyhost;
  const char *proxyport;
  const char *password;
  const char *nmea;
  const char *data;
  int         bitrate;
  int         mode;

  int         udpport;
  int         initudp;
  int         baud;
  char *serdevice;
};

/* option parsing */
#ifdef NO_LONG_OPTS
#define LONG_OPT(a)
#else
#define LONG_OPT(a) a
static struct option opts[] = {
{ "bitrate",    no_argument,       0, 'b'},
{ "data",       required_argument, 0, 'd'}, /* compatibility */
{ "mountpoint", required_argument, 0, 'm'},
{ "initudp",    no_argument,       0, 'I'},
{ "udpport",    required_argument, 0, 'P'},
{ "server",     required_argument, 0, 's'},
{ "password",   required_argument, 0, 'p'},
{ "port",       required_argument, 0, 'r'},
{ "proxyport",  required_argument, 0, 'R'},
{ "proxyhost",  required_argument, 0, 'S'},
{ "user",       required_argument, 0, 'u'},
{ "nmea",       required_argument, 0, 'n'},
{ "mode",       required_argument, 0, 'M'},
{ "serdevice",  required_argument, 0, 'D'},
{ "baud",       required_argument, 0, 'B'},
{ "stopbits",   required_argument, 0, 'T'},
{ "protocol",   required_argument, 0, 'C'},
{ "parity",     required_argument, 0, 'Y'},
{ "databits",   required_argument, 0, 'A'},
{ "serlogfile", required_argument, 0, 'l'},
{ "help",       no_argument,       0, 'h'},
{0,0,0,0}};
#endif

static const char encodingTable [64] = {
  'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
  'Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f',
  'g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v',
  'w','x','y','z','0','1','2','3','4','5','6','7','8','9','+','/'
};

char *encodeurl(const char *req);
char *geturl(const char *url, struct Args *args);
int encode(char *buf, int size, const char *user, const char *pwd);
int getargs(int argc, char **argv, struct Args *args);
int ntrip_initialise(struct Args *args, char *buf);
int ntrip(struct Args *args, char *buf, int *byt);

#ifdef __cplusplus
}
#endif

#endif