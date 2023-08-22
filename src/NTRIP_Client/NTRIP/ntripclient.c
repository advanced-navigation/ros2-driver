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

#include "ntripclient.h"
#include "../../rs232/rs232.h"


/* CVS revision and version */
static char revisionstr[] = "$Revision: 1.51 $";
static char datestr[]     = "$Date: 2009/09/11 09:49:19 $";

int myerror = 0;
int sleeptime = 0;
long i;
sockettype sockfd = 0;
int numbytes;
int stop = 0;



#ifndef _WIN32
int sigstop = 0;
#ifdef __GNUC__
static __attribute__ ((noreturn)) void sighandler_alarm(
int sig __attribute__((__unused__)))
#else /* __GNUC__ */
static void sighandler_alarm(int sig)
#endif /* __GNUC__ */
{
  if(!sigstop)
    fprintf(stderr, "ERROR: more than %d seconds no activity\n", ALARMTIME);
  else
    fprintf(stderr, "ERROR: user break\n");
  exit(1);
}
#ifdef __GNUC__
static void sighandler_int(int sig __attribute__((__unused__)))
#else /* __GNUC__ */
static void sighandler_alarm(int sig)
#endif /* __GNUC__ */
{
  sigstop = 1;
  alarm(2);
  stop = 1;
}
#endif /* _WIN32 */

#ifdef _WIN32
void myperror(char *s)
{
    fprintf(stderr, "%s: %d\n", s, WSAGetLastError());
}

#endif

int ntrip_initialise(struct Args *args, char *buf)
{

#ifndef _WIN32
  signal(SIGALRM,sighandler_alarm);
  signal(SIGINT,sighandler_int);
  alarm(ALARMTIME);
#else
  WSADATA wsaData;
  if(WSAStartup(MAKEWORD(1,1),&wsaData))
  {
    fprintf(stderr, "Could not init network access.\n");
    return 20;
  }
#endif
 
  struct sockaddr_in their_addr; /* connector's address information */
  struct hostent *he;
  struct servent *se;
  const char *server, *port, *proxyserver = 0;
  char proxyport[6];
  char *b;

  if(sleeptime)
  {
#ifdef _WIN32
      Sleep(sleeptime*1000);
#else
      sleep(sleeptime);
#endif
      sleeptime += 2;
  }
  else
  {
    sleeptime = 1;
  }
#ifndef _WIN32
  alarm(ALARMTIME);
#endif

  setbuf(stdout, 0);
  setbuf(stdin, 0);
  setbuf(stderr, 0);

  if(args->proxyhost)
  {
    int p;
    if((i = strtol(args->port, &b, 10)) && (!b || !*b))
      p = i;
    else if(!(se = getservbyname(args->port, 0)))
    {
      fprintf(stderr, "Can't resolve port %s.", args->port);
      stop = 1;
    }
    else
    {
      p = ntohs(se->s_port);
    }
    if(!stop && !myerror)
    {
      snprintf(proxyport, sizeof(proxyport), "%d", p);
      port = args->proxyport;
      proxyserver = args->server;
      server = args->proxyhost;
    }
  }
  else
  {
    server = args->server;
    port = args->port;
  }

  if(!stop && !myerror)
  {
    memset(&their_addr, 0, sizeof(struct sockaddr_in));
    if((i = strtol(port, &b, 10)) && (!b || !*b))
      their_addr.sin_port = htons(i);
    else if(!(se = getservbyname(port, 0)))
    {
      fprintf(stderr, "Can't resolve port %s.", port);
      stop = 1;
    }
    else
    {
      their_addr.sin_port = se->s_port;
    }
    if(!stop && !myerror)
    {
      if(!(he=gethostbyname(server)))
      {
        fprintf(stderr, "Server name lookup failed for '%s'.\n", server);
        myerror = 1;
      }
      else if((sockfd = socket(AF_INET, (args->mode == UDP ? SOCK_DGRAM :
      SOCK_STREAM), 0)) == -1)
      {
        //myperror("socket");
        myerror = 1;
      }
      else
      {
        their_addr.sin_family = AF_INET;
        their_addr.sin_addr = *((struct in_addr *)he->h_addr);
      }
    }
  }
      
  if(!stop && !myerror)
  {
    if(connect(sockfd, (struct sockaddr *)&their_addr,
    sizeof(struct sockaddr)) == -1)
    {
      //myperror("connect");
      myerror = 1;
    }
    if(!stop && !myerror)
    {
      if(!args->data)
      {
        i = snprintf(buf, MAXDATASIZE,
          "GET %s%s%s%s/ HTTP/1.1\r\n"
          "Host: %s\r\n%s"
          "User-Agent: %s/%s\r\n"
          "Ntrip-GGA: %s"
          "Connection: close\r\n"
          "\r\n"
          "%s",
          proxyserver ? "http://" : "", 
          proxyserver ? proxyserver : "",
          proxyserver ? ":" : "", 
          proxyserver ? proxyport : "",
          args->server, 
          args->mode == NTRIP1 ? "" : "Ntrip-Version: Ntrip/2.0\r\n",
          AGENTSTRING, 
          revisionstr,
          args->nmea,
          args->nmea);
      }
      else
      {
        const char *nmeahead = (args->nmea && args->mode == HTTP) ? args->nmea : 0;

        i=snprintf(buf, MAXDATASIZE-40, /* leave some space for login */
          "GET %s%s%s%s/%s HTTP/1.1\r\n"
          "Host: %s\r\n%s"
          "User-Agent: %s/%s\r\n"
          "%s%s%s"
          "Connection: close%s", 
          proxyserver ? "http://" : "", 
          proxyserver ? proxyserver : "",
          proxyserver ? ":" : "", 
          proxyserver ? proxyport : "",
          args->data, 
          args->server,
          args->mode == NTRIP1 ? "" : "Ntrip-Version: Ntrip/2.0\r\n",
          AGENTSTRING, 
          revisionstr,
          nmeahead ? "Ntrip-GGA: " : "", 
          nmeahead ? nmeahead : "",
          nmeahead ? "\r\n" : "",
          (*args->user || *args->password) ? "\r\nAuthorization: Basic " : "");
        
        if(i > MAXDATASIZE-40 || i < 0) /* second check for old glibc */
        {
          fprintf(stderr, "Requested data too long\n");
          stop = 1;
        }
        else
        {
          i += encode(buf+i, MAXDATASIZE-i-4, args->user, args->password);
          if(i > MAXDATASIZE-4)
          {
            fprintf(stderr, "Username and/or password too long\n");
            stop = 1;
          }
          else
          {
            buf[i++] = '\r';
            buf[i++] = '\n';
            buf[i++] = '\r';
            buf[i++] = '\n';
            if(args->nmea && !nmeahead)
            {
              int j = snprintf(buf+i, MAXDATASIZE-i, "%s\r\n", args->nmea);
              if(j >= 0 && j < MAXDATASIZE-i)
                i += j;
              else
              {
                fprintf(stderr, "NMEA string too long\n");
                stop = 1;
              }
           }
          }
        }
      }
    }
  }

    if(send(sockfd, buf, (size_t)i, 0) != i)
    {
      //myperror("send");
      myerror = 1;
    }
  return (myerror || stop);
}

int ntrip(struct Args *args, char *buf, int *byt)
{
 
  
    if(args->data && *args->data != '%')
    {
      static int k = 0;
      static int chunkymode = 0;
      int starttime = time(0);
      int lastout = starttime;
      static int totalbytes = 0;
      static int chunksize = 0;
      char temp[MAXDATASIZE];

      //Make all dec's above here static and move error checks to initlize. 
      while(!stop && !myerror && (numbytes=recv(sockfd, buf, MAXDATASIZE-1, 0)) > 0)
      {
#ifndef _WIN32
        alarm(ALARMTIME);
#endif
       
        if(!k)
        { 
          buf[numbytes] = 0; /* latest end mark for strstr */
          if( numbytes > 17 && !strstr(buf, "ICY 200 OK")  && (!strncmp(buf, "HTTP/1.1 200 OK\r\n", 17) || !strncmp(buf, "HTTP/1.0 200 OK\r\n", 17)) )
          {
            const char *datacheck = "Content-Type: gnss/data\r\n";
            const char *chunkycheck = "Transfer-Encoding: chunked\r\n";
            int l = strlen(datacheck)-1;
            int j=0;
            for(i = 0; j != l && i < numbytes-l; ++i)
            {
              for(j = 0; j < l && buf[i+j] == datacheck[j]; ++j)
                ;
            }
            if(i == numbytes-l)
            {
              fprintf(stderr, "No 'Content-Type: gnss/data' found\n");
              myerror = 1;
            }
              l = strlen(chunkycheck)-1;
              j=0;
              for(i = 0; j != l && i < numbytes-l; ++i)
              {
                for(j = 0; j < l && buf[i+j] == chunkycheck[j]; ++j)
                  ;
              }
              if(i < numbytes-l)
                chunkymode = 1;
            }
            else if(!strstr(buf, "ICY 200 OK"))
            {
              fprintf(stderr, "Could not get the requested data: ");
              for(k = 0; k < numbytes && buf[k] != '\n' && buf[k] != '\r'; ++k)
              {
                fprintf(stderr, "%c", isprint(buf[k]) ? buf[k] : '.');
              }
              fprintf(stderr, "\n");
              myerror = 1;
            }
            else if(args->mode != NTRIP1)
            {
              fprintf(stderr, "NTRIP version 2 HTTP connection failed%s.\n",
              args->mode == AUTO ? ", falling back to NTRIP1" : "");
              if(args->mode == HTTP)
                stop = 1;
            }
            k = 1;
            if(args->mode == NTRIP1)
              continue; /* skip old headers for NTRIP1 */
            else
            {
              char *ep = strstr(buf, "\r\n\r\n");
              if(!ep || ep+4 == buf+numbytes)
                continue;
              ep += 4;
              memmove(buf, ep, numbytes-(ep-buf));
              numbytes -= (ep-buf);
            }
          }
          sleeptime = 0;
         
          int srt = 0;
          int p_chunksize =0;
          *byt = 0;
          if(chunkymode)
          {
            //printf("Chun88ky\n");
            int cstop = 0;
            int pos = 0;
            

            while(!stop && !cstop && !myerror && pos < numbytes)
            { 
              
              switch(chunkymode)
              {
              case 1: /* reading number starts */
                chunksize = 0;
                ++chunkymode; /* no break */
              case 2: /* during reading number */
                i = buf[pos++];
                if(i >= '0' && i <= '9') chunksize = chunksize*16+i-'0';
                else if(i >= 'a' && i <= 'f') chunksize = chunksize*16+i-'a'+10;
                else if(i >= 'A' && i <= 'F') chunksize = chunksize*16+i-'A'+10;
                else if(i == '\r') ++chunkymode;
                else if(i == ';') chunkymode = 5;
                else cstop = 1;
                break;
              case 3: /* scanning for return */
                if(buf[pos++] == '\n') chunkymode = chunksize ? 4 : 1; //Does chunkymode = chunksize ? yes then 4 otherwise 1
                else cstop = 1;
                break;
              case 4: /* output data */
                
                i = numbytes-pos;
                
                if(i > chunksize) 
                {
                  i = chunksize;
                }
                
                //Move data to a temp buffer to send back to main loop
                if(srt==0)
                {
                  memmove(temp,buf+pos,(size_t)i);
                  srt = 1; 
                }
                else
                {
                  memmove(temp+p_chunksize,buf+pos,(size_t)i);
                }
                //printf("%d\n",p_chunksize);
                p_chunksize += chunksize;
                //fwrite(buf+pos, (size_t)i, 1, stdout);
                totalbytes += i;
               *byt += i;
                chunksize -= i;
                pos += i;
                if(!chunksize)
                  chunkymode = 1;
                break;
              case 5:
                if(i == '\r') chunkymode = 3;
                break;
              }
            }
            if(cstop)
            {
              fprintf(stderr, "Error in chunky transfer encoding\n");
              myerror = 1;
            }
          }
          else
          { 
            totalbytes += numbytes;
            memcpy(temp,buf,(size_t)numbytes);
             return(myerror || stop);
          }
          
          //SendBuf(temp, byt);
          memcpy(buf,temp,*byt);  
             
          fflush(stdout);
          if(totalbytes < 0) /* overflow */
          {
            totalbytes = 0;
            starttime = time(0);
            lastout = starttime;
          }
       
          if(args->bitrate)
          {
            int t = time(0);
            if(t > lastout + 60)
            {
              lastout = t;
              fprintf(stderr, "Bitrate is %dbyte/s (%d seconds accumulated).\n",
              totalbytes/(t-starttime), t-starttime);
            }
          }
          
          return(myerror || stop);
        }
      

      }
    else
    {
      sleeptime = 0;
      while(!stop && (numbytes=recv(sockfd, buf, MAXDATASIZE-1, 0)) > 0)
      {
#ifndef _WIN32
        alarm(ALARMTIME);
#endif
        fwrite(buf, (size_t)numbytes, 1, stdout);
      }
    }
  

  if(sockfd) closesocket(sockfd);
  sleep(10);

  return(myerror || stop);
  
}

char *encodeurl(const char *req)
{
  char *h = "0123456789abcdef";
  static char buf[128];
  char *urlenc = buf;
  char *bufend = buf + sizeof(buf) - 3;

  while(*req && urlenc < bufend)
  {
    if(isalnum(*req)
    || *req == '-' || *req == '_' || *req == '.')
      *urlenc++ = *req++;
    else
    {
      *urlenc++ = '%';
      *urlenc++ = h[*req >> 4];
      *urlenc++ = h[*req & 0x0f];
      req++;
    }
  }
  *urlenc = 0;
  return buf;
}

char *geturl(const char *url, struct Args *args)
{
  static char buf[1000];
  static char *Buffer = buf;
  static char *Bufend = buf+sizeof(buf);
  char *h = "0123456789abcdef";

  if(strncmp("ntrip:", url, 6))
    return "URL must start with 'ntrip:'.";
  url += 6; /* skip ntrip: */

  if(*url != '@' && *url != '/')
  {
    /* scan for mountpoint */
    args->data = Buffer;
    if(*url != '?')
    {
       while(*url && *url != '@' &&  *url != ';' && *url != '/' && Buffer != Bufend)
         *(Buffer++) = *(url++);
    }
    else
    {
       while(*url && *url != '@' &&  *url != '/' && Buffer != Bufend)
       {
          if(isalnum(*url) || *url == '-' || *url == '_' || *url == '.')
            *Buffer++ = *url++;
          else
          {
            *Buffer++ = '%';
            *Buffer++ = h[*url >> 4];
            *Buffer++ = h[*url & 0x0f];
            url++;
          }
       }
    }
    if(Buffer == args->data)
      return "Mountpoint required.";
    else if(Buffer >= Bufend-1)
      return "Parsing buffer too short.";
    *(Buffer++) = 0;
  }

  if(*url == '/') /* username and password */
  {
    ++url;
    args->user = Buffer;
    while(*url && *url != '@' && *url != ';' && *url != ':' && Buffer != Bufend)
      *(Buffer++) = *(url++);
    if(Buffer == args->user)
      return "Username cannot be empty.";
    else if(Buffer >= Bufend-1)
      return "Parsing buffer too short.";
    *(Buffer++) = 0;

    if(*url == ':') ++url;

    args->password = Buffer;
    while(*url && *url != '@' && *url != ';' && Buffer != Bufend)
      *(Buffer++) = *(url++);
    if(Buffer == args->password)
      return "Password cannot be empty.";
    else if(Buffer >= Bufend-1)
      return "Parsing buffer too short.";
    *(Buffer++) = 0;
  }

  if(*url == '@') /* server */
  {
    ++url;
    if(*url != '@' && *url != ':')
    {
      args->server = Buffer;
      while(*url && *url != '@' && *url != ':' && *url != ';' && Buffer != Bufend)
        *(Buffer++) = *(url++);
      if(Buffer == args->server)
        return "Servername cannot be empty.";
      else if(Buffer >= Bufend-1)
        return "Parsing buffer too short.";
      *(Buffer++) = 0;
    }

    if(*url == ':')
    {
      ++url;
      args->port = Buffer;
      while(*url && *url != '@' && *url != ';' && Buffer != Bufend)
        *(Buffer++) = *(url++);
      if(Buffer == args->port)
        return "Port cannot be empty.";
      else if(Buffer >= Bufend-1)
        return "Parsing buffer too short.";
      *(Buffer++) = 0;
    }

    if(*url == '@') /* proxy */
    {
      ++url;
      args->proxyhost = Buffer;
      while(*url && *url != ':' && *url != ';' && Buffer != Bufend)
        *(Buffer++) = *(url++);
      if(Buffer == args->proxyhost)
        return "Proxy servername cannot be empty.";
      else if(Buffer >= Bufend-1)
        return "Parsing buffer too short.";
      *(Buffer++) = 0;

      if(*url == ':')
      {
        ++url;
        args->proxyport = Buffer;
        while(*url && *url != ';' && Buffer != Bufend)
          *(Buffer++) = *(url++);
        if(Buffer == args->proxyport)
          return "Proxy port cannot be empty.";
        else if(Buffer >= Bufend-1)
          return "Parsing buffer too short.";
        *(Buffer++) = 0;
      }
    }
  }
  if(*url == ';') /* NMEA */
  {
    args->nmea = ++url;
    while(*url)
      ++url;
  }

  return *url ? "Garbage at end of server string." : 0;
}

int getargs(int argc, char **argv, struct Args *args)
{
  int res = 1;
  int getoptr;
  char *a;
  int i = 0, help = 0;

  args->server = "";
  args->port = "2101";
  args->user = "";
  args->password = "";
  args->nmea = 0;
  args->data = 0;
  args->bitrate = 0;
  args->proxyhost = 0;
  args->proxyport = "2101";
  args->mode = AUTO;
  args->initudp = 0;
  args->udpport = 0;
  args->baud = 0;
  args->serdevice = 0;
  help = 0;

  do
  {
#ifdef NO_LONG_OPTS
    switch((getoptr = getopt(argc, argv, ARGOPT)))
#else
    switch((getoptr = getopt_long(argc, argv, ARGOPT, opts, 0)))
#endif
    {
    case 's': args->server = optarg; break;
    case 'u': args->user = optarg; break;
    case 'p': args->password = optarg; break;
    case 'd': /* legacy option, may get removed in future */
      fprintf(stderr, "Option -d or --data is deprecated. Use -m instead.\n");
    case 'm':
      if(optarg && *optarg == '?')
        args->data = encodeurl(optarg);
      else
        args->data = optarg;
      break;
    case 'B':
      {
        args->baud = strtol(optarg, 0, 10);
      }
      break;
    case 'D': args->serdevice = optarg; break;
    case 'I': args->initudp = 1; break;
    case 'P': args->udpport = strtol(optarg, 0, 10); break;
    case 'n': args->nmea = optarg; break;
    case 'b': args->bitrate = 1; break;
    case 'h': help=1; break;
    case 'r': args->port = optarg; break;
    case 'S': args->proxyhost = optarg; break;
    case 'R': args->proxyport = optarg; break;
    case 'M':
      args->mode = 0;
      if (!strcmp(optarg,"n") || !strcmp(optarg,"ntrip1"))
        args->mode = NTRIP1;
      else if(!strcmp(optarg,"h") || !strcmp(optarg,"http"))
        args->mode = HTTP;
      else if(!strcmp(optarg,"r") || !strcmp(optarg,"rtsp"))
        args->mode = RTSP;
      else if(!strcmp(optarg,"u") || !strcmp(optarg,"udp"))
        args->mode = UDP;
      else if(!strcmp(optarg,"a") || !strcmp(optarg,"auto"))
        args->mode = AUTO;
      else args->mode = atoi(optarg);
      if((args->mode == 0) || (args->mode >= END))
      {
        fprintf(stderr, "Mode %s unknown\n", optarg);
        res = 0;
      }
      break;
    case 1:
      {
        const char *err;
        if((err = geturl(optarg, args)))
        {
          fprintf(stderr, "%s\n\n", err);
          res = 0;
        }
      }
      break;
    case -1: break;
    }
  } while(getoptr != -1 && res);

  for(a = revisionstr+11; *a && *a != ' '; ++a)
    revisionstr[i++] = *a;
  revisionstr[i] = 0;
  datestr[0] = datestr[7];
  datestr[1] = datestr[8];
  datestr[2] = datestr[9];
  datestr[3] = datestr[10];
  datestr[5] = datestr[12];
  datestr[6] = datestr[13];
  datestr[8] = datestr[15];
  datestr[9] = datestr[16];
  datestr[4] = datestr[7] = '-';
  datestr[10] = 0;

  if(!res || help)
  {
    fprintf(stderr, "Version %s (%s) GPL" COMPILEDATE "\nUsage:\n%s -s server -u user ...\n"
    " -m " LONG_OPT("--mountpoint ") "the requested data set or sourcetable filtering criteria\n"
    " -s " LONG_OPT("--server     ") "the server name or address\n"
    " -p " LONG_OPT("--password   ") "the login password\n"
    " -r " LONG_OPT("--port       ") "the server port number (default 2101)\n"
    " -u " LONG_OPT("--user       ") "the user name\n"
    " -M " LONG_OPT("--mode       ") "mode for data request\n"
    "     Valid modes are:\n"
    "     1, h, http     NTRIP Version 2.0 Caster in TCP/IP mode\n"
    "     3, n, ntrip1   NTRIP Version 1.0 Caster\n"
    "     4, a, auto     automatic detection (default)\n"
    "\nExpert options:\n"
    " -S " LONG_OPT("--proxyhost  ") "proxy name or address\n"
    " -R " LONG_OPT("--proxyport  ") "proxy port, optional (default 2101)\n"
    "\nSerial input/output:\n"
    " -D " LONG_OPT("--serdevice  ") "serial device for output\n"
    " -B " LONG_OPT("--baud       ") "baudrate for serial device\n"
    , revisionstr, datestr, argv[0]);
    exit(1);
  }
  return res;
}

/* does not buffer overrun, but breaks directly after an error */
/* returns the number of required bytes */
int encode(char *buf, int size, const char *user, const char *pwd)
{
  unsigned char inbuf[3];
  char *out = buf;
  int i, sep = 0, fill = 0, bytes = 0;

  while(*user || *pwd)
  {
    i = 0;
    while(i < 3 && *user) inbuf[i++] = *(user++);
    if(i < 3 && !sep)    {inbuf[i++] = ':'; ++sep; }
    while(i < 3 && *pwd)  inbuf[i++] = *(pwd++);
    while(i < 3)         {inbuf[i++] = 0; ++fill; }
    if(out-buf < size-1)
      *(out++) = encodingTable[(inbuf [0] & 0xFC) >> 2];
    if(out-buf < size-1)
      *(out++) = encodingTable[((inbuf [0] & 0x03) << 4)
               | ((inbuf [1] & 0xF0) >> 4)];
    if(out-buf < size-1)
    {
      if(fill == 2)
        *(out++) = '=';
      else
        *(out++) = encodingTable[((inbuf [1] & 0x0F) << 2)
                 | ((inbuf [2] & 0xC0) >> 6)];
    }
    if(out-buf < size-1)
    {
      if(fill >= 1)
        *(out++) = '=';
      else
        *(out++) = encodingTable[inbuf [2] & 0x3F];
    }
    bytes += 4;
  }
  if(out-buf < size)
    *out = 0;
  return bytes;
}
