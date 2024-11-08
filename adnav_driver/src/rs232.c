/*
    Cross-platform serial / RS232 library
    Version 0.23, 21/12/2022
    -> All platforms implementation
    -> rs232.c

    The MIT License (MIT)

    Copyright (c) 2013-2015 Frédéric Meslin, Florent Touchard
    Email: fredericmeslin@hotmail.com
    Website: www.fredslab.net
    Twitter: @marzacdev

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

*/

#include "rs232.h"

#include <stdio.h>
#include <string.h>

#ifdef _WIN32

/*****************************************************************************/
typedef int bool;
#define true -1
#define false 0

typedef struct
{
    int port;
    void *handle;
} COMDevice;

/*****************************************************************************/
#if !defined(COM_MAXDEVICES)
#define COM_MAXDEVICES 64
#endif
static COMDevice comDevices[COM_MAXDEVICES];
static int noDevices = 0;

#if !defined(COM_MINDEVNAME)
#define COM_MINDEVNAME 16384
#endif
const char *comPtn = "COM???";

/*****************************************************************************/
const char *findPattern(const char *string, const char *pattern, int *value);
const char *portInternalName(int index);

/*****************************************************************************/
typedef struct _COMMTIMEOUTS
{
    uint32_t ReadIntervalTimeout;
    uint32_t ReadTotalTimeoutMultiplier;
    uint32_t ReadTotalTimeoutConstant;
    uint32_t WriteTotalTimeoutMultiplier;
    uint32_t WriteTotalTimeoutConstant;
} COMMTIMEOUTS;

typedef struct _DCB
{
    uint32_t DCBlength;
    uint32_t BaudRate;
    uint32_t fBinary : 1;
    uint32_t fParity : 1;
    uint32_t fOutxCtsFlow : 1;
    uint32_t fOutxDsrFlow : 1;
    uint32_t fDtrControl : 2;
    uint32_t fDsrSensitivity : 1;
    uint32_t fTXContinueOnXoff : 1;
    uint32_t fOutX : 1;
    uint32_t fInX : 1;
    uint32_t fErrorChar : 1;
    uint32_t fNull : 1;
    uint32_t fRtsControl : 2;
    uint32_t fAbortOnError : 1;
    uint32_t fDummy2 : 17;
    uint16_t wReserved;
    uint16_t XonLim;
    uint16_t XoffLim;
    uint8_t ByteSize;
    uint8_t Parity;
    uint8_t StopBits;
    int8_t XonChar;
    int8_t XoffChar;
    int8_t ErrorChar;
    int8_t EofChar;
    int8_t EvtChar;
    uint16_t wReserved1;
} DCB;

/*****************************************************************************/
/** Windows system constants */
#define ERROR_INSUFFICIENT_BUFFER 122
#define INVALID_HANDLE_VALUE ((void *)-1)
#define GENERIC_READ 0x80000000
#define GENERIC_WRITE 0x40000000
#define OPEN_EXISTING 3
#define MAX_DWORD 0xFFFFFFFF

#define SETRTS 3
#define CLRRTS 4
#define SETDTR 5
#define CLRDTR 6

/*****************************************************************************/
/** Windows system functions */
void *__stdcall CreateFileA(const char *lpFileName, uint32_t dwDesiredAccess, uint32_t dwShareMode, void *lpSecurityAttributes, uint32_t dwCreationDisposition, uint32_t dwFlagsAndAttributes, void *hTemplateFile);
bool __stdcall WriteFile(void *hFile, const void *lpBuffer, uint32_t nNumberOfBytesToWrite, uint32_t *lpNumberOfBytesWritten, void *lpOverlapped);
bool __stdcall ReadFile(void *hFile, void *lpBuffer, uint32_t nNumberOfBytesToRead, uint32_t *lpNumberOfBytesRead, void *lpOverlapped);
bool __stdcall CloseHandle(void *hFile);

uint32_t __stdcall GetLastError(void);
void __stdcall SetLastError(uint32_t dwErrCode);

uint32_t __stdcall QueryDosDeviceA(const char *lpDeviceName, char *lpTargetPath, uint32_t ucchMax);

bool __stdcall GetCommState(void *hFile, DCB *lpDCB);
bool __stdcall GetCommTimeouts(void *hFile, COMMTIMEOUTS *lpCommTimeouts);
bool __stdcall SetCommState(void *hFile, DCB *lpDCB);
bool __stdcall SetCommTimeouts(void *hFile, COMMTIMEOUTS *lpCommTimeouts);
bool __stdcall SetupComm(void *hFile, uint32_t dwInQueue, uint32_t dwOutQueue);
bool __stdcall EscapeCommFunction(void *hFile, uint32_t dwFunc);

/*****************************************************************************/
int comEnumerate()
{
    // Get devices information text
    size_t size = COM_MINDEVNAME;
    char *list = (char *)malloc(size);
    SetLastError(0);
    QueryDosDeviceA(NULL, list, (uint32_t)size);
    while (GetLastError() == ERROR_INSUFFICIENT_BUFFER)
    {
        size *= 2;
        char *nlist = realloc(list, size);
        if (!nlist)
        {
            free(list);
            return 0;
        }
        list = nlist;
        SetLastError(0);
        QueryDosDeviceA(NULL, list, (uint32_t)size);
    }
    // Gather all COM ports
    int port;
    const char *nlist = findPattern(list, comPtn, &port);
    noDevices = 0;
    while (port > 0 && noDevices < COM_MAXDEVICES)
    {
        COMDevice *com = &comDevices[noDevices++];
        com->port = port;
        com->handle = 0;
        nlist = findPattern(nlist, comPtn, &port);
    }
    free(list);
    return noDevices;
}

void comTerminate()
{
    comCloseAll();
}

int comGetNoPorts()
{
    return noDevices;
}

/*****************************************************************************/
const char *comGetPortName(int index)
{
#define COM_MAXNAME 32
    static char name[COM_MAXNAME];
    if (index < 0 || index >= noDevices)
        return 0;
    sprintf(name, "COM%i", comDevices[index].port);
    return name;
}

int comFindPort(const char *name)
{
    for (int i = 0; i < noDevices; i++)
        if (strcmp(name, comGetPortName(i)) == 0)
            return i;
    return -1;
}

const char *comGetInternalName(int index)
{
#define COM_MAXNAME 32
    static char name[COM_MAXNAME];
    if (index < 0 || index >= noDevices)
        return 0;
    sprintf(name, "//./COM%i", comDevices[index].port);
    return name;
}

/*****************************************************************************/
int comOpen(int index, int baudrate_and_parity)
{
    DCB config;
    COMMTIMEOUTS timeouts;
    if (index < 0 || index >= noDevices)
        return 0;
    // Close if already open
    COMDevice *com = &comDevices[index];
    if (com->handle)
        comClose(index);
    // Open COM port
    void *handle = CreateFileA(comGetInternalName(index), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (handle == INVALID_HANDLE_VALUE)
        return 0;
    com->handle = handle;
    // Prepare read / write timeouts
    SetupComm(handle, 64, 64);
    timeouts.ReadIntervalTimeout = MAX_DWORD;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutConstant = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    SetCommTimeouts(handle, &timeouts);
    // Prepare serial communication format
    GetCommState(handle, &config);
    config.BaudRate = baudrate_and_parity & BAUDRATE_BITMASK;
    config.fBinary = 1;
    config.fParity = 1;
    config.fErrorChar = 0;
    config.fNull = 0;
    config.fAbortOnError = 0;
    config.ByteSize = 8;
    switch (baudrate_and_parity & PARITY_BITMASK)
    {
    case PARITY_NONE:
        config.Parity = 0;
        break;
    case PARITY_ODD:
        config.Parity = 1;
        break;
    case PARITY_EVEN:
        config.Parity = 2;
        break;
    case PARITY_SPACE:
        config.Parity = 3;
        break;
    case PARITY_MARK:
        config.Parity = 4;
        break;
    }
    config.StopBits = 0;
    config.EvtChar = '\n';
    // Set the port state
    if (SetCommState(handle, &config) == 0)
    {
        CloseHandle(handle);
        return 0;
    }
    return 1;
}

void comClose(int index)
{
    if (index < 0 || index >= noDevices)
        return;
    COMDevice *com = &comDevices[index];
    if (!com->handle)
        return;
    CloseHandle(com->handle);
    com->handle = 0;
}

void comCloseAll()
{
    for (int i = 0; i < noDevices; i++)
        comClose(i);
}

/*****************************************************************************/
int comWrite(int index, const unsigned char *buffer, size_t len)
{
    if (index < 0 || index >= noDevices)
        return 0;
    COMDevice *com = &comDevices[index];
    uint32_t bytes = 0;
    WriteFile(com->handle, buffer, (uint32_t)len, &bytes, NULL);
    return bytes;
}

int comRead(int index, unsigned char *buffer, size_t len)
{
    if (index < 0 || index >= noDevices)
        return 0;
    COMDevice *com = &comDevices[index];
    uint32_t bytes = 0;
    ReadFile(com->handle, buffer, (uint32_t)len, &bytes, NULL);
    return bytes;
}

int comReadBlocking(int index, unsigned char *buffer, size_t len, unsigned timeout)
{
    if (index < 0 || index >= noDevices)
        return 0;
    COMDevice *com = &comDevices[index];
    COMMTIMEOUTS timeouts;

    timeouts.ReadIntervalTimeout = MAX_DWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = (uint32_t)timeout;
    timeouts.WriteTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    SetCommTimeouts(com->handle, &timeouts);

    uint32_t bytes = 0;
    ReadFile(com->handle, buffer, (uint32_t)len, &bytes, NULL);

    timeouts.ReadIntervalTimeout = MAX_DWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    SetCommTimeouts(com->handle, &timeouts);

    return bytes;
}

/*****************************************************************************/
const char *findPattern(const char *string, const char *pattern, int *value)
{
    char c, n = 0;
    const char *sp = string;
    const char *pp = pattern;
    // Check for the string pattern
    while (1)
    {
        c = *sp++;
        if (c == '\0')
        {
            if (*pp == '?')
                break;
            if (*sp == '\0')
                break;
            n = 0;
            pp = pattern;
        }
        else
        {
            if (*pp == '?')
            {
                // Expect a digit
                if (c >= '0' && c <= '9')
                {
                    n = n * 10 + (c - '0');
                    if (*pp++ == '\0')
                        break;
                }
                else
                {
                    n = 0;
                    pp = comPtn;
                }
            }
            else
            {
                // Expect a character
                if (c == *pp)
                {
                    if (*pp++ == '\0')
                        break;
                }
                else
                {
                    n = 0;
                    pp = comPtn;
                }
            }
        }
    }
    // Return the value
    *value = n;
    return sp;
}

int comSetDtr(int index, int state)
{
    if (index < 0 || index >= noDevices)
        return 0;
    return EscapeCommFunction(comDevices[index].handle, state ? SETDTR : CLRDTR);
}

int comSetRts(int index, int state)
{
    if (index < 0 || index >= noDevices)
        return 0;
    return EscapeCommFunction(comDevices[index].handle, state ? SETRTS : CLRRTS);
}

#endif // _WIN32

#if defined(__unix__) || defined(__unix) || \
    defined(__APPLE__) && defined(__MACH__)

#define _DARWIN_C_SOURCE

#include <unistd.h>
#if !defined(__USE_MISC)
#define __USE_MISC // For CRTSCTS
#endif
#include <termios.h>
#include <fcntl.h>
#include <dirent.h>

#if !defined(__USE_SVID)
#define __USE_SVID // For strdup
#endif
#include <stdlib.h>
#include <sys/ioctl.h>

/*****************************************************************************/
/** Base name for COM devices */
#if defined(__APPLE__) && defined(__MACH__)
static const char *devBases[] = {
    "tty."};
static int noBases = 1;
#else
static const char *devBases[] = {
    "ttyACM", "ttyUSB", "rfcomm", "ttyS"};
static int noBases = 4;
#endif

/*****************************************************************************/
typedef struct
{
    char *port;
    int handle;
} COMDevice;

#if !defined(COM_MAXDEVICES)
#define COM_MAXDEVICES 64
#endif
static COMDevice comDevices[COM_MAXDEVICES];
static int noDevices = 0;

/*****************************************************************************/
/** Private functions */
void _AppendDevices(const char *base);
int _BaudFlag(int BaudRate);

/*****************************************************************************/
int comEnumerate()
{
    for (int i = 0; i < noDevices; i++)
    {
        if (comDevices[i].port)
            free(comDevices[i].port);
        comDevices[i].port = NULL;
    }
    noDevices = 0;
    for (int i = 0; i < noBases; i++)
        _AppendDevices(devBases[i]);
    return noDevices;
}

void comTerminate()
{
    comCloseAll();
    for (int i = 0; i < noDevices; i++)
    {
        if (comDevices[i].port)
            free(comDevices[i].port);
        comDevices[i].port = NULL;
    }
}

int comGetNoPorts()
{
    return noDevices;
}

/*****************************************************************************/
int comFindPort(const char *name)
{
    int p;
    for (p = 0; p < noDevices; p++)
        if (strcmp(name, comDevices[p].port) == 0)
            return p;
    return -1;
}

const char *comGetInternalName(int index)
{
#define COM_MAXNAME 128
    static char name[COM_MAXNAME];
    if (index >= noDevices || index < 0)
        return NULL;
    sprintf(name, "/dev/%s", comDevices[index].port);
    return name;
}

const char *comGetPortName(int index)
{
    if (index >= noDevices || index < 0)
        return NULL;
    return comDevices[index].port;
}

/*****************************************************************************/
int comOpen(int index, int baudrate_and_parity)
{
    if (index >= noDevices || index < 0)
        return 0;
    // Close if already open
    COMDevice *com = &comDevices[index];
    if (com->handle >= 0)
        comClose(index);
    // Open port
    printf("Try %s \n", comGetInternalName(index));
    int handle = open(comGetInternalName(index), O_RDWR | O_NOCTTY | O_NDELAY);
    if (handle < 0)
        return 0;
    printf("Open %s \n", comGetInternalName(index));
    // General configuration
    struct termios config;
    memset(&config, 0, sizeof(config));
    tcgetattr(handle, &config);
    config.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    config.c_iflag |= IGNPAR | IGNBRK;
    config.c_oflag &= ~(OPOST | ONLCR | OCRNL);
    config.c_cflag &= ~(PARENB | PARODD | CSTOPB | CSIZE | CRTSCTS);
    config.c_cflag |= CLOCAL | CREAD | CS8;
    config.c_lflag &= ~(ICANON | ISIG | ECHO);
    switch (baudrate_and_parity & PARITY_BITMASK)
    {
    case PARITY_NONE:
        break;
    case PARITY_ODD:
        config.c_cflag |= PARENB | PARODD;
        break;
    case PARITY_EVEN:
        config.c_cflag |= PARENB;
        break;
#if !defined(_DARWIN_C_SOURCE)
    case PARITY_SPACE:
        config.c_cflag |= PARENB | CMSPAR;
        break;
    case PARITY_MARK:
        config.c_cflag |= PARENB | CMSPAR | PARODD;
        break;
#endif
    }
    int flag = _BaudFlag(baudrate_and_parity & BAUDRATE_BITMASK);
    cfsetospeed(&config, flag);
    cfsetispeed(&config, flag);
    // Timeouts configuration
    config.c_cc[VTIME] = 1;
    config.c_cc[VMIN] = 0;
    // fcntl(handle, F_SETFL, FNDELAY);
    // Validate configuration
    if (tcsetattr(handle, TCSANOW, &config) < 0)
    {
        close(handle);
        return 0;
    }
    com->handle = handle;
    return 1;
}

void comClose(int index)
{
    if (index >= noDevices || index < 0)
        return;
    COMDevice *com = &comDevices[index];
    if (com->handle < 0)
        return;
    tcdrain(com->handle);
    close(com->handle);
    com->handle = -1;
}

void comCloseAll()
{
    for (int i = 0; i < noDevices; i++)
        comClose(i);
}

/*****************************************************************************/
int comWrite(int index, const unsigned char *buffer, size_t len)
{
    if (index >= noDevices || index < 0)
        return 0;
    if (comDevices[index].handle <= 0)
        return 0;
    int res = write(comDevices[index].handle, buffer, len);
    if (res < 0)
        res = 0;
    return res;
}

int comRead(int index, unsigned char *buffer, size_t len)
{
    if (index >= noDevices || index < 0)
        return 0;
    if (comDevices[index].handle <= 0)
        return 0;
    int res = read(comDevices[index].handle, buffer, len);
    if (res < 0)
        res = 0;
    return res;
}

int comReadBlocking(int index, unsigned char *buffer, size_t len, unsigned timeout)
{
    fd_set set;
    struct timeval to;
    int rv, res;

    if (index >= noDevices || index < 0)
        return 0;
    if (comDevices[index].handle <= 0)
        return 0;

    FD_ZERO(&set); /* clear the set */
    FD_SET(comDevices[index].handle, &set);

    to.tv_sec = timeout / 1000;
    to.tv_usec = (timeout % 1000) * 1000;

    rv = select(comDevices[index].handle + 1, &set, NULL, NULL, &to);
    if (rv <= 0)
        return 0;
    res = (int)read(comDevices[index].handle, buffer, len);
    if (res < 0)
        res = 0;
    return res;
}

/*****************************************************************************/
int _BaudFlag(int BaudRate)
{
    switch (BaudRate)
    {
    case 50:
        return B50;
        break;
    case 110:
        return B110;
        break;
    case 134:
        return B134;
        break;
    case 150:
        return B150;
        break;
    case 200:
        return B200;
        break;
    case 300:
        return B300;
        break;
    case 600:
        return B600;
        break;
    case 1200:
        return B1200;
        break;
    case 1800:
        return B1800;
        break;
    case 2400:
        return B2400;
        break;
    case 4800:
        return B4800;
        break;
    case 9600:
        return B9600;
        break;
    case 19200:
        return B19200;
        break;
    case 38400:
        return B38400;
        break;
    case 57600:
        return B57600;
        break;
    case 115200:
        return B115200;
        break;
    case 230400:
        return B230400;
        break;
#if defined(B500000)
    case 500000:
        return B500000;
        break;
    case 576000:
        return B576000;
        break;
    case 921600:
        return B921600;
        break;
    case 1000000:
        return B1000000;
        break;
    case 1152000:
        return B1152000;
        break;
    case 1500000:
        return B1500000;
        break;
    case 2000000:
        return B2000000;
        break;
    case 2500000:
        return B2500000;
        break;
    case 3000000:
        return B3000000;
        break;
    case 3500000:
        return B3500000;
        break;
    case 4000000:
        return B4000000;
        break;
#endif
    default:
        return B0;
        break;
    }
}

void _AppendDevices(const char *base)
{
    int baseLen = strlen(base);
    struct dirent *dp;
    // Enumerate devices
    DIR *dirp = opendir("/dev");
    while ((dp = readdir(dirp)) && noDevices < COM_MAXDEVICES)
    {
        if (strlen(dp->d_name) >= baseLen)
        {
            if (memcmp(base, dp->d_name, baseLen) == 0)
            {
                COMDevice *com = &comDevices[noDevices++];
                com->port = (char *)strdup(dp->d_name);
                com->handle = -1;
            }
        }
    }
    closedir(dirp);
}

int comSetDtr(int index, int state)
{
    int cmd = state ? TIOCMBIS : TIOCMBIC;
    int flag = TIOCM_DTR;
    if (index >= noDevices || index < 0)
        return 0;
    if (comDevices[index].handle <= 0)
        return 0;
    return ioctl(comDevices[index].handle, cmd, &flag) != -1;
}

int comSetRts(int index, int state)
{
    int cmd = state ? TIOCMBIS : TIOCMBIC;
    int flag = TIOCM_RTS;
    if (index >= noDevices || index < 0)
        return 0;
    if (comDevices[index].handle <= 0)
        return 0;
    return ioctl(comDevices[index].handle, cmd, &flag) != -1;
}

#endif // unix
