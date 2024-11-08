/****************************************************************/
/*                                                              */
/*                       Advanced Navigation                    */
/*         			    ROS2 Driver Utilities			  		*/
/*          Copyright 2023, Advanced Navigation Pty Ltd         */
/*                                                              */
/****************************************************************/
/*
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */


#ifndef ADNAV_UTILS_H_
#define ADNAV_UTILS_H_

#include <string>
#include <cstring>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <math.h>
#include <stdexcept>
#include <ctime>
#include "ins_packets.h"

#if defined(WIN32) || defined(_WIN32)
    #pragma comment(lib, "ws2_32.lib") // Winsock Library
    #pragma comment(lib, "iphlpapi.lib") // IP Help API Library
    #include<winsock2.h>
    #include<iphlpapi.h>
    #include<netioapi.h>
    #include<WS2tcpip.h>
#else
    #include <unistd.h>
    #include <netdb.h>
    #include <ifaddrs.h>
    #include <sys/socket.h>
    #include <arpa/inet.h>
#endif

namespace adnav{

namespace utils{
    bool validateIP(const std::string& ip);
    bool chkNumber(const std::string& str);
    std::vector<std::string> splitStr(const std::string& str, char delim);
    std::string getLocalInterfaces();

#if defined(WIN32) || defined(_WIN32)
    std::string getConnectionInfo(SOCKET s, sockaddr_in* servAddr = nullptr);
#else
    std::string getConnectionInfo(int socket_fd, sockaddr_in* servAddr = nullptr);
#endif

    int BccCheckSumCompareForGGA(char const* src);
    int Base64Encode(std::string const& raw, std::string* out);
    int Base64Decode(std::string const& raw, std::string* out);
    int GenerateGGAString(std::string& gga_out, double latitude, double longitude,
        double altitude, gnss_fix_type_e gnss_fix = gnss_fix_3d, int sats = 10, float hdop = 2.0);

    // ANSI Escape Character constant expressions
    //Regular Colored text
    constexpr char BLK[8] = "\x1b[0;30m";
    constexpr char RED[8] = "\x1b[0;31m";
    constexpr char GRN[8] = "\x1b[0;32m";
    constexpr char YEL[8] = "\x1b[0;33m";
    constexpr char BLU[8] = "\x1b[0;34m";
    constexpr char MAG[8] = "\x1b[0;35m";
    constexpr char CYN[8] = "\x1b[0;36m";
    constexpr char WHT[8] = "\x1b[0;37m";

    //Regular bold text
    constexpr char BBLK[8] = "\x1b[1;30m";
    constexpr char BRED[8] = "\x1b[1;31m";
    constexpr char BGRN[8] = "\x1b[1;32m";
    constexpr char BYEL[8] = "\x1b[1;33m";
    constexpr char BBLU[8] = "\x1b[1;34m";
    constexpr char BMAG[8] = "\x1b[1;35m";
    constexpr char BCYN[8] = "\x1b[1;36m";
    constexpr char BWHT[8] = "\x1b[1;37m";

    //Regular underline text
    constexpr char UBLK[8] = "\x1b[4;30m";
    constexpr char URED[8] = "\x1b[4;31m";
    constexpr char UGRN[8] = "\x1b[4;32m";
    constexpr char UYEL[8] = "\x1b[4;33m";
    constexpr char UBLU[8] = "\x1b[4;34m";
    constexpr char UMAG[8] = "\x1b[4;35m";
    constexpr char UCYN[8] = "\x1b[4;36m";
    constexpr char UWHT[8] = "\x1b[4;37m";

    //Regular background
    constexpr char BLKB[6] = "\x1b[40m";
    constexpr char REDB[6] = "\x1b[41m";
    constexpr char GRNB[6] = "\x1b[42m";
    constexpr char YELB[6] = "\x1b[43m";
    constexpr char BLUB[6] = "\x1b[44m";
    constexpr char MAGB[6] = "\x1b[45m";
    constexpr char CYNB[6] = "\x1b[46m";
    constexpr char WHTB[6] = "\x1b[47m";

    //High intensty background 
    constexpr char BLKHB[9] = "\x1b[0;100m";
    constexpr char REDHB[9] = "\x1b[0;101m";
    constexpr char GRNHB[9] = "\x1b[0;102m";
    constexpr char YELHB[9] = "\x1b[0;103m";
    constexpr char BLUHB[9] = "\x1b[0;104m";
    constexpr char MAGHB[9] = "\x1b[0;105m";
    constexpr char CYNHB[9] = "\x1b[0;106m";
    constexpr char WHTHB[9] = "\x1b[0;107m";

    //High intensty text
    constexpr char HBLK[8] = "\x1b[0;90m";
    constexpr char HRED[8] = "\x1b[0;91m";
    constexpr char HGRN[8] = "\x1b[0;92m";
    constexpr char HYEL[8] = "\x1b[0;93m";
    constexpr char HBLU[8] = "\x1b[0;94m";
    constexpr char HMAG[8] = "\x1b[0;95m";
    constexpr char HCYN[8] = "\x1b[0;96m";
    constexpr char HWHT[8] = "\x1b[0;97m";

    //Bold high intensity text
    constexpr char BHBLK[8] = "\x1b[1;90m";
    constexpr char BHRED[8] = "\x1b[1;91m";
    constexpr char BHGRN[8] = "\x1b[1;92m";
    constexpr char BHYEL[8] = "\x1b[1;93m";
    constexpr char BHBLU[8] = "\x1b[1;94m";
    constexpr char BHMAG[8] = "\x1b[1;95m";
    constexpr char BHCYN[8] = "\x1b[1;96m";
    constexpr char BHWHT[8] = "\x1b[1;97m";

    //Reset
    constexpr char reset[5] = "\x1b[0m";
    constexpr char CRESET[5] = "\x1b[0m";
    constexpr char COLOR_RESET[5] = "\x1b[0m";
} // namespace adnav
}

#endif //ADNAV_UTILS_H_