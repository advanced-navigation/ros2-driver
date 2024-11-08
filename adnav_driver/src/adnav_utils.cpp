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


#include "adnav_utils.h"

namespace adnav{

namespace utils{

    /**
     * @brief Function to split a string into a vector of strings at a delimiter. 
     * 
     * @param str String to parse and split.
     * @param delim delimiter character for the string to be split at. 
     */
    std::vector<std::string> splitStr(const std::string& str, char delim) {
        size_t i = 0;
        std::vector<std::string> list;
        size_t pos = str.find(delim);
        while (pos != std::string::npos){
            list.push_back(str.substr(i, pos - i));
            i = ++pos;
            pos = str.find(delim, pos);
        }
        list.push_back(str.substr(i, str.length()));
        return list;
    }

    /**
     * @brief Function to validate if a string is a IPv4 Address. 
     * 
     * @param ip Standard c++ string of the IPv4 address to be tested. 
     */
    bool validateIP(const std::string& ip) {
        std::vector<std::string> strList = splitStr(ip, '.');

        if(strList.size() != 4)
            return false;

        for(std::string str : strList){
            if(!adnav::utils::chkNumber(str) || stoi(str) < 0 || stoi(str) > 255)
            return false;
        }
        return true;
    }

    /**
     * @brief Function to check if a string is a numerical value. 
     * 
     * @param str string to check if numeric. 
     */
    bool chkNumber(const std::string& str) {
        return !str.empty() && 
        (str.find_last_not_of("[0123456789]") == std::string::npos);
    }

    /**
     * @brief Function to get all local interfaces and present them in a formatted string
     * 
     * Only returns interfaces that use IPv4 addresses (AF_INET).
     * 
     * @return Formatted std::string of all different local IPv4 Interfaces
     */
    std::string getLocalInterfaces(){

        std::stringstream ss;
        char hostname[256];
        int i = 0;

        #if defined(WIN32) || defined(_WIN32)
            WSADATA wsadata;
            if (WSAStartup(MAKEWORD(2,2), &wsadata)) {
                throw std::runtime_error("Failed to initialize Winsock.\n");
            }

            // Get current host name
            ::gethostname(hostname, sizeof(hostname));
            ss << "Hostname: " << hostname;

            DWORD asize = 20000;
            PIP_ADAPTER_ADDRESSES adapters;

            // Get a linked list of adapters. 
            do {
                adapters = (PIP_ADAPTER_ADDRESSES)malloc(asize);

                if (!adapters) {
                    WSACleanup();
                    throw std::runtime_error("Unable to allocate memory for adapters.\n");
                }

                int r = GetAdaptersAddresses(AF_INET, GAA_FLAG_INCLUDE_PREFIX, 0, adapters, &asize);

                if (r == ERROR_BUFFER_OVERFLOW) {
                    // If the result is overflow the adapter addresses will set asize to
                    // the required size. So free adapters and let the do while trigger again. 
                    free(adapters);
                    printf("GetAdaptersAddresses Overflow! Increasing size.");
                }
                else if (r == ERROR_SUCCESS) {
                    // Successfully retrieved adapters. 
                    break;
                }
                else {
                    free(adapters);
                    WSACleanup();
                    throw std::runtime_error("GetAdaptersAddresses Failure");
                }
            } while (!adapters);

            // Go through each adapter
            while (adapters) {
                // Stream in adapter number and name.
                ss << "\n\t" << std::setw(2) << std::right << i << " | Adapter name: " << adapters->FriendlyName << "\n\t";
                
                // Get the linked list of addresses in the adapter pointing to the first address.  
                PIP_ADAPTER_UNICAST_ADDRESS address = adapters->FirstUnicastAddress;
                
                // Go through each unicast address in the adapter.
                while (address) {
                    // We only care about IPv4 addresses
                    if (address->Address.lpSockaddr->sa_family == AF_INET) {
                        PULONG netmask;
                        struct in_addr netmask_addr;
                        char nm[INET_ADDRSTRLEN];
                        char ap[INET_ADDRSTRLEN];

                        // Get the netmask from the adress object.
                        ConvertLengthToIpv4Mask(address->OnLinkPrefixLength, netmask);
                        // Put the netmask into the in_addr struct. 
                        netmask_addr.S_un.S_addr = *netmask;
                        // Convert the ULONG into a human readable netmask in 'nm'
                        inet_ntop(AF_INET, &netmask_addr, nm, INET_ADDRSTRLEN);

                        // get the ip address details
                        getnameinfo(address->Address.lpSockaddr, address->Address.iSockaddrLength,
                                ap, INET_ADDRSTRLEN, 0, 0, NI_NUMERICHOST);

                        // Stream it to the buffer.
                        ss << "\tIPv4: " << ap << "\tNetMask: " << nm << "\n\t";
                    }
                    // Move down the linked list
                    address = address->Next;
                }
                adapters = adapters->Next;
                i++;
            }

            // Cleanup
            free(adapters);
            WSACleanup();

        #else
            struct ifaddrs* ptr_ifaddrs = nullptr;

            // Get current host name
            gethostname(hostname, sizeof(hostname));
            ss << "Hostname: " << hostname;

            // Get all local interfaces
            if (getifaddrs(&ptr_ifaddrs) != 0) {
                std::cout << "[WARN] Unable to get local interfaces" << std::endl;
                return ss.str();
            }

            // For every entry in the list of local interfaces
            for (struct ifaddrs* ptr_entry = ptr_ifaddrs;
                ptr_entry != nullptr;
                ptr_entry = ptr_entry->ifa_next) {
                // Stream in interface number and name. 
                ss << "\n\t" << std::setw(2) << std::right << i << " | IF Name: " << ptr_entry->ifa_name << "\t";

                // Check that the interface is IPv4
                sa_family_t address_family = ptr_entry->ifa_addr->sa_family;
                if (address_family == AF_INET) {
                    // IPv4

                    // the fields of ptr_entry might be nullptr. Dereferencing nullptr causes 
                    // Undefined behaviour so checking is a required before saving data.

                    // Address
                    if (ptr_entry->ifa_addr != nullptr) {
                        char buffer[INET_ADDRSTRLEN] = { 0, };
                        // get the ip address and put it into the buffer.
                        inet_ntop(address_family, &((struct sockaddr_in*)(ptr_entry->ifa_addr))->sin_addr,
                            buffer, INET_ADDRSTRLEN);

                        ss << "IP Address: " << std::setw(INET_ADDRSTRLEN) << std::left << buffer << "\t";
                    }

                    // Netmask
                    if (ptr_entry->ifa_netmask != nullptr) {
                        char buffer[INET_ADDRSTRLEN] = { 0, };
                        // get the netmask and put it into the buffer.
                        inet_ntop(address_family, &((struct sockaddr_in*)(ptr_entry->ifa_netmask))->sin_addr,
                            buffer, INET_ADDRSTRLEN);

                        ss << "Netmask: " << std::setw(INET_ADDRSTRLEN) << std::left << buffer << "\t";
                    }
                } // if family = AF_INET

                i++;
            }

            // Free the list so it doesn't leak memory.
            freeifaddrs(ptr_ifaddrs);
        #endif // WIN/UNIX

        // Return std::string from the stringstream.
        return ss.str();
    }

    

#if defined(WIN32) || defined(_WIN32)
    /**
     * @brief Function to retrieve information about a connection to a socket and return it in a formatted
     * string. If calling from the server side the server address should also be given.
     *
     * @param socket SOCKET object of the connection to retrieve information about.
     * @param servAddr [Optional] sockaddr_in* address details of the host server to be placed into the
     * formatted string
     *
     * @return Returns a formatted std::string with the server's and peer's IP and Port.
     */
    std::string getConnectionInfo(SOCKET s, sockaddr_in* servAddr) {
        sockaddr_in address;
        socklen_t addr_len = sizeof(address);
        std::stringstream ss;
        char* buffer;
        int port;
        WSADATA wsadata;

        // Startup winsock
        if (WSAStartup(MAKEWORD(2, 2), &wsadata)) {
            throw std::runtime_error("Failed to initialize Winsock.\n");
        }

        // Get socket info (server side)
        // If no destination server has been passed. 
        if (servAddr == nullptr) {
            getsockname(s, (struct sockaddr*)&address, &addr_len);
            buffer = inet_ntoa(address.sin_addr);
            port = ntohs(address.sin_port);
        }
        else {
            buffer = inet_ntoa(servAddr->sin_addr);
            port = ntohs(servAddr->sin_port);
        }

        ss << std::setw(12) << std::left << "Server IP: " << std::setw(INET_ADDRSTRLEN) <<
            buffer << "\tPort: " << port << "\n";

        // reset the address variable
        memset(&address, 0, addr_len);

        // Get peer info (connection side)
        getpeername(s, (struct sockaddr*)&address, &addr_len);
        buffer = inet_ntoa(address.sin_addr);
        ss << std::setw(12) << std::left << "Peer IP:" << buffer << "\tPort: " << ntohs(address.sin_port);

        // return the std::string from the stringstream 
        return ss.str();

        // Cleanup this winsock usage
        WSACleanup();
    }
#else
    /**
     * @brief Function to retrieve information about a connection to a socket and return it in a formatted
     * string. If calling from the server side the server address should also be given.
     *
     * @param socket_fd file descriptor of the socket to retrieve information about.
     * @param servAddr [Optional] sockaddr_in* address details of the host server to be placed into the
     * formatted string
     *
     * @return Returns a formatted std::string with the server's and peer's IP and Port.
     */
    std::string getConnectionInfo(int socket_fd, sockaddr_in* servAddr) {
        sockaddr_in address;
        socklen_t addr_len = sizeof(address);
        std::stringstream ss;
        char* buffer;
        int port;

        // Get socket info (server side)
        // If no destination server has been passed. 
        if (servAddr == nullptr) {
            getsockname(socket_fd, (struct sockaddr*)&address, &addr_len);
            buffer = inet_ntoa(address.sin_addr);
            port = ntohs(address.sin_port);
        }
        else {
            buffer = inet_ntoa(servAddr->sin_addr);
            port = ntohs(servAddr->sin_port);
        }

        ss << std::setw(12) << std::left << "Server IP: " << std::setw(INET_ADDRSTRLEN) <<
            buffer << "\tPort: " << port << "\n";

        // reset the address variable
        memset(&address, 0, addr_len);

        // Get peer info (connection side)
        getpeername(socket_fd, (struct sockaddr*)&address, &addr_len);
        buffer = inet_ntoa(address.sin_addr);
        ss << std::setw(12) << std::left << "Peer IP:" << buffer << "\tPort: " << ntohs(address.sin_port);

        // return the std::string from the stringstream 
        return ss.str();
    }
#endif

    /**
     * @brief Function to turn a decimal degree into degree decimal minutes formatt used in NMEA GPGGA String
     *
     * @param degree double constant reference of the degree value to convert
     * @param longitude boolean constant for formatting the DDM string. 
     * 0 = Latitude
     * 1 = Longitude
     * 
     * @return value in decimal degrees minutes
     */
    std::string DD2DDM(double const& decimal_degrees, bool longitude) {
        int degree = static_cast<int>(floor(fabs(decimal_degrees)));
        double minutes = (fabs(decimal_degrees) - degree) * 60;
        char buf[20];
        if(longitude){
            snprintf(buf, sizeof(buf), "%03d%010.7f", degree, minutes);
        }else{
            snprintf(buf, sizeof(buf), "%02d%010.7f", degree, minutes);
        }
        return buf;
    }

    // Function to compare checksum in gga string. 
    int BccCheckSumCompareForGGA(const char *src) {
        int sum = 0;
        int num = 0;
        sscanf(src, "%*[^*]*%x", &num);
        for (int i = 1; src[i] != '*'; ++i) {
            sum ^= src[i];
        }
        return sum - num;
    }

    std::string Base64CodeTable =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    inline
    int base64_index(char in) {
        return Base64CodeTable.find(in);
    }

    int Base64Encode(std::string const& raw, std::string* out) {
        int len = raw.size();
        for (int i = 0; i < len; i += 3) {
            out->push_back(Base64CodeTable[(raw[i]&0xFC)>>2]);
            if (i+1 >= len) {
            out->push_back(Base64CodeTable[(raw[i]&0x03)<<4]);
            break;
            }
            out->push_back(Base64CodeTable[(raw[i]&0x03)<<4|(raw[i+1]&0xF0)>>4]);
            if (i+2 >= len) {
            out->push_back(Base64CodeTable[(raw[i+1]&0x0F)<<2]);
            break;
            } else {
            out->push_back(Base64CodeTable[(raw[i+1]&0x0F)<<2|(raw[i+2]&0xC0)>>6]);
            }
            out->push_back(Base64CodeTable[raw[i+2]&0x3F]);
        }
        len = out->size();
        if (len % 4 != 0) {
            out->append(std::string(4-len%4, '='));
        }
        return 0;
    }

    int Base64Decode(std::string const& raw, std::string* out) {
        if (out == nullptr) return -1;
        int len = raw.size();
        if ((len == 0) || (len%4 != 0)) return -1;

        out->clear();
        for (int i = 0; i < len; i += 4) {
            out->push_back(((base64_index(raw[i])&0x3F)<<2) |
                ((base64_index(raw[i+1])&0x3F)>>4));
            if (raw[i+2] == '=') {
            out->push_back(((base64_index(raw[i+1])&0x0F)<<4));
            break;
            }
            out->push_back(((base64_index(raw[i+1])&0x0F)<<4) |
                ((base64_index(raw[i+2])&0x3F)>>2));
            if (raw[i+3] == '=') {
            out->push_back(((base64_index(raw[i+2])&0x03)<<6));
            break;
            }
            out->push_back(((base64_index(raw[i+2])&0x03)<<6) |
                (base64_index(raw[i+3])&0x3F));
        }
        return 0;
    }


    /**
     * @brief Function to take GPGGA data fields and fill a NEMA GPGGA string into the given std::string buffer. 
     * Warning this function will erase anything in the buffer given to it. 
     *
     * @param gga_out standard string reference that the function places the output string into
     * @param latitude latitude to place into the GGA string in degrees. 
     * @param longitude longitude to place into the GGA string in degrees.
     * @param altitude altitude to place into teh GGA string in meters.
     * @param gnss_fix [optional] the current fix of the gnss sensor. Defaults to a 3D fix.
     * @param sats [optional] the number of satellites in use. May be different to number in view. Defaults to 10.
     * @param hdop [optional] horizontal dilution of precision. Defaults to 2.0.
     *
     * @return negative result on failure
     */
    int GenerateGGAString(std::string& gga_out, double latitude, double longitude,
        double altitude, gnss_fix_type_e gnss_fix, int sats, float hdop){
        
        // Clear the string object passed by reference. 
        gga_out.clear();

        int quality = 1;

        switch (gnss_fix)
        {
        case gnss_fix_none:
            quality = 0;
            break;
        case gnss_fix_2d:
        case gnss_fix_3d:
            quality = 1;
            break;
        case gnss_fix_differential:
            quality = 2;
            break;
        case gnss_fix_rtk_fixed:
            quality = 4;
            break;
        case gnss_fix_rtk_float:
        case gnss_fix_omnistar:
            quality = 5;
            break;
        case gnss_fix_sbas:
            quality = 9;
            break;
        default:
            break;
        }
        
        // Get the current time and get convert to a UTC tm struct
        std::time_t time_now = time(nullptr);
        std::tm* gm_now;
        #if defined(WIN32) || defined(_WIN32)
        gmtime_s(tm_now, &time_now);
        #else
        gm_now = gmtime(&time_now);
        #endif

        std::stringstream ss;
        
        ss << "$GPGGA," << std::put_time(gm_now, "%H%M%S.000,") << 
        DD2DDM(latitude, false) << "," <<
        (latitude > 0.0 ? "N" : "S") << "," <<
        DD2DDM(longitude, true) << "," << 
        (longitude > 0.0 ? "E" : "W") <<  "," <<
        quality << "," <<
        sats << "," <<
        std::setprecision(1) << hdop << "," << 
        std::setprecision(2) << altitude << ",M," << altitude << ",M,,";
        gga_out = ss.str();

        uint8_t checksum = 0;
        
        // Checksum does not include the '$' character or '*' character
        for (char& c : gga_out){
            if (c != '$') checksum = checksum ^ c;
        }

        // Place the checksum with the '*' delimiter at the end of the string. 
        ss << "*" << std::uppercase << std::hex << std::setw(2) << static_cast<int>(checksum) << "\r\n"; 
        gga_out = ss.str();

        return BccCheckSumCompareForGGA(gga_out.c_str());
    }

}// namespace Utils
}// namespace adnav
