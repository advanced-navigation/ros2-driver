/****************************************************************/
/*                                                              */
/*                       Advanced Navigation                    */
/*         				Device Communications		  			*/
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

#ifndef ADNAV_COMMS_H_
#define ADNAV_COMMS_H_

#include "rs232.h"
#include "adnav_utils.h"
#include <stdio.h>
#include <string>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>

#include <signal.h>

#if defined(WIN32) || defined(_WIN32)
    #ifndef _WIN32_WINNT
        #define _WIN32_WINNT 0x0600
    #endif
    #pragma comment(lib, "ws2_32.lib") // winsock library
    #include <winsock2.h>
    #include <ws2tcpip.h>
#else
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <netdb.h>
    #include <unistd.h>
    #include <errno.h>
    #include <ifaddrs.h>
#endif

namespace adnav{

constexpr int MAX_CONNECTION_TRYS = 3;
constexpr int CONNECTION_RETRY_TIMEOUT = 2;

typedef enum{
    CONNECTION_NOT_OPEN = -1,
    CONNECTION_SERIAL,
    CONNECTION_TCP_CLIENT, 
    CONNECTION_TCP_SERVER, 
    CONNECTION_UDP_CLIENT
}adnav_connection_e;

typedef struct{
	int method;
	std::string com_port;
	std::string ip_address;
	int port;
	int baud_rate;
    int index;
}adnav_connections_data_t;

class Communicator{
    public:
        // Should not be clonable
        Communicator(Communicator &other) = delete;
        
        // Should not be assignable
        void operator=(const Communicator &) = delete;

        // Requires initialisation to work
        Communicator() = delete;
        
        /**
         * @brief Options initialized constructor
         * Creates and initializes a communicator. 
         * @param ops Options data structure. see adnav_connections_data_t 
        */
        Communicator(const adnav_connections_data_t& ops){initComms(ops);}
        ~Communicator(){close();}
        bool isOpen(){return isOpen_;}
        
        void initComms(const adnav_connections_data_t& ops);
        void open();
        void close();
        
        int read(void* buf, size_t len);
        int write(void* buf, size_t len);
        int getMethod(){return connection_ops_.method;}

        
    private:
        #if defined(WIN32) || defined(_WIN32)
            WSADATA wsa_data_;
            SOCKET sock_ = INVALID_SOCKET;
            SOCKET connection_ = INVALID_SOCKET;
            SOCKET server_ = INVALID_SOCKET;
        #else
            int sock_ = -1;
            int connection_ = -1;
            int server_ = -1;
        #endif   

        // Boolean to check if a method is open. 
        bool isOpen_ = false;

        // Defines what communication method to use, refer to adnav_connection_e.
        adnav_connections_data_t connection_ops_;

        // Has a UDP Packet been recieved. 
        bool UDPDatagramRecv = false;

        // Structures to hold connection address and server addresses details
        struct sockaddr_in address_, servAddr_;
        // Lengths of the sockaddr_in structs. 
        socklen_t addressLen_, servAddrLen_;

        // Private Validation and error handling methods. 
        bool validateBaudRate();
        bool validateComPort();
        bool validateIpAddress();
        bool validatePort();

        void throwRuntime(const char* reason) {
#if defined(WIN32) || defined(_WIN32)
            WSACleanup();
#endif
            throw std::runtime_error(reason);
        }
};

}// namespace adnav

#endif // ADNAV_COMMS_H_