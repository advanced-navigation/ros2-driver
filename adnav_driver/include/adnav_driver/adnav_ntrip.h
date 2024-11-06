/****************************************************************/
/*                                                              */
/*                       Advanced Navigation                    */
/*               			NTRIP Client			  		    */
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

#ifndef ADNAV_NTRIP_H_
#define ADNAV_NTRIP_H_

#if defined(WIN32) || defined(_WIN32)
    #include <winsock2.h>
#endif // defined(WIN32) || defined(_WIN32)
#if defined(__linux__)
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <netinet/tcp.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
#endif  // defined(__linux__)

#include <atomic>
#include <string>
#include <thread>
#include <functional>
#include <errno.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <list>
#include <memory>

#include "adnav_utils.h"
#include "ins_packets.h"

#define NTRIP_TIMEOUT_PERIOD 3
#define NTRIP_BUFFER_SIZE 4096

namespace adnav{
namespace ntrip{

    typedef enum{
        NTRIP_NO_FAIL,
        NTRIP_CONNECTION_TIMEOUT_FAILURE, 
        NTRIP_CREATE_SOCK_FAILURE,
        NTRIP_REMOTE_CLOSE,
        NTRIP_REMOTE_SOCKET_FAILURE,
        NTRIP_CASTER_CONNECTION_FAILURE,
        NTRIP_SEND_GPGGA_FAILURE,
        NTRIP_SEND_REQUEST_FAILURE,
        NTRIP_UNRECOGNIZED_RETURN
    }adnav_ntrip_connection_e;

    class Client {
        public:
            Client() = default;
            Client(Client const&) = delete;
            Client(Client&&) = delete;
            Client& operator=(Client const&) = delete;
            Client& operator=(Client&&) = delete;
            // Destructor
            ~Client(){this->stop();}

            // Constructor
            Client(std::string const& ip, int const& port, 
                std::string const& user, std::string const& passwd,
                std::string const& mountpoint, std::string const& agent = std::string()){
                // load the variables into memory
                server_ip_ = ip;
                server_port_ = port;
                user_ = user;
                passwd_ = passwd;
                mountpoint_ = mountpoint; 

                if (!agent.empty()){
                    user_agent_ = agent;
                }
            }

            void set_location(double latitude, double longitude, double altitude){
                // Don't proceed unless something changed. 
                if(latitude_ == latitude && longitude_ == longitude && altitude_ == altitude) return;
                
                latitude_ = latitude;
                longitude_ = longitude;
                altitude_ = altitude;

                // flag that the gga_string neds to be regenerated. 
                gga_update_.store(false);
            }

            void set_gnss_connection_status(gnss_fix_type_e gnss_fix, int sats, float hdop){
                // Don't proceed unless something changed.
                if(gnss_fix_ == gnss_fix && sats_ == sats && hdop_ == hdop) return;

                gnss_fix_ = gnss_fix;
                sats_ = sats;
                hdop_ = hdop;

                // flag that the gga_string needs to be regenerated.
                gga_update_.store(false);
            }

            void set_report_interval(int interval){
                report_interval_ = interval;
            }

            // Status Functions
            bool run(void);
            bool retrieve_sourcetable(void);
            void stop(void);

            /**
             * @brief Function to make public the running status of the client in
             * a thread safe manner.
             * 
             * @return Boolean service running indicator.
            */
            bool service_running(void) const {
                return service_is_running_.load();
            }

            /**
             * @brief Function to make public the error status of the client in a
             * thread safe manner. 
             * 
             * @return Enum value for error
            */
            adnav_ntrip_connection_e service_failure(void) const {
                return service_failure_.load();
            }

            /**
             * @brief Functon to set the callback called when the NTRIP server
             * provides data. 
             * 
             * @param callback A std::function that has no return, takes a pointer
             * to a buffer containing the sent data and a size of that buffer. 
            */
            void OnReceived(const std::function<void (char const* _buffer, int _size)>&callback) { callback_ = callback; }

        private:

            void thread_handler(void);
            bool establish_new_connection(void);
            std::string encode_credentials(void);

            std::atomic<adnav_ntrip_connection_e> service_failure_ = {NTRIP_NO_FAIL};
            std::atomic_bool service_is_running_ = {false};
            std::atomic_bool gga_update_ = {false};
            int report_interval_ = 10;  // seconds
            double latitude_ = 0;
            double longitude_ = 0;
            double altitude_ = 0;
            gnss_fix_type_e gnss_fix_ = gnss_fix_3d;
            int sats_ = 10;
            float hdop_ = 2.0;
            std::string gga_string_;
            std::string user_agent_ = "AdNav C++ Ntrip/2.0";

            // Server Details
            std::string server_ip_;
            int server_port_;
            std::string user_;
            std::string passwd_;
            std::string mountpoint_;        
            #if defined(WIN32) || defined(_WIN32)
                SOCKET socket_fd_ = INVALID_SOCKET;
            #else
                int socket_fd_ = -1;
            #endif

            // Thread to handle server interactions
            std::thread thread_;

            std::function<void (char const* _buffer, int _size)> callback_ = [] (char const*, int) -> void {};
    };

}// namespace ntrip
}// namespace adnav

#endif // ADNAV_NTRIP_H_