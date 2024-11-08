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
#if defined(WIN32) || defined(_WIN32)
#else
#endif
#include "adnav_comms.h"

namespace adnav{

void Communicator::initComms(const adnav_connections_data_t& ops){
	connection_ops_ = ops;

	switch(ops.method){
		case CONNECTION_NOT_OPEN:
			throw std::runtime_error("Invalid method Selected");
			break;

		case CONNECTION_SERIAL:
			validateBaudRate();
			validateComPort();
			break;

		case CONNECTION_TCP_CLIENT:
			validateIpAddress();
			validatePort();
			memset(&address_, 0, sizeof(address_));

			// Startup winsock if on windows. 
			#if defined(WIN32) || defined(_WIN32)
				if (WSAStartup(MAKEWORD(2, 2), &wsa_data_)) {
					throw std::runtime_error("Failed to initialize Winsock.\n");
				}
			#endif

			address_.sin_family = AF_INET;
			address_.sin_port = htons(connection_ops_.port);
			if(inet_pton(AF_INET, connection_ops_.ip_address.c_str(), &address_.sin_addr) <= 0) {
				throw std::invalid_argument("Invalid IP Address");
			}
			break;

		case CONNECTION_TCP_SERVER:
		case CONNECTION_UDP_CLIENT:
			validatePort();
			memset(&servAddr_, 0, sizeof(servAddr_));

			// Startup Winsock if on windows. 
			#if defined(WIN32) || defined(_WIN32)
				if (WSAStartup(MAKEWORD(2, 2), &wsa_data_)) {
					throw std::runtime_error("Failed to initialize Winsock.\n");
				}
			#endif

			servAddr_.sin_port = htons(connection_ops_.port);
			servAddr_.sin_family = AF_INET;
			servAddr_.sin_addr.s_addr = htonl(INADDR_ANY);
			break;

		default:
			throw std::runtime_error("Unknown communication type");
			break;

	}
}


void Communicator::open(){
	std::stringstream ss;
	char ip[INET_ADDRSTRLEN];
	int trys = 0;

	switch(connection_ops_.method)
	{
	case CONNECTION_NOT_OPEN:
		throw std::runtime_error("Unable to open uninitialized method");
		break;
	
	case CONNECTION_SERIAL:
		// Give user info
		std::cout << std::endl << "Connection Type: Serial\nBaud Rate : " << connection_ops_.baud_rate << std::endl 
		<< "Com Port: " << connection_ops_.com_port << std::endl;

		// Open connection
		if (comOpen(connection_ops_.index, connection_ops_.baud_rate) == false){
			ss << "Could not open serial port: " << connection_ops_.com_port << " at " << connection_ops_.baud_rate << " baud\nInvalid Serial Configuration.";
			throw std::runtime_error(ss.str().c_str());
		}
		break;

	case CONNECTION_TCP_CLIENT:
		// Generate Socket
		if ((sock_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
			throw std::runtime_error("Unable to generate TCP socket");
		}

		// Give user info
		inet_ntop(AF_INET, &(address_.sin_addr), ip, INET_ADDRSTRLEN);
		std::cout << std::endl << "Connection Type: TCP Client\nIP Address: " << ip << std::endl 
		<< "Port: " << ntohs(address_.sin_port) << std::endl << std::endl;

		// While unable to connect try 5 times with a 3 second delay. Leave if ctrl+c is given.
		while((connection_ = connect(sock_, (struct sockaddr*)&address_, sizeof(address_))) < 0) {
			trys++;
			// High intensity Bold Yellow Text
			std::cout << adnav::utils::BHYEL << "TCP Client Connection Failed..." << adnav::utils::reset << "\n\n";
			std::this_thread::sleep_for(std::chrono::seconds(CONNECTION_RETRY_TIMEOUT));
			std::cout << adnav::utils::BHYEL << "Trying Again" << adnav::utils::reset << "\n";
			if(trys > MAX_CONNECTION_TRYS){
				throw std::runtime_error("Unable to open a connection to the TCP Server.");
			}
		}

		// Bold Green text
		std::cout << adnav::utils::BGRN << "Connection made: \n" << adnav::utils::getConnectionInfo(connection_, &address_).c_str() <<
			adnav::utils::reset << std::endl << std::endl;
		break;
	
	case CONNECTION_TCP_SERVER:
		// Generate Socket
		if ((server_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
			throw std::runtime_error("Unable to generate TCP socket");
		}

		if(bind(server_, (struct sockaddr*) &servAddr_, sizeof(servAddr_)) < 0){
			throw std::runtime_error("Error binding socket to local address");
		}

		// Give user info
		std::cout << adnav::utils::BBLU << "Awaiting Connection on:\n" << adnav::utils::getLocalInterfaces().c_str() <<
		std::endl << "Port: " << ntohs(servAddr_.sin_port) << adnav::utils::reset << std::endl << std::endl;

		// listen for a connection
		listen(server_, 1);

		addressLen_ = sizeof(address_);

		// accept connection and create a new descriptor for it.
		if((sock_ = accept(server_, (sockaddr*) &address_, &addressLen_)) < 0){
			throwRuntime("Error accepting request from client");
		}
		
		std::cout <<adnav::utils::BGRN << "Connection Accepted: \n" << adnav::utils::getConnectionInfo(sock_).c_str() <<
			adnav::utils::reset << std::endl << std::endl;
		break;
	
	case CONNECTION_UDP_CLIENT:
		// Generate Socket		
		if ((sock_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
			throwRuntime("Unable to generate TCP socket");
		}

		addressLen_ = sizeof(address_);
		servAddrLen_ = sizeof(servAddr_);

		// Bind the socket to the incoming packet (server) address. 
		if (bind(sock_, (const struct sockaddr*)&servAddr_, sizeof(servAddr_)) < 0) {
			throwRuntime("Socket Bind Failed");
		}

		// Notify the user of available options to send data.
		std::cout << adnav::utils::BBLU << "Connection Available on:\n" << adnav::utils::getLocalInterfaces().c_str() <<
			std::endl << "Port: " << ntohs(servAddr_.sin_port) << adnav::utils::reset << std::endl << std::endl;
		break;
	}

	isOpen_ = true;
}

void Communicator::close(){
	if(!this->isOpen()) std::cerr << "Unable to close an unopened socket." << std::endl;
	switch(connection_ops_.method)
	{
		case CONNECTION_NOT_OPEN:
			throwRuntime("Unable to close an uninitialized communications method.");
			break;

		case CONNECTION_SERIAL:
			comClose(connection_ops_.index);
			break;

		case CONNECTION_TCP_CLIENT:
			#if defined(WIN32) || defined(_WIN32)
				closesocket(connection_);
			#else
				::close(connection_);
			#endif
			break;
		
		case CONNECTION_TCP_SERVER:
			#if defined(WIN32) || defined(_WIN32)
				closesocket(sock_);
				shutdown(server_, SD_BOTH);
			#else	
				::close(sock_);
				shutdown(server_, SHUT_RDWR);
			#endif
			break;

		case CONNECTION_UDP_CLIENT:
			#if defined(WIN32) || defined(_WIN32)
				closesocket(sock_);
			#else
				::close(sock_);
			#endif
			break;

		default:
			throw std::runtime_error("Unable to close an unknown communications method");
			break;
	}

	#if defined(WIN32) || defined(_WIN32)
		WSACleanup();
	#endif
}

int Communicator::read(void* buf, size_t len){
	char ip[INET_ADDRSTRLEN];
	// Ensure that the communications are open.
	if(!this->isOpen()) throw std::runtime_error("Unable to read from unopened socket");
	
	// get the bytes from the communication method, and load them into the decoder
	int received;
	switch(connection_ops_.method)
	{
		case CONNECTION_NOT_OPEN:
			throw std::runtime_error("Unable to read from an uninitialized communications method.");
			break;

		case CONNECTION_SERIAL: 
			received = comRead(connection_ops_.index, (unsigned char*) buf, len);
			break;

		case CONNECTION_TCP_CLIENT: 
		case CONNECTION_TCP_SERVER:
			#if defined(WIN32) || defined(_WIN32)
				received = recv(sock_,(char*) buf, len, 0);
			#else
				received = ::read(sock_, buf, len);
			#endif
			
			if(received < 0){
				std::cerr << "Error reading TCP Packet: ";
			#if defined(WIN32) || defined(_WIN32)
				std::cerr << WSAGetLastError() << std::endl;
			#else
				std::cerr << strerror(errno) << std::endl;
			#endif
			}
			break;
		
		case CONNECTION_UDP_CLIENT:
			// receive both data and address of data source
			received = recvfrom(sock_,(char*) buf, len, 0, (struct sockaddr *)&address_, &addressLen_);
			if(received < 0){
				std::cerr << "Error reading UDP Packet: ";
			#if defined(WIN32) || defined(_WIN32)
				std::cerr << WSAGetLastError() << std::endl;
			#else
				std::cerr << strerror(errno) << std::endl;
			#endif
			}

			// If this is the first time receiving a datagram, tell the user and
			// ensure we send back to the same port the data came from.
			if(!UDPDatagramRecv){
				address_.sin_port = servAddr_.sin_port;
				inet_ntop(AF_INET, &(address_.sin_addr), ip, INET_ADDRSTRLEN);
				std::cout << adnav::utils::BGRN << "Datagram Recieved: \nIP: " << ip << std::endl 
					<< "Port: " << ntohs(address_.sin_port) << adnav::utils::reset << std::endl << std::endl;
				UDPDatagramRecv = true;
			}

			break;
		default:
			throw std::runtime_error("Unable to read from unknown communication method.");
			break;
	}
	return received;
}

int Communicator::write(void* buf, size_t len){
	if(!this->isOpen()) throw std::runtime_error("Unable to write to unopened socket");
	int sent = 0;
	// Send encoded packet through the selected communication medium
	switch(connection_ops_.method){
		case CONNECTION_NOT_OPEN:
			throw std::runtime_error("Unable to write to uninitialized communication method");
			break;

		case CONNECTION_SERIAL:	
			sent = comWrite(connection_ops_.index, (unsigned char*) buf, len);
			break;	

		case CONNECTION_TCP_CLIENT: 
		case CONNECTION_TCP_SERVER: 			
			if((sent = send(sock_,(char*) buf, len, 0)) < 0){
				std::cerr << "Error sending TCP Packet. Size " << len << "\tError: ";
			#if defined(WIN32) || defined(_WIN32)
				std::cerr << WSAGetLastError() << std::endl;
			#else
				std::cerr << strerror(errno) << std::endl;
			#endif
			}
			break;
		
		case CONNECTION_UDP_CLIENT:
			// Leave if we don't know where to send to
			if(!UDPDatagramRecv) return sent;
			// send to the address loaded into the client address struct. 
			if((sent = sendto(sock_,(char*) buf, len, 0, (const sockaddr*) &address_, addressLen_)) < 0){
				std::cerr << "Error sending UDP Packet. Size " << len << "\tError: ";
			#if defined(WIN32) || defined(_WIN32)
				std::cerr << WSAGetLastError() << std::endl;
			#else	
				std::cerr << strerror(errno) << std::endl;
			#endif
			}
			break;

		default:
			throw std::runtime_error("Cannot write to unknown communication method");
			break;
	}
	return sent;
}

bool Communicator::validateBaudRate(){
	switch(connection_ops_.baud_rate){
		case 2400:
		case 4800:
		case 9600:
		case 19200:
		case 38400:
		case 57600:
		case 115200:
		case 230400:
		case 460800:
		case 500000:
		case 576000:
		case 921600:
		case 1000000:
		case 2000000:
			break;
		
		default:
			throw std::invalid_argument("Invalid Baud Rate supplied");
			return false;
	}
	return true;
}

bool Communicator::validateComPort(){
	comEnumerate();
	connection_ops_.index = comFindPort(connection_ops_.com_port.c_str());
	if(connection_ops_.index == -1){
		throw std::invalid_argument("Invalid Com Port");
		return false;
	}
	return true;
}

bool Communicator::validateIpAddress(){
	if(!adnav::utils::validateIP(this->connection_ops_.ip_address)){
		throw std::invalid_argument("Invalid IP Address");
		return false;
	}
	return true;
}

bool Communicator::validatePort(){
	if((connection_ops_.port < 0 )||(connection_ops_.port > 65535)){
		throw std::invalid_argument("Invalid Port");
		return false;
	}
	return true;
}

} // namespace adnav