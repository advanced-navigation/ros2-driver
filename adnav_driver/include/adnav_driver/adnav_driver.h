#ifndef ADVANCED_NAVIGATION_DRIVER_H
#define ADVANCED_NAVIGATION_DRIVER_H

// #include "ntripclient.h"
#include "rs232.h"
#include "an_packet_protocol.h"
#include "ins_packets.h"
#include "adnav_utils.h"
#include "adnav_comms.h"
#include "adnav_logger.h"
#include "adnav_ntrip.h"


// Adnav_interfaces
#include <adnav_interfaces/srv/packet_periods.hpp>
#include <adnav_interfaces/srv/packet_timer_period.hpp>
#include <adnav_interfaces/srv/request_packet.hpp>
#include <adnav_interfaces/srv/ntrip.hpp>
#include <adnav_interfaces/msg/llh.hpp>

#include <chrono>       // Time, std::chrono
#include <functional>   // std::placeholder
#include <memory>       // smart pointers
#include <string>       // std::string
#include <mutex>        // std::mutex, std:unique_lock
#include <condition_variable>   // std::condition_variable
#include <stdio.h>      // FILE
#include <sstream>      // Stringstream

#include <fstream>

// ROS2 Packages, Services, Messages
#include <rclcpp/rclcpp.hpp>                                
#include <tf2/LinearMath/Quaternion.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>  
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <std_srvs/srv/empty.hpp>

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

#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>


namespace adnav{

constexpr const double RADIANS_TO_DEGREES = (180.0/M_PI);
constexpr const int    DEFAULT_BAUD_RATE = 115200;
constexpr const int    DEFAULT_TIMER_PERIOD = 20000;
constexpr const int    DEFAULT_PACKET_TIMER_PERIOD = 10000;
constexpr const char * DEFAULT_COM_PORT = "ttyUSB0";
constexpr const int    DEFAULT_PACKET_REQUEST[4] = {20, 10, 28, 10};
constexpr const char * DEFAULT_PACKET_REQUEST_STR = "20, 10, 28, 10";
constexpr const char * DEFAULT_IP_ADDRESS = "0.0.0.0";
constexpr const bool   DEFAULT_NTRIP_STATE = false;
constexpr const int    DEFAULT_GPGGA_REPORT_PERIOD = 1; // Second(s)
constexpr const int    DEFAULT_TIMEOUT = 5;
constexpr const int    MAX_TIMER_PERIOD = 65535;
constexpr const int    MIN_TIMER_PERIOD = 1000;
constexpr const int    MIN_PACKET_PERIOD = 1;
constexpr const int    MAX_PACKET_PERIOD = 65535;
constexpr const int    MIN_PORT = 0;
constexpr const int    MAX_PORT = 65535;

typedef struct{
    bool en;
    std::string ip;
    int port;
    struct hostent *he;
    std::string username;
    std::string password;
    std::string mountpoint;
}ntrip_client_state_t;

class Driver : public rclcpp::Node // Inheriting gives every "this->" as a pointer to the node.
{
    public:
        // ~~~~ Constructors
        Driver();
        ~Driver();
        
    private:

        // Debug variables
        int pub_num_ = 0, P28_num_ = 0, P20_num_ = 0, P27_num_ = 0, P33_num_ = 0, P0_num_ = 0;

        // Defines what communication method to use, refer to adnav_driver_connection_e.
        int communication_state_;

		// String to hold frame_id
		std::string frame_id_ = "imu_link";

        // String to hold node name.
        std::string node_name_;

		// device communication settings
        std::unique_ptr<adnav::Communicator> communicator_;
        adnav_connections_data_t comms_data_;

        // Packet settings;
        std::vector<int64_t> packet_request_;
        int packet_timer_period_;

		// Log files.
        std::string log_path_;
        adnav::Logger anpp_logger_;
        adnav::Logger rtcm_logger_;

        // ANPP Packet variables
        acknowledge_packet_t acknowledge_packet_; // only access with protection of acknowledge_mutex_
        device_information_packet_t device_information_packet_;

        // Msgs. Only access with protection of messages_mutex_
        tf2::Quaternion                 orientation_;
        sensor_msgs::msg::Imu           imu_msg_;
        sensor_msgs::msg::Imu           imu_raw_msg_;
        sensor_msgs::msg::MagneticField mag_field_msg_;
        sensor_msgs::msg::NavSatFix     nav_fix_msg_;
        sensor_msgs::msg::FluidPressure baro_msg_;
        sensor_msgs::msg::Temperature   temp_msg_;
        geometry_msgs::msg::Twist       twist_msg_;
        geometry_msgs::msg::Pose        pose_msg_;
        diagnostic_msgs::msg::DiagnosticStatus system_status_msg_;
        diagnostic_msgs::msg::DiagnosticStatus filter_status_msg_;

        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr             		imu_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr             		imu_raw_pub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr       		nav_sat_fix_pub_;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr 			magnetic_field_pub_;
        rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr 			barometric_pressure_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr 			temperature_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr 				twist_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr 					pose_pub_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr 	system_status_pub_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr 	filter_status_pub_;
        
        // ~~~~~~~~~~~~~~~ Callback handles and parameters
        // Callback groups Allows the callbacks to be processed on a different thread by
        // the Multithread executor.
        rclcpp::CallbackGroup::SharedPtr publishing_group_;
        rclcpp::CallbackGroup::SharedPtr reading_group_;
        rclcpp::CallbackGroup::SharedPtr service_group_;

        // Parameter callbacks and handlers
        OnSetParametersCallbackHandle::SharedPtr param_set_cb_;
        std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> publish_us_cb_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> read_us_cb_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> packet_request_cb_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> packet_timer_cb_;

        // Timers 
        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::TimerBase::SharedPtr read_timer_;
        // Timer intervals
        std::chrono::microseconds publish_timer_interval_;
        std::chrono::microseconds read_timer_interval_;
        
        //Service Handlers
        rclcpp::Service<adnav_interfaces::srv::PacketPeriods>::SharedPtr packet_period_srv_;
        rclcpp::Service<adnav_interfaces::srv::PacketTimerPeriod>::SharedPtr packet_period_timer_srv_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr restart_pub_srv_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr restart_read_srv_;
        rclcpp::Service<adnav_interfaces::srv::RequestPacket>::SharedPtr request_packet_srv_;
        rclcpp::Service<adnav_interfaces::srv::Ntrip>::SharedPtr ntrip_srv_;

        // Threading variables
        std::mutex messages_mutex_;
        std::condition_variable msg_cv_;
        bool msg_write_done_;

        std::mutex acknowledge_mutex_;
        std::condition_variable srv_cv_;
        bool acknowledge_recieve_;

        // NTRIP Variables
        std::unique_ptr<adnav::ntrip::Client> ntrip_client_;
        ntrip_client_state_t ntrip_state_;
        adnav_interfaces::msg::LLH llh_;
        std::ofstream rtcm_log_file_;
        char rtcm_filename_[200];

        //~~~~~~~~~~~~~~~~~~~~~ Private Methods. 

        //~~~~~~ Setup Functions
        void waitForDevicePacket();
        void requestDeviceInfo();
        void createPublishers();
        void createServices();
        void deviceSetup();
        void setupParamService();
        void setupParams();

        //~~~~~~ Control Functions
        void recievePackets();
        void publishTimerCallback();
        void RestartPublisher();
        void RestartReader();

        //~~~~~~ Logging Functions
        void openLogFile();
        void statusErrLog(const std::string& errmsg);
        void statusWarnLog(const std::string& warnmsg);

        //~~~~~~ ROS Services
        void srvPacketPeriods(const std::shared_ptr<adnav_interfaces::srv::PacketPeriods::Request> request, 
		        std::shared_ptr<adnav_interfaces::srv::PacketPeriods::Response> response);
        void srvPacketTimerPeriod(const std::shared_ptr<adnav_interfaces::srv::PacketTimerPeriod::Request> request, 
		        std::shared_ptr<adnav_interfaces::srv::PacketTimerPeriod::Response> response);
        void srvRequestPacket(const std::shared_ptr<adnav_interfaces::srv::RequestPacket::Request> request, 
		    std::shared_ptr<adnav_interfaces::srv::RequestPacket::Response> response);
        void srvNtrip(const std::shared_ptr<adnav_interfaces::srv::Ntrip::Request> request, 
		    std::shared_ptr<adnav_interfaces::srv::Ntrip::Response> response);
        
        //~~~~~~ Parameter Functions
        rcl_interfaces::msg::SetParametersResult ParamSetCallback(const std::vector<rclcpp::Parameter>& Params);
        rcl_interfaces::msg::SetParametersResult validateBaudRate(const rclcpp::Parameter& parameter);
        rcl_interfaces::msg::SetParametersResult validateComPort(const rclcpp::Parameter& parameter);
        rcl_interfaces::msg::SetParametersResult validatePublishUs(const rclcpp::Parameter& parameter);
        void updatePublishUs(const rclcpp::Parameter& parameter);
        rcl_interfaces::msg::SetParametersResult validateReadUs(const rclcpp::Parameter& parameter);
        void updateReadUs(const rclcpp::Parameter& parameter);
        rcl_interfaces::msg::SetParametersResult validatePacketRequest(const rclcpp::Parameter& parameter);
        void updatePacketRequest(const rclcpp::Parameter& parameter);
        rcl_interfaces::msg::SetParametersResult validatePacketTimer(const rclcpp::Parameter& parameter);
        void updatePacketTimer(const rclcpp::Parameter& parameter);
        void validateAndSaveIPAddress(const rclcpp::Parameter& parameter);
        
        //~~~~~~ NTRIP Functions
        void updateNTRIPClientService();
        void getDataFromHostStr(const std::string& host);
        void NtripReceiveFunction(const char* buffer, int size);


        //~~~~~~ Device Communication Functions
        void encodeAndSend(an_packet_t* an_packet);
        adnav_interfaces::msg::RawAcknowledge AcknowledgeHandler();
        adnav_interfaces::msg::RawAcknowledge SendPacketTimer(int packet_timer_period, bool utc_sync = true , bool permanent = true);
        adnav_interfaces::msg::RawAcknowledge SendPacketPeriods(const std::vector<adnav_interfaces::msg::PacketPeriod>& periods, 
            bool clear_existing = true, bool permanent = true);
            
        //~~~~~~ Decoders
        void decodePackets(an_decoder_t &an_decoder, const int &bytes_received);
        void acknowledgeDecoder(an_packet_t* an_packet);
        void deviceInfoDecoder(an_packet_t* an_packet);
        void systemStateRosDecoder(an_packet_t* an_packet);
        void ecefPosRosDecoder(an_packet_t* an_packet);
        void quartOrientSDRosDriver(an_packet_t* an_packet);
        void rawSensorsRosDecoder(an_packet_t* an_packet);
};

}// namespace adnav

#endif //ADVANCED_NAVIGATION_DRIVER_H