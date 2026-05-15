/****************************************************************/
/*                                                              */
/*                       Advanced Navigation                    */
/*         				Device Communications		  			*/
/*          Copyright 2024, Advanced Navigation Pty Ltd         */
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

#pragma once

#ifndef ADVANCED_NAVIGATION_DRIVER_H
#define ADVANCED_NAVIGATION_DRIVER_H

#include <cstdio>
#include <fstream>

// C++ System Headers
#include <chrono>           // Time, std::chrono
#include <memory>           // smart pointers
#include <vector>           // std::vector
#include <string>           // std::string
#include <mutex>            // std::mutex, std::unique_lock
#include <future>           // std::promise, std::future
#include <queue>            // std::queue
#include <unordered_map>    // std::unordered_map
#include <thread>

#include <an_packet_protocol.h>
#include <ins_packets.h>
#include <adnav_logger.h>
#include <adnav_ntrip.h>

#include "anpp.hpp"
#include "utils.hpp"

#include "uvw.hpp"

#include <adnav_driver/adnav_parameters.hpp>

// Adnav_interfaces
#include <adnav_interfaces/srv/packet_periods.hpp>
#include <adnav_interfaces/srv/packet_timer_period.hpp>
#include <adnav_interfaces/srv/request_packets.hpp>
#include <adnav_interfaces/srv/ntrip.hpp>
#include <adnav_interfaces/msg/llh.hpp>

// ROS2 Packages, Services, Messages
#include <rclcpp/rclcpp.hpp>
#include "diagnostic_updater/diagnostic_updater.hpp"
#include <tf2/LinearMath/Quaternion.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geographic_msgs/msg/geo_pose.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>

namespace adnav {

    inline constexpr std::array<uint8_t, 6> REQUIRED_PACKETS = {
        {
            packet_id_system_state,
            packet_id_body_velocity,
            packet_id_euler_orientation_standard_deviation,
            packet_id_velocity_standard_deviation,
            packet_id_running_time,
            packet_id_gnss_position_velocity_time
        }};

constexpr double RADIANS_TO_DEGREES = (180.0/M_PI);
constexpr int    DEFAULT_GPGGA_REPORT_PERIOD = 1;  // Second(s)
constexpr int    DEFAULT_TIMEOUT = 2;
constexpr int    MAX_TIMER_PERIOD = 65535;
constexpr int    MIN_TIMER_PERIOD = 1000;
constexpr int    MIN_PACKET_PERIOD = 1;
constexpr int    MAX_PACKET_PERIOD = 65535;

typedef struct {
    bool en;
    std::string ip;
    int port;
    struct hostent *he;
    std::string username;
    std::string password;
    std::string mountpoint;
} ntrip_client_state_t;

class Driver : public rclcpp::Node
{
 public:
    Driver();
    ~Driver() override;

 private:
    std::shared_ptr<adnav::ParamListener> param_listener_;

    // String to hold frame_id
    std::string frame_id_ = "imu_link";
    std::string external_frame_id_ = "map";

    // device communication settings
    std::jthread connection_thread_;
    std::jthread setup_thread_;                     // runs deviceSetup() off the event loop
    std::shared_ptr<uvw::async_handle> close_async_;
    std::shared_ptr<uvw::async_handle> write_async_;

    // Write queue — encodeAndSend pushes here; write_async_ drains it on the loop thread
    std::mutex write_q_mutex_;
    std::queue<std::pair<std::unique_ptr<char[]>, unsigned int>> write_q_;

    // Log files.
    adnav::Logger anpp_logger_;
    adnav::Logger rtcm_logger_;

    std::unordered_map<uint8_t, rclcpp::Time> packet_receive_times;
    std::optional<rclcpp::Duration> device_running_time_;

    // ANPP Packet variables
    std::optional<device_information_packet_t> device_information_packet_;
    TimedBox<system_state_packet_t> system_state_packet_;
    TimedBox<raw_sensors_packet_t> raw_sensors_packet_;
    TimedBox<euler_orientation_standard_deviation_packet_t> euler_orientation_standard_deviation_packet_;
    TimedBox<velocity_standard_deviation_packet_t> velocity_standard_deviation_packet_;
    TimedBox<gnss_position_velocity_time_packet_t> gnss_position_velocity_time_packet_;

    // Msgs. Only access with protection of messages_mutex_
    tf2::Quaternion                 orientation_frd_ned_;
    tf2::Quaternion                 orientation_flu_;
    sensor_msgs::msg::Imu           imu_msg_;
    sensor_msgs::msg::Imu           imu_raw_msg_;
    sensor_msgs::msg::MagneticField mag_field_msg_;
    sensor_msgs::msg::NavSatFix     nav_fix_msg_;
    sensor_msgs::msg::FluidPressure baro_msg_;
    sensor_msgs::msg::Temperature   temp_msg_;
    geometry_msgs::msg::Twist       twist_flu_msg_;
    geometry_msgs::msg::TwistStamped twist_stamped_msg_;
    geometry_msgs::msg::TwistStamped twist_stamped_msg_body; // this should be the same as post ENU --> FLU
    geometry_msgs::msg::TwistStamped twist_stamped_msg_enu;
    geometry_msgs::msg::TwistStamped twist_stamped_msg_external_body;
    geometry_msgs::msg::PoseStamped pose_ecef_stamped_msg_;
    geometry_msgs::msg::PoseStamped pose_utm_stamped_msg_;
    geographic_msgs::msg::GeoPose   geo_pose_msg_;
    geographic_msgs::msg::GeoPoseStamped geo_pose_stamped_msg_;


    std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr             		imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr             		imu_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr       		nav_sat_fix_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr 			magnetic_field_pub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr 			barometric_pressure_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr 			temperature_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr 				twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr 			twist_stamped_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr 			twist_stamped_enu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr 			twist_stamped_body_velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr 			twist_stamped_external_body_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr 			pose_ecef_stamped_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr 			pose_utm_stamped_pub_;
    rclcpp::Publisher<geographic_msgs::msg::GeoPose>::SharedPtr 			geo_pose_pub_;
    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr 		geo_pose_stamped_pub_;

    // ~~~~~~~~~~~~~~~ Callback handles and parameters
    // Callback groups Allows the callbacks to be processed on a different thread by
    // the Multithread executor.
    rclcpp::CallbackGroup::SharedPtr publishing_group_;
    rclcpp::CallbackGroup::SharedPtr service_group_;

    // Parameter callbacks and handlers
    OnSetParametersCallbackHandle::SharedPtr param_set_cb_;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> publish_us_cb_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> packet_request_cb_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> packet_timer_cb_;

    // Timers
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // Service Handlers
    rclcpp::Service<adnav_interfaces::srv::PacketTimerPeriod>::SharedPtr packet_period_timer_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr restart_pub_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr restart_read_srv_;
    rclcpp::Service<adnav_interfaces::srv::Ntrip>::SharedPtr ntrip_srv_;

    // Threading variables
    std::mutex messages_mutex_;

    // Pending request-reply tracking. Keyed by the expected reply packet_id.
    // Works for any packet type — register a PendingReply before sending, and
    // decodePackets will dispatch it when that packet_id arrives.
    struct PendingReply {
        std::function<void(an_packet_t*)>       on_packet;  // called on the loop thread with the reply
        std::function<void(std::exception_ptr)> on_cancel;  // called when the connection drops
    };
    std::mutex pending_replies_mutex_;
    std::unordered_map<uint8_t, PendingReply> pending_replies_;

    // NTRIP Variables
    std::unique_ptr<adnav::ntrip::Client> ntrip_client_;
    ntrip_client_state_t ntrip_state_;
    adnav_interfaces::msg::LLH llh_;
    std::ofstream rtcm_log_file_;
    char rtcm_filename_[200];

    //~~~~~~~~~~~~~~~~~~~~~ Private Methods.

    //~~~~~~ Setup Functions
    bool requestDeviceInfo();
    void deviceSetup();

    void createPublishers();
    void createServices();
    void setupParamService();

    //~~~~~~ Control Functions
    void receivePackets(std::span<const uint8_t> buffer);
    void publishTimerCallback();
    void RestartPublisher();

    //~~~~~~ Logging Functions
    void system_status_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void filter_status_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void accuracy_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void packets_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);

    //~~~~~~ ROS Services
    void srvPacketTimerPeriod(const std::shared_ptr<adnav_interfaces::srv::PacketTimerPeriod::Request> request,
            std::shared_ptr<adnav_interfaces::srv::PacketTimerPeriod::Response> response);
    void srvNtrip(const std::shared_ptr<adnav_interfaces::srv::Ntrip::Request> request,
        std::shared_ptr<adnav_interfaces::srv::Ntrip::Response> response);

    //~~~~~~ Parameter Functions
    std::vector<adnav_interfaces::msg::PacketPeriod> getPacketRequest();
    rcl_interfaces::msg::SetParametersResult ParamSetCallback(const std::vector<rclcpp::Parameter>& Params);
    rcl_interfaces::msg::SetParametersResult validateComPort(const rclcpp::Parameter& parameter);
    rcl_interfaces::msg::SetParametersResult validatePublishUs(const rclcpp::Parameter& parameter);
    void updatePublishUs(const rclcpp::Parameter& parameter);
    rcl_interfaces::msg::SetParametersResult validatePacketRequest(const rclcpp::Parameter& parameter);
    void updatePacketRequest(const rclcpp::Parameter& parameter);
    rcl_interfaces::msg::SetParametersResult validatePacketTimer(const rclcpp::Parameter& parameter);
    void updatePacketTimer(const rclcpp::Parameter& parameter);

    //~~~~~~ NTRIP Functions
    void updateNTRIPClientService();
    void getDataFromHostStr(const std::string& host);
    void NtripReceiveFunction(const char* buffer, int size);


    //~~~~~~ Device Communication Functions
    void encodeAndSend(an_packet_t* an_packet);
    adnav_interfaces::msg::RawAcknowledge sendAndAwaitAck(an_packet_t* an_packet);
    void failAllPendingReplies();
    adnav_interfaces::msg::RawAcknowledge SendPacketTimer(int packet_timer_period, bool utc_sync = true , bool permanent = true);
    adnav_interfaces::msg::RawAcknowledge SendPacketPeriods(const std::vector<adnav_interfaces::msg::PacketPeriod>& periods,
        bool clear_existing = true, bool permanent = true);

    //~~~~~~ Decoders
    void decodePackets(an_decoder_t &an_decoder, const int &bytes_received);
    void deviceInfoDecoder(an_packet_t* an_packet);
    void systemStateRosDecoder(an_packet_t* an_packet);
    void gnssPosVelTimeRosDecoder(an_packet_t* an_packet);
    void ecefPosRosDecoder(an_packet_t* an_packet);
    void utmPosRosDecoder(an_packet_t* an_packet);
    void quartOrientSDRosDriver(an_packet_t* an_packet);
    void rawSensorsRosDecoder(an_packet_t* an_packet);
    void extBodyVelRosDecoder(an_packet_t *an_packet);
    void bodyVelRosDecoder(an_packet_t *an_packet);

    bool packet_is_recent(uint8_t packet_id, std::chrono::microseconds max_age);
};

}  // namespace adnav

#endif  // ADVANCED_NAVIGATION_DRIVER_H
