// 
//
// 
// 
// 
//
//    
//
// 
//
// 
// 
// 

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <time.h>

#include "NTRIP_Client/NTRIP/ntripclient.h"
#include "rs232/rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#define RADIANS_TO_DEGREES (180.0/M_PI)
const double PI = 4*atan(1);



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //ros::Nodehandle nh;
  auto node = std::make_shared<rclcpp::Node>("AdNav_Node");

  std_msgs::msg::String sample_msg;

  RCLCPP_INFO(node->get_logger(), "argc: %d\n", argc);
  for(int i = 0; i < argc; i++){
    RCLCPP_INFO(node->get_logger(), "argv[%d}: %s\n", i, argv[i]);
  }
  RCLCPP_INFO(node->get_logger(), "Your Advanced Navigation ROS driver is currently running\nPress Ctrl-C to interrupt\n");  
  
  // Set up for Log File
  static const uint8_t request_all_configuration[] = { 0xE2, 0x01, 0x10, 0x9A, 0x73, 0xB6, 0xB4, 0xB5, 0xB8, 0xB9, 0xBA, 0xBC, 0xBD, 0xC0, 0xC2, 0xC3, 0xC4, 0x03, 0xC6, 0x45, 0xC7 };
  FILE *log_file;
  char filename[32];
  time_t rawtime;
  struct tm * timeinfo;
  int write_counter = 0;

  // Set up the COM port
	int baud_rate;
	std::string com_port;
	std::string imu_frame_id;
	std::string nav_sat_frame_id;
	std::string rawsensors_magnetometer_frame_id;
	std::string barometric_pressure_frame_id;
	std::string temperature_frame_id;
	std::string gnss_fix_type_id;
	std::string topic_prefix;
	std::stringstream gnssFixType;

  // NTRIP Varialbles	
	int error = 0;
	int bytes_received;
	int numbytes = 0;
	int remain = numbytes;
	int pos = 0;
	int state; // 0 = NTRIP Info provided, 1 = No Arguement, 3 = Baudrate and Port for serial 
	struct Args args;
	char buf[MAXDATASIZE];

  // Configuring based on the state, what sort of driver to run
	if (argc == 1){
		com_port = std::string("/dev/ttyUSB0");  
    baud_rate = 115200;
		state = 1;
	}
	else if (argc == 3) {
		com_port = std::string(argv[2]);
		baud_rate = atoi(argv[1]);
		state = 3;
	}
	else{
		getargs(argc, argv, &args);
		state = 0;
	}
  
  // Creating the ROS2 Publishers
  auto adnav_pub = node->create_publisher<std_msgs::msg::String>("/sample_publisher", 10);
  auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("/Imu", 10);
  auto nav_sat_fix_pub = node->create_publisher<sensor_msgs::msg::NavSatFix>("/NavSatFix", 10);
  auto magnetic_field_pub = node->create_publisher<sensor_msgs::msg::MagneticField>("/MagneticField", 10);
  auto barometric_pressure_pub = node->create_publisher<sensor_msgs::msg::FluidPressure>("/BarometricPressure", 10);
  auto temperature_pub = node->create_publisher<sensor_msgs::msg::Temperature>("/Temperature", 10);
  auto twist_pub = node->create_publisher<geometry_msgs::msg::Twist>("/Twist", 10);
  auto pose_pub = node->create_publisher<geometry_msgs::msg::Pose>("/Pose", 10);
  auto system_status_pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/SystemStatus", 10);
  auto filter_status_pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/FilterStatus", 10);
  auto gnss_fix_type_pub = node->create_publisher<std_msgs::msg::String>("/GNSSFixType", 10);
  rclcpp::Rate loop_rate(1000);
  int count = 0;
  
  // IMU sensor_msgs/Imu
	sensor_msgs::msg::Imu imu_msg;
	imu_msg.header.stamp.sec=0;
	imu_msg.header.stamp.nanosec=0;
	imu_msg.header.frame_id='0';
	imu_msg.orientation.x=0.0;
	imu_msg.orientation.y=0.0;
	imu_msg.orientation.z=0.0;
	imu_msg.orientation.w=0.0;
	imu_msg.orientation_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	imu_msg.angular_velocity.x=0.0;
	imu_msg.angular_velocity.y=0.0;
	imu_msg.angular_velocity.z=0.0;
	imu_msg.angular_velocity_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // fixed
	imu_msg.linear_acceleration.x=0.0;
	imu_msg.linear_acceleration.y=0.0;
	imu_msg.linear_acceleration.z=0.0;
	imu_msg.linear_acceleration_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // fixed

  // NavSatFix sensor_msgs/NavSatFix 
	sensor_msgs::msg::NavSatFix nav_sat_fix_msg;
	nav_sat_fix_msg.header.stamp.sec=0;
	nav_sat_fix_msg.header.stamp.nanosec=0;
	nav_sat_fix_msg.header.frame_id='0';
	nav_sat_fix_msg.status.status=0;
	nav_sat_fix_msg.status.service=1; // fixed to GPS
	nav_sat_fix_msg.latitude=0.0;
	nav_sat_fix_msg.longitude=0.0;
	nav_sat_fix_msg.altitude=0.0;
	nav_sat_fix_msg.position_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	nav_sat_fix_msg.position_covariance_type=2; // fixed to variance on the diagonal

  // MagneticField geometry_msg/magnetic_field
	sensor_msgs::msg::MagneticField magnetic_field_msg;
	magnetic_field_msg.magnetic_field.x=0;
	magnetic_field_msg.magnetic_field.y=0;
	magnetic_field_msg.magnetic_field.z=0;

	// Barometric Pressure sensor_msgs/fluidPressure
	sensor_msgs::msg::FluidPressure barometric_pressure_msg;
	barometric_pressure_msg.fluid_pressure=0;

	// Temperature sensor_msgs/Temperature
	sensor_msgs::msg::Temperature temperature_msg;
	temperature_msg.temperature=0;

	// Twist sensor_msgs/twist
	geometry_msgs::msg::Twist twist_msg;
	twist_msg.linear.x=0.0;
	twist_msg.linear.y=0.0;
	twist_msg.linear.z=0.0;
	twist_msg.angular.x=0.0;
	twist_msg.angular.y=0.0;
	twist_msg.angular.z=0.0;

  // Position in ECEF Postion (Packet 33) and Orientation in Quartenion Format (Same as IMU)
	geometry_msgs::msg::Pose pose_msg;
	pose_msg.position.x = 0;
	pose_msg.position.y = 0;
	pose_msg.position.z = 0;
	pose_msg.orientation.x=0.0;
	pose_msg.orientation.y=0.0;
	pose_msg.orientation.z=0.0;
	pose_msg.orientation.w=0.0;

	// DiagnosticsStatus messages for System Status
	diagnostic_msgs::msg::DiagnosticStatus system_status_msg;
	system_status_msg.level = 0; // default OK state
	system_status_msg.name = "System Status";
	system_status_msg.message = "";

	// DiagnosticsStatus messages for Filter Status
	diagnostic_msgs::msg::DiagnosticStatus filter_status_msg;
	filter_status_msg.level = 0; // default OK state
	filter_status_msg.name = "Filter Status";
	filter_status_msg.message = "";

  // Intialising for the log files
	rawtime = time(NULL);
	timeinfo = localtime(&rawtime);
	sprintf(filename, "Log_%02d-%02d-%02d_%02d-%02d-%02d.anpp", timeinfo->tm_year-100, timeinfo->tm_mon+1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
	log_file = fopen(filename, "wb");
  
  
  // Initialise packets
	an_decoder_t an_decoder;
	an_packet_t *an_packet;
	system_state_packet_t system_state_packet;
	quaternion_orientation_standard_deviation_packet_t quaternion_orientation_standard_deviation_packet;
	raw_sensors_packet_t raw_sensors_packet;
	ecef_position_packet_t ecef_position_packet;

  if(OpenComport(const_cast<char*>(com_port.c_str()), baud_rate))
  {
    printf("Could not open serial port: %s \n",com_port.c_str());
    exit(EXIT_FAILURE);
  }

  // Request Config packets and also start decoding anpp packets
	SendBuf((unsigned char*)request_all_configuration, sizeof(request_all_configuration));
	an_decoder_initialise(&an_decoder);
  
  while(rclcpp::ok()){
    std::stringstream ss;
    ss << "Sample Message [" << count++ << "]";
    sample_msg.data = ss.str();
    RCLCPP_DEBUG(node->get_logger(), "SOMETHING DEBUG");
    RCLCPP_WARN(node->get_logger(), "SOMETHING WARN");
    RCLCPP_INFO(node->get_logger(), "SOMETHING INFO");
    RCLCPP_ERROR(node->get_logger(), "SOMETHING ERROR");
    adnav_pub->publish(sample_msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
 
  return 0;
}
