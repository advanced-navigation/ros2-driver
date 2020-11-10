/****************************************************************/
/*                                                              */
/*                       Advanced Navigation                    */
/*         					  ROS2 Driver			  			*/
/*          Copyright 2020, Advanced Navigation Pty Ltd         */
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <tf2/LinearMath/Quaternion.h>
#include "NTRIP_Client/NTRIP/ntripclient.h"
#include "rs232/rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
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

//RCLCPP_WARN(node->get_logger(), "WARNING MSG PRINT");
//RCLCPP_INFO(node->get_logger(), "INFORMATION MSG PRINT");
//RCLCPP_ERROR(node->get_logger(), "ERROR MSG PRINT");

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>("AdNav_Node");
	
	/* For Debugging
	RCLCPP_INFO(node->get_logger(), "argc: %d\n", argc);
	for(int i = 0; i < argc; i++){
		RCLCPP_INFO(node->get_logger(), "argv[%d}: %s\n", i, argv[i]);
	}
	*/
	if(argc == 1){
		printf("usage: ros2 run package_name executable_name [baud_rate] [comm_port]\npackage_name     Name of the ROS package\nexecutable_name  Name of the executable\nbaud_rate        The Baud rate configured on the device. Default 115200\ncomm_port        The COM port of the connected device. Default /dev/ttyUSB0\n");
		exit(EXIT_FAILURE);	
	}
	else{
		RCLCPP_INFO(node->get_logger(), "Your Advanced Navigation ROS driver is currently running\nPress Ctrl-C to interrupt\n");  
	}

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

	// String ID for all publishers
	std::string imu_frame_id;
	std::string nav_sat_frame_id;
	std::string rawsensors_magnetometer_frame_id;
	std::string barometric_pressure_frame_id;
	std::string temperature_frame_id;
	std::string gnss_fix_type_id;
	std::string topic_prefix;
	std::stringstream gnssFixType;
	tf2::Quaternion orientation;
	std_msgs::msg::String gnss_fix_type_msgs;

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
		printf("usage: ros2 run package_name executable_name [baud_rate] [comm_port]\npackage_name     Name of the ROS package\nexecutable_name  Name of the executable\nbaud_rate        The Baud rate configured on the device. Default 115200\ncomm_port        The COM port of the connected device. Default /dev/ttyUSB0\n");
		exit(EXIT_FAILURE);	
		//com_port = std::string("/dev/ttyUSB0");  
    	//baud_rate = 115200;
		//state = 1;
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
	rclcpp::Rate loop_rate(10);

	// IMU sensor_msgs/Imu
	sensor_msgs::msg::Imu imu_msg;
	imu_msg.header.stamp.sec = 0;
	imu_msg.header.stamp.nanosec = 0;
	imu_msg.header.frame_id = "imu";
	imu_msg.orientation.x = 0.0;
	imu_msg.orientation.y = 0.0;
	imu_msg.orientation.z = 0.0;
	imu_msg.orientation.w = 0.0;
	imu_msg.orientation_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	imu_msg.angular_velocity.x = 0.0;
	imu_msg.angular_velocity.y = 0.0;
	imu_msg.angular_velocity.z = 0.0;
	imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // fixed
	imu_msg.linear_acceleration.x = 0.0;
	imu_msg.linear_acceleration.y = 0.0;
	imu_msg.linear_acceleration.z = 0.0;
	imu_msg.linear_acceleration_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // fixed

	// NavSatFix sensor_msgs/NavSatFix 
	sensor_msgs::msg::NavSatFix nav_sat_fix_msg;
	nav_sat_fix_msg.header.stamp.sec = 0;
	nav_sat_fix_msg.header.stamp.nanosec = 0;
	nav_sat_fix_msg.header.frame_id = "gps";
	nav_sat_fix_msg.status.status = 0;
	nav_sat_fix_msg.status.service = 1; // fixed to GPS
	nav_sat_fix_msg.latitude = 0.0;
	nav_sat_fix_msg.longitude = 0.0;
	nav_sat_fix_msg.altitude = 0.0;
	nav_sat_fix_msg.position_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	nav_sat_fix_msg.position_covariance_type = 2; // fixed to variance on the diagonal

  	// MagneticField geometry_msg/magnetic_field
	sensor_msgs::msg::MagneticField magnetic_field_msg;
	magnetic_field_msg.magnetic_field.x = 0;
	magnetic_field_msg.magnetic_field.y = 0;
	magnetic_field_msg.magnetic_field.z = 0;
	magnetic_field_msg.header.frame_id = "rawsensors_magnetometer";

	// Barometric Pressure sensor_msgs/fluidPressure
	sensor_msgs::msg::FluidPressure barometric_pressure_msg;
	barometric_pressure_msg.fluid_pressure=0;
	barometric_pressure_msg.header.frame_id = "barometric_pressure";

	// Temperature sensor_msgs/Temperature
	sensor_msgs::msg::Temperature temperature_msg;
	temperature_msg.temperature=0;
	temperature_msg.header.frame_id = "temperature";

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

	if(state == 0){
		if (OpenComport(args.serdevice, args.baud)){
			printf("Could not open serial port\n");
			exit(EXIT_FAILURE);			
		}
		error = ntrip_initialise(&args, buf);
		if(error){
			printf("ERROR\n");
		}
		else{
			//printf("NOT ERROR\n");
			error += 0;
		}
	}
	if(state == 1 || state == 3){
		if (OpenComport(const_cast<char*>(com_port.c_str()), baud_rate))
		{
			printf("Could not open serial port: %s \n",com_port.c_str());
			exit(EXIT_FAILURE);
		}
	}

  	// Request Config packets and also start decoding anpp packets
	SendBuf((unsigned char*)request_all_configuration, sizeof(request_all_configuration));
	an_decoder_initialise(&an_decoder);
  
	while(rclcpp::ok() && !error){
		std::stringstream ss;
		if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0)
		{
			fwrite(an_decoder_pointer(&an_decoder), sizeof(uint8_t), bytes_received, log_file);
			// Increment the decode buffer length by the number of bytes received 
			an_decoder_increment(&an_decoder, bytes_received);

			// Decode all the packets in the buffer 
			while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
			{				
				
				// System State Packet Decoding
				if (an_packet->id == packet_id_system_state)
				{
					if(decode_system_state_packet(&system_state_packet, an_packet) == 0)
					{
						// GNSS FIX TYPE
						switch(system_state_packet.filter_status.b.gnss_fix_type)
						{
							case 0: 
								gnssFixType.str("No GNSS fix");
								break;
							case 1:
								gnssFixType.str("2D Fix");
								break;
							case 2:
								gnssFixType.str("3D Fix");
								break;
							case 3:
								gnssFixType.str("SBAS Fix");
								break;
							case 4:
								gnssFixType.str("Differential Fix");
								break;
							case 5:
								gnssFixType.str("Omnistar/Starfire Fix");
								break;
							case 6:
								gnssFixType.str("RTK Float");
								break;
							case 7:
								gnssFixType.str("RTK Fixed");
								break;
							default:
								gnssFixType.str("NOT CONNECTED");
						}						
						gnss_fix_type_msgs.data = gnssFixType.str();

						// NAVSATFIX
						nav_sat_fix_msg.header.stamp.sec=system_state_packet.unix_time_seconds;
						nav_sat_fix_msg.header.stamp.nanosec=system_state_packet.microseconds*1000;
						nav_sat_fix_msg.header.frame_id=nav_sat_frame_id;
						if ((system_state_packet.filter_status.b.gnss_fix_type == 1) ||
							(system_state_packet.filter_status.b.gnss_fix_type == 2))
						{
							nav_sat_fix_msg.status.status=0;
						}
						else if ((system_state_packet.filter_status.b.gnss_fix_type == 3) ||
							 (system_state_packet.filter_status.b.gnss_fix_type == 5))
						{
							nav_sat_fix_msg.status.status=1;
						}
						else if ((system_state_packet.filter_status.b.gnss_fix_type == 4) ||
							 (system_state_packet.filter_status.b.gnss_fix_type == 6) ||
							 (system_state_packet.filter_status.b.gnss_fix_type == 7))
						{
							nav_sat_fix_msg.status.status=2;
						}
						else
						{
							nav_sat_fix_msg.status.status=-1;
						}
						nav_sat_fix_msg.latitude=system_state_packet.latitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.longitude=system_state_packet.longitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.altitude=system_state_packet.height;
						nav_sat_fix_msg.position_covariance={pow(system_state_packet.standard_deviation[1],2), 0.0, 0.0,
							0.0, pow(system_state_packet.standard_deviation[0],2), 0.0,
							0.0, 0.0, pow(system_state_packet.standard_deviation[2],2)};						


						// TWIST
						twist_msg.linear.x=system_state_packet.velocity[0];
						twist_msg.linear.y=system_state_packet.velocity[1];
						twist_msg.linear.z=system_state_packet.velocity[2];
						twist_msg.angular.x=system_state_packet.angular_velocity[0];
						twist_msg.angular.y=system_state_packet.angular_velocity[1];
						twist_msg.angular.z=system_state_packet.angular_velocity[2];


						// IMU
						imu_msg.header.stamp.sec=system_state_packet.unix_time_seconds;
						imu_msg.header.stamp.nanosec=system_state_packet.microseconds*1000;
						imu_msg.header.frame_id=imu_frame_id;
						// Using the RPY orientation as done by cosama
						orientation.setRPY(
							system_state_packet.orientation[0],
							system_state_packet.orientation[1],
							PI/2.0f - system_state_packet.orientation[2] //REP 103
						);
						imu_msg.orientation.x = orientation[0];
						imu_msg.orientation.y = orientation[1];
						imu_msg.orientation.z = orientation[2];
						imu_msg.orientation.w = orientation[3];

						// POSE Orientation
						pose_msg.orientation.x = orientation[0];
						pose_msg.orientation.y = orientation[1];
						pose_msg.orientation.z = orientation[2];
						pose_msg.orientation.w = orientation[3];

						imu_msg.angular_velocity.x=system_state_packet.angular_velocity[0]; // These the same as the TWIST msg values
						imu_msg.angular_velocity.y=system_state_packet.angular_velocity[1];
						imu_msg.angular_velocity.z=system_state_packet.angular_velocity[2];

						//The IMU linear acceleration is now coming from the RAW Sensors Accelerometer 
						//imu_msg.linear_acceleration.x=system_state_packet.body_acceleration[0];
						//imu_msg.linear_acceleration.y=system_state_packet.body_acceleration[1];
						//imu_msg.linear_acceleration.z=system_state_packet.body_acceleration[2];

						// SYSTEM STATUS
						system_status_msg.message = "";
						system_status_msg.level = 0; // default OK state
						if (system_state_packet.system_status.b.system_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "0. System Failure! ";
						}
						if (system_state_packet.system_status.b.accelerometer_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "1. Accelerometer Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.gyroscope_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "2. Gyroscope Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.magnetometer_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "3. Magnetometer Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.pressure_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "4. Pressure Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.gnss_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "5. GNSS Failure! ";
						}
						if (system_state_packet.system_status.b.accelerometer_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "6. Accelerometer Over Range! ";
						}
						if (system_state_packet.system_status.b.gyroscope_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "7. Gyroscope Over Range! ";
						}
						if (system_state_packet.system_status.b.magnetometer_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "8. Magnetometer Over Range! ";
						}
						if (system_state_packet.system_status.b.pressure_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "9. Pressure Over Range! ";
						}
						if (system_state_packet.system_status.b.minimum_temperature_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "10. Minimum Temperature Alarm! ";
						}
						if (system_state_packet.system_status.b.maximum_temperature_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "11. Maximum Temperature Alarm! ";
						}
						if (system_state_packet.system_status.b.low_voltage_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "12. Low Voltage Alarm! ";
						}
						if (system_state_packet.system_status.b.high_voltage_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "13. High Voltage Alarm! ";
						}
						if (system_state_packet.system_status.b.gnss_antenna_disconnected) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "14. GNSS Antenna Disconnected! ";
						}
						if (system_state_packet.system_status.b.serial_port_overflow_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "15. Data Output Overflow Alarm! ";
						}

						// FILTER STATUS
						filter_status_msg.message = "";
						filter_status_msg.level = 0; // default OK state
						if (system_state_packet.filter_status.b.orientation_filter_initialised) {
							filter_status_msg.message = filter_status_msg.message + "0. Orientation Filter Initialised. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "0. Orientation Filter NOT Initialised. ";
						}
						if (system_state_packet.filter_status.b.ins_filter_initialised) {
							filter_status_msg.message = filter_status_msg.message + "1. Navigation Filter Initialised. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "1. Navigation Filter NOT Initialised. ";
						}
						if (system_state_packet.filter_status.b.heading_initialised) {
							filter_status_msg.message = filter_status_msg.message + "2. Heading Initialised. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "2. Heading NOT Initialised. ";
						}
						if (system_state_packet.filter_status.b.utc_time_initialised) {
							filter_status_msg.message = filter_status_msg.message + "3. UTC Time Initialised. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "3. UTC Time NOT Initialised. ";
						}
						if (system_state_packet.filter_status.b.event1_flag) {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "7. Event 1 Occured. ";
						}
						else {
							filter_status_msg.message = filter_status_msg.message + "7. Event 1 NOT Occured. ";
						}
						if (system_state_packet.filter_status.b.event2_flag) {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "8. Event 2 Occured. ";
						}
						else {
							filter_status_msg.message = filter_status_msg.message + "8. Event 2 NOT Occured. ";
						}
						if (system_state_packet.filter_status.b.internal_gnss_enabled) {
							filter_status_msg.message = filter_status_msg.message + "9. Internal GNSS Enabled. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "9. Internal GNSS NOT Enabled. ";
						}
						if (system_state_packet.filter_status.b.magnetic_heading_enabled) {
							filter_status_msg.message = filter_status_msg.message + "10. Magnetic Heading Active. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "10. Magnetic Heading NOT Active. ";
						}
						if (system_state_packet.filter_status.b.velocity_heading_enabled) {
							filter_status_msg.message = filter_status_msg.message + "11. Velocity Heading Enabled. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "11. Velocity Heading NOT Enabled. ";
						}
						if (system_state_packet.filter_status.b.atmospheric_altitude_enabled) {
							filter_status_msg.message = filter_status_msg.message + "12. Atmospheric Altitude Enabled. ";
						}
						else {
							filter_status_msg.message = filter_status_msg.message + "12. Atmospheric Altitude NOT Enabled. ";
							filter_status_msg.level = 1; // WARN state
						}
						if (system_state_packet.filter_status.b.external_position_active) {
							filter_status_msg.message = filter_status_msg.message + "13. External Position Active. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "13. External Position NOT Active. ";
						}
						if (system_state_packet.filter_status.b.external_velocity_active) {
							filter_status_msg.message = filter_status_msg.message + "14. External Velocity Active. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "14. External Velocity NOT Active. ";
						}
						if (system_state_packet.filter_status.b.external_heading_active) {
							filter_status_msg.message = filter_status_msg.message + "15. External Heading Active. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "15. External Heading NOT Active. ";
						}
					}
				}

				// ECEF Position (in meters) Packet for Pose Message
				if(an_packet->id == packet_id_ecef_position)
				{		
					if(decode_ecef_position_packet(&ecef_position_packet, an_packet) == 0)
					{
						pose_msg.position.x = ecef_position_packet.position[0];
						pose_msg.position.y = ecef_position_packet.position[1];
						pose_msg.position.z = ecef_position_packet.position[2];
					}
				}

				// QUATERNION ORIENTATION STANDARD DEVIATION PACKET 
				if (an_packet->id == packet_id_quaternion_orientation_standard_deviation)
				{
					// copy all the binary data into the typedef struct for the packet 
					// this allows easy access to all the different values             
					if(decode_quaternion_orientation_standard_deviation_packet(&quaternion_orientation_standard_deviation_packet, an_packet) == 0)
					{
						// IMU
						imu_msg.orientation_covariance[0] = quaternion_orientation_standard_deviation_packet.standard_deviation[0];
						imu_msg.orientation_covariance[4] = quaternion_orientation_standard_deviation_packet.standard_deviation[1];
						imu_msg.orientation_covariance[8] = quaternion_orientation_standard_deviation_packet.standard_deviation[2];
					}
				}

				// Setting up the magnetic field to display
				if(an_packet->id == packet_id_raw_sensors){
					if(decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0){
						// Time Stamp from the System State Packet
						magnetic_field_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
						barometric_pressure_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
						temperature_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
						magnetic_field_msg.header.stamp.nanosec = system_state_packet.microseconds*1000;
						barometric_pressure_msg.header.stamp.nanosec = system_state_packet.microseconds*1000;
						temperature_msg.header.stamp.nanosec = system_state_packet.microseconds*1000;
					
						// RAW MAGNETICFIELD VALUE FROM IMU
						magnetic_field_msg.header.frame_id = rawsensors_magnetometer_frame_id;
						magnetic_field_msg.magnetic_field.x = raw_sensors_packet.magnetometers[0];
						magnetic_field_msg.magnetic_field.y = raw_sensors_packet.magnetometers[1];
						magnetic_field_msg.magnetic_field.z = raw_sensors_packet.magnetometers[2];
						imu_msg.linear_acceleration.x = raw_sensors_packet.accelerometers[0];
						imu_msg.linear_acceleration.y = raw_sensors_packet.accelerometers[1];
						imu_msg.linear_acceleration.z = raw_sensors_packet.accelerometers[2];

						// BAROMETRIC PRESSURE
						barometric_pressure_msg.header.frame_id = barometric_pressure_frame_id;
						barometric_pressure_msg.fluid_pressure = raw_sensors_packet.pressure;

						// TEMPERATURE
						temperature_msg.header.frame_id = rawsensors_magnetometer_frame_id;
						temperature_msg.temperature = raw_sensors_packet.pressure_temperature;
					}
				}

				// Ensure that you free the an_packet when your done with it or you will leak memory                                  
				an_packet_free(&an_packet);

				// PUBLISH MESSAGES
				nav_sat_fix_pub->publish(nav_sat_fix_msg);
				twist_pub->publish(twist_msg);
				imu_pub->publish(imu_msg);
				system_status_pub->publish(system_status_msg);
				filter_status_pub->publish(filter_status_msg);
				magnetic_field_pub->publish(magnetic_field_msg);
				barometric_pressure_pub->publish(barometric_pressure_msg);
				temperature_pub->publish(temperature_msg);
				pose_pub->publish(pose_msg);
				gnss_fix_type_pub->publish(gnss_fix_type_msgs);
			}
			
			// Write the logs to the logger reset when counter is full
			if(write_counter++ >= 100){
				fflush(log_file);
				write_counter = 0;
			}
		}
		
		if(state == 0){
			error = ntrip(&args, buf, &numbytes);
			remain = numbytes;
			
			// Send Buffer in 255 Byte chunks to the Spatial 
			// Loop till the entire rtcm corrections message is encoded. 
			while(remain)
			{	
				int toCpy = remain > AN_MAXIMUM_PACKET_SIZE ? AN_MAXIMUM_PACKET_SIZE : remain;
				an_packet = encode_rtcm_corrections_packet(toCpy, buf+pos);				
				an_packet_encode(an_packet);
				SendBuf(an_packet_pointer(an_packet), an_packet_size(an_packet));
				an_packet_free(&an_packet);
				pos += toCpy;			

				// Increment buffer
				remain -= toCpy;
			}			
			pos=0;
		}	
		
		rclcpp::spin_some(node);
		//loop_rate.sleep();
	} 
  return 0;
}
