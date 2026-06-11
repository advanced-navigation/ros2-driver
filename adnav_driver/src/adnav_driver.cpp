/****************************************************************/
/*                                                              */
/*                       Advanced Navigation                    */
/*         					  ROS2 Driver			  			*/
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

#include "adnav_driver.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace adnav {


/**
 * @brief Constructor for the Advanced Navigation Driver node
 *
 * This will create a Driver instance. Declare node parameters, setup threading groups and callbacks.
 * Initialize the log file, and open the communications to the device.
 */
Driver::Driver(): rclcpp::Node("adnav_driver")
{
	// ~~~~~~~~~~ Create the callback groups
	// Multithreaded group for publishing ROS messages
	publishing_group_	= this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

	// Group for completing incoming services.
	service_group_		= this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(this);

	// Setup parameters for the node
	param_listener_ = std::make_shared<adnav::ParamListener>(this);
	setupParamService();

	// Create timers for callbacks
	const auto params = param_listener_->get_params();
	// Reset the timer using the class stored interval.
	publish_timer_ = this->create_wall_timer(
		std::chrono::microseconds(params.publish_us), std::bind(&Driver::publishTimerCallback, this),
		publishing_group_);

	createServices();

	// Open a new ANPP Log file for data logging
	anpp_logger_.openFile("Log_", ".anpp", params.log_path);

	createPublishers();

	system_state_packet_.set_lifespan(std::chrono::microseconds(params.publish_us * 3));
	raw_sensors_packet_.set_lifespan(std::chrono::microseconds(params.publish_us * 3));
	euler_orientation_standard_deviation_packet_.set_lifespan(std::chrono::microseconds(params.publish_us * 3));
	velocity_standard_deviation_packet_.set_lifespan(std::chrono::microseconds(params.publish_us * 3));

	connection_thread_ = std::jthread([&](std::stop_token st){
		auto loop = uvw::loop::get_default();

		while (!st.stop_requested()) {
			const auto _params = param_listener_->get_params();
			auto resolver = loop->resource<uvw::get_addr_info_req>();

			uvw::socket_address dest;

			std::shared_ptr<uvw::tcp_handle> tcp_handle;
			std::shared_ptr<uvw::udp_handle> udp_handle;

			if (_params.connection_type == "tcp-client")
			{
				tcp_handle = loop->resource<uvw::tcp_handle>();
			} else if (_params.connection_type == "udp")
			{
				udp_handle = loop->resource<uvw::udp_handle>();
			}

			close_async_ = loop->resource<uvw::async_handle>();
			close_async_->on<uvw::async_event>([&](const uvw::async_event &, uvw::async_handle &async) {
				// This fires on the loop thread — safe to close handles here
				if (tcp_handle) {
					tcp_handle->stop();
				}
				if (udp_handle) {
					udp_handle->stop();
				}
				if (write_async_) {
					write_async_->close();
				}
				async.close();  // close the async handle itself so the loop can exit
			});

			write_async_ = loop->resource<uvw::async_handle>();
			write_async_->on<uvw::async_event>([&](const uvw::async_event &, uvw::async_handle &) {
				std::unique_lock lock(write_q_mutex_);
				while (!write_q_.empty()) {
					auto [data, size] = std::move(write_q_.front());
					write_q_.pop();
					lock.unlock();
					if (tcp_handle)
					{
						tcp_handle->write(std::move(data), size);
					}
					if (udp_handle)
					{
						udp_handle->send(dest, std::move(data), size);
					}
					lock.lock();
				}
			});

			if (tcp_handle)
			{
				tcp_handle->on<uvw::error_event>([&](const uvw::error_event &err, uvw::tcp_handle &)
				{
					RCLCPP_ERROR(this->get_logger(), "TCP: Connection Error: %s - stopping", err.what());
					failAllPendingReplies();
					close_async_->send();
				});

				tcp_handle->on<uvw::end_event>([&](const uvw::end_event &, uvw::tcp_handle &) {
					RCLCPP_INFO(this->get_logger(), "TCP: End Event");
					failAllPendingReplies();
					close_async_->send();
				});

				tcp_handle->on<uvw::connect_event>([&](const uvw::connect_event &, uvw::tcp_handle &tcp) {
					RCLCPP_INFO(this->get_logger(), "TCP: Connection established");
					// tcp.keep_alive(true, uvw::tcp_handle::time{2});

#ifdef GR_PIXI_BUILD
					// pixi/conda libuv exposes the 3-arg keepalive; the extended
					// (count/interval) form may be unavailable on the vendored libuv.
					if (const auto err = uv_tcp_keepalive(tcp.raw(), 1, 1); err != 0) {
#else
					// deb path — unchanged from main.
					if (const auto err = uv_tcp_keepalive_ex(tcp.raw(), 1, 1, 1, 2); err != 0) {
#endif
						RCLCPP_ERROR(this->get_logger(), "Failed to set TCP keep-alive: %s", uv_strerror(err));
						throw std::runtime_error("Failed to set TCP keep-alive");
					}
					tcp.read();

					// Run setup in a separate thread so it can block on request-reply futures
					// without stalling the event loop. join() is fast — failAllPendingReplies() will
					// have unblocked any in-flight future before we reach here on reconnect.
					if (setup_thread_.joinable()) setup_thread_.join();
					setup_thread_ = std::jthread([this] {
						requestDeviceInfo();
						deviceSetup();
					});
				});

				tcp_handle->on<uvw::data_event>([&](const uvw::data_event &event, uvw::tcp_handle &) {
					std::span<const uint8_t> buffer(
						reinterpret_cast<const uint8_t*>(event.data.get()), event.length);
					RCLCPP_DEBUG(this->get_logger(), "Received %zu bytes", buffer.size());
					receivePackets(buffer);
				});

				auto resolved_addr = resolver->node_addr_info_sync(_params.ip_address.c_str());
				if (!resolved_addr.first)
				{
					RCLCPP_ERROR(this->get_logger(), "Could not resolve hostname: %s.", _params.ip_address.c_str());
					close_async_->send();
				} else
				{
					dest = uvw::details::sock_addr(*resolved_addr.second->ai_addr);
					dest.port = static_cast<unsigned int>(_params.port);
					RCLCPP_INFO(this->get_logger(), "TCP: Attempting to connect to %s:%u", dest.ip.c_str(), dest.port);
					tcp_handle->connect(dest);
				}
			}

			if (udp_handle)
			{
				udp_handle->on<uvw::error_event>([&](const uvw::error_event &err, uvw::udp_handle &)
				{
					RCLCPP_ERROR(this->get_logger(), "UDP: Connection Error: %s - stopping", err.what());
					failAllPendingReplies();
					close_async_->send();
				});

				udp_handle->on<uvw::udp_data_event>([&](const uvw::udp_data_event &event, uvw::udp_handle &) {
					std::span<const uint8_t> buffer(
						reinterpret_cast<const uint8_t*>(event.data.get()), event.length);
					RCLCPP_DEBUG(this->get_logger(), "Received %zu bytes", buffer.size());
					receivePackets(buffer);
				});

				auto resolved_addr = resolver->node_addr_info_sync(_params.ip_address.c_str());
				if (!resolved_addr.first)
				{
					RCLCPP_ERROR(this->get_logger(), "Could not resolve hostname: %s.", _params.ip_address.c_str());
					close_async_->send();
				} else
				{
					dest = uvw::details::sock_addr(*resolved_addr.second->ai_addr);
					dest.port = static_cast<unsigned int>(_params.port);

					// determine the correct local address to listen on
					udp_handle->connect(dest);
					auto local_addr = udp_handle->sock();
					local_addr.port = static_cast<unsigned int>(_params.port);
					udp_handle->disconnect();

					udp_handle->bind(local_addr);
					udp_handle->recv();

					RCLCPP_INFO(this->get_logger(), "UDP: Created %s:%u --> endpoint %s:%u", local_addr.ip.c_str(), local_addr.port,
						dest.ip.c_str(), dest.port);

					// Run setup in a separate thread so it can block on request-reply futures
					// without stalling the event loop. join() is fast — failAllPendingReplies() will
					// have unblocked any in-flight future before we reach here on reconnect.
					if (setup_thread_.joinable()) setup_thread_.join();
					setup_thread_ = std::jthread([this] {
						if (!requestDeviceInfo())
						{
							RCLCPP_ERROR(this->get_logger(), "Did not get a Device Information reply - stopping");
							close_async_->send();
							return;
						}
						deviceSetup();
					});
				}
			}

			loop->run();
			RCLCPP_DEBUG(this->get_logger(), "Event loop ended");

			if (tcp_handle)
			{
				tcp_handle->close();
			}

			if (udp_handle)
			{
				udp_handle->close();
			}

			if (!st.stop_requested() && rclcpp::ok())
			{
				RCLCPP_ERROR(this->get_logger(), "Attempting reconnect in 2 seconds");
				std::condition_variable_any cv;
				std::mutex mtx;
				std::stop_callback cb(st, [&]
				{
					cv.notify_all();
				});
				std::unique_lock lock(mtx);
				cv.wait_for(lock, st, std::chrono::seconds(2), [&st]{ return st.stop_requested(); });
			}
		}
	});

	diagnostic_updater_->add("System Status", this, &Driver::system_status_diagnostic);
	diagnostic_updater_->add("Filter Status", this, &Driver::filter_status_diagnostic);
	diagnostic_updater_->add("Accuracy", this, &Driver::accuracy_diagnostic);
	diagnostic_updater_->add("Packets", this, &Driver::packets_diagnostic);
}

/**
 * @brief Deconstructor for the Node
 *
 * Closes the Logfiles and gives name to the user. Shutsdown the communication method
 */
Driver::~Driver() {
	RCLCPP_DEBUG(this->get_logger(), "Deconstructing Adnav_Driver node");

	anpp_logger_.closeFile();

	// if the Ntrip client is initialised and running or the logger is open.
	if (ntrip_client_ && (ntrip_client_->service_running() || rtcm_logger_.is_open())) {
		ntrip_client_->stop();

		// Close the logfile
		rtcm_logger_.closeFile();
	}

	connection_thread_.request_stop();
	if (close_async_) {
		close_async_->send();  // ← thread-safe wakeup
	}
	connection_thread_.join();
}

rclcpp::Time time_from_seconds_microseconds(uint32_t seconds, uint32_t microseconds) {
	auto nanosecs = microseconds * 1000;

	// Handle possible rollover of nanoseconds
	if (nanosecs >= 1000000000) {
		seconds += nanosecs / 1000000000;
		nanosecs = nanosecs % 1000000000;
	}

	return {static_cast<int32_t>(seconds), nanosecs, RCL_ROS_TIME};
}

rclcpp::Duration duration_from_seconds_microseconds(uint32_t seconds, uint32_t microseconds) {
	auto nanosecs = microseconds * 1000;

	// Handle possible rollover of nanoseconds
	if (nanosecs >= 1000000000) {
		seconds += nanosecs / 1000000000;
		nanosecs = nanosecs % 1000000000;
	}

	return {static_cast<int32_t>(seconds), nanosecs};
}

/**
 * @brief Function to request a Advanced Navigation Devices info using ANPP.
 */
bool Driver::requestDeviceInfo() {
	RCLCPP_DEBUG(this->get_logger(), "Requesting Device Info");
	auto promise = std::make_shared<std::promise<device_information_packet_t>>();
	auto future = promise->get_future();

	{
		std::lock_guard lock(pending_replies_mutex_);
		pending_replies_[packet_id_device_information] = {
			[promise](an_packet_t* pkt) {
				device_information_packet_t decoded_pkt{};
				if (decode_device_information_packet(&decoded_pkt, pkt)) {
					promise->set_exception(std::make_exception_ptr(
						std::runtime_error("Decode error")));
					return;
				}
				try { promise->set_value(decoded_pkt); } catch (const std::future_error&) {}
			},
			[promise](std::exception_ptr ep) {
				try { promise->set_exception(ep); } catch (const std::future_error&) {}
			}
		};
	}

	an_packet_t* an_packet = encode_request_packet(packet_id_device_information);
	encodeAndSend(an_packet);

	if (future.wait_for(std::chrono::seconds(DEFAULT_TIMEOUT)) != std::future_status::ready) {
		RCLCPP_ERROR(this->get_logger(), "Reply timeout for Device Information");
		std::lock_guard lock(pending_replies_mutex_);
		pending_replies_.erase(packet_id_device_information);
		return false;
	}

	try {
		[[maybe_unused]] const auto pkt = future.get();
		return true;
	} catch (const std::exception& e) {
		// Promise was broken by failAllPendingReplies() — connection dropped mid-wait
		RCLCPP_ERROR(this->get_logger(), "ACK aborted for Device Information: %s", e.what());
		return false;
	}
}

/**
 * @brief Function to declare topics and publishers for the adnav_driver node.
 */
void Driver::createPublishers() {
	const auto params = param_listener_->get_params();

	// Following packets are derived from system state
	nav_sat_fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("nav_sat_fix", 10);
	twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("twist", 10);
	twist_stamped_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist_stamped", 10);
	twist_stamped_enu_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist_stamped_enu", 10);
	geo_pose_pub_ = this->create_publisher<geographic_msgs::msg::GeoPose>("geopose", 10);
	geo_pose_stamped_pub_ = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>("geopose_stamped", 10);
	imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

	if (params.additional_packets.raw_sensors)
	{
		imu_raw_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_raw", 10);
		magnetic_field_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("magnetic_field", 10);
		barometric_pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("barometric_pressure", 10);
		temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);
	}

	if (params.additional_packets.body_velocity)
	{
		twist_stamped_body_velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist_stamped_body", 10);
	}

	if (params.additional_packets.external_body_velocity)
	{
		twist_stamped_external_body_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist_stamped_external_body", 10);
	}

	if (params.additional_packets.ecef_position)
	{
		pose_ecef_stamped_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_stamped", 10);
	}

	if (params.additional_packets.utm_position)
	{
		pose_utm_stamped_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_utm", 10);
	}
}

/**
 * @brief Function to declare and assign callbacks to services for the adnav_driver node.
 */
void Driver::createServices() {
	packet_period_timer_srv_ = this->create_service<adnav_interfaces::srv::PacketTimerPeriod>(
		"~/packet_timer_period",
		std::bind(
			&Driver::srvPacketTimerPeriod,
			this,
			std::placeholders::_1,
			std::placeholders::_2),
		rclcpp::ServicesQoS(),
		service_group_);

	ntrip_srv_ = this->create_service<adnav_interfaces::srv::Ntrip>(
		"~/ntrip",
		std::bind(
			&Driver::srvNtrip,
			this,
			std::placeholders::_1,
			std::placeholders::_2),
		rclcpp::ServicesQoS(),
		service_group_);
}

void Driver::deviceSetup() {
	RCLCPP_INFO(this->get_logger(), "Sending requested configuration to device:");

	const auto params = param_listener_->get_params();
	SendPacketTimer(params.packet_timer_period);
	SendPacketPeriods(getPacketRequest(), params.override_existing_packets);
}

/**
 * @brief Method to setup node parameters, callbacks and event handlers
 *
 * This method will initialize the event handlers and callbacks for parameter updates,
 * as well as declaring, describing, and assigning values to node parameters.
 * Will assign a default value if one is not provided already.
 */
void Driver::setupParamService() {
	// Register the callback for parameter changes.
	// Will be called whenever a parameter is update is requested.
	param_set_cb_ = this->add_on_set_parameters_callback(
            std::bind(&Driver::ParamSetCallback,
					this,
					std::placeholders::_1));
	// Set Parameter Event Handler will call attached callbacks when verification is completed.
	param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

	// publish period updater callback
	publish_us_cb_ = param_handler_->add_parameter_callback("publish_us",
			std::bind(&Driver::updatePublishUs,
					this,
					std::placeholders::_1));

	// packet request updater callback
	packet_request_cb_ = param_handler_->add_parameter_callback("packet_request",
			std::bind(&Driver::updatePacketRequest,
					this,
					std::placeholders::_1));

	// packet timer updater callback
	packet_timer_cb_ = param_handler_->add_parameter_callback("packet_timer_period",
			std::bind(&Driver::updatePacketTimer,
					this,
					std::placeholders::_1));
}

//~~~~~~ Control Functions

/**
 * @brief Function to receive ANPP Packets from the COMPORT as specified in user input.
 *
 * Uses the ANPP SDK to decode the packets and call the relevant ROS decoder.
 *
 * This Function will also create a log file session and log incoming data to it.
 */
void Driver::receivePackets(const std::span<const uint8_t> buffer) {
	an_decoder_t an_decoder;
	an_decoder_initialise(&an_decoder);

	// Copy incoming data into the decoder buffer
	const auto buffer_bytes = std::min(buffer.size(), an_decoder_size(&an_decoder));
	std::memcpy(an_decoder_pointer(&an_decoder), buffer.data(), buffer_bytes);

	anpp_logger_.writeAndIncrement(reinterpret_cast<char*>(an_decoder_pointer(&an_decoder)), buffer_bytes);
	an_decoder_increment(&an_decoder, buffer_bytes);
	decodePackets(an_decoder, buffer_bytes);
}

	bool Driver::packet_is_recent(uint8_t packet_id, std::chrono::microseconds max_age) {
		auto now = this->get_clock()->now();
		auto pkt_it = packet_receive_times.find(packet_id);
		if (pkt_it == packet_receive_times.end())
		{
			return false;
		}
		auto packet_time = pkt_it->second;

		return now - packet_time <= rclcpp::Duration(max_age);
	}

/**
 * @brief Periodic callback for publishing ROS messages.
 *
 * This function accesses in a thread safe manner the class stored ROS messages, and publishes them to their respective topics.
 * Awaits a signal from a ROS Decoder method before publishing and accessing shared data.
 */
void Driver::publishTimerCallback() {
	if (!write_async_ || !write_async_->active())
	{
		return;
	}

	// Since the messages can be filled in other threads ensure exclusive access.
	std::unique_lock<std::mutex> lock(messages_mutex_);
	const auto params = param_listener_->get_params();

	if (packet_is_recent(packet_id_system_state, std::chrono::microseconds(params.publish_us)))
	{
		nav_sat_fix_pub_->publish(nav_fix_msg_);
		twist_pub_->publish(twist_flu_msg_);
		twist_stamped_enu_pub_->publish(twist_stamped_msg_enu);
		twist_stamped_pub_->publish(twist_stamped_msg_);
		imu_pub_->publish(imu_msg_);
		geo_pose_pub_->publish(geo_pose_msg_);
		geo_pose_stamped_pub_->publish(geo_pose_stamped_msg_);
	}

	if (packet_is_recent(packet_id_body_velocity, std::chrono::microseconds(params.publish_us)) && twist_stamped_body_velocity_pub_)
	{
		twist_stamped_body_velocity_pub_->publish(twist_stamped_msg_body);
	}

	if (packet_is_recent(packet_id_external_body_velocity, std::chrono::microseconds(params.publish_us)) && twist_stamped_external_body_pub_)
	{
		twist_stamped_external_body_pub_->publish(twist_stamped_msg_external_body);
	}

	if (packet_is_recent(packet_id_raw_sensors, std::chrono::microseconds(params.publish_us)) && imu_raw_pub_ && magnetic_field_pub_ && barometric_pressure_pub_ && temperature_pub_)
	{
		imu_raw_pub_->publish(imu_raw_msg_);
		magnetic_field_pub_->publish(mag_field_msg_);
		barometric_pressure_pub_->publish(baro_msg_);
		temperature_pub_->publish(temp_msg_);
	}

	if (packet_is_recent(packet_id_ecef_position, std::chrono::microseconds(params.publish_us)) && pose_ecef_stamped_pub_)
	{
		pose_ecef_stamped_pub_->publish(pose_ecef_stamped_msg_);
	}

	if (packet_is_recent(packet_id_utm_position, std::chrono::microseconds(params.publish_us)) && pose_utm_stamped_pub_ && pose_utm_stamped_pub_)
	{
		pose_utm_stamped_pub_->publish(pose_utm_stamped_msg_);
	}
}

/**
 * @brief Function to cancel and restart the Publisher timer.
 */
void Driver::RestartPublisher() {
	RCLCPP_INFO(this->get_logger(), "Restart Publisher Timer");

	// Cancel old timer to stop callbacks from triggering while remaking timer.
	publish_timer_->cancel();

	const auto params = param_listener_->get_params();
	// Reset the timer using the class stored interval.
	publish_timer_ = this->create_wall_timer(
      	std::chrono::microseconds(params.publish_us), std::bind(&Driver::publishTimerCallback, this), publishing_group_);
}

//~~~~~~ Diagnostic Functions

void Driver::system_status_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	const auto sys_state_pkt = system_state_packet_.value();
	if (sys_state_pkt)
	{
		stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;

		if (sys_state_pkt->system_status.b.system_failure) {
			stat.add("System", "Fail");
			stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
		}
		if (sys_state_pkt->system_status.b.accelerometer_sensor_failure) {
			stat.add("Accelerometer", "Fail");
			stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
		}
		if (sys_state_pkt->system_status.b.gyroscope_sensor_failure) {
			stat.add("Gyroscope", "Fail");
			stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
		}
		if (sys_state_pkt->system_status.b.magnetometer_sensor_failure) {
			stat.add("Magnetometer", "Fail");
			stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
		}
		if (sys_state_pkt->system_status.b.pressure_sensor_failure) {
			stat.add("Pressure Sensor", "Fail");
			stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
		}
		if (sys_state_pkt->system_status.b.gnss_failure) {
			stat.add("GNSS", "Fail");
			stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
		}
		if (sys_state_pkt->system_status.b.minimum_temperature_alarm) {
			stat.add("Temperature Alarm", "Minimum Temperature Alarm");
			stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
		}
		if (sys_state_pkt->system_status.b.maximum_temperature_alarm) {
			stat.add("Temperature Alarm", "Maximum Temperature Alarm");
			stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
		}
		if (sys_state_pkt->system_status.b.internal_data_logging_error) {
			stat.add("Data Logging", "Internal Error");
			stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
		}
		if (sys_state_pkt->system_status.b.high_voltage_alarm) {
			stat.add("Power Supply", "High Voltage Alarm");
			stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
		}
		if (sys_state_pkt->system_status.b.gnss_antenna_fault) {
			stat.add("GNSS Antenna", "Fault Detected");
			stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
		}
		if (sys_state_pkt->system_status.b.serial_port_overflow_alarm) {
			stat.add("Serial Port", "Data Overflow");
			stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
		}

		if (stat.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
			stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
		} else
		{
			stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "System Failure Detected");
		}
	} else {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::format("System State packet: {}", to_string(sys_state_pkt.error())));
	}
}

void Driver::filter_status_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	const auto sys_state_pkt = system_state_packet_.value();
	if (sys_state_pkt) {
		stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;

		stat.add("Orientation Filter Initialised", sys_state_pkt->filter_status.b.orientation_filter_initialised ? "Yes" : "No");
		stat.add("Navigation Filter Initialised", sys_state_pkt->filter_status.b.ins_filter_initialised ? "Yes" : "No");
		stat.add("Heading Initialised", sys_state_pkt->filter_status.b.heading_initialised ? "Yes" : "No");
		stat.add("UTC Time Initialised", sys_state_pkt->filter_status.b.utc_time_initialised ? "Yes" : "No");
		stat.add("Internal GNSS Enabled", sys_state_pkt->filter_status.b.internal_gnss_enabled ? "Yes" : "No");
		stat.add("Dual Antenna Heading Active", sys_state_pkt->filter_status.b.dual_antenna_heading_active ? "Yes" : "No");
		stat.add("Velocity Heading Enabled", sys_state_pkt->filter_status.b.velocity_heading_enabled ? "Yes" : "No");
		stat.add("Atmospheric Altitude Enabled", sys_state_pkt->filter_status.b.atmospheric_altitude_enabled ? "Yes" : "No");
		stat.add("External Position Active", sys_state_pkt->filter_status.b.external_position_active ? "Yes" : "No");
		stat.add("External Velocity Active", sys_state_pkt->filter_status.b.external_velocity_active ? "Yes" : "No");
		stat.add("External Heading Active", sys_state_pkt->filter_status.b.external_heading_active ? "Yes" : "No");

		const auto gnss_fix = static_cast<GnssFixStatus>(sys_state_pkt->filter_status.b.gnss_fix_type);
		stat.add("GNSS Fix Status", to_string(gnss_fix));

		// TODO parameters need to be added to inform what is considered abnormal operation

		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
	} else {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::format("System State packet: {}", to_string(sys_state_pkt.error())));
	}
}

	void Driver::packets_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;

	auto packets = getPacketRequest();

	for (const auto& packet: packets)
	{
		const auto packet_name = packet_id_to_string(static_cast<packet_id_e>(packet.packet_id)).value_or("Packet ID " + std::to_string(packet.packet_id));

		if (packet_receive_times.contains(packet.packet_id))
		{
			auto last_rx_time = packet_receive_times[packet.packet_id];
			auto time_since_last = this->get_clock()->now() - last_rx_time;
			stat.add(std::format("{} ({}) - Last Seen (ms)", packet_name, packet.packet_id), std::to_string(time_since_last.nanoseconds() / 1e6));

			if (time_since_last > rclcpp::Duration(1, 0))
			{
				stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::format("{} stale", packet_name).c_str());
			}
		} else
		{
			stat.add(std::format("{} ({}) - Last Seen (ms)", packet_name, packet.packet_id), "Never");
			stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::format("{} packet never received", packet_name).c_str());
		}

		if (stat.level == diagnostic_msgs::msg::DiagnosticStatus::OK)
		{
			stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
		}
	}
}

/**
 * Calculate positional accuracy from the standard deviation values in the system state packet.
 * @param state_packet
 * @return double, double: 2D RMS, 3D RMS
 */
std::tuple<double, double> position_accuracy_rms(const system_state_packet_t& state_packet) {
	const auto [x, y, z] = state_packet.standard_deviation;
	return {
		std::sqrt(std::pow(x, 2) + std::pow(y, 2)),
		std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2))
	};
}

double orientation_accuracy_stdev(const euler_orientation_standard_deviation_packet_t& orientation_packet) {
	const auto [roll_stdev, pitch_stdev, yaw_stdev] = orientation_packet.standard_deviation;
	return std::max({roll_stdev, pitch_stdev, yaw_stdev});
}

/**
 * Calculate velocity accuracy from the standard deviation values
 * @param velocity_standard_deviation
 * @return double, double: 2D RMS, 3D RMS
 */
	std::tuple<double, double> velocity_accuracy_rms(const velocity_standard_deviation_packet_t& velocity_standard_deviation) {
	const auto [x, y, z] = velocity_standard_deviation.standard_deviation;
	return {
		std::sqrt(std::pow(x, 2) + std::pow(y, 2)),
		std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2))
	};
}

void Driver::accuracy_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	const auto sys_state_pkt = system_state_packet_.value();
	if (sys_state_pkt)
	{
		const auto params = param_listener_->get_params();

		stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;

		stat.add("Accelerometer In Range", sys_state_pkt->system_status.b.accelerometer_over_range ? "Fail" : "OK");
		stat.add("Gyroscope In Range", sys_state_pkt->system_status.b.gyroscope_over_range ? "Fail" : "OK");
		stat.add("Magnetometer In Range", sys_state_pkt->system_status.b.magnetometer_over_range ? "Fail" : "OK");
		stat.add("Pressure Sensor In Range", sys_state_pkt->system_status.b.pressure_over_range ? "Fail" : "OK");

		if (params.health.sensors_in_range)
		{
			if (sys_state_pkt->system_status.b.accelerometer_over_range) {
				stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Accelerometer Over Range");
			}
			if (sys_state_pkt->system_status.b.gyroscope_over_range) {
				stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Gyroscope Over Range");
			}
			if (sys_state_pkt->system_status.b.magnetometer_over_range) {
				stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Magnetometer Over Range");
			}
			if (sys_state_pkt->system_status.b.pressure_over_range) {
				stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Pressure Sensor Over Range");
			}
		}

		if (params.health.filters_initialised)
		{
			if (!sys_state_pkt->filter_status.b.orientation_filter_initialised) {
				stat.add("Orientation Filter", "Not Initialised");
				stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Orientation Filter not initialised");
			}

			if (!sys_state_pkt->filter_status.b.ins_filter_initialised)
			{
				stat.add("Navigation Filter", "Not Initialised");
				stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Navigation Filter not initialised");
			}

			if (!sys_state_pkt->filter_status.b.heading_initialised)
			{
				stat.add("Heading", "Not Initialised");
				stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Heading not initialised");
			}
		}

		const auto gnss_fix = static_cast<GnssFixStatus>(sys_state_pkt->filter_status.b.gnss_fix_type);
		stat.add("GNSS Fix Status", to_string(gnss_fix));

		if (params.health.gnss_fix && (gnss_fix == GnssFixStatus::NoFix || gnss_fix == GnssFixStatus::Fix2D))
		{
			stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No precise GNSS Fix");
		}

		if (params.health.external_velocity)
		{
			stat.add("External Velocity Active", sys_state_pkt->filter_status.b.external_velocity_active ? "Yes" : "No");
			if (!sys_state_pkt->filter_status.b.external_velocity_active)
			{
				stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No External Velocity");
			}
		}

		auto [rms_2d, rms_3d] = position_accuracy_rms(*sys_state_pkt);
		stat.add("Position Accuracy (2D RMS) (m)", std::to_string(rms_2d));
		stat.add("Position Accuracy (3D RMS) (m)", std::to_string(rms_3d));

		if (params.health.thresholds.position_accuracy_rms_2d != 0.0 && rms_2d > params.health.thresholds.position_accuracy_rms_2d)
		{
			stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "2D Position RMS above threshold");
		}
		if (params.health.thresholds.position_accuracy_rms_3d != 0.0 && rms_3d > params.health.thresholds.position_accuracy_rms_3d)
		{
			stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "3D Position RMS above threshold");
		}

		const auto euler_stddev_pkt = euler_orientation_standard_deviation_packet_.value();
		if (euler_stddev_pkt) {
			const auto orientation_deviation = orientation_accuracy_stdev(*euler_stddev_pkt);
			stat.add("Orientation Deviation (max of roll, pitch, yaw) (rads)", std::to_string(orientation_deviation));
			if (params.health.thresholds.orientation_accuracy_stdev != 0.0 && orientation_deviation > params.health.thresholds.orientation_accuracy_stdev)
			{
				stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Orientation stdev above threshold");
			}
		} else if (params.health.thresholds.orientation_accuracy_stdev != 0.0)
		{
			stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::format("Euler Orientation Stddev packet: {}", to_string(euler_stddev_pkt.error())));
		}

		const auto velocity_stddev_pkt = velocity_standard_deviation_packet_.value();
		if (velocity_stddev_pkt)
		{
			auto [vel_rms_2d, vel_rms_3d] = velocity_accuracy_rms(*velocity_stddev_pkt);
			stat.add("Velocity Accuracy (2D RMS) (m)", std::to_string(vel_rms_2d));
			stat.add("Velocity Accuracy (3D RMS) (m)", std::to_string(vel_rms_3d));

			if (params.health.thresholds.velocity_accuracy_rms_2d != 0.0 && vel_rms_2d > params.health.thresholds.velocity_accuracy_rms_2d)
			{
				stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "2D Velocity RMS above threshold");
			}

			if (params.health.thresholds.velocity_accuracy_rms_3d != 0.0 && vel_rms_3d > params.health.thresholds.velocity_accuracy_rms_3d)
			{
				stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "3D Velocity RMS above threshold");
			}
		} else if (params.health.thresholds.velocity_accuracy_rms_2d != 0.0 || params.health.thresholds.velocity_accuracy_rms_3d != 0.0)
		{
			stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::format("Velocity Standard Deviation packet: {}", to_string(velocity_stddev_pkt.error())));
		}

		const auto pvt_pkt = gnss_position_velocity_time_packet_.value();
		if (pvt_pkt)
		{
			const auto spoofing_status = static_cast<spoofing_interference_status_e>(pvt_pkt.value().status_bitfield.b.spoofing_status);
			const auto interference_status = static_cast<spoofing_interference_status_e>(pvt_pkt.value().status_bitfield.b.interference_status);
			stat.add("Spoofing Status", to_string(spoofing_status));
			stat.add("Interference Status", to_string(interference_status));

			if (spoofing_status != spoofing_interference_status_unknown)
			{
				if (params.health.spoofing && spoofing_status != spoofing_interference_status_none)
				{
					stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Spoofing detected");
				}
			}

			if (interference_status != spoofing_interference_status_unknown)
			{
				if (params.health.spoofing && interference_status != spoofing_interference_status_none)
				{
					stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Interference detected");
				}
			}
		} else
		{
			stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::format("GNSS Position Velocity Time packet: {}", to_string(pvt_pkt.error())));
		}

		if (stat.level == diagnostic_msgs::msg::DiagnosticStatus::OK)
		{
			stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
		}
	} else {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "System State packet not received");
	}
}

//~~~~~~ Service Functions

/**
 * @brief Service to request a change of Packet Timer Period from the device.
 *
 * @param request Const shared pointer to a PacketTimerPeriod Request msg
 * @param response Shared pointer to a PacketTimerPeriod Response.
 */
void Driver::srvPacketTimerPeriod(const std::shared_ptr<adnav_interfaces::srv::PacketTimerPeriod::Request> request,
		std::shared_ptr<adnav_interfaces::srv::PacketTimerPeriod::Response> response) {

	response->acknowledgement = SendPacketTimer(request->packet_timer_period, request->utc_synchronisation, request->permanent);

	char buf[15];
	snprintf(buf, sizeof(buf)/sizeof(buf[0]), "%s%d%s", "Failure: ", response->acknowledgement.result, "\n");
	RCLCPP_INFO(this->get_logger(), "Received Update acknowledgement:\nID: %d\tCRC: %d\nOutcome: %s", response->acknowledgement.id,
            response->acknowledgement.crc, (response->acknowledgement.result == 0) ? "Success\n":buf);
}

void Driver::srvNtrip(const std::shared_ptr<adnav_interfaces::srv::Ntrip::Request> request,
		std::shared_ptr<adnav_interfaces::srv::Ntrip::Response> response) {
	std::stringstream dbg_ss;
	dbg_ss << "NTRIP Service Called:\n\tEnable: " << request->enable << "\n\tUsername: " << request->username <<
		"\n\tPassword: " << request->password << "\n\tHost: " << request->host << "\n\tMountpoint: " << request->mountpoint << "\n";
	RCLCPP_DEBUG(this->get_logger(), "%s", dbg_ss.str().c_str());

	// Reset the state
	ntrip_state_ = {};

	// Fill out data from request
	ntrip_state_.en = request->enable;

	if (ntrip_state_.en) {
		ntrip_state_.username = request->username;
		ntrip_state_.password = request->password;
		ntrip_state_.mountpoint = request->mountpoint;
		// Decode the host str into IP and Port.
		getDataFromHostStr(request->host);

	} else {
		if(ntrip_client_.get() != nullptr && ntrip_client_->service_running()) ntrip_client_->stop();
		if(rtcm_logger_.is_open()) rtcm_logger_.closeFile();
		response->reason = std::string("Successfully disabled NTRIP Client.");
		response->success = true;
		return;
	}

	// Update the client
	updateNTRIPClientService();

	// If no mountpoint has been specified print out the sourcetable.
	if (ntrip_state_.mountpoint.empty()) {
		if (ntrip_client_->retrieve_sourcetable()) {
			response->success = true;
			response->reason = std::string("Successfully Requested Sourcetable");
		} else {
			response->success = false;
			switch (ntrip_client_->service_failure())
			 {
			case adnav::ntrip::NTRIP_CONNECTION_TIMEOUT_FAILURE:
				response->reason = std::string("NTRIP Response Timeout.");
				break;
			case adnav::ntrip::NTRIP_UNRECOGNIZED_RETURN:
				response->reason = std::string("Unrecognized Sourcetable Header");
				break;
			default:
				response->reason = std::string("Sourcetable Retieval Failure.");
				break;
			}
		}
		return;
	}

	// Start the service
	if(!ntrip_client_->run()) {
		ntrip_client_->stop();
		rtcm_logger_.closeFile();
	}

	// If the client is initialized
	if (ntrip_client_.get() != nullptr) {
		int attempts = 0;

		// While the ntrip service is not what is requested and hasn't failed
		while(ntrip_client_->service_running() != ntrip_state_.en &&
				ntrip_client_->service_failure() == adnav::ntrip::NTRIP_NO_FAIL) {
			attempts++;
			std::this_thread::sleep_for(std::chrono::seconds(1));
			if(attempts > NTRIP_TIMEOUT_PERIOD + 1) {
				response->success = false;
				response->reason = std::string("NTRIP Client Failure");
				if(ntrip_state_.he == nullptr) response->reason.append(": Bad Host");
				return;
			}
		}

		// if the service fails.
		if(ntrip_client_->service_failure() != adnav::ntrip::NTRIP_NO_FAIL) {
			response->success = false;
			std::stringstream ss;
			switch (ntrip_client_->service_failure())
			 {
			case adnav::ntrip::NTRIP_CONNECTION_TIMEOUT_FAILURE:
				ss << "NTRIP Connection Timeout.\n";
				break;
			case adnav::ntrip::NTRIP_CREATE_SOCK_FAILURE:
				ss << "NTRIP Socket Creation Failure.\n";
				break;
			case adnav::ntrip::NTRIP_REMOTE_CLOSE:
				ss << "NTRIP Remote Socket Closed.\n";
				break;
			case adnav::ntrip::NTRIP_REMOTE_SOCKET_FAILURE:
				ss << "NTRIP Remote Socket Failure.\n";
				break;
			case adnav::ntrip::NTRIP_CASTER_CONNECTION_FAILURE:
				ss << "NTRIP Caster Connection Failure.\n";
				if(ntrip_state_.he == nullptr) ss << ":Bad Host";
				break;
			case adnav::ntrip::NTRIP_SEND_GPGGA_FAILURE:
				ss << "NTRIP SEND GPGGA FAILURE.\n";
				break;
			case adnav::ntrip::NTRIP_UNAUTHORIZED:
				ss << "NTRIP Caster Access Authorisation Failure.\n";
				break;
			case adnav::ntrip::NTRIP_FORBIDDEN:
				ss << "NTRIP Caster returned HTTP 403 Forbidden.\n";
				break;
			case adnav::ntrip::NTRIP_NOT_FOUND:
				ss << "NTRIP Caster returned HTTP 404 Not Found. Perhaps a bad mountpoint or host was supplied?\n";
				break;
			case adnav::ntrip::NTRIP_UNRECOGNIZED_RETURN:
				ss << "NTRIP Caster returned an unknown HTTP header.\n";
				break;
			default:
				ss << "Unknown NTRIP Error.\n";
				break;
			}
			response->reason = ss.str();
			RCLCPP_ERROR(this->get_logger(), "%s", ss.str().c_str());
			rtcm_logger_.closeFile();
			return;
		}
	}

	// it is uninitialized and request is to enable ntrip
	else if (ntrip_state_.en == true) {
		RCLCPP_ERROR(this->get_logger(), "Unable to enable NTRIP service");
		response->success = false;
		response->reason = std::string("Unable to enable NTRIP service");
		return;
	}

	// Successful result reached
	response->success = true;
	if(ntrip_state_.en == true) response->reason = std::string("Successfuly enabled NTRIP client.");
	else response->reason = std::string("Successfully disabled NTRIP client.");

}

//~~~~~~ Parameter Functions

/**
 * @brief Callback to handle dynamic reconfiguration of node parameters.
 *
 * @param Params Vector of changed parameters
 * @return Result class for success/failure of parameter settings
 */
rcl_interfaces::msg::SetParametersResult Driver::ParamSetCallback(const std::vector<rclcpp::Parameter> &Params) {
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason = "";

	// For every parameter in the vector
	for (const auto &parameter : Params) {

		RCLCPP_INFO(this->get_logger(), "Received Param Update Request\n\tname:\t%s\n\ttype:\t%s\n\tvalue:\t%s\n", parameter.get_name().c_str(),
					parameter.get_type_name().c_str(), parameter.value_to_string().c_str());

		// Validation of parameters
		// Only if they require validation
		// comport
		if (parameter.get_name() == "com_port") result.reason += validateComPort(parameter).reason;
		// Publisher Interval Duration
		else if (parameter.get_name() == "publish_us") result.reason += validatePublishUs(parameter).reason;
		// Packet IDs and Rates
		else if (parameter.get_name() == "packet_request") result.reason += validatePacketRequest(parameter).reason;
		// Packet Timer Period
		else if (parameter.get_name() == "packet_timer_period") result.reason += validatePacketTimer(parameter).reason;
	} // for each parameter

	if(result.reason != "") {
		result.successful = false;
	}
	return result;
}

/**
 * @brief Function to validate the proposed parameter change to the Com Port parameter.
 *
 * @param parameter const rclcpp::Parameter. proposed change to the Com Port parameter.
 * @return returns a SetParametersResult message containing a success value and a string descriptor.
 */
rcl_interfaces::msg::SetParametersResult Driver::validateComPort(const rclcpp::Parameter& parameter) {
	rcl_interfaces::msg::SetParametersResult result;
	result.reason = "";
	result.successful = true;

	// Stringstream to stream in faults.
	std::stringstream ss;

	// Guard for parameter type
	if(parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
		result.successful = false;
		ss << "\n[Error] Passed parameter is not of String Type.\n";
	}

	if(!result.successful)	 {
		result.reason = ss.str();
	}
	return result;
}

/**
 * @brief Function to validate the proposed parameter change to the Publish us parameter.
 *
 * @param parameter const rclcpp::Parameter. proposed change to the Publish us parameter.
 * @return returns a SetParametersResult message containing a success value and a string descriptor.
 */
rcl_interfaces::msg::SetParametersResult Driver::validatePublishUs(const rclcpp::Parameter& parameter) {
	rcl_interfaces::msg::SetParametersResult result;
	result.reason = "";
	result.successful = true;

	// Stringstream to stream in faults.
	std::stringstream ss;

	// Guard for parameter type
	if(parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
		result.successful = false;
		ss << "\n[Error] Passed parameter is not of Integer Type.\n";
	}

	// Guard for timer range
	if(parameter.as_int() < 1000 || parameter.as_int() > 1000000) {
		result.successful = false;
		ss << "\n[Error] Publish Period " << parameter.as_int() << " is invalid. Out of range.";
		result.reason = ss.str();
	}

	if(!result.successful)	 {
		result.reason = ss.str();
	}
	return result;
}

/**
 * @brief Function to Update the Publish us parameter, create temporary service client and update the node using services.
 *
 * @param parameter const rclcpp::Parameter. updated Publish us parameter
 */
void Driver::updatePublishUs([[maybe_unused]] const rclcpp::Parameter& parameter) {
	RestartPublisher();
}

/**
 * @brief Function to validate the proposed parameter change to the Packet Request parameter.
 *
 * @param parameter const rclcpp::Parameter. proposed change to the Packet Request parameter.
 * @return returns a SetParametersResult message containing a success value and a string descriptor.
 */
rcl_interfaces::msg::SetParametersResult Driver::validatePacketRequest(const rclcpp::Parameter& parameter) {
	rcl_interfaces::msg::SetParametersResult result;
	result.reason = "";
	result.successful = true;

	// Stringstream to stream in faults.
	std::stringstream ss;

	// Guard for parameter type
	if(parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
		result.successful = false;
		ss << "\n[Error] Passed parameter is not of Integer Array Type.\n";
	}

	// Guard to ensure that a packet is included in the array
	if(parameter.as_integer_array().size() < 2) {
		result.successful = false;
		ss << "\n[Error] ID Array must contain one or more requested packet\n" <<
			"Usage: [ID1, Rate1, ID2, Rate2, ...]\n";
	}

	// Guard to ensure that there is even entries
	if(parameter.as_integer_array().size() % 2 != 0) {
		result.successful = false;
		ss << "\n[Error] ID Array must contain an even number of entries\n"<<
			"Usage: [ID1, Rate1, ID2, Rate2, ...]\n";
	}

	// Check all elements of the request array
	for(std::size_t i = 0; i < parameter.as_integer_array().size(); i++) {
		// if even (ID)
		int64_t element = parameter.as_integer_array().at(i);
		if(i%2 == 0) {
			// Check for valid ID
			if( element < 0 || 	// below range
			(element >= end_system_packets && element < START_STATE_PACKETS) || // system-state gap
			(element >= end_state_packets && element < START_CONFIGURATION_PACKETS) || // state-config gap
			element >= end_configuration_packets) { // above range
				result.successful = false;
				ss << "\n[Error] ID: " << element << "\t isn't a valid packet.\n";
			}
		}else { // If odd (Period)
			if(element < MIN_PACKET_PERIOD || element > MAX_PACKET_PERIOD) {
				result.successful = false;
				ss << "\n[Error] Period: " << element << "\t isn't a valid period.\n";
			}
		}
	}

	if(!result.successful)	 {
		result.reason = ss.str();
	}
	return result;
}

std::vector<adnav_interfaces::msg::PacketPeriod> Driver::getPacketRequest() {
	std::vector<adnav_interfaces::msg::PacketPeriod> packet_periods;
	const auto params = param_listener_->get_params();

	if (params.send_packet_periods)
	{
		for (const auto pkt_id : REQUIRED_PACKETS)
		{
			adnav_interfaces::msg::PacketPeriod period;
			period.packet_id = pkt_id;
			period.packet_period = params.default_packet_period;
			packet_periods.push_back(period);
		}

		if (params.additional_packets.body_velocity) {
			adnav_interfaces::msg::PacketPeriod period;
			period.packet_id = packet_id_body_velocity;
			period.packet_period = params.default_packet_period;
			packet_periods.push_back(period);
		}

		if (params.additional_packets.external_body_velocity) {
			adnav_interfaces::msg::PacketPeriod period;
			period.packet_id = packet_id_external_body_velocity;
			period.packet_period = params.default_packet_period;
			packet_periods.push_back(period);
		}

		if (params.additional_packets.ecef_position) {
			adnav_interfaces::msg::PacketPeriod period;
			period.packet_id = packet_id_ecef_position;
			period.packet_period = params.default_packet_period;
			packet_periods.push_back(period);
		}

		if (params.additional_packets.utm_position) {
			adnav_interfaces::msg::PacketPeriod period;
			period.packet_id = packet_id_utm_position;
			period.packet_period = params.default_packet_period;
			packet_periods.push_back(period);
		}

		if (params.additional_packets.raw_sensors) {
			adnav_interfaces::msg::PacketPeriod period;
			period.packet_id = packet_id_raw_sensors;
			period.packet_period = params.default_packet_period;
			packet_periods.push_back(period);
		}
	}
	return packet_periods;
}

/**
 * @brief Function to Update the Packet Request parameter
 *
 * @param parameter const rclcpp::Parameter. updated Packet Request parameter
 */
void Driver::updatePacketRequest([[maybe_unused]] const rclcpp::Parameter& parameter) {
	SendPacketPeriods(getPacketRequest());
}

/**
 * @brief Function to validate the proposed parameter change to the Packet Timer parameter.
 *
 * @param parameter const rclcpp::Parameter. proposed change to the Packet Timer parameter.
 * @return returns a SetParametersResult message containing a success value and a string descriptor.
 */
rcl_interfaces::msg::SetParametersResult Driver::validatePacketTimer(const rclcpp::Parameter& parameter) {
	rcl_interfaces::msg::SetParametersResult result;
	result.reason = "";
	result.successful = true;

	// Stringstream to stream in faults.
	std::stringstream ss;

	// Guard for parameter type
	if(parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
		result.successful = false;
		ss << "\n[Error] Passed parameter is not of Integer Type.\n";
	}

	// Guard for timer range
	if(parameter.as_int() < MIN_TIMER_PERIOD || parameter.as_int() > MAX_TIMER_PERIOD) {
		result.successful = false;
		ss << "\n[Error] Packet Timer Period " << parameter.as_int() << " is invalid.";
		result.reason = ss.str();
	}

	if(!result.successful)	 {
		result.reason = ss.str();
	}
	return result;
}

/**
 * @brief Function to Update the PacketTimer parameter, then call for it to be sent to the device.
 *
 * @param parameter const rclcpp::Parameter. updated PacketTimer parameter
 */
void Driver::updatePacketTimer([[maybe_unused]] const rclcpp::Parameter& parameter) {
	// Send to the device.
	const auto params = param_listener_->get_params();
	SendPacketTimer(params.packet_timer_period);
}

//~~~~~~ NTRIP Functions

/**
 * @brief Function to Update the NTRIP service with new parameters.
 */
void Driver::updateNTRIPClientService() {
	// check if the service is running, if so stop it
	if (ntrip_client_.get() != nullptr && ntrip_client_->service_running()) {
		ntrip_client_->stop();
	}

	if (rtcm_logger_.is_open()) rtcm_logger_.closeFile();

	// Guard to stop execution if request enable is false.
	if (ntrip_state_.en == false) return;

	// create new client instance with new parameters. Deletes old instance.
	ntrip_client_ = std::make_unique<adnav::ntrip::Client>(ntrip_state_.ip,
		ntrip_state_.port,
		ntrip_state_.username,
		ntrip_state_.password,
		ntrip_state_.mountpoint,
		[this](const std::string& msg) { RCLCPP_INFO(this->get_logger(), "%s", msg.c_str()); },
		[this](const std::string& msg) { RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str()); },
		"NTRIP AdNavRos/2.0");

	ntrip_client_->OnReceived(std::bind(
			&Driver::NtripReceiveFunction,
			this,
			std::placeholders::_1,
			std::placeholders::_2));

	ntrip_client_->set_report_interval(DEFAULT_GPGGA_REPORT_PERIOD);
	ntrip_client_->set_location(llh_.latitude, llh_.longitude, llh_.height);

	const auto params = param_listener_->get_params();
	// Guard to stop bad hostend return.
	if(ntrip_state_.he != nullptr) {
		rtcm_logger_.openFile((std::string("Log_") + std::string(ntrip_state_.he->h_name)), ".rtcm", params.log_path);
	}
}

/**
 * @brief Function to decode the host string and place relevant data into struct.
 *
 * @param host string containing host or ip followed by port
 */
void Driver::getDataFromHostStr(const std::string& host) {
	// split the string using the ':' character
	std::vector<std::string> split_string = adnav::utils::splitStr(host, ':');

	// if the string is invalid.
	if(split_string.size() != 2) {
		RCLCPP_ERROR(this->get_logger(), "Invalid Host String");
		this->set_parameter(rclcpp::Parameter("ntrip_enable", false));
	}

	struct in_addr addr;
	// Get the IP from the string.
	if(adnav::utils::validateIP(split_string.front()) == true) {
		if(inet_pton(AF_INET, split_string.front().c_str(), &addr) == 0) {
			RCLCPP_ERROR(this->get_logger(), "Invalid Address.");
			return;
		}
		ntrip_state_.he = gethostbyaddr((char*) &addr, sizeof(addr), AF_INET);
		if(ntrip_state_.he == nullptr) {
			ntrip_state_.he = gethostbyname(split_string.front().c_str());
		}
	} else {
		ntrip_state_.he = gethostbyname(split_string.front().c_str());
	}

	if(ntrip_state_.he == nullptr) return;

	char ip[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, (struct in_addr*) ntrip_state_.he->h_addr_list[0], ip, INET_ADDRSTRLEN);

	ntrip_state_.ip = std::string(ip);

	// Get the Port from the string
	ntrip_state_.port = atoi(split_string.back().c_str());
}

/**
 * @brief Function to take the incoming buffer from the NTRIP client, place it into ANPP Packet 55
 * (RTCM Corrections Packet) and send it to the device.
 *
 * @param buffer the character buffer containing RTCM data.
 * @param size size of the RTCM corrections buffer.
 */
void Driver::NtripReceiveFunction(const char* buffer, int size) {
	std::stringstream ss;
	an_packet_t* an_packet;
	rtcm_corrections_packet_t rtcm_corrections_packet;
	std::string received(buffer, size);

	// Write to the logfile.
	rtcm_logger_.writeAndIncrement(buffer, size);

	ss << "Recieved [" << size << "]:\n";
	// for each byte in the buffer
	int buffer_idx = 0;
	int number_of_full_size_anpp = floor(size / 255);
	int size_of_last_packet = size % 255;
	for (int i = 0; i < number_of_full_size_anpp; ++i) {
		// Write to the debug string
		char buf[10];
		sprintf(buf, "%02X ", static_cast<uint8_t>(buffer[buffer_idx]));
		ss << buf;

		rtcm_corrections_packet.packet_data = (uint8_t*)(&buffer[buffer_idx]);
		// The packet is full send it to the device.
		an_packet = encode_rtcm_corrections_packet(&rtcm_corrections_packet, 255);
		RCLCPP_DEBUG(this->get_logger(), "Sending RTCM Corrections Packet with %d bytes", an_packet->length);
		encodeAndSend(an_packet);
		buffer_idx += 255;
	}

	// Send the final partially filled packet
	rtcm_corrections_packet.packet_data = (uint8_t*)(&buffer[buffer_idx]);
	an_packet = encode_rtcm_corrections_packet(&rtcm_corrections_packet, size_of_last_packet);
	RCLCPP_DEBUG(this->get_logger(), "Sending RTCM Corrections Packet with %d bytes", an_packet->length);
	encodeAndSend(an_packet);

	ss << std::endl << std::endl;
	RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
}

//~~~~~~ Device Communication Functions

/**
 * @brief Function to encode a ANPP packet with a header CRC and LRC, and send it over the open communications method
 *
 * @param an_packet a pointer to an an_packet_t object which will be encoded and send to the device
 */
void Driver::encodeAndSend(an_packet_t* an_packet) {
	an_packet_encode(an_packet);

	const auto size = static_cast<unsigned int>(an_packet_size(an_packet));
	auto data = std::make_unique<char[]>(size);
	std::memcpy(data.get(), an_packet_pointer(an_packet), size);
	an_packet_free(&an_packet);

	{
		std::lock_guard lock(write_q_mutex_);
		write_q_.push({std::move(data), size});
	}
	write_async_->send();
}

/**
 * @brief Register a pending reply for the acknowledge of @p an_packet, send it,
 *        then block until the device replies or @c DEFAULT_TIMEOUT seconds elapse.
 *
 * Uses pending_replies_ keyed on packet_id_acknowledge, so any caller can use the
 * same mechanism to await any other packet type by registering their own PendingReply.
 *
 * Thread-safe: may be called from any thread (e.g. setup_thread_ or a ROS service thread).
 * The actual TCP write is always dispatched onto the event loop via write_async_.
 *
 * @param an_packet Encoded ANPP packet whose reply we are waiting for.
 * @return RawAcknowledge message; result field is 0xFF on timeout or connection drop.
 */
adnav_interfaces::msg::RawAcknowledge Driver::sendAndAwaitAck(an_packet_t* an_packet) {
	const uint8_t packet_id = an_packet->id;

	auto promise = std::make_shared<std::promise<acknowledge_packet_t>>();
	std::future<acknowledge_packet_t> future = promise->get_future();

	// Register BEFORE sending to avoid a race where the ack arrives before the entry exists.
	{
		std::lock_guard lock(pending_replies_mutex_);
		pending_replies_[packet_id_acknowledge] = {
			[promise](an_packet_t* pkt) {
				acknowledge_packet_t ack{};
				if (decode_acknowledge_packet(&ack, pkt)) {
					promise->set_exception(std::make_exception_ptr(
						std::runtime_error("Decode error")));
					return;
				}
				try { promise->set_value(ack); } catch (const std::future_error&) {}
			},
			[promise](std::exception_ptr ep) {
				try { promise->set_exception(ep); } catch (const std::future_error&) {}
			}
		};
	}

	encodeAndSend(an_packet);

	adnav_interfaces::msg::RawAcknowledge msg;

	if (future.wait_for(std::chrono::seconds(DEFAULT_TIMEOUT)) != std::future_status::ready) {
		RCLCPP_ERROR(this->get_logger(), "ACK timeout for packet %d", packet_id);
		std::lock_guard lock(pending_replies_mutex_);
		pending_replies_.erase(packet_id_acknowledge);
		msg.result = 0xFF;
		return msg;
	}

	try {
		const auto pkt = future.get();
		msg.id     = pkt.packet_id;
		msg.crc    = pkt.packet_crc;
		msg.result = pkt.acknowledge_result;
	} catch (const std::exception& e) {
		// Promise was broken by failAllPendingReplies() — connection dropped mid-wait
		RCLCPP_ERROR(this->get_logger(), "ACK aborted for packet %d: %s", packet_id, e.what());
		msg.result = 0xFF;
	}

	return msg;
}

/**
 * @brief Fail all in-flight pending replies with an exception.
 *
 * Called on disconnect (error_event / end_event) so that any thread blocked in
 * sendAndAwaitAck() or any other future.get() unblocks immediately.
 */
void Driver::failAllPendingReplies() {
	auto ep = std::make_exception_ptr(std::runtime_error("Connection lost"));
	std::lock_guard lock(pending_replies_mutex_);
	for (auto& [id, reply] : pending_replies_) {
		reply.on_cancel(ep);
	}
	pending_replies_.clear();
}

/**
 * @brief Function to send the packet Timer to the device and await the acknowledgement.
 *
 * @param packet_timer_period Period for the packet rates in microseconds.
 * @param utc_sync Synchronize with UTC time. True by default.
 * @param permanent Is this a permanent change. True by default.
 * @return acknowledgement Message
 */
adnav_interfaces::msg::RawAcknowledge Driver::SendPacketTimer(int packet_timer_period, bool utc_sync, bool permanent) {
	RCLCPP_DEBUG_STREAM(this->get_logger(), "Sending Packet Timer Request:\nUTC Sync: " <<
		(utc_sync ? "True\n":"False\n") <<
		"\tPermanent: " << (permanent ? "True\n":"False\n") <<
		"\tPeriod (μs): " << packet_timer_period << "\n");

	// Create a periods packet.
	packet_timer_period_packet_t packet_timer_period_packet;
	an_packet_t *an_packet;

	// set packet to 0
	memset(&packet_timer_period_packet, 0, sizeof(packet_timer_period_packet));

	// Fill the packet
	packet_timer_period_packet.permanent = permanent;
	packet_timer_period_packet.utc_synchronisation = utc_sync;
	packet_timer_period_packet.packet_timer_period = packet_timer_period;

	// Send the packet and block until the device acknowledges or timeout.
	RCLCPP_INFO(this->get_logger(), "Sending Packet Timer Request to device.");
	an_packet = encode_packet_timer_period_packet(&packet_timer_period_packet);
	return sendAndAwaitAck(an_packet);
}

/**
 * @brief Function to send the packet Timer to the device and await the acknowledgement.
 *
 * @param periods Vector of packet periods to be sent to the device.
 * @param clear_existing Boolean value for overwriting existing packet periods. Default = True.
 * @param permanent Boolean value for overwriting configuration memory. Default = True.
 * @return acknowledgement Message
 */
adnav_interfaces::msg::RawAcknowledge Driver::SendPacketPeriods(const std::vector<adnav_interfaces::msg::PacketPeriod>& periods,
	bool clear_existing, bool permanent) {
	std::stringstream ss;
	ss << "Sending Packet Request:\nClear: " << (clear_existing ? "True\n":"False\n") <<
		"Permanent: " << (permanent ? "True\n":"False\n") <<
		"Number Requested: " << periods.size() << std::endl;

	// Create a periods packet.
	packet_periods_packet_t packet_periods_packet;
	an_packet_t *an_packet;

	// Set contents of packet to 0
	memset(&packet_periods_packet, 0, sizeof(packet_periods_packet));

	// Fill in the packet
	packet_periods_packet.permanent = permanent;
	packet_periods_packet.clear_existing_packets = clear_existing;

	int j = 0;
	for(adnav_interfaces::msg::PacketPeriod i : periods) {
		ss << "\tID: " << static_cast<int>(i.packet_id) << "\tPeriod: "<< i.packet_period << std::endl;
		packet_periods_packet.packet_periods[j].packet_id = i.packet_id;
		packet_periods_packet.packet_periods[j].period = i.packet_period;
		j++;
	}

	RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());

	RCLCPP_INFO(this->get_logger(), "Sending Packet Periods Request to device.");
	an_packet = encode_packet_periods_packet(&packet_periods_packet);
	return sendAndAwaitAck(an_packet);
}

//~~~~~~ Decoders

/**
 * @brief Function to decode packets in the decoder and fill out ROS messages
 *
 * @param an_decoder instance of the decoder loaded with data to be decoded.
 * @param bytes number of bytes within the decoders buffer.
 */
void Driver::decodePackets(an_decoder_t &an_decoder, const int &bytes) {
	// If there are bytes to be decoded.
	an_packet_t* an_packet;
	if (bytes > 0) {
		// Decode all the packets in the buffer
		std::unique_lock<std::mutex> lock(messages_mutex_);

		auto buffer_ts = now();

		while ((an_packet = an_packet_decode(&an_decoder)) != nullptr)
		 {
			RCLCPP_DEBUG(this->get_logger(), "RX Packet IDs %hhu", an_packet->id);

			bool handled = true;
			switch (an_packet->id)
			 {
			case packet_id_device_information: deviceInfoDecoder(an_packet);
				break;

			case packet_id_system_state: systemStateRosDecoder(an_packet);
				break;

			case packet_id_ecef_position: ecefPosRosDecoder(an_packet);
				break;

			case packet_id_utm_position: utmPosRosDecoder(an_packet);
				break;

			case packet_id_gnss_position_velocity_time: gnssPosVelTimeRosDecoder(an_packet);
				break;

			case packet_id_quaternion_orientation_standard_deviation: quartOrientSDRosDriver(an_packet);
				break;

			case packet_id_euler_orientation_standard_deviation:
				{
					euler_orientation_standard_deviation_packet_t pkt;
					if (decode_euler_orientation_standard_deviation_packet(&pkt, an_packet) == 0)
					{
						euler_orientation_standard_deviation_packet_ = pkt;
					}
					break;
				}

			case packet_id_velocity_standard_deviation:
				{
					velocity_standard_deviation_packet_t pkt;
					if (decode_velocity_standard_deviation_packet(&pkt, an_packet) == 0)
					{
						velocity_standard_deviation_packet_ = pkt;
					}
					break;
				}

			case packet_id_raw_sensors:
				rawSensorsRosDecoder(an_packet);
				break;

			case packet_id_body_velocity:
				bodyVelRosDecoder(an_packet);
				break;

			case packet_id_external_body_velocity:
				extBodyVelRosDecoder(an_packet);
				break;

			case packet_id_running_time:
				{
					running_time_packet_t pkt;
					if (decode_running_time_packet(&pkt, an_packet) == 0)
					{
						auto uptime = duration_from_seconds_microseconds(pkt.seconds, pkt.microseconds);
						if (device_running_time_ && uptime < *device_running_time_)
						{
							RCLCPP_ERROR(this->get_logger(), "Device running time has gone backwards. Previous: %f seconds, Current: %f seconds. This may indicate a device reset or clock issue.",
								device_running_time_->seconds(), uptime.seconds());
							close_async_->send();
						}
						device_running_time_ = uptime;
					}
					break;
				}
			default:
				handled = false;
				break;
			}

			packet_receive_times.insert_or_assign(an_packet->id, buffer_ts);

			// Generic pending reply dispatch — fires for any packet_id registered in pending_replies_.
			// Runs alongside named case handlers, so both can fire for the same packet (e.g. device info
			// is always decoded above AND can also satisfy a request-reply future registered below).
			{
				std::unique_lock reply_lock(pending_replies_mutex_);
				auto it = pending_replies_.find(an_packet->id);
				if (it != pending_replies_.end()) {
					auto cb = std::move(it->second.on_packet);
					pending_replies_.erase(it);
					reply_lock.unlock();
					cb(an_packet);
					handled = true;
				}
			}

			if (!handled) {
				RCLCPP_WARN/*_THROTTLE*/(this->get_logger(), /* *this->get_clock(), 500, */
					"Unsupported packet definition for ROS driver. PACKET_ID: %d", an_packet->id);
			}

			// Ensure that you free the an_packet when your done with it or you will leak memory
			an_packet_free(&an_packet);
		}
	}
}

/**
 * @brief Function to decode a device information packet and output its contents to the ROS logger
 *
 * @param an_packet pointer to a an_packet_t object from which to decode the information.
 */
void Driver::deviceInfoDecoder(an_packet_t* an_packet) {
	// Decode packet and warn if error.
	device_information_packet_t device_information_packet;
	if(decode_device_information_packet(&device_information_packet, an_packet))
	{
		RCLCPP_WARN(this->get_logger(), "Error decoding device information Packet");
	} else
	{
		std::stringstream serial_num;
		serial_num << std::hex << device_information_packet.serial_number[0] <<
			device_information_packet.serial_number[1] << device_information_packet.serial_number[2];
		diagnostic_updater_->setHardwareID(serial_num.str());
		device_information_packet_ = device_information_packet;
	}

	// since multiple packets may be requested before the device responds. ensure only one gets printed per second.
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s",
		std::format("Device ID: {} - Software Rev: {} - Hardware Rev: {} - Serial Number: {:X}{:X}{:X}",
			device_information_packet.device_id,
			device_information_packet.software_version,
			device_information_packet.hardware_revision,
			device_information_packet.serial_number[0],
			device_information_packet.serial_number[1],
			device_information_packet.serial_number[2]).c_str());
}

/**
 * @brief Function to decode the System State ANPP Packet (ANPP.20).
 *
 * This function accesses in a thread safe manner the class stored ROS messages, placed relevant information into them,
 * then using the publishing control variable, requests a publisher thread to publish the message.
 *
 * @param an_packet a pointer to an an_packet_t object which will be decoded.
 */
void Driver::systemStateRosDecoder(an_packet_t* an_packet) {
	const auto params = param_listener_->get_params();
	system_state_packet_t system_state_packet;
	std::stringstream ss;

	if(decode_system_state_packet(&system_state_packet, an_packet) == 0)
	 {
			// NAVSATFIX
			nav_fix_msg_.header.stamp = time_from_seconds_microseconds(system_state_packet.unix_time_seconds,
				system_state_packet.microseconds);
			nav_fix_msg_.header.frame_id = frame_id_;
			if ((system_state_packet.filter_status.b.gnss_fix_type == gnss_fix_2d) ||
				(system_state_packet.filter_status.b.gnss_fix_type == gnss_fix_3d))
			 {
				nav_fix_msg_.status.status= sensor_msgs::msg::NavSatStatus::STATUS_FIX;
				nav_fix_msg_.status.status= sensor_msgs::msg::NavSatStatus::STATUS_FIX;
			}
			else if ((system_state_packet.filter_status.b.gnss_fix_type == gnss_fix_sbas) ||
					(system_state_packet.filter_status.b.gnss_fix_type == gnss_fix_omnistar))
			 {
				nav_fix_msg_.status.status= sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
				nav_fix_msg_.status.status= sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
			}
			else if ((system_state_packet.filter_status.b.gnss_fix_type == gnss_fix_differential) ||
					(system_state_packet.filter_status.b.gnss_fix_type == gnss_fix_rtk_float) ||
					(system_state_packet.filter_status.b.gnss_fix_type == gnss_fix_rtk_fixed))
			 {
				nav_fix_msg_.status.status= sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
				nav_fix_msg_.status.status= sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
			}
			else
			 {
				nav_fix_msg_.status.status= sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
				nav_fix_msg_.status.status= sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
			}
			nav_fix_msg_.latitude = system_state_packet.latitude * RADIANS_TO_DEGREES;
			nav_fix_msg_.longitude = system_state_packet.longitude * RADIANS_TO_DEGREES;
			nav_fix_msg_.altitude = system_state_packet.height;
			nav_fix_msg_.position_covariance = { pow(system_state_packet.standard_deviation[1], 2), 0.0, 0.0,
				0.0, pow(system_state_packet.standard_deviation[0], 2), 0.0,
				0.0, 0.0, pow(system_state_packet.standard_deviation[2], 2)};
			nav_fix_msg_.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

			llh_.latitude = system_state_packet.latitude * RADIANS_TO_DEGREES;
			llh_.longitude = system_state_packet.longitude * RADIANS_TO_DEGREES;
			llh_.height = system_state_packet.height;

			if(ntrip_client_) {
				ntrip_client_->set_location(llh_.latitude, llh_.longitude, llh_.height);
			}

			// the orientation fields in the packet are roll, pitch, heading (0 = North, pi/2 = east)
			// system packet roll pitch yaw is in FRD with a NED world frame
			orientation_frd_ned_.setRPY(
				system_state_packet.orientation[0],
				system_state_packet.orientation[1],
				system_state_packet.orientation[2]
			);

			orientation_flu_ = transform_frd_ned_to_flu_enu(orientation_frd_ned_);

			// system packet velocities are NED, so it needs to be rearranged for ENU
			twist_stamped_msg_enu.twist.linear.x = system_state_packet.velocity[1];
			twist_stamped_msg_enu.twist.linear.y = system_state_packet.velocity[0];
			twist_stamped_msg_enu.twist.linear.z = -system_state_packet.velocity[2];

			geometry_msgs::msg::Vector3 angular_velocity_frd;
			// system packet angular velocity is in body frame, FRD
			angular_velocity_frd.x = system_state_packet.angular_velocity[0];
			angular_velocity_frd.y = system_state_packet.angular_velocity[1];
			angular_velocity_frd.z = system_state_packet.angular_velocity[2];

			twist_flu_msg_.linear = transform_enu_to_flu(twist_stamped_msg_enu.twist.linear, orientation_flu_);
			twist_flu_msg_.angular = transform_frd_to_flu(angular_velocity_frd);

			// convert the FRD angular vel to ENU for the enu twist for completion's sake
			twist_stamped_msg_enu.twist.angular = transform_frd_to_enu(angular_velocity_frd, orientation_frd_ned_);

			twist_stamped_msg_.twist = twist_flu_msg_;
			twist_stamped_msg_.header = nav_fix_msg_.header;
			twist_stamped_msg_enu.header.frame_id = "enu";
			twist_stamped_msg_enu.header.stamp = nav_fix_msg_.header.stamp;

			geometry_msgs::msg::Vector3 body_frd_acceleration;
			body_frd_acceleration.x = system_state_packet.body_acceleration[0];
			body_frd_acceleration.y = system_state_packet.body_acceleration[1];
			body_frd_acceleration.z = system_state_packet.body_acceleration[2];

			// IMU
			imu_msg_.header = nav_fix_msg_.header;
			imu_msg_.orientation = tf2::toMsg(orientation_flu_);

			imu_msg_.angular_velocity = twist_flu_msg_.angular;
			imu_msg_.linear_acceleration = transform_frd_to_flu(body_frd_acceleration);

			geo_pose_msg_.orientation = imu_msg_.orientation;
			geo_pose_msg_.position.latitude = nav_fix_msg_.latitude;
		    geo_pose_msg_.position.longitude = nav_fix_msg_.longitude;
		    geo_pose_msg_.position.altitude = nav_fix_msg_.altitude;
			geo_pose_stamped_msg_.pose = geo_pose_msg_;
			geo_pose_stamped_msg_.header = nav_fix_msg_.header;

			system_state_packet_ = system_state_packet;

			if (system_state_packet.filter_status.b.event1_flag) {
				RCLCPP_INFO(this->get_logger(), "Event 1 Occurred.");
			}

			if (system_state_packet.filter_status.b.event2_flag) {
				RCLCPP_INFO(this->get_logger(), "Event 1 Occurred.");
			}
	}
}

void Driver::gnssPosVelTimeRosDecoder(an_packet_t* an_packet) {
	gnss_position_velocity_time_packet_t pvt_packet;

	if(decode_gnss_position_velocity_time_packet(&pvt_packet, an_packet) == 0)
	{
		gnss_position_velocity_time_packet_ = pvt_packet;
	}
}

/**
 * @brief Function to decode the ECEF Position ANPP Packet (ANPP.33).
 *
 * This function accesses in a thread safe manner the class stored ROS messages, placed relevant information into them,
 * then using the publishing control variable, requests a publisher thread to publish the message.
 *
 * @param an_packet a pointer to an an_packet_t object which will be decoded.
 */
void Driver::ecefPosRosDecoder(an_packet_t* an_packet) {
	ecef_position_packet_t ecef_position_packet;

	// ECEF Position (in meters) Packet for Pose Message
	if(decode_ecef_position_packet(&ecef_position_packet, an_packet) == 0)
	 {
		// Position is in X, Y, Z
		pose_ecef_stamped_msg_.pose.position.x = ecef_position_packet.position[0];
		pose_ecef_stamped_msg_.pose.position.y = ecef_position_packet.position[1];
		pose_ecef_stamped_msg_.pose.position.z = ecef_position_packet.position[2];

		pose_ecef_stamped_msg_.header.stamp = nav_fix_msg_.header.stamp;
		pose_ecef_stamped_msg_.header.frame_id = "ecef";
	}
}

void Driver::utmPosRosDecoder(an_packet_t* an_packet) {
	utm_position_packet_t utm_position_packet;

	if(decode_utm_position_packet(&utm_position_packet, an_packet) == 0)
	{
		// Position is in Northing, Easting, Height
		pose_utm_stamped_msg_.pose.position.x = utm_position_packet.position[1];
		pose_utm_stamped_msg_.pose.position.y = utm_position_packet.position[0];
		pose_utm_stamped_msg_.pose.position.z = utm_position_packet.position[2];

		pose_utm_stamped_msg_.pose.orientation = imu_msg_.orientation;
		pose_utm_stamped_msg_.header.stamp = nav_fix_msg_.header.stamp;
		pose_utm_stamped_msg_.header.frame_id = fmt::format("UTM_{}{}",
			utm_position_packet.zone_number, std::toupper(utm_position_packet.zone_char));
	}
}

/**
 * @brief Function to decode the Quaternion Orientation Standard Deviation ANPP Packet (ANPP.27).
 *
 * This function accesses in a thread safe manner the class stored ROS messages, placed relevant information into them,
 * then using the publishing control variable, requests a publisher thread to publish the message.
 *
 * @param an_packet a pointer to an an_packet_t object which will be decoded.
 */
void Driver::quartOrientSDRosDriver(an_packet_t* an_packet) {
	quaternion_orientation_standard_deviation_packet_t quaternion_orientation_standard_deviation_packet;

	if(decode_quaternion_orientation_standard_deviation_packet(&quaternion_orientation_standard_deviation_packet, an_packet) == 0)
	 {
		// IMU message
		imu_msg_.orientation_covariance[0] = quaternion_orientation_standard_deviation_packet.standard_deviation[0];
		imu_msg_.orientation_covariance[4] = quaternion_orientation_standard_deviation_packet.standard_deviation[1];
		imu_msg_.orientation_covariance[8] = quaternion_orientation_standard_deviation_packet.standard_deviation[2];
	}
}

/**
 * @brief Function to decode the Raw Sensor ANPP Packet (ANPP.28).
 *
 * This function accesses in a thread safe manner the class stored ROS messages, placed relevant information into them,
 * then using the publishing control variable, requests a publisher thread to publish the message.
 *
 * @param an_packet a pointer to an an_packet_t object which will be decoded.
 */
void Driver::rawSensorsRosDecoder(an_packet_t* an_packet) {
	raw_sensors_packet_t raw_sensors_packet;

	// Fill the messages
	if(decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0) {
		raw_sensors_packet_ = raw_sensors_packet;
		// RAW MAGNETICFIELD VALUE FROM IMU
		mag_field_msg_.header.frame_id = frame_id_;
		mag_field_msg_.magnetic_field.x = raw_sensors_packet.magnetometers[0];
		mag_field_msg_.magnetic_field.y = raw_sensors_packet.magnetometers[1];
		mag_field_msg_.magnetic_field.z = raw_sensors_packet.magnetometers[2];
		mag_field_msg_.magnetic_field = transform_frd_to_flu(mag_field_msg_.magnetic_field);

		imu_raw_msg_.header.frame_id = frame_id_;
		imu_raw_msg_.orientation_covariance[0] = -1; // Tell recievers that no orientation is sent.
		imu_raw_msg_.linear_acceleration.x = raw_sensors_packet.accelerometers[0];
		imu_raw_msg_.linear_acceleration.y = raw_sensors_packet.accelerometers[1];
		imu_raw_msg_.linear_acceleration.z = raw_sensors_packet.accelerometers[2];
		imu_raw_msg_.linear_acceleration = transform_frd_to_flu(imu_raw_msg_.linear_acceleration);

		imu_raw_msg_.angular_velocity.x = raw_sensors_packet.gyroscopes[0];
		imu_raw_msg_.angular_velocity.y = raw_sensors_packet.gyroscopes[1];
		imu_raw_msg_.angular_velocity.z = raw_sensors_packet.gyroscopes[2];
		imu_raw_msg_.angular_velocity = transform_frd_to_flu(imu_raw_msg_.angular_velocity);

		// BAROMETRIC PRESSURE
		baro_msg_.header.frame_id = frame_id_;
		baro_msg_.fluid_pressure = raw_sensors_packet.pressure;

		// TEMPERATURE
		temp_msg_.header.frame_id = frame_id_;
		temp_msg_.temperature = raw_sensors_packet.pressure_temperature;
	}
}

void Driver::bodyVelRosDecoder(an_packet_t *an_packet)
{
	body_velocity_packet_t body_velocity_packet;

	const auto stamp_time = this->get_clock().get()->now();

	if (decode_body_velocity_packet(&body_velocity_packet, an_packet) == 0)
	{
		// body velocity is in body frame (FRD)
		twist_stamped_msg_body.twist.linear.x = body_velocity_packet.velocity[0];
		twist_stamped_msg_body.twist.linear.y = body_velocity_packet.velocity[1];
		twist_stamped_msg_body.twist.linear.z = body_velocity_packet.velocity[2];

		if (const auto sys_pkt = system_state_packet_.value(); sys_pkt.has_value())
		{
			// borrow the angular rates from the system packet (in FRD)
			twist_stamped_msg_body.twist.angular.x = sys_pkt.value().angular_velocity[0];
			twist_stamped_msg_body.twist.angular.y = sys_pkt.value().angular_velocity[1];
			twist_stamped_msg_body.twist.angular.z = sys_pkt.value().angular_velocity[2];
		}
		twist_stamped_msg_body.twist = transform_frd_to_flu(twist_stamped_msg_body.twist);
		twist_stamped_msg_body.header.stamp = stamp_time;
		twist_stamped_msg_body.header.frame_id = frame_id_;
	}
}

void Driver::extBodyVelRosDecoder(an_packet_t *an_packet)
{
	external_body_velocity_packet_t external_body_velocity_packet;
	const auto stamp_time = this->get_clock().get()->now();

	if (decode_external_body_velocity_packet(&external_body_velocity_packet, an_packet) == 0)
	{
		twist_stamped_msg_external_body.twist.linear.x = external_body_velocity_packet.velocity[0];
		twist_stamped_msg_external_body.twist.linear.y = external_body_velocity_packet.velocity[1];
		twist_stamped_msg_external_body.twist.linear.z = external_body_velocity_packet.velocity[2];
		twist_stamped_msg_external_body.twist = transform_frd_to_flu(twist_stamped_msg_external_body.twist);

		twist_stamped_msg_external_body.header.stamp = stamp_time;
		twist_stamped_msg_external_body.header.frame_id = external_frame_id_;
	}
}

}// namespace adnav
