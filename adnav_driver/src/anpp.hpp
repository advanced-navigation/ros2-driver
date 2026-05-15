#pragma once
#include <cstdint>
#include <string>
#include "ins_packets.h"

enum class FilterStatusBit : uint16_t {
    OrientationFilterInitialised = (1u << 0),
    NavigationFilterInitialised  = (1u << 1),
    HeadingInitialised           = (1u << 2),
    UtcTimeInitialised           = (1u << 3),
    // Bits 4–6: GNSS fix status — NOT individual flags, use gnss_fix_status()
    Event1Occurred               = (1u << 7),
    Event2Occurred               = (1u << 8),
    InternalGnssEnabled          = (1u << 9),
    DualAntennaHeadingActive     = (1u << 10),
    VelocityHeadingEnabled       = (1u << 11),
    AtmosphericAltitudeEnabled   = (1u << 12),
    ExternalPositionActive       = (1u << 13),
    ExternalVelocityActive       = (1u << 14),
    ExternalHeadingActive        = (1u << 15),
};

enum class GnssFixStatus : uint8_t {
    NoFix          = 0,
    Fix2D          = 1,
    Fix3D          = 2,
    SbasFix        = 3,
    DifferentialFix = 4,
    PppFix         = 5,
    RtkFloat       = 6,
    RtkFixed       = 7,
};

inline std::string to_string(const FilterStatusBit bit) {
    switch (bit) {
        case FilterStatusBit::OrientationFilterInitialised: return "Orientation Filter Initialised";
        case FilterStatusBit::NavigationFilterInitialised: return "Navigation Filter Initialised";
        case FilterStatusBit::HeadingInitialised: return "Heading Initialised";
        case FilterStatusBit::UtcTimeInitialised: return "UTC Time Initialised";
        case FilterStatusBit::Event1Occurred: return "Event 1 Occurred";
        case FilterStatusBit::Event2Occurred: return "Event 2 Occurred";
        case FilterStatusBit::InternalGnssEnabled: return "Internal GNSS Enabled";
        case FilterStatusBit::DualAntennaHeadingActive: return "Dual Antenna Heading Active";
        case FilterStatusBit::VelocityHeadingEnabled: return "Velocity Heading Enabled";
        case FilterStatusBit::AtmosphericAltitudeEnabled: return "Atmospheric Altitude Enabled";
        case FilterStatusBit::ExternalPositionActive: return "External Position Active";
        case FilterStatusBit::ExternalVelocityActive: return "External Velocity Active";
        case FilterStatusBit::ExternalHeadingActive: return "External Heading Active";
    }
    return "Unknown Bit";
}

inline std::string to_string(const GnssFixStatus status) {
    switch (status) {
        case GnssFixStatus::NoFix: return "No Fix";
        case GnssFixStatus::Fix2D: return "2D Fix";
        case GnssFixStatus::Fix3D: return "3D Fix";
        case GnssFixStatus::SbasFix: return "SBAS Fix";
        case GnssFixStatus::DifferentialFix: return "Differential Fix";
        case GnssFixStatus::PppFix: return "PPP Fix";
        case GnssFixStatus::RtkFloat: return "RTK Float";
        case GnssFixStatus::RtkFixed: return "RTK Fixed";
    }
    return "Unknown Status";
}

inline std::string to_string(const spoofing_interference_status_e status) {
	switch (status) {
		case spoofing_interference_status_unknown: return "Unknown";
		case spoofing_interference_status_none: return "None";
		case spoofing_interference_status_detected_mitigated: return "Detected, Mitigated";
		case spoofing_interference_status_detected_unmitigated: return "Detected, Unmitigated";
	}
	return "Unknown Status";
}

inline std::optional<std::string> packet_id_to_string(packet_id_e p_id)
{
	switch (p_id)
	{
		case packet_id_acknowledge: return "Acknowledge";
		case packet_id_request: return "Request";
		case packet_id_boot_mode: return "Boot Mode";
		case packet_id_device_information: return "Device Information";
		case packet_id_restore_factory_settings: return "Restore Factory Settings";
		case packet_id_reset: return "Reset";
		case packet_id_file_transfer_request: return "File Transfer Request";
		case packet_id_file_transfer_acknowledge: return "File Transfer Acknowledge";
		case packet_id_file_transfer: return "File Transfer";
		case packet_id_serial_port_passthrough: return "Serial Port Passthrough";
		case packet_id_ip_configuration: return "IP Configuration";
		case packet_id_extended_device_information: return "Extended Device Information";
		case packet_id_subcomponent_information: return "Subcomponent Information";

		case packet_id_system_state: return "System State";
		case packet_id_unix_time: return "Unix Time";
		case packet_id_formatted_time: return "Formatted Time";
		case packet_id_status: return "Status";
		case packet_id_position_standard_deviation: return "Position Standard Deviation";
		case packet_id_velocity_standard_deviation: return "Velocity Standard Deviation";
		case packet_id_euler_orientation_standard_deviation: return "Euler Orientation Standard Deviation";
		case packet_id_quaternion_orientation_standard_deviation: return "Quaternion Orientation Standard Deviation";
		case packet_id_raw_sensors: return "Raw Sensors";
		case packet_id_raw_gnss: return "Raw GNSS";
		case packet_id_satellites: return "Satellites";
		case packet_id_satellites_detailed: return "Satellites Detailed";
		case packet_id_geodetic_position: return "Geodetic Position";
		case packet_id_ecef_position: return "ECEF Position";
		case packet_id_utm_position: return "UTM Position";
		case packet_id_ned_velocity: return "NED Velocity";
		case packet_id_body_velocity: return "Body Velocity";
		case packet_id_acceleration: return "Acceleration";
		case packet_id_body_acceleration: return "Body Acceleration";
		case packet_id_euler_orientation: return "Euler Orientation";
		case packet_id_quaternion_orientation: return "Quaternion Orientation";
		case packet_id_dcm_orientation: return "DCM Orientation";
		case packet_id_angular_velocity: return "Angular Velocity";
		case packet_id_angular_acceleration: return "Angular Acceleration";
		case packet_id_external_position_velocity: return "External Position Velocity";
		case packet_id_external_position: return "External Position";
		case packet_id_external_velocity: return "External Velocity";
		case packet_id_external_body_velocity: return "External Body Velocity";
		case packet_id_external_heading: return "External Heading";
		case packet_id_running_time: return "Running Time";
		case packet_id_local_magnetics: return "Local Magnetics";
		case packet_id_odometer_state: return "Odometer State";
		case packet_id_external_time: return "External Time";
		case packet_id_external_depth: return "External Depth";
		case packet_id_geoid_height: return "Geoid Height";
		case packet_id_rtcm_corrections: return "RTCM Corrections";
		case packet_id_wind: return "Wind";
		case packet_id_heave: return "Heave";
		case packet_id_raw_satellite_data: return "Raw Satellite Data";
		case packet_id_raw_satellite_ephemeris: return "Raw Satellite Ephemeris";
		case packet_id_external_odometer: return "External Odometer";
		case packet_id_external_air_data: return "External Air Data";
		case packet_id_gnss_receiver_information: return "GNSS Receiver Information";
		case packet_id_raw_dvl_data: return "Raw DVL Data";
		case packet_id_north_seeking_status: return "North Seeking Status";
		case packet_id_gimbal_state: return "Gimbal State";
		case packet_id_automotive: return "Automotive";
		case packet_id_external_magnetometers: return "External Magnetometers";
		case packet_id_basestation: return "Basestation";
		case packet_id_zero_angular_velocity: return "Zero Angular Velocity";
		case packet_id_extended_satellites: return "Extended Satellites";
		case packet_id_sensor_temperatures: return "Sensor Temperatures";
		case packet_id_system_temperature: return "System Temperature";
		case packet_id_quantum_sensor: return "Quantum Sensor";
		case packet_id_vessel_motion: return "Vessel Motion";
		case packet_id_gnss_position_velocity_time: return "GNSS Position Velocity Time";
		case packet_id_gnss_orientation: return "GNSS Orientation";

		case packet_id_packet_timer_period: return "Packet Timer Period";
		case packet_id_packet_periods: return "Packet Periods";
		case packet_id_baud_rates: return "Baud Rates";
		case packet_id_sensor_ranges: return "Sensor Ranges";
		case packet_id_installation_alignment: return "Installation Alignment";
		case packet_id_filter_options: return "Filter Options";
		case packet_id_gpio_configuration: return "GPIO Configuration";
		case packet_id_magnetic_calibration_values: return "Magnetic Calibration Values";
		case packet_id_magnetic_calibration_configuration: return "Magnetic Calibration Configuration";
		case packet_id_magnetic_calibration_status: return "Magnetic Calibration Status";
		case packet_id_odometer_configuration: return "Odometer Configuration";
		case packet_id_zero_alignment: return "Zero Alignment";
		case packet_id_reference_offsets: return "Reference Offsets";
		case packet_id_gpio_output_configuration: return "GPIO Output Configuration";
		case packet_id_dual_antenna_configuration: return "Dual Antenna Configuration";
		case packet_id_gnss_configuration: return "GNSS Configuration";
		case packet_id_user_data: return "User Data";
		case packet_id_gpio_input_configuration: return "GPIO Input Configuration";
		case packet_id_ip_dataports_configuration: return "IP Dataports Configuration";
		case packet_id_can_configuration: return "CAN Configuration";

		default:
			return std::nullopt;
	}
}