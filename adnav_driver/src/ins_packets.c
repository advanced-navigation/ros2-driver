/******************************************************************************/
/*                                                                            */
/*                Advanced Navigation Packet Protocol Library                 */
/*                 C Language Dynamic Boreas SDK, Version 6.0                 */
/*                    Copyright 2023, Advanced Navigation                     */
/*                                                                            */
/******************************************************************************/
/*
 * Copyright (C) 2023 Advanced Navigation
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

#include <string.h>
#include "an_packet_protocol.h"
#include "ins_packets.h"

/*
 * This file contains functions to decode and encode packets
 *
 * Decode functions take an an_packet_t and turn it into a type specific
 * to that packet so that the fields can be conveniently accessed. Decode
 * functions return 0 for success and 1 for failure. Decode functions are
 * used when receiving packets.
 *
 * Example decode
 *
 * an_packet_t an_packet
 * acknowledge_packet_t acknowledge_packet
 * ...
 * decode_acknowledge_packet(&acknowledge_packet, &an_packet);
 * printf("acknowledge id %d with result %d\n", acknowledge_packet.packet_id, acknowledge_packet.acknowledge_result);
 *
 * Encode functions take a type specific structure and turn it into an
 * an_packet_t. Encode functions are used when sending packets. Don't
 * forget to free the returned packet with an_packet_free().
 *
 * Example encode
 *
 * an_packet_t* an_packet;
 * boot_mode_packet_t boot_mode_packet;
 * ...
 * boot_mode_packet.boot_mode = boot_mode_bootloader;
 * an_packet = encode_boot_mode_packet(&boot_mode_packet);
 * serial_port_transmit(an_packet_pointer(an_packet), an_packet_size(an_packet));
 * an_packet_free(&an_packet);
 *
 */

int decode_acknowledge_packet(acknowledge_packet_t* acknowledge_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_acknowledge && an_packet->length == 4)
	{
		acknowledge_packet->packet_id = an_packet->data[0];
		memcpy(&acknowledge_packet->packet_crc, &an_packet->data[1], sizeof(uint16_t));
		acknowledge_packet->acknowledge_result = an_packet->data[3];
		return 0;
	}
	else return 1;
}

an_packet_t* encode_request_packet(uint8_t requested_packet_id)
{
	an_packet_t* an_packet = an_packet_allocate(1, packet_id_request);
	an_packet->data[0] = requested_packet_id;
	return an_packet;
}

int decode_boot_mode_packet(boot_mode_packet_t* boot_mode_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_boot_mode && an_packet->length == 1)
	{
		boot_mode_packet->boot_mode = an_packet->data[0];
		return 0;
	}
	else return 1;
}

an_packet_t* encode_boot_mode_packet(boot_mode_packet_t* boot_mode_packet)
{
	an_packet_t* an_packet = an_packet_allocate(1, packet_id_boot_mode);
	an_packet->data[0] = boot_mode_packet->boot_mode;
	return an_packet;
}

int decode_device_information_packet(device_information_packet_t* device_information_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_device_information && an_packet->length == 24)
	{
		memcpy(&device_information_packet->software_version, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&device_information_packet->device_id, &an_packet->data[4], sizeof(uint32_t));
		memcpy(&device_information_packet->hardware_revision, &an_packet->data[8], sizeof(uint32_t));
		memcpy(&device_information_packet->serial_number[0], &an_packet->data[12], 3 * sizeof(uint32_t));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_restore_factory_settings_packet()
{
	uint32_t verification = 0x85429E1C;
	an_packet_t* an_packet = an_packet_allocate(4, packet_id_restore_factory_settings);
	memcpy(&an_packet->data[0], &verification, sizeof(uint32_t));
	return an_packet;
}

an_packet_t* encode_reset_packet()
{
	uint32_t verification = 0x21057A7E;
	an_packet_t* an_packet = an_packet_allocate(4, packet_id_reset);
	memcpy(&an_packet->data[0], &verification, sizeof(uint32_t));
	return an_packet;
}

an_packet_t* encode_file_transfer_request_packet(file_transfer_first_packet_t* file_transfer_first_packet, int metadata_size, int data_size)
{
	an_packet_t* an_packet = an_packet_allocate(3 * sizeof(uint32_t) + 2 * sizeof(uint8_t) + metadata_size + data_size, packet_id_file_transfer_request);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &file_transfer_first_packet->unique_id, sizeof(uint32_t));
		memcpy(&an_packet->data[4], &file_transfer_first_packet->data_index, sizeof(uint32_t));
		memcpy(&an_packet->data[8], &file_transfer_first_packet->total_size, sizeof(uint32_t));
		an_packet->data[12] = file_transfer_first_packet->data_encoding;
		an_packet->data[13] = file_transfer_first_packet->metadata_type;
		memcpy(&an_packet->data[14], &file_transfer_first_packet->metadata, metadata_size);
		memcpy(&an_packet->data[14 + metadata_size], &file_transfer_first_packet->packet_data, data_size);
	}
	return an_packet;
}

int decode_file_transfer_acknowledge_packet(file_transfer_acknowledge_packet_t* file_transfer_acknowledge_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_file_transfer_acknowledge && an_packet->length == 9)
	{
		memcpy(&file_transfer_acknowledge_packet->unique_id, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&file_transfer_acknowledge_packet->data_index, &an_packet->data[4], sizeof(uint32_t));
		file_transfer_acknowledge_packet->response_code = an_packet->data[8];
		return 0;
	}
	else return 1;
}

an_packet_t* encode_file_transfer_packet(file_transfer_ongoing_packet_t* file_transfer_ongoing_packet, int data_size)
{
	an_packet_t* an_packet = an_packet_allocate(2 * sizeof(uint32_t) + data_size, packet_id_file_transfer_request);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &file_transfer_ongoing_packet->unique_id, sizeof(uint32_t));
		memcpy(&an_packet->data[4], &file_transfer_ongoing_packet->data_index, sizeof(uint32_t));
		memcpy(&an_packet->data[8], &file_transfer_ongoing_packet->packet_data, data_size);
	}
	return an_packet;
}

int decode_serial_port_passthrough_packet(serial_port_passthrough_packet_t* serial_port_passthrough_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_serial_port_passthrough && an_packet->length <= 255)
	{
		serial_port_passthrough_packet->passthrough_route = an_packet->data[0];
		memcpy(&serial_port_passthrough_packet->passthrough_data, &an_packet->data[1], an_packet->length - sizeof(uint8_t));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_serial_port_passthrough_packet(serial_port_passthrough_packet_t* serial_port_passthrough_packet, int data_size)
{
	an_packet_t* an_packet = an_packet_allocate(sizeof(uint8_t) + data_size, packet_id_serial_port_passthrough);
	if(an_packet != NULL)
	{
		an_packet->data[0] = serial_port_passthrough_packet->passthrough_route;
		memcpy(&an_packet->data[1], &serial_port_passthrough_packet->passthrough_data, data_size);
	}
	return an_packet;
}

int decode_ip_configuration_packet(ip_configuration_packet_t* ip_configuration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_ip_configuration && an_packet->length == 30)
	{
		ip_configuration_packet->permanent = an_packet->data[0];
		memcpy(&ip_configuration_packet->dhcp_mode, &an_packet->data[1], sizeof(uint8_t));
		memcpy(&ip_configuration_packet->ip_address, &an_packet->data[2], sizeof(uint32_t));
		memcpy(&ip_configuration_packet->ip_netmask, &an_packet->data[6], sizeof(uint32_t));
		memcpy(&ip_configuration_packet->ip_gateway, &an_packet->data[10], sizeof(uint32_t));
		memcpy(&ip_configuration_packet->dns_server, &an_packet->data[14], sizeof(uint32_t));
		memcpy(&ip_configuration_packet->serial_number[0], &an_packet->data[18], 3 * sizeof(uint32_t));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_ip_configuration_packet(ip_configuration_packet_t* ip_configuration_packet)
{
	an_packet_t* an_packet = an_packet_allocate(30, packet_id_ip_configuration);
	if(an_packet != NULL)
	{
		an_packet->data[0] = ip_configuration_packet->permanent;
		memcpy(&an_packet->data[1], &ip_configuration_packet->dhcp_mode, sizeof(uint8_t));
		memcpy(&an_packet->data[2], &ip_configuration_packet->ip_address, sizeof(uint32_t));
		memcpy(&an_packet->data[6], &ip_configuration_packet->ip_netmask, sizeof(uint32_t));
		memcpy(&an_packet->data[10], &ip_configuration_packet->ip_gateway, sizeof(uint32_t));
		memcpy(&an_packet->data[14], &ip_configuration_packet->dns_server, sizeof(uint32_t));
		memcpy(&an_packet->data[18], &ip_configuration_packet->serial_number[0], 3 * sizeof(uint32_t));
	}
	return an_packet;
}

int decode_subcomponent_information_packet(subcomponent_information_packet_t* subcomponent_information_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_subcomponent_information && an_packet->length % 24 == 0)
	{
		int number_of_subcomponents = an_packet->length / 24;
		int i;
		for(i = 0; i < MAXIMUM_SUBCOMPONENTS; i++)
		{
			if (i < number_of_subcomponents)
			{
				memcpy(&subcomponent_information_packet->subcomponent_information[i].software_version, &an_packet->data[0], sizeof(uint32_t));
				memcpy(&subcomponent_information_packet->subcomponent_information[i].device_id, &an_packet->data[4], sizeof(uint32_t));
				memcpy(&subcomponent_information_packet->subcomponent_information[i].hardware_revision, &an_packet->data[8], sizeof(uint32_t));
				memcpy(&subcomponent_information_packet->subcomponent_information[i].serial_number[0], &an_packet->data[12], 3 * sizeof(uint32_t));
			}
			else memset(&subcomponent_information_packet->subcomponent_information[i], 0, sizeof(subcomponent_information_t));
		}
		return 0;
	}
	else return 1;
}

int decode_system_state_packet(system_state_packet_t* system_state_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_system_state && an_packet->length == 100)
	{
		memcpy(&system_state_packet->system_status, &an_packet->data[0], sizeof(uint16_t));
		memcpy(&system_state_packet->filter_status, &an_packet->data[2], sizeof(uint16_t));
		memcpy(&system_state_packet->unix_time_seconds, &an_packet->data[4], sizeof(uint32_t));
		memcpy(&system_state_packet->microseconds, &an_packet->data[8], sizeof(uint32_t));
		memcpy(&system_state_packet->latitude, &an_packet->data[12], sizeof(double));
		memcpy(&system_state_packet->longitude, &an_packet->data[20], sizeof(double));
		memcpy(&system_state_packet->height, &an_packet->data[28], sizeof(double));
		memcpy(&system_state_packet->velocity[0], &an_packet->data[36], 3 * sizeof(float));
		memcpy(&system_state_packet->body_acceleration[0], &an_packet->data[48], 3 * sizeof(float));
		memcpy(&system_state_packet->g_force, &an_packet->data[60], sizeof(float));
		memcpy(&system_state_packet->orientation[0], &an_packet->data[64], 3 * sizeof(float));
		memcpy(&system_state_packet->angular_velocity[0], &an_packet->data[76], 3 * sizeof(float));
		memcpy(&system_state_packet->standard_deviation[0], &an_packet->data[88], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_unix_time_packet(unix_time_packet_t* unix_time_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_unix_time && an_packet->length == 8)
	{
		memcpy(&unix_time_packet->unix_time_seconds, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&unix_time_packet->microseconds, &an_packet->data[4], sizeof(uint32_t));
		return 0;
	}
	else return 1;
}

int decode_formatted_time_packet(formatted_time_packet_t* formatted_time_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_formatted_time && an_packet->length == 14)
	{
		memcpy(&formatted_time_packet->microseconds, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&formatted_time_packet->year, &an_packet->data[4], sizeof(uint16_t));
		memcpy(&formatted_time_packet->year_day, &an_packet->data[6], sizeof(uint16_t));
		formatted_time_packet->month = an_packet->data[8];
		formatted_time_packet->month_day = an_packet->data[9];
		formatted_time_packet->week_day = an_packet->data[10];
		formatted_time_packet->hour = an_packet->data[11];
		formatted_time_packet->minute = an_packet->data[12];
		formatted_time_packet->second = an_packet->data[13];
		return 0;
	}
	else return 1;
}

int decode_status_packet(status_packet_t* status_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_status && an_packet->length == 4)
	{
		memcpy(&status_packet->system_status, &an_packet->data[0], sizeof(uint16_t));
		memcpy(&status_packet->filter_status, &an_packet->data[2], sizeof(uint16_t));
		return 0;
	}
	else return 1;
}

int decode_position_standard_deviation_packet(position_standard_deviation_packet_t* position_standard_deviation_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_position_standard_deviation && an_packet->length == 12)
	{
		memcpy(&position_standard_deviation_packet->standard_deviation[0], &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_velocity_standard_deviation_packet(velocity_standard_deviation_packet_t* velocity_standard_deviation_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_velocity_standard_deviation && an_packet->length == 12)
	{
		memcpy(&velocity_standard_deviation_packet->standard_deviation[0], &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_euler_orientation_standard_deviation_packet(euler_orientation_standard_deviation_packet_t* euler_orientation_standard_deviation, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_euler_orientation_standard_deviation && an_packet->length == 12)
	{
		memcpy(&euler_orientation_standard_deviation->standard_deviation[0], &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_quaternion_orientation_standard_deviation_packet(quaternion_orientation_standard_deviation_packet_t* quaternion_orientation_standard_deviation_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_quaternion_orientation_standard_deviation && an_packet->length == 16)
	{
		memcpy(&quaternion_orientation_standard_deviation_packet->standard_deviation[0], &an_packet->data[0], 4 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_raw_sensors_packet(raw_sensors_packet_t* raw_sensors_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_raw_sensors && an_packet->length == 48)
	{
		memcpy(&raw_sensors_packet->accelerometers[0], &an_packet->data[0], 3 * sizeof(float));
		memcpy(&raw_sensors_packet->gyroscopes[0], &an_packet->data[12], 3 * sizeof(float));
		memcpy(&raw_sensors_packet->magnetometers[0], &an_packet->data[24], 3 * sizeof(float));
		memcpy(&raw_sensors_packet->imu_temperature, &an_packet->data[36], sizeof(float));
		memcpy(&raw_sensors_packet->pressure, &an_packet->data[40], sizeof(float));
		memcpy(&raw_sensors_packet->pressure_temperature, &an_packet->data[44], sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_raw_gnss_packet(raw_gnss_packet_t* raw_gnss_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_raw_gnss && an_packet->length == 74)
	{
		memcpy(&raw_gnss_packet->unix_time_seconds, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&raw_gnss_packet->microseconds, &an_packet->data[4], sizeof(uint32_t));
		memcpy(&raw_gnss_packet->position[0], &an_packet->data[8], 3 * sizeof(double));
		memcpy(&raw_gnss_packet->velocity[0], &an_packet->data[32], 3 * sizeof(float));
		memcpy(&raw_gnss_packet->position_standard_deviation[0], &an_packet->data[44], 3 * sizeof(float));
		memcpy(&raw_gnss_packet->tilt, &an_packet->data[56], sizeof(float));
		memcpy(&raw_gnss_packet->heading, &an_packet->data[60], sizeof(float));
		memcpy(&raw_gnss_packet->tilt_standard_deviation, &an_packet->data[64], sizeof(float));
		memcpy(&raw_gnss_packet->heading_standard_deviation, &an_packet->data[68], sizeof(float));
		memcpy(&raw_gnss_packet->flags.r, &an_packet->data[72], sizeof(uint16_t));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_raw_gnss_packet(raw_gnss_packet_t* raw_gnss_packet)
{
	an_packet_t* an_packet = an_packet_allocate(74, packet_id_raw_gnss);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &raw_gnss_packet->unix_time_seconds, sizeof(uint32_t));
		memcpy(&an_packet->data[4], &raw_gnss_packet->microseconds, sizeof(uint32_t));
		memcpy(&an_packet->data[8], &raw_gnss_packet->position[0], 3 * sizeof(double));
		memcpy(&an_packet->data[32], &raw_gnss_packet->velocity[0], 3 * sizeof(float));
		memcpy(&an_packet->data[44], &raw_gnss_packet->position_standard_deviation[0], 3 * sizeof(float));
		memcpy(&an_packet->data[56], &raw_gnss_packet->tilt, sizeof(float));
		memcpy(&an_packet->data[60], &raw_gnss_packet->heading, sizeof(float));
		memcpy(&an_packet->data[64], &raw_gnss_packet->tilt_standard_deviation, sizeof(float));
		memcpy(&an_packet->data[68], &raw_gnss_packet->heading_standard_deviation, sizeof(float));
		memcpy(&an_packet->data[72], &raw_gnss_packet->flags.r, sizeof(uint16_t));
	}
	return an_packet;
}

int decode_satellites_packet(satellites_packet_t* satellites_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_satellites && an_packet->length == 13)
	{
		memcpy(&satellites_packet->hdop, &an_packet->data[0], sizeof(float));
		memcpy(&satellites_packet->vdop, &an_packet->data[4], sizeof(float));
		memcpy(&satellites_packet->gps_satellites, &an_packet->data[8], 5 * sizeof(uint8_t));
		return 0;
	}
	else return 1;
}

int decode_geodetic_position_packet(geodetic_position_packet_t* geodetic_position_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_geodetic_position && an_packet->length == 24)
	{
		memcpy(&geodetic_position_packet->position[0], &an_packet->data[0], 3 * sizeof(double));
		return 0;
	}
	else return 1;
}

int decode_ecef_position_packet(ecef_position_packet_t* ecef_position_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_ecef_position && an_packet->length == 24)
	{
		memcpy(&ecef_position_packet->position[0], &an_packet->data[0], 3 * sizeof(double));
		return 0;
	}
	else return 1;
}

int decode_utm_position_packet(utm_position_packet_t* utm_position_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_utm_position && an_packet->length == 26)
	{
		memcpy(&utm_position_packet->position, &an_packet->data[0], 3 * sizeof(double));
		utm_position_packet->zone_number = an_packet->data[24];
		utm_position_packet->zone_char = an_packet->data[25];
		return 0;
	}
	else return 1;
}

int decode_ned_velocity_packet(ned_velocity_packet_t* ned_velocity_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_ned_velocity && an_packet->length == 12)
	{
		memcpy(&ned_velocity_packet->velocity, &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_body_velocity_packet(body_velocity_packet_t* body_velocity_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_body_velocity && an_packet->length == 12)
	{
		memcpy(&body_velocity_packet->velocity, &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_acceleration_packet(acceleration_packet_t* acceleration, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_acceleration && an_packet->length == 12)
	{
		memcpy(&acceleration->acceleration[0], &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_body_acceleration_packet(body_acceleration_packet_t* body_acceleration, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_body_acceleration && an_packet->length == 16)
	{
		memcpy(&body_acceleration->acceleration[0], &an_packet->data[0], 3 * sizeof(float));
		memcpy(&body_acceleration->g_force, &an_packet->data[12], sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_euler_orientation_packet(euler_orientation_packet_t* euler_orientation_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_euler_orientation && an_packet->length == 12)
	{
		memcpy(&euler_orientation_packet->orientation[0], &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_quaternion_orientation_packet(quaternion_orientation_packet_t* quaternion_orientation_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_quaternion_orientation && an_packet->length == 16)
	{
		memcpy(&quaternion_orientation_packet->orientation[0], &an_packet->data[0], 4 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_dcm_orientation_packet(dcm_orientation_packet_t* dcm_orientation_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_dcm_orientation && an_packet->length == 36)
	{
		memcpy(&dcm_orientation_packet->orientation[0][0], &an_packet->data[0], 9 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_angular_velocity_packet(angular_velocity_packet_t* angular_velocity_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_angular_velocity && an_packet->length == 12)
	{
		memcpy(&angular_velocity_packet->angular_velocity[0], &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_angular_acceleration_packet(angular_acceleration_packet_t* angular_acceleration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_angular_acceleration && an_packet->length == 12)
	{
		memcpy(&angular_acceleration_packet->angular_acceleration[0], &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_external_position_velocity_packet(external_position_velocity_packet_t* external_position_velocity_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_position_velocity && an_packet->length == 60)
	{
		memcpy(&external_position_velocity_packet->position[0], &an_packet->data[0], 3 * sizeof(double));
		memcpy(&external_position_velocity_packet->velocity[0], &an_packet->data[24], 3 * sizeof(float));
		memcpy(&external_position_velocity_packet->position_standard_deviation[0], &an_packet->data[36], 3 * sizeof(float));
		memcpy(&external_position_velocity_packet->velocity_standard_deviation[0], &an_packet->data[48], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_external_position_velocity_packet(external_position_velocity_packet_t* external_position_velocity_packet)
{
	an_packet_t* an_packet = an_packet_allocate(60, packet_id_external_position_velocity);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &external_position_velocity_packet->position[0], 3 * sizeof(double));
		memcpy(&an_packet->data[24], &external_position_velocity_packet->velocity[0], 3 * sizeof(float));
		memcpy(&an_packet->data[36], &external_position_velocity_packet->position_standard_deviation[0], 3 * sizeof(float));
		memcpy(&an_packet->data[48], &external_position_velocity_packet->velocity_standard_deviation[0], 3 * sizeof(float));
	}
	return an_packet;
}

int decode_external_position_packet(external_position_packet_t* external_position_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_position && an_packet->length == 36)
	{
		memcpy(&external_position_packet->position[0], &an_packet->data[0], 3 * sizeof(double));
		memcpy(&external_position_packet->standard_deviation[0], &an_packet->data[24], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_external_position_packet(external_position_packet_t* external_position_packet)
{
	an_packet_t* an_packet = an_packet_allocate(36, packet_id_external_position);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &external_position_packet->position[0], 3 * sizeof(double));
		memcpy(&an_packet->data[24], &external_position_packet->standard_deviation[0], 3 * sizeof(float));
	}
	return an_packet;
}

int decode_external_velocity_packet(external_velocity_packet_t* external_velocity_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_velocity && an_packet->length == 24)
	{
		memcpy(&external_velocity_packet->velocity[0], &an_packet->data[0], 3 * sizeof(float));
		memcpy(&external_velocity_packet->standard_deviation[0], &an_packet->data[12], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_external_velocity_packet(external_velocity_packet_t* external_velocity_packet)
{
	an_packet_t* an_packet = an_packet_allocate(24, packet_id_external_velocity);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &external_velocity_packet->velocity[0], 3 * sizeof(float));
		memcpy(&an_packet->data[12], &external_velocity_packet->standard_deviation[0], 3 * sizeof(float));
	}
	return an_packet;
}

int decode_external_body_velocity_packet(external_body_velocity_packet_t* external_body_velocity_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_body_velocity && an_packet->length == 16)
	{
		memcpy(&external_body_velocity_packet->velocity, &an_packet->data[0], 3 * sizeof(float));
		memcpy(&external_body_velocity_packet->standard_deviation, &an_packet->data[12], sizeof(float));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_external_body_velocity_packet(external_body_velocity_packet_t* external_body_velocity_packet)
{
	an_packet_t* an_packet = an_packet_allocate(16, packet_id_external_body_velocity);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &external_body_velocity_packet->velocity[0], 3 * sizeof(float));
		memcpy(&an_packet->data[12], &external_body_velocity_packet->standard_deviation, sizeof(float));
	}
	return an_packet;
}

int decode_external_heading_packet(external_heading_packet_t* external_heading_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_heading && an_packet->length == 8)
	{
		memcpy(&external_heading_packet->heading, &an_packet->data[0], sizeof(float));
		memcpy(&external_heading_packet->standard_deviation, &an_packet->data[4], sizeof(float));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_external_heading_packet(external_heading_packet_t* external_heading_packet)
{
	an_packet_t* an_packet = an_packet_allocate(8, packet_id_external_heading);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &external_heading_packet->heading, sizeof(float));
		memcpy(&an_packet->data[4], &external_heading_packet->standard_deviation, sizeof(float));
	}
	return an_packet;
}

int decode_running_time_packet(running_time_packet_t* running_time_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_running_time && an_packet->length == 8)
	{
		memcpy(&running_time_packet->seconds, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&running_time_packet->microseconds, &an_packet->data[4], sizeof(uint32_t));
		return 0;
	}
	else return 1;
}

int decode_local_magnetics_packet(local_magnetics_packet_t* local_magnetics_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_local_magnetics && an_packet->length == 12)
	{
		memcpy(&local_magnetics_packet->magnetic_field[0], &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_odometer_state_packet(odometer_state_packet_t* odometer_state_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_odometer_state && an_packet->length == 20)
	{
		memcpy(&odometer_state_packet->pulse_count, &an_packet->data[0], sizeof(int32_t));
		memcpy(&odometer_state_packet->distance, &an_packet->data[4], sizeof(float));
		memcpy(&odometer_state_packet->speed, &an_packet->data[8], sizeof(float));
		memcpy(&odometer_state_packet->slip, &an_packet->data[12], sizeof(float));
		odometer_state_packet->active = an_packet->data[16];
		return 0;
	}
	else return 1;
}

int decode_external_time_packet(external_time_packet_t* external_time_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_time && an_packet->length == 8)
	{
		memcpy(&external_time_packet->unix_time_seconds, &an_packet->data[0], sizeof(float));
		memcpy(&external_time_packet->microseconds, &an_packet->data[4], sizeof(float));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_external_time_packet(external_time_packet_t* external_time_packet)
{
	an_packet_t* an_packet = an_packet_allocate(8, packet_id_external_time);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &external_time_packet->unix_time_seconds, sizeof(float));
		memcpy(&an_packet->data[4], &external_time_packet->microseconds, sizeof(float));
	}
	return an_packet;
}

int decode_external_depth_packet(external_depth_packet_t* external_depth_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_depth && an_packet->length == 8)
	{
		memcpy(&external_depth_packet->depth, &an_packet->data[0], sizeof(float));
		memcpy(&external_depth_packet->standard_deviation, &an_packet->data[4], sizeof(float));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_external_depth_packet(external_depth_packet_t* external_depth_packet)
{
	an_packet_t* an_packet = an_packet_allocate(8, packet_id_external_depth);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &external_depth_packet->depth, sizeof(float));
		memcpy(&an_packet->data[4], &external_depth_packet->standard_deviation, sizeof(float));
	}
	return an_packet;
}

int decode_geoid_height_packet(geoid_height_packet_t* geoid_height_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_geoid_height && an_packet->length == 4)
	{
		memcpy(&geoid_height_packet->geoid_height, &an_packet->data[0], sizeof(float));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_rtcm_corrections_packet(rtcm_corrections_packet_t* rtcm_corrections_packet, int data_size)
{
	an_packet_t* an_packet = an_packet_allocate(data_size, packet_id_rtcm_corrections);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], rtcm_corrections_packet->packet_data, data_size);
	}
	return an_packet;
}

int decode_wind_packet(wind_packet_t* wind_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_wind && an_packet->length == 12)
	{
		memcpy(&wind_packet->wind_velocity[0], &an_packet->data[0], 2 * sizeof(float));
		memcpy(&wind_packet->wind_standard_deviation, &an_packet->data[8], sizeof(float));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_wind_packet(wind_packet_t* wind_packet)
{
	an_packet_t* an_packet = an_packet_allocate(12, packet_id_wind);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &wind_packet->wind_velocity[0], 2 * sizeof(float));
		memcpy(&an_packet->data[8], &wind_packet->wind_standard_deviation, sizeof(float));
	}
	return an_packet;
}

int decode_heave_packet(heave_packet_t* heave_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_heave && an_packet->length == 16)
	{
		memcpy(&heave_packet->heave_point_1, &an_packet->data[0], sizeof(float));
		memcpy(&heave_packet->heave_point_2, &an_packet->data[4], sizeof(float));
		memcpy(&heave_packet->heave_point_3, &an_packet->data[8], sizeof(float));
		memcpy(&heave_packet->heave_point_4, &an_packet->data[12], sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_raw_satellite_ephemeris_packet(raw_satellite_ephemeris_packet_t* raw_satellite_ephemeris_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_raw_satellite_ephemeris)
	{
		uint8_t satellite_system = an_packet->data[4];
		if(an_packet->length == 132 && satellite_system == satellite_system_gps)
		{
			memcpy(&raw_satellite_ephemeris_packet->gps.unix_time, &an_packet->data[0], sizeof(uint32_t));
			raw_satellite_ephemeris_packet->gps.satellite_system = an_packet->data[4];
			raw_satellite_ephemeris_packet->gps.satellite_number = an_packet->data[5];
			memcpy(&raw_satellite_ephemeris_packet->gps.time_of_ephemeris, &an_packet->data[6], sizeof(uint32_t));
			memcpy(&raw_satellite_ephemeris_packet->gps.issue_of_data_clock, &an_packet->data[10], sizeof(uint16_t));
			memcpy(&raw_satellite_ephemeris_packet->gps.issue_of_data_ephemeris, &an_packet->data[12], sizeof(uint16_t));
			memcpy(&raw_satellite_ephemeris_packet->gps.satellite_clock_bias, &an_packet->data[14], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.satellite_clock_drift, &an_packet->data[18], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.satellite_clock_drift_rate, &an_packet->data[22], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.crs, &an_packet->data[26], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.deltaN, &an_packet->data[30], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.m0, &an_packet->data[34], sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps.cuc, &an_packet->data[42], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.eccentricity, &an_packet->data[46], sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps.cus, &an_packet->data[54], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.sqrtA, &an_packet->data[58], sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps.cic, &an_packet->data[66], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.omega0, &an_packet->data[70], sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps.cis, &an_packet->data[78], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.i0, &an_packet->data[82], sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps.crc, &an_packet->data[90], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.omega, &an_packet->data[94], sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps.omega_dot, &an_packet->data[102], sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps.i_dot, &an_packet->data[110], sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps.tgd, &an_packet->data[118], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.ephemeris_week_number, &an_packet->data[122], sizeof(uint16_t));
			memcpy(&raw_satellite_ephemeris_packet->gps.transmission_time, &an_packet->data[124], sizeof(uint32_t));
			memcpy(&raw_satellite_ephemeris_packet->gps.user_range_accuracy, &an_packet->data[128], sizeof(uint16_t));
			memcpy(&raw_satellite_ephemeris_packet->gps.flags, &an_packet->data[130], sizeof(uint16_t));
			return 0;
		}
		else if(an_packet->length == 94 && satellite_system == satellite_system_glonass)
		{
			memcpy(&raw_satellite_ephemeris_packet->glo.unix_time, &an_packet->data[0], sizeof(uint32_t));
			raw_satellite_ephemeris_packet->glo.satellite_system = an_packet->data[4];
			raw_satellite_ephemeris_packet->glo.satellite_number = an_packet->data[5];
			memcpy(&raw_satellite_ephemeris_packet->glo.clock_bias, &an_packet->data[6], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->glo.frequency_bias, &an_packet->data[10], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->glo.position[0], &an_packet->data[14], 3 * sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->glo.velocity[0], &an_packet->data[38], 3 * sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->glo.acceleration[0], &an_packet->data[62], 3 * sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->glo.frame_start_time, &an_packet->data[86], sizeof(uint32_t));
			raw_satellite_ephemeris_packet->glo.age = an_packet->data[90];
			raw_satellite_ephemeris_packet->glo.frequency_slot = an_packet->data[91];
			raw_satellite_ephemeris_packet->glo.satellite_health = an_packet->data[92];
			return 0;
		}
		else if(an_packet->length == 92 && satellite_system == satellite_system_gps)
		{
			memcpy(&raw_satellite_ephemeris_packet->gps_iono.unix_time, &an_packet->data[0], sizeof(uint32_t));
			raw_satellite_ephemeris_packet->gps_iono.satellite_system = an_packet->data[4];
			raw_satellite_ephemeris_packet->gps_iono.satellite_number = an_packet->data[5];
			memcpy(&raw_satellite_ephemeris_packet->gps_iono.alpha[0], &an_packet->data[6], 4 * sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps_iono.beta[0], &an_packet->data[38], 4 * sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps_iono.asub[0], &an_packet->data[70], 2 * sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps_iono.delta_ls, &an_packet->data[86], sizeof(int16_t));
			memcpy(&raw_satellite_ephemeris_packet->gps_iono.delta_tlsf, &an_packet->data[88], sizeof(int16_t));
			raw_satellite_ephemeris_packet->gps_iono.wnsub_lsf = an_packet->data[90];
			raw_satellite_ephemeris_packet->gps_iono.dn = an_packet->data[91];
			return 0;
		}
		else return 1;
	}
	else return 1;
}

int decode_external_odometer_packet(odometer_packet_t* external_odometer_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_odometer && an_packet->length == 13)
	{
		memcpy(&external_odometer_packet->delay, &an_packet->data[0], sizeof(float));
		memcpy(&external_odometer_packet->speed, &an_packet->data[4], sizeof(float));
		memcpy(&external_odometer_packet->distance_travelled, &an_packet->data[8], sizeof(float));
		external_odometer_packet->flags.r = an_packet->data[12];
		return 0;
	}
	else return 1;
}

an_packet_t* encode_external_odometer_packet(odometer_packet_t* external_odometer_packet)
{
	an_packet_t* an_packet = an_packet_allocate(13, packet_id_external_odometer);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &external_odometer_packet->delay, sizeof(float));
		memcpy(&an_packet->data[4], &external_odometer_packet->speed, sizeof(float));
		memcpy(&an_packet->data[8], &external_odometer_packet->distance_travelled, sizeof(float));
		an_packet->data[12] = external_odometer_packet->flags.r;
	}
	return an_packet;
}

int decode_external_air_data_packet(external_air_data_packet_t* external_air_data_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_air_data && an_packet->length == 25)
	{
		memcpy(&external_air_data_packet->barometric_altitude_delay, &an_packet->data[0], sizeof(float));
		memcpy(&external_air_data_packet->airspeed_delay, &an_packet->data[4], sizeof(float));
		memcpy(&external_air_data_packet->barometric_altitude, &an_packet->data[8], sizeof(float));
		memcpy(&external_air_data_packet->airspeed, &an_packet->data[12], sizeof(float));
		memcpy(&external_air_data_packet->barometric_altitude_standard_deviation, &an_packet->data[16], sizeof(float));
		memcpy(&external_air_data_packet->airspeed_standard_deviation, &an_packet->data[20], sizeof(float));
		external_air_data_packet->flags.r = an_packet->data[24];
		return 0;
	}
	else return 1;
}

an_packet_t* encode_external_air_data_packet(external_air_data_packet_t* external_air_data_packet)
{
	an_packet_t* an_packet = an_packet_allocate(25, packet_id_external_air_data);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &external_air_data_packet->barometric_altitude_delay, sizeof(float));
		memcpy(&an_packet->data[4], &external_air_data_packet->airspeed_delay, sizeof(float));
		memcpy(&an_packet->data[8], &external_air_data_packet->barometric_altitude, sizeof(float));
		memcpy(&an_packet->data[12], &external_air_data_packet->airspeed, sizeof(float));
		memcpy(&an_packet->data[16], &external_air_data_packet->barometric_altitude_standard_deviation, sizeof(float));
		memcpy(&an_packet->data[20], &external_air_data_packet->airspeed_standard_deviation, sizeof(float));
		an_packet->data[24] = external_air_data_packet->flags.r;
	}
	return an_packet;
}

int decode_gnss_information_packet(gnss_receiver_information_packet_t* gnss_information_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_gnss_receiver_information && an_packet->length == 48)
	{
		gnss_information_packet->gnss_manufacturer_id = an_packet->data[0];
		gnss_information_packet->gnss_receiver_model = an_packet->data[1];
		memcpy(gnss_information_packet->serial_number, &an_packet->data[2], 11 * sizeof(char));
		gnss_information_packet->serial_number[10] = '\0';
		memcpy(&gnss_information_packet->firmware_version, &an_packet->data[12], sizeof(uint32_t));
		memcpy(&gnss_information_packet->software_license, &an_packet->data[16], 3 * sizeof(uint32_t));
		memcpy(&gnss_information_packet->omnistar_serial_number, &an_packet->data[28], sizeof(uint32_t));
		memcpy(&gnss_information_packet->omnistar_subscription_start_unix_time, &an_packet->data[32], sizeof(uint32_t));
		memcpy(&gnss_information_packet->omnistar_subscription_expiry_unix_time, &an_packet->data[36], sizeof(uint32_t));
		gnss_information_packet->omnistar_engine_mode = an_packet->data[40];
		gnss_information_packet->rtk_accuracy = an_packet->data[41];
		return 0;
	}
	else return 1;
}

int decode_raw_dvl_data_packet(raw_dvl_data_packet_t* raw_dvl_data_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_raw_dvl_data && an_packet->length == 60)
	{
		memcpy(&raw_dvl_data_packet->unix_timestamp, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&raw_dvl_data_packet->microseconds, &an_packet->data[4], sizeof(uint32_t));
		memcpy(&raw_dvl_data_packet->status.r, &an_packet->data[8], sizeof(uint32_t));
		memcpy(&raw_dvl_data_packet->bottom_velocity[0], &an_packet->data[12], 3 * sizeof(float));
		memcpy(&raw_dvl_data_packet->bottom_velocity_standard_deviation, &an_packet->data[24], sizeof(float));
		memcpy(&raw_dvl_data_packet->water_velocity[0], &an_packet->data[28], 3 * sizeof(float));
		memcpy(&raw_dvl_data_packet->water_velocity_standard_deviation, &an_packet->data[40], sizeof(float));
		memcpy(&raw_dvl_data_packet->water_velocity_layer_depth, &an_packet->data[44], sizeof(float));
		memcpy(&raw_dvl_data_packet->depth, &an_packet->data[48], sizeof(float));
		memcpy(&raw_dvl_data_packet->altitude, &an_packet->data[52], sizeof(float));
		memcpy(&raw_dvl_data_packet->temperature, &an_packet->data[56], sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_north_seeking_status_packet(north_seeking_status_packet_t* north_seeking_status_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_north_seeking_status && an_packet->length == 28)
	{
		memcpy(&north_seeking_status_packet->north_seeking_status.r, &an_packet->data[0], sizeof(uint16_t));
		memcpy(&north_seeking_status_packet->quadrant_data_collection_progress, &an_packet->data[4], 4 * sizeof(uint8_t));
		memcpy(&north_seeking_status_packet->current_rotation_angle, &an_packet->data[8], sizeof(float));
		memcpy(&north_seeking_status_packet->current_gyroscope_bias_solution[0], &an_packet->data[12], 3 * sizeof(float));
		memcpy(&north_seeking_status_packet->current_gyroscope_bias_solution_error, &an_packet->data[24], sizeof(float));
		return 0;
	}
	else return 1;
}


int decode_gimbal_state_packet(gimbal_state_packet_t* gimbal_state_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_gimbal_state && an_packet->length == 8)
	{
		memcpy(&gimbal_state_packet->current_angle, &an_packet->data[0], sizeof(float));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_gimbal_state_packet(gimbal_state_packet_t* gimbal_state_packet)
{
	an_packet_t* an_packet = an_packet_allocate(8, packet_id_gimbal_state);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &gimbal_state_packet->current_angle, sizeof(float));
		memset(&an_packet->data[4], 0, 4 * sizeof(uint8_t));
	}
	return an_packet;
}

int decode_automotive_packet(automotive_packet_t* automotive_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_automotive && an_packet->length == 24)
	{
		memcpy(&automotive_packet->virtual_odometer_distance, &an_packet->data[0], sizeof(float));
		memcpy(&automotive_packet->slip_angle, &an_packet->data[4], sizeof(float));
		memcpy(&automotive_packet->velocity_x, &an_packet->data[8], sizeof(float));
		memcpy(&automotive_packet->velocity_y, &an_packet->data[12], sizeof(float));
		memcpy(&automotive_packet->distance_standard_deviation, &an_packet->data[16], sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_external_magnetometers_packet(external_magnetometers_packet_t* external_magnetometers_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_magnetometers && an_packet->length == 17)
	{
		memcpy(&external_magnetometers_packet->delay, &an_packet->data[0], sizeof(float));
		memcpy(&external_magnetometers_packet->magnetometer[0], &an_packet->data[4], 3 * sizeof(float));
		memcpy(&external_magnetometers_packet->flags, &an_packet->data[16], sizeof(uint8_t));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_external_magnetometers_packet(external_magnetometers_packet_t* external_magnetometers_packet)
{
	an_packet_t* an_packet = an_packet_allocate(17, packet_id_external_magnetometers);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &external_magnetometers_packet->delay, sizeof(float));
		memcpy(&an_packet->data[4], &external_magnetometers_packet->magnetometer[0], 3 * sizeof(float));
		memcpy(&an_packet->data[16], &external_magnetometers_packet->flags, sizeof(uint8_t));
	}
	return an_packet;
}

an_packet_t* encode_zero_angular_velocity_packet(zero_angular_velocity_packet_t* zero_angular_velocity_packet)
{
	an_packet_t* an_packet = an_packet_allocate(8, packet_id_zero_angular_velocity);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &zero_angular_velocity_packet->duration, sizeof(float));
	}
	return an_packet;
}

int decode_extended_satellites_packet(extended_satellites_packet_t* extended_satellites_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_extended_satellites && (an_packet->length - 2) % 9 == 0)
	{
		extended_satellites_packet->total_number_of_packets = an_packet->data[0];
		extended_satellites_packet->packet_number = an_packet->data[1];
		int number_of_satellites = (an_packet->length - 2) / 9;
		int i;
		for(i = 0; i < MAXIMUM_EXTENDED_SATELLITES; i++)
		{
			if(i < number_of_satellites)
			{
				extended_satellites_packet->extended_satellites[i].satellite_system = an_packet->data[9 * i + 2];
				extended_satellites_packet->extended_satellites[i].number = an_packet->data[9 * i + 3];
				extended_satellites_packet->extended_satellites[i].frequencies.r = an_packet->data[9 * i + 4];
				extended_satellites_packet->extended_satellites[i].elevation = an_packet->data[9 * i + 5];
				memcpy(&extended_satellites_packet->extended_satellites[i].azimuth, &an_packet->data[9 * i + 6], sizeof(uint16_t));
				extended_satellites_packet->extended_satellites[i].snr1 = an_packet->data[9 * i + 8];
				extended_satellites_packet->extended_satellites[i].snr2 = an_packet->data[9 * i + 9];
				memcpy(&extended_satellites_packet->extended_satellites[i].flags, &an_packet->data[9 * i + 10], sizeof(uint8_t));
			}
			else memset(&extended_satellites_packet->extended_satellites[i], 0, sizeof(extended_satellite_t));
		}
		return 0;
	}
	else return 1;
}

int decode_packet_timer_period_packet(packet_timer_period_packet_t* packet_timer_period_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_packet_timer_period && an_packet->length == 4)
	{
		packet_timer_period_packet->permanent = an_packet->data[0];
		packet_timer_period_packet->utc_synchronisation = an_packet->data[1];
		memcpy(&packet_timer_period_packet->packet_timer_period, &an_packet->data[2], sizeof(uint16_t));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_packet_timer_period_packet(packet_timer_period_packet_t* packet_timer_period_packet)
{
	an_packet_t* an_packet = an_packet_allocate(4, packet_id_packet_timer_period);
	if(an_packet != NULL)
	{
		an_packet->data[0] = packet_timer_period_packet->permanent > 0;
		an_packet->data[1] = packet_timer_period_packet->utc_synchronisation > 0;
		memcpy(&an_packet->data[2], &packet_timer_period_packet->packet_timer_period, sizeof(uint16_t));
	}
	return an_packet;
}

int decode_packet_periods_packet(packet_periods_packet_t* packet_periods_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_packet_periods && (an_packet->length - 2) % 5 == 0)
	{
		int i;
		int packet_periods_count = (an_packet->length - 2) / 5;
		packet_periods_packet->permanent = an_packet->data[0];
		packet_periods_packet->clear_existing_packets = an_packet->data[1];
		for(i = 0; i < MAXIMUM_PACKET_PERIODS; i++)
		{
			if(i < packet_periods_count)
			{
				packet_periods_packet->packet_periods[i].packet_id = an_packet->data[2 + 5 * i];
				memcpy(&packet_periods_packet->packet_periods[i].period, &an_packet->data[2 + 5 * i + 1], sizeof(uint32_t));
			}
			else memset(&packet_periods_packet->packet_periods[i], 0, sizeof(packet_period_t));
		}
		return 0;
	}
	else return 1;
}

an_packet_t* encode_packet_periods_packet(packet_periods_packet_t* packet_periods_packet)
{
	int i;
	an_packet_t* an_packet = an_packet_allocate(252, packet_id_packet_periods);
	if(an_packet != NULL)
	{
		an_packet->data[0] = packet_periods_packet->permanent > 0;
		an_packet->data[1] = packet_periods_packet->clear_existing_packets;
		for(i = 0; i < MAXIMUM_PACKET_PERIODS; i++)
		{
			if(packet_periods_packet->packet_periods[i].packet_id)
			{
				an_packet->data[2 + 5 * i] = packet_periods_packet->packet_periods[i].packet_id;
				memcpy(&an_packet->data[2 + 5 * i + 1], &packet_periods_packet->packet_periods[i].period, sizeof(uint32_t));
			}
			else break;
		}
		an_packet->length = 2 + 5 * i;
	}
	return an_packet;
}

int decode_baud_rates_packet(baud_rates_packet_t* baud_rates_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_baud_rates && an_packet->length == 17)
	{
		baud_rates_packet->permanent = an_packet->data[0];
		memcpy(&baud_rates_packet->primary_baud_rate, &an_packet->data[1], sizeof(uint32_t));
		memcpy(&baud_rates_packet->gpio_1_2_baud_rate, &an_packet->data[5], sizeof(uint32_t));
		memcpy(&baud_rates_packet->auxiliary_baud_rate, &an_packet->data[9], sizeof(uint32_t));
		memcpy(&baud_rates_packet->reserved, &an_packet->data[13], sizeof(uint32_t));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_baud_rates_packet(baud_rates_packet_t* baud_rates_packet)
{
	an_packet_t* an_packet = an_packet_allocate(17, packet_id_baud_rates);
	if(an_packet != NULL)
	{
		an_packet->data[0] = baud_rates_packet->permanent;
		memcpy(&an_packet->data[1], &baud_rates_packet->primary_baud_rate, sizeof(uint32_t));
		memcpy(&an_packet->data[5], &baud_rates_packet->gpio_1_2_baud_rate, sizeof(uint32_t));
		memcpy(&an_packet->data[9], &baud_rates_packet->auxiliary_baud_rate, sizeof(uint32_t));
		memcpy(&an_packet->data[13], &baud_rates_packet->reserved, sizeof(uint32_t));
	}
	return an_packet;
}

int decode_installation_alignment_packet(installation_alignment_packet_t* installation_alignment_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_installation_alignment && an_packet->length == 73)
	{
		installation_alignment_packet->permanent = an_packet->data[0];
		memcpy(&installation_alignment_packet->alignment_dcm[0][0], &an_packet->data[1], 9 * sizeof(float));
		memcpy(&installation_alignment_packet->gnss_antenna_offset[0], &an_packet->data[37], 3 * sizeof(float));
		memcpy(&installation_alignment_packet->odometer_offset[0], &an_packet->data[49], 3 * sizeof(float));
		memcpy(&installation_alignment_packet->external_data_offset[0], &an_packet->data[61], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_installation_alignment_packet(installation_alignment_packet_t* installation_alignment_packet)
{
	an_packet_t* an_packet = an_packet_allocate(73, packet_id_installation_alignment);
	if(an_packet != NULL)
	{
		an_packet->data[0] = installation_alignment_packet->permanent;
		memcpy(&an_packet->data[1], &installation_alignment_packet->alignment_dcm[0][0], 9 * sizeof(float));
		memcpy(&an_packet->data[37], &installation_alignment_packet->gnss_antenna_offset[0], 3 * sizeof(float));
		memcpy(&an_packet->data[49], &installation_alignment_packet->odometer_offset[0], 3 * sizeof(float));
		memcpy(&an_packet->data[61], &installation_alignment_packet->external_data_offset[0], 3 * sizeof(float));
	}
	return an_packet;
}

int decode_filter_options_packet(filter_options_packet_t* filter_options_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_filter_options && an_packet->length == 17)
	{
		memcpy(filter_options_packet, &an_packet->data[0], 9 * sizeof(uint8_t));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_filter_options_packet(filter_options_packet_t* filter_options_packet)
{
	an_packet_t* an_packet = an_packet_allocate(17, packet_id_filter_options);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], filter_options_packet, 9 * sizeof(uint8_t));
		memset(&an_packet->data[9], 0, 8 * sizeof(uint8_t));
	}
	return an_packet;
}

int decode_gpio_configuration_packet(gpio_configuration_packet_t* gpio_configuration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_gpio_configuration && an_packet->length == 13)
	{
		gpio_configuration_packet->permanent = an_packet->data[0];
		memcpy(gpio_configuration_packet, &an_packet->data[1], 4 * sizeof(uint8_t));
		gpio_configuration_packet->gpio_voltage_selection = an_packet->data[5];
		return 0;
	}
	else return 1;
}

an_packet_t* encode_gpio_configuration_packet(gpio_configuration_packet_t* gpio_configuration_packet)
{
	an_packet_t* an_packet = an_packet_allocate(13, packet_id_gpio_configuration);
	if(an_packet != NULL)
	{
		an_packet->data[0] = gpio_configuration_packet->permanent;
		memcpy(&an_packet->data[1], gpio_configuration_packet, 4 * sizeof(uint8_t));
		an_packet->data[5] = gpio_configuration_packet->gpio_voltage_selection;
		memset(&an_packet->data[6], 0, 7 * sizeof(uint8_t));
	}
	return an_packet;
}

int decode_odometer_configuration_packet(odometer_configuration_packet_t* odometer_configuration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_odometer_configuration && an_packet->length == 8)
	{
		odometer_configuration_packet->permanent = an_packet->data[0];
		odometer_configuration_packet->automatic_calibration = an_packet->data[1];
		memcpy(&odometer_configuration_packet->pulse_length, &an_packet->data[4], sizeof(float));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_odometer_configuration_packet(odometer_configuration_packet_t* odometer_configuration_packet)
{
	an_packet_t* an_packet = an_packet_allocate(8, packet_id_odometer_configuration);
	if(an_packet != NULL)
	{
		an_packet->data[0] = odometer_configuration_packet->permanent;
		an_packet->data[1] = odometer_configuration_packet->automatic_calibration;
		memset(&an_packet->data[2], 0, 2 * sizeof(uint8_t));
		memcpy(&an_packet->data[4], &odometer_configuration_packet->pulse_length, sizeof(float));
	}
	return an_packet;
}

an_packet_t* encode_zero_alignment_packet(zero_alignment_packet_t* zero_alignment_packet)
{
	uint32_t verification = 0x9A4E8055;
	an_packet_t* an_packet = an_packet_allocate(5, packet_id_zero_alignment);
	if(an_packet != NULL)
	{
		an_packet->data[0] = zero_alignment_packet->permanent;
		memcpy(&an_packet->data[1], &verification, sizeof(uint32_t));
	}
	return an_packet;
}

int decode_heave_offset_packet(heave_offset_packet_t* heave_offset_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_reference_offsets && an_packet->length == 49)
	{
		heave_offset_packet->permanent = an_packet->data[0];
		memcpy(&heave_offset_packet->heave_point_1_offset[0], &an_packet->data[1], 3 * sizeof(float));
		memcpy(&heave_offset_packet->heave_point_2_offset[0], &an_packet->data[13], 3 * sizeof(float));
		memcpy(&heave_offset_packet->heave_point_3_offset[0], &an_packet->data[25], 3 * sizeof(float));
		memcpy(&heave_offset_packet->heave_point_4_offset[0], &an_packet->data[37], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_heave_offset_packet(heave_offset_packet_t* heave_offset_packet)
{
	an_packet_t* an_packet = an_packet_allocate(49, packet_id_reference_offsets);
	if(an_packet != NULL)
	{
		an_packet->data[0] = heave_offset_packet->permanent;
		memcpy(&an_packet->data[1], &heave_offset_packet->heave_point_1_offset[0], 3 * sizeof(float));
		memcpy(&an_packet->data[13], &heave_offset_packet->heave_point_2_offset[0], 3 * sizeof(float));
		memcpy(&an_packet->data[25], &heave_offset_packet->heave_point_3_offset[0], 3 * sizeof(float));
		memcpy(&an_packet->data[37], &heave_offset_packet->heave_point_4_offset[0], 3 * sizeof(float));
	}
	return an_packet;
}

int decode_gpio_output_configuration_packet(gpio_output_configuration_packet_t* gpio_output_configuration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_gpio_output_configuration && an_packet->length == 183)
	{
		gpio_output_configuration_packet->permanent = an_packet->data[0];
		memcpy(&gpio_output_configuration_packet->auxiliary_port, &an_packet->data[1], 18 * sizeof(uint8_t));
		memcpy(&gpio_output_configuration_packet->gpio_port, &an_packet->data[27], 18 * sizeof(uint8_t));
		memcpy(&gpio_output_configuration_packet->logging_port, &an_packet->data[53], 18 * sizeof(uint8_t));
		memcpy(&gpio_output_configuration_packet->data_port_1, &an_packet->data[79], 18 * sizeof(uint8_t));
		memcpy(&gpio_output_configuration_packet->data_port_2, &an_packet->data[105], 18 * sizeof(uint8_t));
		memcpy(&gpio_output_configuration_packet->data_port_3, &an_packet->data[131], 18 * sizeof(uint8_t));
		memcpy(&gpio_output_configuration_packet->data_port_4, &an_packet->data[157], 18 * sizeof(uint8_t));
		return 0;
	}
	else return 1;
}

an_packet_t* encode_gpio_output_configuration_packet(gpio_output_configuration_packet_t* gpio_output_configuration_packet)
{
	an_packet_t* an_packet = an_packet_allocate(183, packet_id_gpio_output_configuration);
	if(an_packet != NULL)
	{
		an_packet->data[0] = gpio_output_configuration_packet->permanent;
		memcpy(&an_packet->data[1], &gpio_output_configuration_packet->auxiliary_port, 18 * sizeof(uint8_t));
		memset(&an_packet->data[19], 0, 8 * sizeof(uint8_t));
		memcpy(&an_packet->data[27], &gpio_output_configuration_packet->gpio_port, 18 * sizeof(uint8_t));
		memset(&an_packet->data[45], 0, 8 * sizeof(uint8_t));
		memcpy(&an_packet->data[53], &gpio_output_configuration_packet->logging_port, 18 * sizeof(uint8_t));
		memset(&an_packet->data[71], 0, 8 * sizeof(uint8_t));
		memcpy(&an_packet->data[79], &gpio_output_configuration_packet->data_port_1, 18 * sizeof(uint8_t));
		memset(&an_packet->data[97], 0, 8 * sizeof(uint8_t));
		memcpy(&an_packet->data[105], &gpio_output_configuration_packet->data_port_2, 18 * sizeof(uint8_t));
		memset(&an_packet->data[123], 0, 8 * sizeof(uint8_t));
		memcpy(&an_packet->data[131], &gpio_output_configuration_packet->data_port_3, 18 * sizeof(uint8_t));
		memset(&an_packet->data[149], 0, 8 * sizeof(uint8_t));
		memcpy(&an_packet->data[157], &gpio_output_configuration_packet->data_port_4, 18 * sizeof(uint8_t));
		memset(&an_packet->data[175], 0, 8 * sizeof(uint8_t));
	}
	return an_packet;
}

int decode_dual_antenna_configuration_packet(dual_antenna_configuration_packet_t* dual_antenna_configuration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_dual_antenna_configuration && an_packet->length == 17)
	{
		dual_antenna_configuration_packet->permanent = an_packet->data[0];
		memcpy(&dual_antenna_configuration_packet->options.r, &an_packet->data[1], sizeof(uint16_t));
		dual_antenna_configuration_packet->automatic_offset_orientation = an_packet->data[3];
		dual_antenna_configuration_packet->reserved = an_packet->data[4];
		memcpy(&dual_antenna_configuration_packet->manual_offset, &an_packet->data[5], 3 * sizeof(float));
		return 0;
	}
	return 1;
}

an_packet_t* encode_dual_antenna_configuration_packet(dual_antenna_configuration_packet_t* dual_antenna_configuration_packet)
{
	an_packet_t* an_packet = an_packet_allocate(17, packet_id_dual_antenna_configuration);
	if(an_packet != NULL)
	{
		an_packet->data[0] = dual_antenna_configuration_packet->permanent;
		memcpy(&an_packet->data[1], &dual_antenna_configuration_packet->options.r, sizeof(uint16_t));
		an_packet->data[3] = dual_antenna_configuration_packet->automatic_offset_orientation;
		an_packet->data[4] = 0;
		memcpy(&an_packet->data[5], &dual_antenna_configuration_packet->manual_offset, 3 * sizeof(float));
	}
	return an_packet;
}

int decode_gnss_configuration_packet(gnss_configuration_packet_t* gnss_configuration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_gnss_configuration && an_packet->length == 85)
	{
		gnss_configuration_packet->permanent = an_packet->data[0];
		memcpy(&gnss_configuration_packet->gnss_frequencies, &an_packet->data[1], sizeof(uint64_t));
		memcpy(&gnss_configuration_packet->pdop, &an_packet->data[9], sizeof(float));
		memcpy(&gnss_configuration_packet->tdop, &an_packet->data[13], sizeof(float));
		gnss_configuration_packet->elevation_mask = an_packet->data[17];
		gnss_configuration_packet->snr_mask = an_packet->data[18];
		gnss_configuration_packet->sbas_corrections_enabled = an_packet->data[19];
		gnss_configuration_packet->lband_mode = an_packet->data[20];
		memcpy(&gnss_configuration_packet->lband_frequency, &an_packet->data[21], sizeof(uint32_t));
		memcpy(&gnss_configuration_packet->lband_baud, &an_packet->data[25], sizeof(uint32_t));
		memcpy(&gnss_configuration_packet->primary_antenna_type, &an_packet->data[29], sizeof(uint32_t));
		memcpy(&gnss_configuration_packet->secondary_antenna_type, &an_packet->data[33], sizeof(uint32_t));
		gnss_configuration_packet->lband_satellite_id = an_packet->data[37];
		return 0;
	}
	return 1;
}

an_packet_t* encode_gnss_configuration_packet(gnss_configuration_packet_t* gnss_configuration_packet)
{
	an_packet_t* an_packet = an_packet_allocate(85, packet_id_gnss_configuration);
	if(an_packet != NULL)
	{
		an_packet->data[0] = gnss_configuration_packet->permanent;
		memcpy(&an_packet->data[1], &gnss_configuration_packet->gnss_frequencies, sizeof(uint64_t));
		memcpy(&an_packet->data[9], &gnss_configuration_packet->pdop, sizeof(float));
		memcpy(&an_packet->data[13], &gnss_configuration_packet->tdop, sizeof(float));
		an_packet->data[17] = gnss_configuration_packet->elevation_mask;
		an_packet->data[18] = gnss_configuration_packet->snr_mask;
		an_packet->data[19] = gnss_configuration_packet->sbas_corrections_enabled;
		an_packet->data[20] = gnss_configuration_packet->lband_mode;
		memcpy(&an_packet->data[21], &gnss_configuration_packet->lband_frequency, sizeof(uint32_t));
		memcpy(&an_packet->data[25], &gnss_configuration_packet->lband_baud, sizeof(uint32_t));
		memcpy(&an_packet->data[29], &gnss_configuration_packet->primary_antenna_type, sizeof(uint32_t));
		memcpy(&an_packet->data[33], &gnss_configuration_packet->secondary_antenna_type, sizeof(uint32_t));
		an_packet->data[37] = gnss_configuration_packet->lband_satellite_id;
	}
	return an_packet;
}

int decode_user_data_packet(user_data_packet_t* user_data_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_user_data && an_packet->length == 64)
	{
		memcpy(&user_data_packet->user_data, &an_packet->data[0], 64);
		return 0;
	}
	return 1;
}

an_packet_t* encode_user_data_packet(user_data_packet_t* user_data_packet)
{
	an_packet_t* an_packet = an_packet_allocate(64, packet_id_user_data);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &user_data_packet->user_data, 64);
	}
	return an_packet;
}

int decode_gpio_input_configuration_packet(gpio_input_configuration_packet_t* gpio_input_configuration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_gpio_input_configuration && an_packet->length == 65)
	{
		gpio_input_configuration_packet->permanent = an_packet->data[0];
		memcpy(&gpio_input_configuration_packet->gimbal_radians_per_encoder_tick, &an_packet->data[1], sizeof(float));
		return 0;
	}
	return 1;
}

an_packet_t* encode_gpio_input_configuration_packet(gpio_input_configuration_packet_t* gpio_input_configuration_packet)
{
	an_packet_t* an_packet = an_packet_allocate(65, packet_id_gpio_input_configuration);
	if(an_packet != NULL)
	{
		an_packet->data[0] = gpio_input_configuration_packet->permanent;
		memcpy(&an_packet->data[1], &gpio_input_configuration_packet->gimbal_radians_per_encoder_tick, sizeof(float));
	}
	return an_packet;
}

int decode_ip_dataports_configuration_packet(ip_dataports_configuration_packet_t* ip_dataports_configuration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_ip_dataports_configuration && an_packet->length == 30)
	{
		for(int i = 0; i < 4; i++)
		{
			memcpy(&ip_dataports_configuration_packet->ip_dataport_configuration[i].ip_address, &an_packet->data[7 * i + 2], sizeof(uint32_t));
			memcpy(&ip_dataports_configuration_packet->ip_dataport_configuration[i].port, &an_packet->data[7 * i + 6], sizeof(uint16_t));
			ip_dataports_configuration_packet->ip_dataport_configuration[i].ip_dataport_mode = an_packet->data[7 * i + 8];
		}
		return 0;
	}
	return 1;
}

an_packet_t* encode_ip_dataports_configuration_packet(ip_dataports_configuration_packet_t* ip_dataports_configuration_packet)
{
	an_packet_t* an_packet = an_packet_allocate(30, packet_id_ip_dataports_configuration);
	if(an_packet != NULL)
	{
		for(int i = 0; i < 4; i++)
		{
			memcpy(&an_packet->data[7 * i + 2], &ip_dataports_configuration_packet->ip_dataport_configuration[i].ip_address, sizeof(uint32_t));
			memcpy(&an_packet->data[7 * i + 6], &ip_dataports_configuration_packet->ip_dataport_configuration[i].port, sizeof(uint16_t));
			an_packet->data[7 * i + 8] = ip_dataports_configuration_packet->ip_dataport_configuration[i].ip_dataport_mode;
		}
	}
	return an_packet;
}

int decode_can_configuration_packet(can_configuration_packet_t* can_configuration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_can_configuration && an_packet->length == 11)
	{
		can_configuration_packet->permanent = an_packet->data[0];
		can_configuration_packet->enabled = an_packet->data[1];
		memcpy(&can_configuration_packet->baud_rate, &an_packet->data[2], sizeof(uint32_t));
		can_configuration_packet->can_protocol = an_packet->data[6];
		return 0;
	}
	return 1;
}

an_packet_t* encode_can_configuration_packet(can_configuration_packet_t* can_configuration_packet)
{
	an_packet_t* an_packet = an_packet_allocate(11, packet_id_can_configuration);
	if(an_packet != NULL)
	{
		an_packet->data[0] = can_configuration_packet->permanent;
		an_packet->data[1] = can_configuration_packet->enabled;
		memcpy(&an_packet->data[2], &can_configuration_packet->baud_rate, sizeof(uint32_t));
		an_packet->data[6] = can_configuration_packet->can_protocol;
	}
	return an_packet;
}

