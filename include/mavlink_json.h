/*******************************************************************************
 * Copyright 2025 RED DOT DRONE PTE. LTD.
 *
 * Author: Akira Hirakawa
 *
 * Data parsing utilities for VOXL MQTT Client
 * Handles parsing MAVLink messages and VIO data from VOXL pipes and converting them to JSON
 ******************************************************************************/

#pragma once

#include <string>

#if !defined(__x86_64__)
#include <c_library_v2/common/mavlink.h>
#include <mavlink_to_json.h>

/**
 * Parse raw pipe data into MAVLink messages and convert to JSON
 * @param data Raw data buffer from pipe
 * @param bytes Number of bytes in buffer
 * @param json_output Output JSON string (empty if parsing fails)
 * @return true if parsing successful, false otherwise
 */
bool parse_mavlink_to_json(char* data, int bytes, std::string& json_output);

/**
 * Convert VIO data to JSON string for MQTT publishing
 * @param vio_data Pointer to VIO data structure
 * @return JSON string representation of the VIO data
 */
std::string vio_to_json(const void* vio_data);

/**
 * Parse raw pipe data into VIO data and convert to JSON
 * @param data Raw data buffer from pipe
 * @param bytes Number of bytes in buffer
 * @param json_output Output JSON string (empty if parsing fails)
 * @return true if parsing successful, false otherwise
 */
bool parse_vio_to_json(char* data, int bytes, std::string& json_output);

/**
 * Convert IMU data to JSON string for MQTT publishing
 * @param imu_data Pointer to IMU data structure
 * @return JSON string representation of the IMU data
 */
std::string imu_to_json(const void* imu_data);

/**
 * Parse raw pipe data into IMU data and convert to JSON
 * @param data Raw data buffer from pipe
 * @param bytes Number of bytes in buffer
 * @param json_output Output JSON string (empty if parsing fails)
 * @return true if parsing successful, false otherwise
 */
bool parse_imu_to_json(char* data, int bytes, std::string& json_output);

/**
 * Auto-detect data type and parse to JSON
 * Tries MAVLink first, then VIO, then falls back to raw data
 * @param pipe_name Name of the pipe (used for type detection hints)
 * @param data Raw data buffer from pipe
 * @param bytes Number of bytes in buffer
 * @param json_output Output JSON string
 * @return true if any parsing was successful, false otherwise
 */
bool parse_pipe_data_to_json(const std::string& pipe_name, char* data, int bytes, std::string& json_output);

#endif // !defined(__x86_64__)