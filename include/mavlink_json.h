/*******************************************************************************
 * Copyright 2025 RED DOT DRONE PTE. LTD.
 *
 * Author: Akira Hirakawa
 *
 * MAVLink to JSON conversion utilities for VOXL MQTT Client
 * Handles parsing MAVLink messages from VOXL pipes and converting them to JSON
 ******************************************************************************/

#pragma once

#include <string>

#if !defined(__x86_64__)
#include <c_library_v2/common/mavlink.h>

/**
 * Convert MAVLink message to JSON string for MQTT publishing
 * @param msg Pointer to decoded MAVLink message
 * @return JSON string representation of the message
 */
std::string mavlink_to_json(const mavlink_message_t* msg);

/**
 * Parse raw pipe data into MAVLink messages and convert to JSON
 * @param data Raw data buffer from pipe
 * @param bytes Number of bytes in buffer
 * @param json_output Output JSON string (empty if parsing fails)
 * @return true if parsing successful, false otherwise
 */
bool parse_mavlink_to_json(char* data, int bytes, std::string& json_output);

#endif // !defined(__x86_64__)