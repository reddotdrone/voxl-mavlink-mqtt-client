/*******************************************************************************
 * Copyright 2025 RED DOT DRONE PTE. LTD.
 *
 * Author: Akira Hirakawa
 *
 * MAVLink to JSON conversion implementations for VOXL MQTT Client
 ******************************************************************************/

#include "mavlink_json.h"
#include <ctime>
#include <iostream>

#if !defined(__x86_64__)
#include <modal_pipe_interfaces.h>

std::string mavlink_to_json(const mavlink_message_t* msg) {
    // Basic JSON structure with common MAVLink fields
    std::string json = "{";
    json += "\"msgid\":" + std::to_string(msg->msgid) + ",";
    json += "\"sysid\":" + std::to_string(msg->sysid) + ",";
    json += "\"compid\":" + std::to_string(msg->compid) + ",";
    json += "\"seq\":" + std::to_string(msg->seq) + ",";
    json += "\"timestamp\":" + std::to_string(std::time(nullptr));
    
    // Add message-specific data based on message ID
    switch(msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(msg, &heartbeat);
            json += ",\"type\":" + std::to_string(heartbeat.type);
            json += ",\"autopilot\":" + std::to_string(heartbeat.autopilot);
            json += ",\"base_mode\":" + std::to_string(heartbeat.base_mode);
            json += ",\"custom_mode\":" + std::to_string(heartbeat.custom_mode);
            json += ",\"system_status\":" + std::to_string(heartbeat.system_status);
            json += ",\"mavlink_version\":" + std::to_string(heartbeat.mavlink_version);
            break;
        }
        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(msg, &sys_status);
            json += ",\"voltage_battery\":" + std::to_string(sys_status.voltage_battery);
            json += ",\"current_battery\":" + std::to_string(sys_status.current_battery);
            json += ",\"battery_remaining\":" + std::to_string(sys_status.battery_remaining);
            json += ",\"load\":" + std::to_string(sys_status.load);
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(msg, &attitude);
            json += ",\"time_boot_ms\":" + std::to_string(attitude.time_boot_ms);
            json += ",\"roll\":" + std::to_string(attitude.roll);
            json += ",\"pitch\":" + std::to_string(attitude.pitch);
            json += ",\"yaw\":" + std::to_string(attitude.yaw);
            json += ",\"rollspeed\":" + std::to_string(attitude.rollspeed);
            json += ",\"pitchspeed\":" + std::to_string(attitude.pitchspeed);
            json += ",\"yawspeed\":" + std::to_string(attitude.yawspeed);
            break;
        }
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
            mavlink_local_position_ned_t pos;
            mavlink_msg_local_position_ned_decode(msg, &pos);
            json += ",\"time_boot_ms\":" + std::to_string(pos.time_boot_ms);
            json += ",\"x\":" + std::to_string(pos.x);
            json += ",\"y\":" + std::to_string(pos.y);
            json += ",\"z\":" + std::to_string(pos.z);
            json += ",\"vx\":" + std::to_string(pos.vx);
            json += ",\"vy\":" + std::to_string(pos.vy);
            json += ",\"vz\":" + std::to_string(pos.vz);
            break;
        }
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
            mavlink_gps_raw_int_t gps;
            mavlink_msg_gps_raw_int_decode(msg, &gps);
            json += ",\"time_usec\":" + std::to_string(gps.time_usec);
            json += ",\"fix_type\":" + std::to_string(gps.fix_type);
            json += ",\"lat\":" + std::to_string(gps.lat);
            json += ",\"lon\":" + std::to_string(gps.lon);
            json += ",\"alt\":" + std::to_string(gps.alt);
            json += ",\"eph\":" + std::to_string(gps.eph);
            json += ",\"epv\":" + std::to_string(gps.epv);
            json += ",\"vel\":" + std::to_string(gps.vel);
            json += ",\"cog\":" + std::to_string(gps.cog);
            json += ",\"satellites_visible\":" + std::to_string(gps.satellites_visible);
            break;
        }
        // Add more message types as needed
        default:
            json += ",\"raw_data\":\"unsupported_message_type\"";
            json += ",\"message_name\":\"UNKNOWN_MSG_" + std::to_string(msg->msgid) + "\"";
            break;
    }
    
    json += "}";
    return json;
}

bool parse_mavlink_to_json(char* data, int bytes, std::string& json_output) {
    int n_packets;
    mavlink_message_t* msg_array = (mavlink_message_t*)pipe_validate_mavlink_message_t(data, bytes, &n_packets);
    
    if (msg_array != NULL && n_packets > 0) {
        // Convert first MAVLink message to JSON
        json_output = mavlink_to_json(&msg_array[0]);
        
        if (n_packets > 1) {
            std::cout << "Received " << n_packets << " MAVLink messages, converting first one" << std::endl;
        }
        return true;
    } else {
        json_output.clear();
        return false;
    }
}

#endif // !defined(__x86_64__)