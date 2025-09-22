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
#include <cmath>
#include <cstring>

#if !defined(__x86_64__)
#include <modal_pipe_interfaces.h>
#include <modal_json.h>

#define RAD_TO_DEG (180.0/3.14159265358979323846)

bool parse_mavlink_to_json(char* data, int bytes, std::string& json_output) {
    int n_packets;
    mavlink_message_t* msg_array = (mavlink_message_t*)pipe_validate_mavlink_message_t(data, bytes, &n_packets);
    
    if (msg_array != NULL && n_packets > 0) {
        // Convert first MAVLink message to JSON
        json_output = mavlink_to_json_string(&msg_array[0]);    
        if (n_packets > 1) {
            std::cout << "Received " << n_packets << " MAVLink messages, converting first one" << std::endl;
        }
        return true;
    } else {
        json_output.clear();
        return false;
    }
}

/**
 * Convert rotation matrix to Tait-Bryan angles (roll, pitch, yaw)
 * Same implementation as voxl-inspect-vio.c
 */
static void rotation_to_tait_bryan(float R[3][3], float* roll, float* pitch, float* yaw) {
    *roll  = atan2(R[2][1], R[2][2]);
    *pitch = asin(-R[2][0]);
    *yaw   = atan2(R[1][0], R[0][0]);

    if(fabs((double)*pitch - M_PI_2) < 1.0e-3){
        *roll = 0.0;
        *pitch = atan2(R[1][2], R[0][2]);
    }
    else if(fabs((double)*pitch + M_PI_2) < 1.0e-3) {
        *roll = 0.0;
        *pitch = atan2(-R[1][2], -R[0][2]);
    }
}

std::string vio_to_json(const void* vio_data_ptr) {
    // Cast to vio_data_t (assuming this is the structure type from modal_pipe_interfaces.h)
    const vio_data_t* vio = static_cast<const vio_data_t*>(vio_data_ptr);
    
    std::string json = "{";
    json += "\"data_type\":\"vio\",";
    json += "\"timestamp_ns\":" + std::to_string(vio->timestamp_ns) + ",";
    json += "\"timestamp\":" + std::to_string(std::time(nullptr)) + ",";
    
    // Position (T_imu_wrt_vio)
    json += "\"position\":{";
    json += "\"x\":" + std::to_string(vio->T_imu_wrt_vio[0]) + ",";
    json += "\"y\":" + std::to_string(vio->T_imu_wrt_vio[1]) + ",";
    json += "\"z\":" + std::to_string(vio->T_imu_wrt_vio[2]);
    json += "},";
    
    // Rotation (convert rotation matrix to Euler angles)
    float R_tmp[3][3];
    memcpy(R_tmp, vio->R_imu_to_vio, sizeof(R_tmp));
    float roll, pitch, yaw;
    rotation_to_tait_bryan(R_tmp, &roll, &pitch, &yaw);
    
    json += "\"rotation\":{";
    json += "\"roll\":" + std::to_string((double)roll * RAD_TO_DEG) + ",";
    json += "\"pitch\":" + std::to_string((double)pitch * RAD_TO_DEG) + ",";
    json += "\"yaw\":" + std::to_string((double)yaw * RAD_TO_DEG);
    json += "},";
    
    // Velocity
    json += "\"velocity\":{";
    json += "\"x\":" + std::to_string(vio->vel_imu_wrt_vio[0]) + ",";
    json += "\"y\":" + std::to_string(vio->vel_imu_wrt_vio[1]) + ",";
    json += "\"z\":" + std::to_string(vio->vel_imu_wrt_vio[2]);
    json += "},";
    
    // Angular velocity
    json += "\"angular_velocity\":{";
    json += "\"x\":" + std::to_string((double)vio->imu_angular_vel[0] * RAD_TO_DEG) + ",";
    json += "\"y\":" + std::to_string((double)vio->imu_angular_vel[1] * RAD_TO_DEG) + ",";
    json += "\"z\":" + std::to_string((double)vio->imu_angular_vel[2] * RAD_TO_DEG);
    json += "},";
    
    // Quality and features
    json += "\"quality\":" + std::to_string(vio->quality) + ",";
    json += "\"n_feature_points\":" + std::to_string(vio->n_feature_points) + ",";
    
    // State and error codes
    json += "\"state\":" + std::to_string(vio->state) + ",";
    json += "\"error_code\":" + std::to_string(vio->error_code);
    
    json += "}";
    return json;
}

bool parse_vio_to_json(char* data, int bytes, std::string& json_output) {
    int n_packets;
    vio_data_t* vio_array = pipe_validate_vio_data_t(data, bytes, &n_packets);
    
    if (vio_array != NULL && n_packets > 0) {
        // Convert first VIO data to JSON
        json_output = vio_to_json(&vio_array[0]);
        
        if (n_packets > 1) {
            std::cout << "Received " << n_packets << " VIO data packets, converting first one" << std::endl;
        }
        return true;
    } else {
        json_output.clear();
        return false;
    }
}

bool parse_pipe_data_to_json(const std::string& pipe_name, char* data, int bytes, std::string& json_output) {
    
    if (pipe_name.find("vio") != std::string::npos || 
        pipe_name.find("qvio") != std::string::npos ||
        pipe_name.find("openvins") != std::string::npos) {
        
        // Try VIO parsing first for vio-related pipes
        if (parse_vio_to_json(data, bytes, json_output)) {
            return true;
        }
    }
    
    // If pipe name doesn't give us hints, try both parsers
    if (parse_mavlink_to_json(data, bytes, json_output)) {
        return true;
    }
        
    // If both parsers fail, return raw data as fallback
    json_output = "{\"data_type\":\"raw\",\"timestamp\":" + std::to_string(std::time(nullptr)) + 
                  ",\"bytes\":" + std::to_string(bytes) + 
                  ",\"data\":\"" + std::string(data, std::min(bytes, 100)) + "\"}";
    return false;
}

#endif // !defined(__x86_64__)