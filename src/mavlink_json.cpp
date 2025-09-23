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
#include <cJSON.h>

#include <modal_pipe_interfaces.h>
#include <modal_json.h>

#define RAD_TO_DEG (180.0/3.14159265358979323846)

bool parse_mavlink_to_json(char* data, int bytes, std::string& json_output) {
    int n_packets;
    mavlink_message_t* msg_array = (mavlink_message_t*)pipe_validate_mavlink_message_t(data, bytes, &n_packets);
    
    if (msg_array != NULL && n_packets > 0) {
        // Convert first MAVLink message to JSON
        json_output = mavlink_to_json_string(&msg_array[0]);    
        if (n_packets > 1 && g_debug_mode) {
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
    const vio_data_t* vio = static_cast<const vio_data_t*>(vio_data_ptr);

    cJSON* root = cJSON_CreateObject();
    if (!root) return "{}";

    cJSON_AddNumberToObject(root, "timestamp_ns", vio->timestamp_ns);

    // Position (T_imu_wrt_vio)
    cJSON* position = cJSON_CreateObject();
    cJSON_AddNumberToObject(position, "x", vio->T_imu_wrt_vio[0]);
    cJSON_AddNumberToObject(position, "y", vio->T_imu_wrt_vio[1]);
    cJSON_AddNumberToObject(position, "z", vio->T_imu_wrt_vio[2]);
    cJSON_AddItemToObject(root, "position", position);

    // Rotation (convert rotation matrix to Euler angles)
    float R_tmp[3][3];
    memcpy(R_tmp, vio->R_imu_to_vio, sizeof(R_tmp));
    float roll, pitch, yaw;
    rotation_to_tait_bryan(R_tmp, &roll, &pitch, &yaw);

    cJSON* rotation = cJSON_CreateObject();
    cJSON_AddNumberToObject(rotation, "roll", (double)roll * RAD_TO_DEG);
    cJSON_AddNumberToObject(rotation, "pitch", (double)pitch * RAD_TO_DEG);
    cJSON_AddNumberToObject(rotation, "yaw", (double)yaw * RAD_TO_DEG);
    cJSON_AddItemToObject(root, "rotation", rotation);

    // Velocity
    cJSON* velocity = cJSON_CreateObject();
    cJSON_AddNumberToObject(velocity, "x", vio->vel_imu_wrt_vio[0]);
    cJSON_AddNumberToObject(velocity, "y", vio->vel_imu_wrt_vio[1]);
    cJSON_AddNumberToObject(velocity, "z", vio->vel_imu_wrt_vio[2]);
    cJSON_AddItemToObject(root, "velocity", velocity);

    // Angular velocity
    cJSON* angular_velocity = cJSON_CreateObject();
    cJSON_AddNumberToObject(angular_velocity, "x", (double)vio->imu_angular_vel[0] * RAD_TO_DEG);
    cJSON_AddNumberToObject(angular_velocity, "y", (double)vio->imu_angular_vel[1] * RAD_TO_DEG);
    cJSON_AddNumberToObject(angular_velocity, "z", (double)vio->imu_angular_vel[2] * RAD_TO_DEG);
    cJSON_AddItemToObject(root, "angular_velocity", angular_velocity);

    // Quality and features
    cJSON_AddNumberToObject(root, "quality", vio->quality);
    cJSON_AddNumberToObject(root, "n_feature_points", vio->n_feature_points);

    // State and error codes
    cJSON_AddNumberToObject(root, "state", vio->state);
    cJSON_AddNumberToObject(root, "error_code", vio->error_code);

    char* json_string = cJSON_PrintUnformatted(root);
    std::string result = json_string ? json_string : "{}";

    cJSON_Delete(root);
    if (json_string) free(json_string);

    return result;
}

bool parse_vio_to_json(char* data, int bytes, std::string& json_output) {
    int n_packets;
    vio_data_t* vio_array = pipe_validate_vio_data_t(data, bytes, &n_packets);
    
    if (vio_array != NULL && n_packets > 0) {
        // Convert first VIO data to JSON
        json_output = vio_to_json(&vio_array[0]);
        
        if (n_packets > 1 && g_debug_mode) {
            std::cout << "Received " << n_packets << " VIO data packets, converting first one" << std::endl;
        }
        return true;
    } else {
        json_output.clear();
        return false;
    }
}

std::string imu_to_json(const void* imu_data_ptr) {
    const imu_data_t* imu = static_cast<const imu_data_t*>(imu_data_ptr);

    cJSON* root = cJSON_CreateObject();
    if (!root) return "{}";

    // Accelerometer data (m/s²)
    cJSON* accl = cJSON_CreateObject();
    cJSON_AddNumberToObject(accl, "x", imu->accl_ms2[0]);
    cJSON_AddNumberToObject(accl, "y", imu->accl_ms2[1]);
    cJSON_AddNumberToObject(accl, "z", imu->accl_ms2[2]);
    cJSON_AddItemToObject(root, "accl_ms2", accl);

    // Gyroscope data (rad/s)
    cJSON* gyro = cJSON_CreateObject();
    cJSON_AddNumberToObject(gyro, "x", imu->gyro_rad[0]);
    cJSON_AddNumberToObject(gyro, "y", imu->gyro_rad[1]);
    cJSON_AddNumberToObject(gyro, "z", imu->gyro_rad[2]);
    cJSON_AddItemToObject(root, "gyro_rad", gyro);

    // Temperature (°C)
    cJSON_AddNumberToObject(root, "temp_c", imu->temp_c);

    cJSON_AddNumberToObject(root, "timestamp_ns", imu->timestamp_ns);

    char* json_string = cJSON_PrintUnformatted(root);
    std::string result = json_string ? json_string : "{}";

    cJSON_Delete(root);
    if (json_string) free(json_string);

    return result;
}

// https://gitlab.com/voxl-public/voxl-sdk/utilities/voxl-mpa-tools/-/blob/master/tools/voxl-inspect-imu.c
bool parse_imu_to_json(char* data, int bytes, std::string& json_output) {
    int n_packets;
    imu_data_t* data_array = pipe_validate_imu_data_t(data, bytes, &n_packets);

    if (data_array != NULL && n_packets > 0) {
        // Convert latest IMU data to JSON
        json_output = imu_to_json(&data_array[n_packets-1]);

        if (n_packets > 1 && g_debug_mode) {
            std::cout << "Received " << n_packets << " IMU data packets, converting latest one" << std::endl;
        }
        return true;
    } else {
        json_output.clear();
        return false;
    }
}

bool parse_pipe_data_to_json(const std::string& pipe_name, char* data, int bytes, std::string& json_output) {

    if (pipe_name.find("vvhub_aligned_vio") != std::string::npos) {
        // Try VIO parsing first for vio-related pipes
        if (parse_vio_to_json(data, bytes, json_output)) {
            return true;
        }
    }

    if (pipe_name.find("imu_apps") != std::string::npos) {
        // Parse IMU data
        if (parse_imu_to_json(data, bytes, json_output)) {
            return true;
        }
    }
    
    // If pipe name doesn't give us hints, try both parsers
    if (parse_mavlink_to_json(data, bytes, json_output)) {
        return true;
    }

    // If both parsers fail, return raw data as fallback
    cJSON* raw_json = cJSON_CreateObject();
    if (raw_json) {
        cJSON_AddStringToObject(raw_json, "data_type", "raw");
        cJSON_AddNumberToObject(raw_json, "timestamp", std::time(nullptr));
        cJSON_AddNumberToObject(raw_json, "bytes", bytes);

        // Add truncated data string (max 100 bytes)
        std::string data_str(data, std::min(bytes, 100));
        cJSON_AddStringToObject(raw_json, "data", data_str.c_str());

        char* json_string = cJSON_PrintUnformatted(raw_json);
        json_output = json_string ? json_string : "{}";

        cJSON_Delete(raw_json);
        if (json_string) free(json_string);
    } else {
        json_output = "{}";
    }
    return false;
}