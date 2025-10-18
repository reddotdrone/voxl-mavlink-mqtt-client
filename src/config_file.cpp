/*******************************************************************************
 * Copyright 2025 RED DOT DRONE PTE. LTD.
 *
 * Author: Akira Hirakawa
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "config_file.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <sys/stat.h>
#include <unistd.h>

static void set_default_config(mqtt_config_t* config) {
    config->broker_host = "localhost";
    config->broker_port = 1883;
    config->client_id = "voxl-mavlink-mqtt-client";
    config->username = "";
    config->password = "";
    config->use_tls = false;
    config->ca_cert_path = "";
    config->cert_path = "";
    config->key_path = "";
    config->keepalive = 60;
    config->reconnect_delay = 5;
    config->publish_topics.clear();
    config->subscribe_topics.clear();

    mqtt_topic_config_t pub_topic;
    // pub_topic.topic = "voxl/imu";
    // pub_topic.pipe_name = "imu";
    // pub_topic.qos = 0;
    // config->publish_topics.push_back(pub_topic);

    pub_topic.topic = "voxl/vio";
    pub_topic.pipe_name = "vvhub_aligned_vio";
    pub_topic.qos = 0;
    config->publish_topics.push_back(pub_topic);

    pub_topic.topic = "voxl/battery";
    pub_topic.pipe_name = "/run/mpa/mavlink_sys_status/";
    pub_topic.qos = 0;
    config->publish_topics.push_back(pub_topic);

    pub_topic.topic = "voxl/heartbeat";
    pub_topic.pipe_name = "mavlink_ap_heartbeat";
    pub_topic.qos = 0;
    config->publish_topics.push_back(pub_topic);

    // Default subscribe topic for offboard MQTT commands
    mqtt_topic_config_t sub_topic;
    sub_topic.topic = "voxl/offboard_cmd";
    sub_topic.pipe_name = "offboard_mqtt_cmd";
    sub_topic.qos = 0;
    config->subscribe_topics.push_back(sub_topic);
}

static std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(' ');
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(' ');
    return str.substr(first, (last - first + 1));
}

static bool parse_bool(const std::string& value) {
    std::string lower = value;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    return lower == "true" || lower == "1" || lower == "yes" || lower == "on";
}

int load_config(mqtt_config_t* config) {
    set_default_config(config);
    
    std::ifstream file(CONFIG_FILE_PATH);
    if (!file.is_open()) {
        std::cout << "Config file not found, using defaults" << std::endl;
        return 0;
    }
    
    std::string line;
    mqtt_topic_config_t current_topic;
    bool in_publish_section = false;
    bool in_subscribe_section = false;

    while (std::getline(file, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;

        if (line == "[publish_topics]") {
            in_publish_section = true;
            in_subscribe_section = false;
            config->publish_topics.clear();
            continue;
        }

        if (line == "[subscribe_topics]") {
            in_subscribe_section = true;
            in_publish_section = false;
            config->subscribe_topics.clear();
            continue;
        }

        if (line[0] == '[' && line.back() == ']') {
            in_publish_section = false;
            in_subscribe_section = false;
            continue;
        }

        size_t eq_pos = line.find('=');
        if (eq_pos == std::string::npos) continue;

        std::string key = trim(line.substr(0, eq_pos));
        std::string value = trim(line.substr(eq_pos + 1));

        if (value.front() == '"' && value.back() == '"') {
            value = value.substr(1, value.length() - 2);
        }

        if (in_publish_section) {
            if (key == "topic") {
                current_topic.topic = value;
            } else if (key == "pipe_name") {
                current_topic.pipe_name = value;
            } else if (key == "qos") {
                current_topic.qos = std::stoi(value);
                config->publish_topics.push_back(current_topic);
            }
        } else if (in_subscribe_section) {
            if (key == "topic") {
                current_topic.topic = value;
            } else if (key == "pipe_name") {
                current_topic.pipe_name = value;
            } else if (key == "qos") {
                current_topic.qos = std::stoi(value);
                config->subscribe_topics.push_back(current_topic);
            }
        } else {
            if (key == "broker_host") {
                config->broker_host = value;
            } else if (key == "broker_port") {
                config->broker_port = std::stoi(value);
            } else if (key == "client_id") {
                config->client_id = value;
            } else if (key == "username") {
                config->username = value;
            } else if (key == "password") {
                config->password = value;
            } else if (key == "use_tls") {
                config->use_tls = parse_bool(value);
            } else if (key == "ca_cert_path") {
                config->ca_cert_path = value;
            } else if (key == "cert_path") {
                config->cert_path = value;
            } else if (key == "key_path") {
                config->key_path = value;
            } else if (key == "keepalive") {
                config->keepalive = std::stoi(value);
            } else if (key == "reconnect_delay") {
                config->reconnect_delay = std::stoi(value);
            }
        }
    }
    
    file.close();
    return 0;
}

int save_default_config(void) {
    std::string dir = "/etc/modalai";
    mkdir(dir.c_str(), 0755);
    
    std::ofstream file(CONFIG_FILE_PATH);
    if (!file.is_open()) {
        std::cerr << "Failed to create config file: " << CONFIG_FILE_PATH << std::endl;
        return -1;
    }
    
    file << "# VOXL MAVLink MQTT Client Configuration\n";
    file << "# This file configures the MAVLink MQTT client for publishing to topics\n\n";
    
    file << "[broker]\n";
    file << "broker_host = \"localhost\"\n";
    file << "broker_port = 1883\n";
    file << "client_id = \"voxl-mavlink-mqtt-client\"\n";
    file << "username = \"\"\n";
    file << "password = \"\"\n";
    file << "keepalive = 60\n";
    file << "reconnect_delay = 5\n\n";
    
    file << "[tls]\n";
    file << "use_tls = false\n";
    file << "ca_cert_path = \"\"\n";
    file << "cert_path = \"\"\n";
    file << "key_path = \"\"\n\n";
    
    file << "[publish_topics]\n";
    file << "topic = \"voxl/imu\"\n";
    file << "pipe_name = \"imu\"\n";
    file << "qos = 0\n\n";
    file << "topic = \"voxl/qvio\"\n";
    file << "pipe_name = \"qvio\"\n";
    file << "qos = 0\n\n";

    file << "[subscribe_topics]\n";
    file << "# MQTT topics to subscribe to and forward to Modal Pipes\n";
    file << "topic = \"voxl/offboard_cmd\"\n";
    file << "pipe_name = \"offboard_mqtt_cmd\"\n";
    file << "qos = 0\n\n";

    file.close();
    return 0;
}

void print_config(const mqtt_config_t* config) {
    std::cout << "MAVLink MQTT Configuration:\n";
    std::cout << "  Broker: " << config->broker_host << ":" << config->broker_port << "\n";
    std::cout << "  Client ID: " << config->client_id << "\n";
    std::cout << "  Username: " << config->username << "\n";
    std::cout << "  TLS: " << (config->use_tls ? "enabled" : "disabled") << "\n";
    std::cout << "  Keepalive: " << config->keepalive << "s\n";
    std::cout << "  Reconnect delay: " << config->reconnect_delay << "s\n";

    std::cout << "\nPublish Topics (Pipe -> MQTT):\n";
    for (const auto& topic : config->publish_topics) {
        std::cout << "  " << topic.topic << " <- " << topic.pipe_name << " (QoS " << topic.qos << ")\n";
    }

    std::cout << "\nSubscribe Topics (MQTT -> Pipe):\n";
    for (const auto& topic : config->subscribe_topics) {
        std::cout << "  " << topic.topic << " -> " << topic.pipe_name << " (QoS " << topic.qos << ")\n";
    }

    std::cout << std::endl;
}