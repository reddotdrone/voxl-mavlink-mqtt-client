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

#include "mqtt_client.h"
#include <iostream>
#include <cstring>
#include <unistd.h>

// External debug flag
extern bool g_debug_mode;

MQTTClient::MQTTClient() : m_mosq(nullptr), m_connected(false), m_running(false) {
    mosquitto_lib_init();
}

MQTTClient::~MQTTClient() {
    stop();
    if (m_mosq) {
        mosquitto_destroy(m_mosq);
    }
    mosquitto_lib_cleanup();
}

bool MQTTClient::initialize(const mqtt_config_t& config) {
    m_config = config;
    
    m_mosq = mosquitto_new(config.client_id.empty() ? nullptr : config.client_id.c_str(), true, this);
    if (!m_mosq) {
        std::cerr << "Failed to create mosquitto instance" << std::endl;
        return false;
    }
    
    mosquitto_connect_callback_set(m_mosq, on_connect_wrapper);
    mosquitto_disconnect_callback_set(m_mosq, on_disconnect_wrapper);
    mosquitto_message_callback_set(m_mosq, on_message_wrapper);
    mosquitto_log_callback_set(m_mosq, on_log_wrapper);
    
    if (!config.username.empty()) {
        mosquitto_username_pw_set(m_mosq, config.username.c_str(), 
                                 config.password.empty() ? nullptr : config.password.c_str());
    }
    
    if (config.use_tls) {
        setup_tls();
    }
    
    return true;
}

bool MQTTClient::connect() {
    if (!m_mosq) {
        std::cerr << "MQTT client not initialized" << std::endl;
        return false;
    }
    
    int rc = mosquitto_connect(m_mosq, m_config.broker_host.c_str(), m_config.broker_port, m_config.keepalive);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "Failed to connect to MQTT broker: " << mosquitto_strerror(rc) << std::endl;
        return false;
    }
    
    return true;
}

bool MQTTClient::disconnect() {
    if (!m_mosq) {
        return false;
    }
    
    int rc = mosquitto_disconnect(m_mosq);
    return rc == MOSQ_ERR_SUCCESS;
}

bool MQTTClient::publish(const std::string& topic, const std::string& payload, int qos) {
    if (!m_mosq || !m_connected) {
        return false;
    }

    int rc = mosquitto_publish(m_mosq, nullptr, topic.c_str(), payload.length(), payload.c_str(), qos, false);

    if (rc == MOSQ_ERR_SUCCESS) {
        if (g_debug_mode) {
            std::cout << "Published to topic '" << topic << "': " << payload.length() << " bytes" << std::endl;
        }
    } else {
        std::cerr << "Failed to publish to topic '" << topic << "': " << mosquitto_strerror(rc) << std::endl;
    }

    return rc == MOSQ_ERR_SUCCESS;
}

bool MQTTClient::subscribe(const std::string& topic, int qos) {
    if (!m_mosq || !m_connected) {
        return false;
    }

    int rc = mosquitto_subscribe(m_mosq, nullptr, topic.c_str(), qos);

    if (rc == MOSQ_ERR_SUCCESS) {
        if (g_debug_mode) {
            std::cout << "Subscribed to topic '" << topic << "' with QoS " << qos << std::endl;
        }
    } else {
        std::cerr << "Failed to subscribe to topic '" << topic << "': " << mosquitto_strerror(rc) << std::endl;
    }

    return rc == MOSQ_ERR_SUCCESS;
}

bool MQTTClient::unsubscribe(const std::string& topic) {
    if (!m_mosq || !m_connected) {
        return false;
    }
    
    int rc = mosquitto_unsubscribe(m_mosq, nullptr, topic.c_str());
    return rc == MOSQ_ERR_SUCCESS;
}

void MQTTClient::set_on_connect_callback(std::function<void(int)> callback) {
    m_on_connect = callback;
}

void MQTTClient::set_on_disconnect_callback(std::function<void(int)> callback) {
    m_on_disconnect = callback;
}

void MQTTClient::set_on_message_callback(std::function<void(const std::string&, const std::string&)> callback) {
    m_on_message = callback;
}

bool MQTTClient::is_connected() const {
    return m_connected;
}

void MQTTClient::run() {
    m_running = true;
    m_loop_thread = std::thread(&MQTTClient::loop_forever, this);
}

void MQTTClient::stop() {
    m_running = false;
    if (m_loop_thread.joinable()) {
        m_loop_thread.join();
    }
}

void MQTTClient::on_connect_wrapper(struct mosquitto* mosq, void* obj, int result) {
    (void)mosq;
    MQTTClient* client = static_cast<MQTTClient*>(obj);
    client->m_connected = (result == 0);
    
    if (client->m_on_connect) {
        client->m_on_connect(result);
    }
}

void MQTTClient::on_disconnect_wrapper(struct mosquitto* mosq, void* obj, int result) {
    (void)mosq;
    MQTTClient* client = static_cast<MQTTClient*>(obj);
    client->m_connected = false;
    
    if (client->m_on_disconnect) {
        client->m_on_disconnect(result);
    }
}

void MQTTClient::on_message_wrapper(struct mosquitto* mosq, void* obj, const struct mosquitto_message* message) {
    (void)mosq;
    MQTTClient* client = static_cast<MQTTClient*>(obj);

    if (g_debug_mode) {
        std::string topic(message->topic);
        std::cout << "Received message on topic '" << topic << "': " << message->payloadlen << " bytes" << std::endl;
    }

    if (client->m_on_message && message->payload) {
        std::string topic(message->topic);
        std::string payload(static_cast<char*>(message->payload), message->payloadlen);
        client->m_on_message(topic, payload);
    }
}

void MQTTClient::on_log_wrapper(struct mosquitto* mosq, void* obj, int level, const char* str) {
    (void)mosq;
    (void)obj;
    if (g_debug_mode) {
        std::cout << "MQTT Log [" << level << "]: " << str << std::endl;
    }
}

void MQTTClient::setup_tls() {
    if (!m_config.ca_cert_path.empty()) {
        mosquitto_tls_set(m_mosq, 
                         m_config.ca_cert_path.c_str(),
                         nullptr,
                         m_config.cert_path.empty() ? nullptr : m_config.cert_path.c_str(),
                         m_config.key_path.empty() ? nullptr : m_config.key_path.c_str(),
                         nullptr);
    }
}

void MQTTClient::loop_forever() {
    while (m_running) {
        int rc = mosquitto_loop(m_mosq, 100, 1);
        if (rc != MOSQ_ERR_SUCCESS) {
            if (rc == MOSQ_ERR_CONN_LOST) {
                std::cout << "Connection lost, attempting to reconnect..." << std::endl;
                sleep(m_config.reconnect_delay);
                mosquitto_reconnect(m_mosq);
            } else {
                std::cerr << "MQTT loop error: " << mosquitto_strerror(rc) << std::endl;
                break;
            }
        }
    }
}