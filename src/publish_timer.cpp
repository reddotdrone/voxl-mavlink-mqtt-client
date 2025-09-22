/*******************************************************************************
 * Copyright 2025 RED DOT DRONE PTE. LTD.
 *
 * Author: Akira Hirakawa
 *
 * Publish Timer Implementation
 ******************************************************************************/

#include "publish_timer.h"
#include "mqtt_client.h"
#include <iostream>

PublishTimer::PublishTimer(MQTTClient* mqtt_client, int sleep_seconds, bool debug)
    : m_mqtt_client(mqtt_client), m_timer_running(false), m_sleep_seconds(sleep_seconds), m_debug(debug) {
}

PublishTimer::~PublishTimer() {
    stop();
}

void PublishTimer::start() {
    if (!m_timer_running) {
        m_timer_running = true;
        m_timer_thread = std::thread(&PublishTimer::timer_thread, this);
        if (m_debug) {
            std::cout << "Started publish timer (" << m_sleep_seconds << "s interval)" << std::endl;
        }
    }
}

void PublishTimer::stop() {
    if (m_timer_running) {
        m_timer_running = false;
        if (m_timer_thread.joinable()) {
            m_timer_thread.join();
        }
        if (m_debug) {
            std::cout << "Stopped publish timer" << std::endl;
        }
    }
}

void PublishTimer::buffer_data(int channel, const std::string& topic, const std::string& payload, int qos) {
    std::lock_guard<std::mutex> buffer_lock(m_buffer_mutex);
    m_buffered_data[channel] = {
        payload,
        topic,
        qos,
        true,  // has_data = true
        std::chrono::steady_clock::now()
    };
}

void PublishTimer::clear_buffered_data() {
    std::lock_guard<std::mutex> buffer_lock(m_buffer_mutex);
    m_buffered_data.clear();
}

void PublishTimer::timer_thread() {
    while (m_timer_running) {
        // Sleep for configured interval
        std::this_thread::sleep_for(std::chrono::seconds(m_sleep_seconds));

        if (!m_timer_running) break;

        std::lock_guard<std::mutex> buffer_lock(m_buffer_mutex);

        // Publish all buffered data that has been updated
        for (auto& pair : m_buffered_data) {
            BufferedData& buffer = pair.second;

            if (buffer.has_data && m_mqtt_client) {
                m_mqtt_client->publish(buffer.topic, buffer.payload, buffer.qos);
                if (m_debug) {
                    std::cout << "Timer published to topic '" << buffer.topic
                             << "' (" << buffer.payload.length() << " bytes)" << std::endl;
                }

                // Reset the has_data flag after publishing
                buffer.has_data = false;
            }
        }
    }
}