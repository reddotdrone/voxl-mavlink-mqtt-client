/*******************************************************************************
 * Copyright 2025 RED DOT DRONE PTE. LTD.
 *
 * Author: Akira Hirakawa
 *
 * Publish Timer - Manages buffered data publishing at 1Hz
 ******************************************************************************/

#ifndef PUBLISH_TIMER_H
#define PUBLISH_TIMER_H

#include <string>
#include <map>
#include <mutex>
#include <thread>
#include <chrono>

// Forward declaration
class MQTTClient;

struct BufferedData {
    std::string payload;
    std::string topic;
    int qos;
    bool has_data;
    std::chrono::steady_clock::time_point last_update;
};

class PublishTimer {
public:
    PublishTimer(MQTTClient* mqtt_client, int sleep_seconds = 1, bool debug = false);
    ~PublishTimer();

    void start();
    void stop();
    void buffer_data(int channel, const std::string& topic, const std::string& payload, int qos);
    void clear_buffered_data();

private:
    void timer_thread();

    MQTTClient* m_mqtt_client;
    std::map<int, BufferedData> m_buffered_data;
    std::mutex m_buffer_mutex;
    std::thread m_timer_thread;
    bool m_timer_running;
    int m_sleep_seconds;
    bool m_debug;
};

#endif // PUBLISH_TIMER_H