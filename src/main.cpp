/*******************************************************************************
 * Copyright 2025 RED DOT DRONE PTE. LTD.
 *
 * Author: Akira Hirakawa
 *
 * VOXL MQTT Client - Bridges VOXL Modal Pipe system with MQTT brokers
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

// Standard C++ includes
#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <map>
#include <mutex>
#include <ctime>  // For std::time

// ModalAI includes
#if defined(__x86_64__)
// Modal pipe client stubs
#include "modal_start_stop_stub.h"
#include "modal_pipe_stub.h"
#include "modal_pipe_client_stub.h"
#else
#include <c_library_v2/common/mavlink.h>
#include <modal_start_stop.h>
#include <modal_pipe_client.h>
#endif

// MQTT client components
#include "mqtt_client.h"
#include "config_file.h"
#include "mavlink_json.h"
#include "publish_timer.h"

#define PROCESS_NAME "voxl-mqtt-client"

// Global state variables
volatile int main_running = 0;                       // Application running flag
static MQTTClient* g_mqtt_client = nullptr;          // MQTT client instance
static mqtt_config_t g_config;                       // Configuration loaded from file
static std::map<std::string, int> g_publish_pipes;   // Map pipe names to channels for publishing
static std::map<int, std::string> g_channel_to_topic; // Map pipe channel to MQTT topic
static std::mutex g_publish_mutex;                   // Thread safety for publish operations
static PublishTimer* g_publish_timer = nullptr;      // Timer-based publishing system

#define PIPE_READ_BUF_SIZE 4096
#define CLIENT_NAME "voxl-mqtt-client"

/**
 * MQTT connection callback - called when connection status changes
 */
static void on_mqtt_connect(int result) {
    if (result == 0) {
        std::cout << "Connected to MQTT broker" << std::endl;
    } else {
        std::cerr << "Failed to connect to MQTT broker: " << result << std::endl;
    }
}

/**
 * MQTT disconnection callback - called when broker connection is lost
 */
static void on_mqtt_disconnect(int result) {
    std::cout << "Disconnected from MQTT broker with result: " << result << std::endl;
}

/**
 * Pipe client callback - called when data arrives from a VOXL pipe
 * Buffers the data for timer-based publishing at 1Hz
 */
static void pipe_data_callback(int ch, char* data, int bytes, __attribute__((unused)) void* context) {
    // Find the MQTT topic for this channel
    auto topic_it = g_channel_to_topic.find(ch);
    if (topic_it != g_channel_to_topic.end()) {
        std::string payload;

#if defined(__x86_64__)
        // For native builds, just use raw data
        payload = std::string(data, bytes);
#else
        // For VOXL builds, auto-detect and parse data (MAVLink, VIO, etc.)
        std::string pipe_name;
        // Find the pipe name for this channel
        for (const auto& pub_topic : g_config.publish_topics) {
            if (g_publish_pipes.find(pub_topic.pipe_name) != g_publish_pipes.end() &&
                g_publish_pipes[pub_topic.pipe_name] == ch) {
                pipe_name = pub_topic.pipe_name;
                break;
            }
        }

        if (!parse_pipe_data_to_json(pipe_name, data, bytes, payload)) {
            // If all parsing fails, fall back to raw data
            payload = std::string(data, bytes);
            std::cout << "Data parsing failed for pipe '" << pipe_name << "', using raw data" << std::endl;
        }
#endif

        // Find QoS level for this topic
        int qos = 0;
        for (const auto& pub_topic : g_config.publish_topics) {
            if (pub_topic.topic == topic_it->second) {
                qos = pub_topic.qos;
                break;
            }
        }

        // Buffer the data for timer-based publishing
        if (g_publish_timer) {
            g_publish_timer->buffer_data(ch, topic_it->second, payload, qos);
        }

        std::cout << "Buffered " << bytes << " bytes from pipe channel " << ch
                  << " for topic: " << topic_it->second << std::endl;
    }
}

/**
 * Pipe client connect callback - called when pipe client connects
 */
static void pipe_connect_callback(int ch, __attribute__((unused)) void* context) {
    std::cout << "Pipe client channel " << ch << " connected" << std::endl;
}

/**
 * Pipe client disconnect callback - called when pipe client disconnects  
 */
static void pipe_disconnect_callback(int ch, __attribute__((unused)) void* context) {
    std::cout << "Pipe client channel " << ch << " disconnected" << std::endl;
}

/**
 * Initialize all Modal Pipe connections
 * Sets up client pipes for publishing to MQTT
 * @return 0 on success, -1 on failure
 */
static int setup_pipes() {
    std::lock_guard<std::mutex> pub_lock(g_publish_mutex);
    
#if !defined(__x86_64__)
    // Set up client pipes for reading VOXL data and publishing to MQTT
    int ch = 0;
    for (const auto& pub_topic : g_config.publish_topics) {
        int flags = CLIENT_FLAG_EN_SIMPLE_HELPER;
        
        // Set up callbacks for this channel
        pipe_client_set_simple_helper_cb(ch, pipe_data_callback, NULL);
        pipe_client_set_connect_cb(ch, pipe_connect_callback, NULL);
        pipe_client_set_disconnect_cb(ch, pipe_disconnect_callback, NULL);
        
        // Open the pipe client connection
        int ret = pipe_client_open(ch, pub_topic.pipe_name.c_str(), CLIENT_NAME, flags, PIPE_READ_BUF_SIZE);
        
        if (ret != 0) {
            std::cerr << "Failed to open pipe client for " << pub_topic.pipe_name << ": " << ret << std::endl;
            continue;
        }
        
        g_publish_pipes[pub_topic.pipe_name] = ch;
        g_channel_to_topic[ch] = pub_topic.topic;
        std::cout << "Opened publish pipe: " << pub_topic.pipe_name << " on channel " << ch << std::endl;
        ch++;
    }
#endif
    
    return 0;
}

/**
 * Clean up all pipe connections on shutdown
 */
static void cleanup_pipes() {
    // Stop the timer first
    if (g_publish_timer) {
        g_publish_timer->stop();
    }

    std::lock_guard<std::mutex> pub_lock(g_publish_mutex);

#if !defined(__x86_64__)
    pipe_client_close_all();
#endif
    g_publish_pipes.clear();
    g_channel_to_topic.clear();

    // Clear buffered data
    if (g_publish_timer) {
        g_publish_timer->clear_buffered_data();
    }
}

/**
 * Signal handler for graceful shutdown
 * Handles SIGINT (Ctrl+C) and SIGTERM signals
 */
static void signal_handler(int sig) {
    std::cout << "\nReceived signal " << sig << ", shutting down..." << std::endl;
    main_running = 0;  // Set flag to stop main loop
}

/**
 * Print command-line usage information
 */
static void print_usage() {
    std::cout << "Usage: " << PROCESS_NAME << " [options]\n";
    std::cout << "Options:\n";
    std::cout << "  -h, --help         Show this help message\n";
    std::cout << "  -c, --config       Print current configuration\n";
    std::cout << "  -s, --save-config  Save default configuration file\n";
    std::cout << "  -v, --verbose      Enable verbose logging\n";
    std::cout << std::endl;
}

/**
 * Main entry point for VOXL MQTT Client
 * Bridges VOXL Modal Pipe system with MQTT broker
 */
int main(int argc, char* argv[]) {
    bool verbose = false;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            print_usage();
            return 0;
        } else if (arg == "-c" || arg == "--config") {
            if (load_config(&g_config) != 0) {
                std::cerr << "Failed to load configuration" << std::endl;
                return -1;
            }
            print_config(&g_config);
            return 0;
        } else if (arg == "-s" || arg == "--save-config") {
            if (save_default_config() != 0) {
                std::cerr << "Failed to save default configuration" << std::endl;
                return -1;
            }
            std::cout << "Default configuration saved to " << CONFIG_FILE_PATH << std::endl;
            return 0;
        } else if (arg == "-v" || arg == "--verbose") {
            verbose = true;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            print_usage();
            return -1;
        }
    }
    
    // Set up signal handlers for graceful shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Ensure only one instance runs at a time
    if (kill_existing_process(PROCESS_NAME, 2.0) < -2) {
        std::cerr << "Failed to kill existing process" << std::endl;
        return -1;
    }
    
    // Create PID file for process management
    make_pid_file(PROCESS_NAME);
    
    // Load configuration from file
    if (load_config(&g_config) != 0) {
        std::cerr << "Failed to load configuration" << std::endl;
        return -1;
    }
    
    if (verbose) {
        print_config(&g_config);
    }
    
    // Initialize MQTT client with loaded configuration
    g_mqtt_client = new MQTTClient();
    if (!g_mqtt_client->initialize(g_config)) {
        std::cerr << "Failed to initialize MQTT client" << std::endl;
        delete g_mqtt_client;
        return -1;
    }

    // Initialize publish timer
    g_publish_timer = new PublishTimer(g_mqtt_client);
    
    // Register MQTT event callbacks
    g_mqtt_client->set_on_connect_callback(on_mqtt_connect);
    g_mqtt_client->set_on_disconnect_callback(on_mqtt_disconnect);
    
    // Initialize Modal Pipe connections
    if (setup_pipes() != 0) {
        std::cerr << "Failed to setup pipes" << std::endl;
        delete g_mqtt_client;
        return -1;
    }
    
    // Establish initial MQTT connection
    if (!g_mqtt_client->connect()) {
        std::cerr << "Failed to connect to MQTT broker" << std::endl;
        cleanup_pipes();
        delete g_mqtt_client;
        return -1;
    }
    
    // Start MQTT client background thread
    g_mqtt_client->run();

    // Start 1Hz timer for publishing buffered data
    g_publish_timer->start();


    main_running = 1;
    std::cout << "VOXL MQTT Client started" << std::endl;
    
    // Main loop - monitor connection and handle reconnection
    while (main_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // Check if MQTT connection is still alive
        if (!g_mqtt_client->is_connected()) {
            std::cout << "MQTT connection lost, attempting to reconnect..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(g_config.reconnect_delay));
            g_mqtt_client->connect();
        }
    }
    
    // Graceful shutdown sequence
    std::cout << "Shutting down..." << std::endl;
    
    // Stop MQTT client background thread
    g_mqtt_client->stop();
    
    
    // Clean up all pipe connections
    cleanup_pipes();
    delete g_mqtt_client;
    delete g_publish_timer;
    
    // Remove PID file
    remove_pid_file(PROCESS_NAME);
    
    return 0;
}