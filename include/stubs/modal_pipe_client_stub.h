/*******************************************************************************
 * Copyright 2025 RED DOT DRONE PTE. LTD.
 *
 * Author: Akira Hirakawa
 *
 * Native build stub functions for VOXL MQTT Client
 * These provide dummy implementations for ModalAI and pipe functions
 * when building natively for development/testing
 ******************************************************************************/

#ifndef MODAL_PIPE_CLIENT_STUB_H
#define MODAL_PIPE_CLIENT_STUB_H

#if defined(__x86_64__)

#include <cstddef>

// Global variable declaration for native builds
extern volatile int main_running;

extern "C" {
    
    // Pipe client functionality stubs
    inline int pipe_client_open(int ch, const char* name, const char* client_name, int flags, int buf_size) {
        (void)ch; (void)name; (void)client_name; (void)flags; (void)buf_size; return 0;
    }
    
    inline int pipe_client_set_simple_helper_cb(int ch, void (*cb)(int, char*, int, void*), void* context) {
        (void)ch; (void)cb; (void)context; return 0;
    }
    
    inline int pipe_client_set_connect_cb(int ch, void (*cb)(int, void*), void* context) {
        (void)ch; (void)cb; (void)context; return 0;
    }
    
    inline int pipe_client_set_disconnect_cb(int ch, void (*cb)(int, void*), void* context) {
        (void)ch; (void)cb; (void)context; return 0;
    }
    
    inline void pipe_client_close_all() {}
    
    inline int pipe_is_type(const char* name, const char* type) {
        (void)name; (void)type; return 1;
    }
    
    // MAVLink validation stub
    inline void* pipe_validate_mavlink_message_t(char* data, int bytes, int* n_packets) {
        (void)data; (void)bytes; *n_packets = 0; return nullptr;
    }
}

#endif // defined(__x86_64__)

#endif // MODAL_PIPE_CLIENT_STUB_H