#ifndef MODAL_START_STOP_STUB_H
#define MODAL_START_STOP_STUB_H

// Stub implementations for native builds
extern volatile int main_running;

static inline void enable_signal_handler() {}
static inline int kill_existing_process(const char* name, float timeout) { (void)name; (void)timeout; return 0; }
static inline void make_pid_file(const char* name) { (void)name; }
static inline void remove_pid_file(const char* name) { (void)name; }

#endif // MODAL_START_STOP_STUB_H