#ifndef MODAL_PIPE_STUB_H
#define MODAL_PIPE_STUB_H

#ifdef __APPLE__
static inline int pipe_client_get_fd(const char* name) { (void)name; return -1; }
static inline int pipe_client_read(int fd, char* buffer, int size) { (void)fd; (void)buffer; (void)size; return 0; }
static inline void pipe_client_close(int fd) { (void)fd; }
static inline int pipe_server_create(const char* name) { (void)name; return -1; }
static inline int pipe_server_write(int fd, char* buffer, int size) { (void)fd; (void)buffer; (void)size; return 0; }
static inline void pipe_server_close(int fd) { (void)fd; }
#else
#include <modal_pipe.h>
#endif

#endif // MODAL_PIPE_STUB_H