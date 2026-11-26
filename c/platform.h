#pragma once
#include <stdint.h>
#include <stdbool.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <process.h>
typedef SRWLOCK pthread_mutex_t;
#define PTHREAD_MUTEX_INITIALIZER SRWLOCK_INIT
#define pthread_mutex_lock AcquireSRWLockExclusive
#define pthread_mutex_unlock ReleaseSRWLockExclusive
typedef SOCKET socket_t;
#define close_socket closesocket
#define invalid_socket INVALID_SOCKET
#define THREAD_FN(name) static unsigned __stdcall name(void *unused)
#define API __declspec(dllexport)
#else
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>
typedef int socket_t;
#define close_socket close
#define invalid_socket (-1)
#define THREAD_FN(name) static void *name(void *unused)
#define API
#endif

static inline uint64_t now_ms(void) {
#ifdef _WIN32
    return GetTickCount64();
#else
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000 + (unsigned)t.tv_nsec / 1000000;
#endif
}
static inline void sleep_ms(unsigned ms) {
#ifdef _WIN32
    Sleep(ms);
#else
    struct timespec t = {ms / 1000, (long)(ms % 1000) * 1000000};
    nanosleep(&t, NULL);
#endif
}
