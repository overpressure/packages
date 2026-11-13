// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: Unlicense
/*
 * The few bionic (Android C library) declarations pico4-slamd and its stand-ins use. They are
 * built with clang for aarch64-linux-android29 against PICO OS's own bionic libraries, without an
 * NDK (docs/xr.md#head-tracking); types and calls are those of LP64 Linux.
 */
#pragma once
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

typedef long ssize_t;
typedef long off_t;
typedef unsigned int uid_t;
typedef unsigned int gid_t;
typedef unsigned int mode_t;
typedef long pthread_t;
typedef struct
{
	long tv_sec;
	long tv_nsec;
} timespec_t;

int fprintf(void *, const char *, ...);
int snprintf(char *, size_t, const char *, ...);
int vsnprintf(char *, size_t, const char *, va_list);
int fflush(void *);
extern void *stderr;
void *malloc(size_t);
void *calloc(size_t, size_t);
void free(void *);
void *memset(void *, int, size_t);
void *memcpy(void *, const void *, size_t);
int strcmp(const char *, const char *);
int atoi(const char *);
char *getenv(const char *);
int usleep(unsigned);
int clock_gettime(int, timespec_t *);
int open(const char *, int, ...);
int close(int);
ssize_t read(int, void *, size_t);
int ftruncate(int, off_t);
int fchown(int, uid_t, gid_t);
int fchmod(int, mode_t);
void *mmap(void *, size_t, int, int, int, off_t);
void exit(int);
void *dlopen(const char *, int);
void *dlsym(void *, const char *);
char *dlerror(void);
int pthread_create(pthread_t *, const void *, void *(*)(void *), void *);
int pthread_join(pthread_t, void **);
void (*signal(int, void (*)(int)))(int);
ssize_t write(int, const void *, size_t);
int rename(const char *, const char *);
int unlink(const char *);
double atan2(double, double);
double sin(double);
double cos(double);
double sqrt(double);

#define RTLD_NOW 2
#define O_RDONLY 0
#define O_WRONLY 1
#define O_RDWR 2
#define O_TRUNC 01000
#define SIGTERM 15
#define SIGINT 2
#define O_CREAT 0100
#define O_CLOEXEC 02000000
#define CLOCK_MONOTONIC 1
#define CLOCK_PROCESS_CPUTIME_ID 2
#define CLOCK_BOOTTIME 7
#define PROT_READ 1
#define PROT_WRITE 2
#define MAP_SHARED 1

static inline int64_t
now_ns(int clock)
{
	timespec_t t;
	clock_gettime(clock, &t);
	return (int64_t)t.tv_sec * 1000000000 + t.tv_nsec;
}
