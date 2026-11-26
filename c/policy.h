#pragma once
#include "ipc.h"
#include "device.h"
typedef struct {
    int speed, wheel;
    uint64_t stamp, arm, stop;
} Command;
typedef struct {
    int speed, wheel;
    bool armed;
    const char *reason;
    uint64_t stamp;
} Output;
typedef struct {
    bool armed;
    unsigned streak;
    int last_kind;
    uint64_t last_frame, last_capture, arm_seen, stop_seen;
} Policy;
Output decide(Policy *p, uint64_t now, const Detection *ai, Sensor sensor, Command cmd,
              bool drive_ok, bool still, bool physical, int near_mm);
