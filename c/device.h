#pragma once
#include "platform.h"
typedef struct {
    int mm;
    double tilt;
    bool valid;
    uint64_t stamp;
} Sensor;
int devices_open(void);
bool devices_sense(Sensor *s);
bool devices_wheel(int preset);
bool devices_drive(int percent);
bool devices_still(void);
void devices_close(void);
