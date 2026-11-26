#include "device.h"
#include <stdio.h>
/* make hardware selects this branch. All other builds have no device access. */
#ifndef PIPE_HARDWARE
int devices_open(void) {
    return -1;
}
bool devices_sense(Sensor *s) {
    (void)s;
    return false;
}
bool devices_wheel(int preset) {
    (void)preset;
    return false;
}
bool devices_drive(int percent) {
    (void)percent;
    return false;
}
bool devices_still(void) {
    return false;
}
void devices_close(void) {
}
#else
#include <dynamixel_sdk.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <string.h>
static int port = -1, pwm = -1, id[2], direction[2], pulse[2][4], limit;
static double distance_factor;
static const char *axes[3], *distance_file;
static bool port_ready = false, mode_checked = false;
static bool number(const char *key, int low, int high, int *out) {
    const char *v = getenv(key);
    char *end;
    if (!v || !*v)
        return false;
    long n = strtol(v, &end, 0);
    if (*end || n < low || n > high)
        return false;
    *out = (int)n;
    return true;
}
static bool read_value(const char *path, double *value) {
    FILE *f = fopen(path, "r");
    if (!f)
        return false;
    int ok = fscanf(f, "%lf", value);
    fclose(f);
    return ok == 1 && isfinite(*value);
}
static bool dx_ok(void) {
    return getLastTxRxResult(port, 1) == COMM_SUCCESS && getLastRxPacketError(port, 1) == 0;
}
static bool write_reg(int reg, int val) {
    unsigned char b[2] = {(unsigned char)reg, (unsigned char)val};
    return write(pwm, b, 2) == 2;
}
int devices_open(void) {
    const char *serial = getenv("DXL_PORT"), *bus = getenv("PCA_BUS");
    const char *factor = getenv("TOF_MM_PER_UNIT");
    char *end;
    int baud, addr;
    distance_file = getenv("TOF_PATH");
    axes[0] = getenv("ACCEL_X");
    axes[1] = getenv("ACCEL_Y");
    axes[2] = getenv("ACCEL_Z");
    if (!serial || !bus || !distance_file || !factor || !axes[0] || !axes[1] || !axes[2])
        return -1;
    distance_factor = strtod(factor, &end);
    if (*end || !isfinite(distance_factor) || distance_factor <= 0)
        return -1;
    if (!number("DXL_BAUD", 9600, 1000000, &baud) || !number("PCA_ADDR", 3, 119, &addr) ||
        !number("DXL_LIMIT", 1, 1023, &limit))
        return -1;
    for (int i = 0; i < 2; i++) {
        char key[32];
        snprintf(key, sizeof key, "DXL_ID%d", i + 1);
        if (!number(key, 1, 253, id + i))
            return -1;
        snprintf(key, sizeof key, "DXL_SIGN%d", i + 1);
        if (!number(key, -1, 1, direction + i) || !direction[i])
            return -1;
    }
    if (id[0] == id[1])
        return -1;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 2; j++) {
            char key[32];
            snprintf(key, sizeof key, "WHEEL_%d_CH%d_US", j ? 250 : 200, i);
            if (!number(key, 500, 2500, &pulse[j][i]))
                return -1;
        }
    port = portHandler(serial);
    packetHandler();
    if (port < 0 || !openPort(port))
        return -1;
    port_ready = true;
    if (!setBaudRate(port, baud))
        return -1;
    for (int i = 0; i < 2; i++)
        for (int a = 6; a <= 8; a += 2) {
            int lim = read2ByteTxRx(port, 1, id[i], a);
            if (!dx_ok() || lim != 0) {
                fprintf(stderr, "AX wheel mode/communication check failed\n");
                return -1;
            }
        }
    mode_checked = true;
    if (!devices_drive(0))
        return -1;
    pwm = open(bus, O_RDWR);
    if (pwm < 0 || ioctl(pwm, I2C_SLAVE, addr) < 0)
        return -1;
    /* 25 MHz nominal, prescaler 121 -> approximately 50 Hz. */
    if (!write_reg(0, 0x10) || !write_reg(0xfe, 121) || !write_reg(0, 0x20))
        return -1;
    sleep_ms(2);
    for (int i = 0; i < 4; i++)
        if (!write_reg(0x09 + 4 * i, 0x10))
            return -1;
    for (int i = 0; i < 2; i++) {
        write1ByteTxRx(port, 1, id[i], 24, 1);
        if (!dx_ok())
            return -1;
    }
    return 0;
}
bool devices_drive(int percent) {
    bool ok = true;
    for (int i = 0; i < 2; i++) {
        int v = percent * direction[i] * limit / 100;
        int raw = v < 0 ? 1024 - v : v;
        write2ByteTxRx(port, 1, id[i], 32, (uint16_t)raw);
        if (!dx_ok())
            ok = false;
    }
    return ok;
}
bool devices_still(void) {
    for (int i = 0; i < 2; i++) {
        int speed = read2ByteTxRx(port, 1, id[i], 38);
        if (!dx_ok() || (speed & 1023) > 2)
            return false;
    }
    return true;
}
bool devices_sense(Sensor *s) {
    double d, a[3];
    if (!read_value(distance_file, &d))
        return false;
    for (int i = 0; i < 3; i++)
        if (!read_value(axes[i], a + i))
            return false;
    d *= distance_factor;
    if (d < 1 || d > 5000 || hypot(hypot(a[0], a[1]), a[2]) < 1e-6)
        return false;
    s->mm = (int)d;
    s->tilt = atan2(a[0], hypot(a[1], a[2])) * 57.2957795;
    return true;
}
bool devices_wheel(int preset) {
    if (preset != 200 && preset != 250)
        return false;
    unsigned char b[17] = {0x06};
    for (int i = 0; i < 4; i++) {
        unsigned ticks = (unsigned)lround(pulse[preset == 250][i] * 4096.0 / 20000.0);
        b[3 + 4 * i] = (unsigned char)ticks;
        b[4 + 4 * i] = (unsigned char)(ticks >> 8);
    }
    return write(pwm, b, sizeof b) == (ssize_t)sizeof b;
}
void devices_close(void) {
    if (mode_checked) {
        devices_drive(0);
        for (int i = 0; i < 2; i++)
            write1ByteTxRx(port, 1, id[i], 24, 0);
    }
    if (port_ready)
        closePort(port);
    if (pwm >= 0) {
        for (int i = 0; i < 4; i++)
            write_reg(0x09 + 4 * i, 0x10);
        close(pwm);
    }
}
#endif
