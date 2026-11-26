#include "policy.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
/* Each mutex protects an entire snapshot, including its timestamp. */
static pthread_mutex_t sensor_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t command_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t actuator_mutex = PTHREAD_MUTEX_INITIALIZER;
static Sensor sensor;
static Command command;
static Output target = {0, 0, false, "BOOT", 0};
static Detection reported_ai;
static bool drive_ok = true, still = false, servo_ok = true;
static uint64_t settle_until, motor_feedback;
static int applied_speed, preset, sim_mm = 1000, near_mm = 180, settle_ms = 1800;
static bool physical = false;
static volatile sig_atomic_t interrupted = 0;
static pthread_mutex_t run_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool running = true;
static socket_t udp = invalid_socket;
static char trusted_ip[INET_ADDRSTRLEN] = "127.0.0.1";
static struct in_addr trusted_addr;
static void on_signal(int sig) {
    (void)sig;
    interrupted = 1;
}
static bool active(void) {
    pthread_mutex_lock(&run_mutex);
    bool value = running;
    pthread_mutex_unlock(&run_mutex);
    return value;
}
static void stop_all(void) {
    pthread_mutex_lock(&run_mutex);
    running = false;
    pthread_mutex_unlock(&run_mutex);
}
static void next_tick(uint64_t *deadline, unsigned period) {
    *deadline += period;
    uint64_t now = now_ms();
    if (*deadline <= now) {
        *deadline = now;
        return;
    }
    sleep_ms((unsigned)(*deadline - now));
}
THREAD_FN(sensors) {
    (void)unused;
    while (active()) {
        Sensor local = {0};
        local.stamp = now_ms();
        pthread_mutex_lock(&sensor_mutex);
        local.mm = sim_mm;
        pthread_mutex_unlock(&sensor_mutex);
        local.valid = !physical || devices_sense(&local);
        pthread_mutex_lock(&sensor_mutex);
        sensor = local;
        pthread_mutex_unlock(&sensor_mutex);
        pthread_mutex_lock(&actuator_mutex);
        int want = target.wheel;
        bool change = want && want != preset && still && now_ms() - motor_feedback < 150 &&
                      now_ms() - target.stamp < 100 && !target.speed && servo_ok && drive_ok;
        if (change)
            settle_until = UINT64_MAX;
        pthread_mutex_unlock(&actuator_mutex);
        if (change) {
            bool ok = !physical || devices_wheel(want);
            pthread_mutex_lock(&actuator_mutex);
            if (ok)
                preset = want;
            else
                servo_ok = false;
            settle_until = now_ms() + settle_ms;
            pthread_mutex_unlock(&actuator_mutex);
        }
        sleep_ms(40);
    }
    return 0;
}
THREAD_FN(motors) {
    (void)unused;
    uint64_t stopped_since = 0;
    while (active()) {
        uint64_t now = now_ms();
        pthread_mutex_lock(&actuator_mutex);
        int speed = (target.armed && drive_ok && servo_ok && now >= settle_until &&
                     now >= target.stamp && now - target.stamp < 100)
                        ? target.speed
                        : 0;
        if (speed)
            still = false;
        pthread_mutex_unlock(&actuator_mutex);
        bool ok = !physical || devices_drive(speed);
        if (!ok && physical)
            devices_drive(0);
        bool stopped = ok && speed == 0 && (!physical || devices_still());
        if (!stopped)
            stopped_since = 0;
        else if (!stopped_since)
            stopped_since = now;
        pthread_mutex_lock(&actuator_mutex);
        if (!ok)
            drive_ok = false;
        applied_speed = ok ? speed : 0;
        motor_feedback = now_ms();
        still = stopped && now - stopped_since >= 200;
        pthread_mutex_unlock(&actuator_mutex);
        sleep_ms(20);
    }
    if (physical)
        devices_drive(0);
    return 0;
}
THREAD_FN(control) {
    (void)unused;
    Detection ai = {0};
    Policy state = {0};
    uint64_t tick = now_ms();
    while (active()) {
        ipc_snapshot(&ai); /* sem_trywait: retain last sample when busy. */
        pthread_mutex_lock(&sensor_mutex);
        Sensor s = sensor;
        pthread_mutex_unlock(&sensor_mutex);
        pthread_mutex_lock(&command_mutex);
        Command cmd = command;
        pthread_mutex_unlock(&command_mutex);
        pthread_mutex_lock(&actuator_mutex);
        uint64_t now = now_ms();
        bool ok = drive_ok && servo_ok && motor_feedback && now - motor_feedback < 200;
        bool stationary = still;
        pthread_mutex_unlock(&actuator_mutex);
        Output o = decide(&state, now, &ai, s, cmd, ok, stationary, physical, near_mm);
        pthread_mutex_lock(&actuator_mutex);
        if (now < settle_until && o.armed) {
            o.speed = 0;
            o.reason = "ADJUSTING";
        }
        target = o;
        reported_ai = ai;
        pthread_mutex_unlock(&actuator_mutex);
        next_tick(&tick, 20);
    }
    return 0;
}
static void status(char *buf, size_t n, long seq) {
    pthread_mutex_lock(&sensor_mutex);
    Sensor s = sensor;
    pthread_mutex_unlock(&sensor_mutex);
    pthread_mutex_lock(&actuator_mutex);
    Output t = target;
    int speed = applied_speed, w = preset;
    Detection d = reported_ai;
    pthread_mutex_unlock(&actuator_mutex);
    snprintf(buf, n,
             "{\"seq\":%ld,\"reason\":\"%s\",\"armed\":%d,\"speed\":%d,\"requested_output\":%d,"
             "\"wheel\":%d,\"mm\":%d,\"tilt_deg\":%.2f,\"sensor_ok\":%d,"
             "\"defect\":\"%s\",\"confidence\":%.3f,\"frame\":%llu}",
             seq, t.reason, t.armed, speed, t.speed, w, s.mm, s.tilt, s.valid,
             d.kind == CRACK     ? "crack"
             : d.kind == DEPOSIT ? "deposit"
             : d.kind == FOREIGN ? "foreign"
                                 : "clear",
             isfinite(d.confidence) ? d.confidence : 0.0, (unsigned long long)d.frame);
}
static bool parse_int(const char *v, long low, long high, long *out) {
    if (!v || !*v)
        return false;
    errno = 0;
    char *end;
    long num = strtol(v, &end, 10);
    if (errno || *end || num < low || num > high)
        return false;
    *out = num;
    return true;
}
/* Wire command: PB1 <seq> <action> <value>. One ground station per tether. */
THREAD_FN(network) {
    (void)unused;
    unsigned short owner_port = 0;
    long last_seq = 0;
    while (active()) {
        struct sockaddr_in peer;
#ifdef _WIN32
        int len = sizeof peer;
#else
        socklen_t len = sizeof peer;
#endif
        char buf[256], reply[320];
        int n = (int)recvfrom(udp, buf, sizeof buf - 1, 0, (struct sockaddr *)&peer, &len);
        if (n < 1 || n > 200 || memchr(buf, 0, (size_t)n))
            continue;
        if (peer.sin_addr.s_addr != trusted_addr.s_addr)
            continue;
        buf[n] = 0;
        char prot[8], seqtxt[24], op[16], arg[16], extra;
        if (sscanf(buf, "%7s %23s %15s %15s %c", prot, seqtxt, op, arg, &extra) != 4 ||
            strcmp(prot, "PB1"))
            continue;
        long seq, value;
        if (!parse_int(seqtxt, 1, INT_MAX, &seq) || !parse_int(arg, -5000, 5000, &value))
            continue;
        uint64_t now = now_ms();
        pthread_mutex_lock(&command_mutex);
        bool expired = !command.stamp || now - command.stamp > 500;
        bool same = owner_port == peer.sin_port;
        if (expired && !same)
            last_seq = 0;
        bool accept = (same || expired) && seq > last_seq;
        if (accept) {
            if (!strcmp(op, "STOP") && !value) {
                command.stop++;
                command.speed = 0;
                command.wheel = 0;
            } else if (!strcmp(op, "ARM") && !value) {
                command.arm++;
                command.speed = 0;
                command.wheel = 0;
            } else if (!strcmp(op, "DRIVE") && value >= -100 && value <= 100) {
                command.speed = (int)value;
                command.wheel = 0;
            } else if (!strcmp(op, "WHEEL") && (value == 200 || value == 250)) {
                command.speed = 0;
                command.wheel = (int)value;
            } else if (!strcmp(op, "PING") && !value) {
            } else if (!physical && !strcmp(op, "SIMDIST") && value >= 0) {
                pthread_mutex_lock(&sensor_mutex);
                sim_mm = (int)value;
                pthread_mutex_unlock(&sensor_mutex);
            } else
                accept = false;
        }
        if (accept) {
            if (expired) {
                command.stop++;
                command.speed = 0;
                command.wheel = 0;
            }
            owner_port = peer.sin_port;
            last_seq = seq;
            command.stamp = now;
        }
        pthread_mutex_unlock(&command_mutex);
        if (accept) {
            status(reply, sizeof reply, seq);
            sendto(udp, reply, (int)strlen(reply), 0, (struct sockaddr *)&peer, len);
        }
    }
    return 0;
}
int main(int argc, char **argv) {
    const char *bind_ip = "127.0.0.1";
    long portnum = 43771;
    if (argc != 1 && argc != 5 && argc != 7)
        goto usage;
    if (argc >= 5) {
        physical = !strcmp(argv[1], "--hardware");
        if (!physical && strcmp(argv[1], "--sim"))
            goto usage;
        bind_ip = argv[2];
        struct in_addr a;
        if (inet_pton(AF_INET, argv[3], &a) != 1 || strlen(argv[3]) >= sizeof trusted_ip)
            goto usage;
        strcpy(trusted_ip, argv[3]);
        if (!parse_int(argv[4], 1024, 65535, &portnum))
            goto usage;
    }
    if (argc == 7) {
        long d, t;
        if (!parse_int(argv[5], 1, 5000, &d) || !parse_int(argv[6], 100, 30000, &t))
            goto usage;
        near_mm = (int)d;
        settle_ms = (int)t;
    } else if (physical)
        goto usage;
#ifdef _WIN32
    WSADATA w;
    if (WSAStartup(MAKEWORD(2, 2), &w))
        return 1;
#endif
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    if (inet_pton(AF_INET, trusted_ip, &trusted_addr) != 1)
        return 2;
    udp = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons((unsigned short)portnum);
    if (udp == invalid_socket || inet_pton(AF_INET, bind_ip, &addr.sin_addr) != 1 ||
        bind(udp, (struct sockaddr *)&addr, sizeof addr) != 0) {
        fprintf(stderr, "UDP bind failed\n");
        return 1;
    }
#ifdef _WIN32
    DWORD timeout = 100;
#else
    struct timeval timeout = {0, 100000};
#endif
    if (setsockopt(udp, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof timeout) != 0)
        return 1;
    if (ipc_start() != 0) {
        fprintf(stderr, "IPC busy/unavailable\n");
        return 1;
    }
    if (physical && devices_open() != 0) {
        fprintf(stderr, "Hardware/config check failed\n");
        devices_close();
        ipc_close();
        return 1;
    }
#ifdef _WIN32
    HANDLE th[4];
    unsigned(__stdcall * fn[])(void *) = {sensors, motors, control, network};
#else
    pthread_t th[4];
    void *(*fn[])(void *) = {sensors, motors, control, network};
#endif
    int started = 0;
    for (; started < 4; started++) {
#ifdef _WIN32
        th[started] = (HANDLE)_beginthreadex(NULL, 0, fn[started], NULL, 0, NULL);
        if (!th[started])
            break;
#else
        if (pthread_create(&th[started], NULL, fn[started], NULL) != 0)
            break;
#endif
    }
    printf("Pipe robot %s | %s:%ld\n", physical ? "HARDWARE" : "SIM", bind_ip, portnum);
    fflush(stdout);
    while (started == 4 && !interrupted)
        sleep_ms(50);
    stop_all();
    for (int i = 0; i < started; i++) {
#ifdef _WIN32
        WaitForSingleObject(th[i], INFINITE);
        CloseHandle(th[i]);
#else
        pthread_join(th[i], NULL);
#endif
    }
    if (physical)
        devices_close();
    ipc_close();
    close_socket(udp);
    return started == 4 ? 0 : 1;
usage:
    fprintf(stderr, "robot [--sim|--hardware bind_IP station_IP port [stop_mm settle_ms]]\n");
    return 2;
}
