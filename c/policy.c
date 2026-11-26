#include "policy.h"
#include <math.h>
static bool fresh(uint64_t now, uint64_t stamp, uint64_t age) {
    return stamp && stamp <= now && now - stamp <= age;
}
Output decide(Policy *p, uint64_t now, const Detection *ai, Sensor s, Command cmd, bool drive_ok,
              bool still, bool physical, int near_mm) {
    Output out = {0, 0, false, "DISARMED", now};
    bool arm_event = cmd.arm != p->arm_seen;
    bool stop_event = cmd.stop != p->stop_seen;
    p->arm_seen = cmd.arm;
    p->stop_seen = cmd.stop;
    bool ai_ok = ai->frame && fresh(now, ai->captured_ms, 1500) && ai->kind >= CLEAR &&
                 ai->kind <= FOREIGN && isfinite(ai->confidence) && ai->confidence >= 0 &&
                 ai->confidence <= 1 && (!physical || ai->simulated == 0);
    int kind = ai_ok && ai->confidence >= 0.60f ? ai->kind : CLEAR;
    if (!ai_ok) {
        p->streak = 0;
        p->last_kind = CLEAR;
    } else if (ai->frame != p->last_frame || ai->captured_ms != p->last_capture) {
        bool consecutive = ai->frame == p->last_frame + 1 && ai->captured_ms >= p->last_capture &&
                           ai->captured_ms - p->last_capture <= 1500;
        p->streak =
            (kind == CLEAR) ? 0 : (consecutive && kind == p->last_kind && p->streak ? 2 : 1);
        p->last_frame = ai->frame;
        p->last_capture = ai->captured_ms;
        p->last_kind = kind;
    }
    const char *fault = NULL;
    if (stop_event)
        fault = "STOP";
    else if (!drive_ok)
        fault = "DRIVE_FAULT";
    else if (!fresh(now, cmd.stamp, 500))
        fault = "CMD_TIMEOUT";
    else if (!s.valid || !fresh(now, s.stamp, 300))
        fault = "SENSOR_STALE";
    else if (s.mm <= near_mm)
        fault = "OBSTACLE";
    else if (!ai_ok)
        fault = "AI_STALE";
    else if (p->streak >= 2)
        fault = kind == CRACK ? "CRACK" : "BLOCKAGE";
    if (fault) {
        p->armed = false;
        out.reason = fault;
        return out;
    }
    if (arm_event && cmd.speed == 0 && !cmd.wheel && still && kind == CLEAR)
        p->armed = true;
    out.armed = p->armed;
    if (!p->armed)
        return out;
    if (cmd.wheel) {
        out.wheel = still && cmd.speed == 0 ? cmd.wheel : 0;
        out.reason = out.wheel ? "ADJUST" : "WAIT_STOP";
        return out;
    }
    out.speed = cmd.speed;
    if (kind != CLEAR) {
        out.speed = cmd.speed / 2;
        out.reason = "SLOW";
    } else
        out.reason = cmd.speed ? "DRIVE" : "READY";
    return out;
}
