#pragma once
#include "platform.h"
/* All integers are host-native; both participants run on the same Pi.
   bbox uses normalized (left, top, right, bottom) coordinates. */
enum { CLEAR, CRACK, DEPOSIT, FOREIGN };
typedef struct {
    uint64_t frame, captured_ms;
    int32_t kind, simulated;
    float confidence, bbox[4];
    uint32_t reserved;
} Detection;
API uint64_t ipc_clock_ms(void);
API int ipc_publish(const Detection *value);
int ipc_start(void);
bool ipc_snapshot(Detection *out);
void ipc_close(void);
