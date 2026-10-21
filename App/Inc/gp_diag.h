#ifndef GP_DIAG_H
#define GP_DIAG_H
#include "gp_control.h"
typedef struct {
    uint32_t uptime_ms, blockers, reset_flags, panic, state, faults, history;
    uint32_t captured, dropped, decoded, rejected, stale, published, barred;
    uint32_t decoded_ms, recovery_requests;
    uint32_t can_btr, can_esr;
    int32_t recovery_result;
    TargetSample decoded_command, requested;
    MotorOutput applied;
    bool can_ready, motor_ready, watchdog;
} GpDiagStatus;
/* Task context: returns a coherent copy, never a pointer to controller state. */
void GpApp_GetStatus(GpDiagStatus *);
void GpDiag_Start(void);
void GpDiag_Interrupt(void);
#endif
