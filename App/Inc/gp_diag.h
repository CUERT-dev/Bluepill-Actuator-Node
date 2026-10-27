#ifndef GP_DIAG_H
#define GP_DIAG_H
#include "gp_control.h"
typedef struct { uint32_t psc, arr, ccr, cr1, ccer; } GpDiagTimer;
typedef struct {
    uint32_t uptime_ms, blockers, reset_flags, panic, state, faults, history;
    uint32_t captured, dropped, decoded, rejected, stale, published, barred;
    uint32_t decoded_ms, recovery_requests;
    uint32_t can_btr, can_esr;
    int32_t recovery_result;
    TargetSample decoded_command, requested;
    MotorOutput applied;
    bool can_ready, motor_ready, watchdog;
    uint32_t motor_runs, motor_ms, requested_ms, shutdown_reason, shutdown_count;
    int32_t motor_result;
    uint16_t last_rx_id;
    uint8_t last_rx_dlc;
    uint8_t last_rx_data[8];
    uint32_t last_rx_ms;
    bool last_rx_extended, last_rx_remote;
    bool have_requested, pending;
    bool direct_commands;
    bool rx_waiting;
    uint32_t rx_wait_count;
    GpDiagTimer tim2, tim3;
    uint32_t timer_clock_hz;
    uint32_t gpioa_crl, gpiob_crl, gpioa_idr, gpiob_idr, gpioa_odr, gpiob_odr, mapr;
} GpDiagStatus;
/* Pure formatting, also exercised by host tests. No peripheral access. */
int GpDiag_FormatMotion(char *, size_t, const GpDiagStatus *);
int GpDiag_FormatRx(char *, size_t, const GpDiagStatus *);
/* Task context: returns a coherent copy, never a pointer to controller state. */
void GpApp_GetStatus(GpDiagStatus *);
void GpDiag_Start(void);
void GpDiag_Interrupt(void);
#endif
