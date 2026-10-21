#ifndef GP_HARDWARE_H
#define GP_HARDWARE_H
#include "gp_types.h"

typedef struct { uint16_t prescaler, autoreload, compare; } GpTimerPlan;
GpResult GpTimer_PwmPlan(uint32_t clock_hz, uint32_t hz, uint16_t duty_pm, GpTimerPlan *out);
GpResult GpTimer_StepPlan(uint32_t clock_hz, uint32_t hz, uint16_t high_us,
                          uint16_t low_us, GpTimerPlan *out);
bool GpCanTiming_Valid(uint32_t clock_hz, uint32_t bitrate, uint16_t prescaler,
                       uint8_t bs1, uint8_t bs2, uint8_t sjw);

typedef struct {
    uint32_t timer_clock_hz, pwm_hz, max_step_hz;
    uint16_t duty_cap_pm, pulse_us, low_us, dir_setup_us, enable_setup_us;
    bool approved;
} GpMotorHwConfig;
typedef struct {
    void *user;
    uint32_t (*now_us)(void *user);
    void (*off)(void *user);
    bool (*direction)(void *user, int8_t sign);
    bool (*apply)(void *user, const GpTimerPlan *pwm, const GpTimerPlan *step);
    bool (*permitted)(void *user);
    /* Optional bounded power/ENA readiness phase BEFORE changing DIR.
     * GP_PENDING keeps both waveforms off; never starts output asynchronously.
     * off() cancels it. Board implementations may enforce long (e.g. 200 ms)
     * power/ENA settling without truncating the public 16-bit setup fields.
     * NULL means no electrical-readiness mechanism is implemented, never that
     * a driver is electrically ready. Current-board SignalTest uses NULL with
     * motor energy disconnected; powered operation is separately unsupported. */
    GpResult (*prepare)(void *user);
} GpMotorHwPort;
typedef struct {
    GpMotorHwConfig config;
    GpMotorHwPort port;
    uint32_t setup_since_us;
    int8_t direction;
    bool initialized, pending, stepping, preparing;
} GpMotorHw;

/* Callback-compatible policy, used by the real BSP and by host tests.
 * Port off stops waveforms synchronously; it need not remove holding torque.
 * No autonomous deferred start. DIR setup starts after direction() succeeds.
 * enable_setup_us is only an additional inhibit interval when no ENA exists;
 * this module cannot claim ENA timing/control without a wired enable. */
GpResult GpMotorHw_Init(GpMotorHw *, const GpMotorHwConfig *, const GpMotorHwPort *);
GpResult GpMotorHw_Write(void *user, const MotorOutput *);
void GpMotorHw_Off(void *user);

typedef struct { uint32_t motor_ms, can_ms, deadline_ms; bool motor_seen, can_seen; } GpTaskWatchdog;
bool GpTaskWatchdog_Ready(const GpTaskWatchdog *, uint32_t now_ms);
/* Returns remaining wait, advancing only absolute periodic deadlines. */
uint32_t GpSchedule_Wait(uint32_t now_ms, uint32_t period_ms, uint32_t *deadline_ms);
#endif
