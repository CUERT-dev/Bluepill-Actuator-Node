#include "gp_hardware.h"
#include "gp_board_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)

typedef struct {
    uint32_t now;
    unsigned off_count, dir_count, apply_count;
    bool permitted, fail_dir, fail_apply, active;
    int8_t sign;
    GpTimerPlan pwm, step;
} Mock;
static uint32_t clock_us(void *user) { return ((Mock *)user)->now; }
static void off(void *user) { Mock *m = user; ++m->off_count; m->active = false; }
static bool direction(void *user, int8_t sign)
{
    Mock *m = user;
    CHECK(!m->active);
    ++m->dir_count;
    m->sign = sign;
    return !m->fail_dir;
}
static bool apply(void *user, const GpTimerPlan *pwm, const GpTimerPlan *step)
{
    Mock *m = user;
    CHECK(m->permitted);
    ++m->apply_count;
    m->pwm = *pwm;
    m->step = step ? *step : (GpTimerPlan){0};
    m->active = pwm->compare != 0 || step != NULL;
    return !m->fail_apply;
}
static bool allowed(void *user) { return ((Mock *)user)->permitted; }
static GpMotorHw new_hw(Mock *m)
{
    GpMotorHw hw;
    GpMotorHwConfig config = {72000000, 20000, 20000, 600, 5, 5, 5, 6, true};
    GpMotorHwPort port = {m, clock_us, off, direction, apply, allowed, NULL};
    m->permitted = true;
    CHECK(GpMotorHw_Init(&hw, &config, &port) == GP_OK);
    return hw;
}

static void timer_plans(void)
{
    const uint32_t frequencies[] = {1, 17, 1000, 20000, 50000};
    for (size_t i = 0; i < sizeof(frequencies) / sizeof(frequencies[0]); ++i) {
        GpTimerPlan plan;
        uint32_t hz = frequencies[i];
        CHECK(GpTimer_StepPlan(72000000, hz, 5, 5, &plan) == GP_OK);
        uint64_t divisor = (uint64_t)plan.prescaler + 1u;
        uint64_t period = (uint64_t)plan.autoreload + 1u;
        CHECK(divisor * period * hz >= 72000000u); /* never exceed cap */
        CHECK(divisor * plan.compare * 1000000u >= 72000000u * 5u);
        CHECK(divisor * (period - plan.compare) * 1000000u >= 72000000u * 5u);
    }
    GpTimerPlan plan;
    CHECK(GpTimer_PwmPlan(72000000, 20000, 500, &plan) == GP_OK);
    CHECK(plan.prescaler == 0 && plan.autoreload == 3599 && plan.compare == 1800);
    CHECK(GpTimer_PwmPlan(72000000, 20000, 0, &plan) == GP_OK && plan.compare == 0);
    CHECK(GpTimer_StepPlan(72000000, 200000, 5, 5, &plan) == GP_INVALID);
    CHECK(GpTimer_StepPlan(72000000, 0, 5, 5, &plan) != GP_OK);
    CHECK(GpTimer_StepPlan(72000000, 20000, 0, 5, &plan) != GP_OK);
    CHECK(GpTimer_PwmPlan(0, 20000, 500, &plan) != GP_OK);
    CHECK(GpTimer_PwmPlan(72000000, 20000, 1001, &plan) != GP_OK);
    CHECK(GpTimer_PwmPlan(UINT32_MAX, 1, 1000, &plan) == GP_OK);
    CHECK(plan.compare <= plan.autoreload);
}

static void can_timing(void)
{
    CHECK(GpCanTiming_Valid(36000000u, 500000u, 4u, 15u, 2u, 1u));
    /* Synthetic, test-only bitrate; the firmware has no operating default. */
    CHECK(GpCanTiming_Valid(36000000, 500000, 4, 13, 4, 1));
    CHECK(!GpCanTiming_Valid(36000000, 0, 4, 13, 4, 1));
    CHECK(!GpCanTiming_Valid(36000000, 250000, 4, 13, 4, 1));
    CHECK(!GpCanTiming_Valid(36000000, 500000, 4, 13, 4, 5));
    CHECK(!GpCanTiming_Valid(36000000, 500000, 0, 13, 4, 1));
    CHECK(!GpCanTiming_Valid(36000000, 500000, 4, 17, 4, 1));
}

static void setup_pending(void)
{
    Mock mock = {0};
    GpMotorHw hw = new_hw(&mock);
    MotorOutput request = {500, 1000};
    CHECK(GpMotorHw_Write(&hw, &request) == GP_PENDING);
    CHECK(mock.dir_count == 1 && mock.apply_count == 0 && !mock.active);
    mock.now = 5;
    request.steering_rate_pm = 100;
    CHECK(GpMotorHw_Write(&hw, &request) == GP_PENDING);
    CHECK(mock.dir_count == 1); /* slew updates do not restart setup */
    mock.now = 6;
    CHECK(GpMotorHw_Write(&hw, &request) == GP_OK && mock.active);
    CHECK(mock.pwm.compare == 1800);
    request.throttle_duty_pm = 1000;
    CHECK(GpMotorHw_Write(&hw, &request) == GP_OK);
    CHECK(mock.pwm.compare == 2160); /* 600 pm cap */
    request.steering_rate_pm = -100;
    CHECK(GpMotorHw_Write(&hw, &request) == GP_PENDING && !mock.active);
    CHECK(mock.sign == -1);
    mock.now = 12;
    CHECK(GpMotorHw_Write(&hw, &request) == GP_OK);
}

static void cancellation(void)
{
    Mock mock = {0};
    GpMotorHw hw = new_hw(&mock);
    MotorOutput request = {500, 100};
    CHECK(GpMotorHw_Write(&hw, &request) == GP_PENDING);
    GpMotorHw_Off(&hw);
    CHECK(!mock.active && !hw.pending && hw.direction == 0);
    mock.now = 100;
    request = (MotorOutput){0, 0};
    CHECK(GpMotorHw_Write(&hw, &request) == GP_OK && !mock.active);
    request = (MotorOutput){500, 100};
    CHECK(GpMotorHw_Write(&hw, &request) == GP_PENDING);
    mock.now = 101;
    request.steering_rate_pm = 0;
    CHECK(GpMotorHw_Write(&hw, &request) == GP_OK && mock.active);
    CHECK(mock.step.compare == 0);
    GpMotorHw_Off(&hw);
    GpMotorHw_Off(&hw);
    GpMotorHw_Off(NULL);
    CHECK(!mock.active);
}

static void errors(void)
{
    Mock mock = {0};
    GpMotorHw hw = new_hw(&mock);
    MotorOutput request = {500, 100};
    mock.fail_dir = true;
    CHECK(GpMotorHw_Write(&hw, &request) == GP_IO_ERROR && !mock.active);
    mock.fail_dir = false;
    CHECK(GpMotorHw_Write(&hw, &request) == GP_PENDING);
    mock.now = 6;
    mock.fail_apply = true;
    CHECK(GpMotorHw_Write(&hw, &request) == GP_IO_ERROR && !mock.active);
    mock.fail_apply = false;
    mock.permitted = false;
    CHECK(GpMotorHw_Write(&hw, &request) == GP_INVALID && !mock.active);
    CHECK(GpMotorHw_Write(&hw, NULL) == GP_BAD_ARG && !mock.active);
    mock.permitted = true;
    request.throttle_duty_pm = -1;
    CHECK(GpMotorHw_Write(&hw, &request) == GP_INVALID && !mock.active);
    GpMotorHwConfig config = hw.config;
    config.approved = false;
    CHECK(GpMotorHw_Init(&hw, &config, &hw.port) == GP_INVALID);
    CHECK(!hw.initialized);
    CHECK(GpMotorHw_Write(&hw, &request) == GP_INVALID);
}

static void microsecond_rollover(void)
{
    Mock mock = {0};
    GpMotorHw hw = new_hw(&mock);
    MotorOutput request = {500, -100};
    mock.now = UINT32_MAX - 2u;
    CHECK(GpMotorHw_Write(&hw, &request) == GP_PENDING);
    mock.now = 2;
    CHECK(GpMotorHw_Write(&hw, &request) == GP_PENDING);
    mock.now = 3;
    CHECK(GpMotorHw_Write(&hw, &request) == GP_OK);
}

static void watchdog(void)
{
    GpTaskWatchdog watchdog = {0, 0, 40, false, false};
    CHECK(!GpTaskWatchdog_Ready(&watchdog, 0));
    watchdog.motor_seen = watchdog.can_seen = true;
    CHECK(GpTaskWatchdog_Ready(&watchdog, 39));
    CHECK(!GpTaskWatchdog_Ready(&watchdog, 40));
    watchdog.motor_ms = watchdog.can_ms = UINT32_MAX - 10u;
    CHECK(GpTaskWatchdog_Ready(&watchdog, 20));
    watchdog.can_ms = UINT32_MAX - 30u;
    CHECK(!GpTaskWatchdog_Ready(&watchdog, 20));
    watchdog.deadline_ms = 0;
    CHECK(!GpTaskWatchdog_Ready(&watchdog, 20));
}

static void scheduling(void)
{
    uint32_t deadline = 10;
    for (uint32_t now = 0; now < 10; ++now) {
        CHECK(GpSchedule_Wait(now, 10, &deadline) == 10u - now);
        CHECK(deadline == 10); /* notification flood cannot delay the deadline */
    }
    CHECK(GpSchedule_Wait(10, 10, &deadline) == 10 && deadline == 20);
    CHECK(GpSchedule_Wait(105, 10, &deadline) == 5 && deadline == 110);
    deadline = UINT32_MAX - 4;
    CHECK(GpSchedule_Wait(UINT32_MAX - 5, 10, &deadline) == 1);
    CHECK(GpSchedule_Wait(5, 10, &deadline) == 10 && deadline == 15);
}

static void driver_integration(void)
{
    Mock mock = {0};
    GpMotorHw hw = new_hw(&mock);
    MotorDriver driver;
    MotorDriverConfig config = {GpMotorHw_Write, GpMotorHw_Off, &hw, 600, 20000, 5, 5, 6, 10};
    MotorSafety safety = {true, true};
    MotorOutput request = {500, 100}, applied;
    CHECK(MotorDriver_Init(&driver, &config) == GP_OK);
    CHECK(MotorDriver_Apply(&driver, &request, &safety, 0, &applied) == GP_PENDING);
    CHECK(mock.dir_count == 0 && !mock.active);
    mock.now = 10000;
    CHECK(MotorDriver_Apply(&driver, &request, &safety, 10, &applied) == GP_PENDING);
    CHECK(mock.dir_count == 1 && !mock.active);
    mock.now = 10006;
    CHECK(MotorDriver_Apply(&driver, &request, &safety, 10, &applied) == GP_OK);
    request.steering_rate_pm = -100;
    CHECK(MotorDriver_Apply(&driver, &request, &safety, 11, &applied) == GP_PENDING);
    CHECK(!mock.active);
    mock.now = 21000;
    CHECK(MotorDriver_Apply(&driver, &request, &safety, 21, &applied) == GP_PENDING);
    mock.now = 21006;
    CHECK(MotorDriver_Apply(&driver, &request, &safety, 21, &applied) == GP_OK);
    CHECK(mock.sign == -1 && mock.active);
    safety.estop_ok = false;
    CHECK(MotorDriver_Apply(&driver, &request, &safety, 22, &applied) == GP_OK);
    CHECK(!mock.active && !hw.pending);
}

static void default_inhibit(void)
{
    CHECK(!GP_ENABLE_ACTUATION && !GP_ENABLE_CAN && !GP_ENABLE_WATCHDOG);
    CHECK(GP_HAS_DECODER); /* codec availability never grants output permission */
    CHECK(GP_PIN_MAP_APPROVED); /* wiring confirmation alone never enables outputs */
    CHECK(!GP_OUTPUT_SETTINGS_CONFIGURED && !GP_CAN_SETTINGS_APPROVED);
    CHECK(GP_CAN_BITRATE == 0 && GP_THROTTLE_PWM_HZ == 0 && GP_MAX_STEP_HZ == 0);
    CHECK(GP_PWM_ACTIVE_HIGH == -1 && GP_DIR_POSITIVE_HIGH == -1);
    CHECK(!GP_UNPOWERED_SIGNAL_TEST);
}

static void neutral_axes(void)
{
    Mock mock = {0};
    GpMotorHw hw = new_hw(&mock);
    MotorDriver driver;
    MotorDriverConfig config = {GpMotorHw_Write, GpMotorHw_Off, &hw, 600, 20000, 5, 5, 6, 10};
    MotorSafety safety = {true, true};
    MotorOutput request = {500, 300}, applied;
    CHECK(MotorDriver_Init(&driver, &config) == GP_OK);
    CHECK(MotorDriver_Apply(&driver, &request, &safety, 0, &applied) == GP_PENDING);
    mock.now = 10000;
    CHECK(MotorDriver_Apply(&driver, &request, &safety, 10, &applied) == GP_PENDING);
    mock.now = 10006;
    CHECK(MotorDriver_Apply(&driver, &request, &safety, 10, &applied) == GP_OK);
    unsigned off_before = mock.off_count, dir_before = mock.dir_count;
    GpTimerPlan original_step = mock.step, original_pwm = mock.pwm;
    request.throttle_duty_pm = 0;
    CHECK(MotorDriver_Apply(&driver, &request, &safety, 11, &applied) == GP_OK);
    CHECK(mock.pwm.compare == 0 && mock.step.compare == original_step.compare);
    CHECK(mock.step.autoreload == original_step.autoreload && mock.active);
    CHECK(applied.throttle_duty_pm == 0 && applied.steering_rate_pm == 300);
    request = (MotorOutput){500, 0};
    CHECK(MotorDriver_Apply(&driver, &request, &safety, 12, &applied) == GP_OK);
    CHECK(mock.pwm.compare == original_pwm.compare && mock.step.compare == 0 && mock.active);
    CHECK(mock.off_count == off_before && mock.dir_count == dir_before);
    CHECK(!hw.pending && !hw.stepping);
    request = (MotorOutput){0, 0};
    CHECK(MotorDriver_Apply(&driver, &request, &safety, 13, &applied) == GP_OK);
    CHECK(!mock.active && mock.off_count == off_before + 1);
}

static void fault_during_setup(void)
{
    Mock mock = {0};
    GpMotorHw hw = new_hw(&mock);
    MotorDriver driver;
    MotorDriverConfig config = {GpMotorHw_Write, GpMotorHw_Off, &hw, 600, 20000, 5, 5, 6, 10};
    MotorSafety safety = {true, true};
    MotorOutput request = {500, 100}, applied;
    CHECK(MotorDriver_Init(&driver, &config) == GP_OK);
    CHECK(MotorDriver_Apply(&driver, &request, &safety, 0, &applied) == GP_PENDING);
    mock.now = 10000;
    CHECK(MotorDriver_Apply(&driver, &request, &safety, 10, &applied) == GP_PENDING);
    CHECK(hw.pending && mock.dir_count == 1);
    safety.estop_ok = false;
    CHECK(MotorDriver_Apply(&driver, &request, &safety, 10, &applied) == GP_OK);
    CHECK(!hw.pending && !mock.active && mock.apply_count == 0);
    mock.now = 100000;
    CHECK(!mock.active); /* elapsed time alone never starts a waveform */
    request = (MotorOutput){0, 0};
    CHECK(GpMotorHw_Write(&hw, &request) == GP_OK);
    CHECK(mock.apply_count == 0); /* neutral does not release a board inhibit */
}

/* Synthetic time/readback here tests policy only, never represents board wiring. */
static GpResult prepare_power(void *user)
{
    Mock *m = user;
    CHECK(!m->active && m->dir_count == 0);
    if (m->fail_dir) return GP_IO_ERROR;
    return m->now < 200000u ? GP_PENDING : GP_OK;
}
static void power_readiness(void)
{
    Mock mock = {0};
    GpMotorHw hw = new_hw(&mock);
    hw.port.prepare = prepare_power;
    MotorOutput request = {500, 300};
    CHECK(GpMotorHw_Write(&hw, &request) == GP_PENDING);
    unsigned off_count = mock.off_count;
    mock.now = 199999;
    request.steering_rate_pm = 400;
    CHECK(GpMotorHw_Write(&hw, &request) == GP_PENDING);
    CHECK(mock.off_count == off_count && mock.dir_count == 0 && !mock.active);
    mock.now = 200000;
    CHECK(GpMotorHw_Write(&hw, &request) == GP_PENDING); /* separate DIR setup */
    CHECK(mock.dir_count == 1 && hw.pending && !hw.preparing);
    mock.now = 200006;
    CHECK(GpMotorHw_Write(&hw, &request) == GP_OK && mock.active);
    GpMotorHw_Off(&hw);
    mock.now = 0; mock.dir_count = 0;
    CHECK(GpMotorHw_Write(&hw, &request) == GP_PENDING);
    GpMotorHw_Off(&hw); /* STOP cancels all phases, elapsed time cannot start output */
    CHECK(!hw.preparing && !hw.pending);
    mock.now = 300000;
    CHECK(!mock.active);
    request = (MotorOutput){0, 0};
    CHECK(GpMotorHw_Write(&hw, &request) == GP_OK && mock.dir_count == 0);
    request = (MotorOutput){500, 300};
    mock.fail_dir = true;
    CHECK(GpMotorHw_Write(&hw, &request) == GP_IO_ERROR);
    CHECK(!hw.preparing && !hw.pending && !mock.active);
}

int main(int argc, char **argv)
{
    const struct { const char *name; void (*run)(void); } tests[] = {
#define CASE(name) {#name, name}
        CASE(power_readiness), CASE(timer_plans), CASE(can_timing), CASE(setup_pending), CASE(cancellation),
        CASE(errors), CASE(microsecond_rollover), CASE(watchdog), CASE(scheduling),
        CASE(driver_integration), CASE(default_inhibit), CASE(fault_during_setup), CASE(neutral_axes)
#undef CASE
    };
    CHECK(argc == 2);
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) {
        if (!strcmp(argv[1], tests[i].name)) { tests[i].run(); return 0; }
    }
    return 2;
}
