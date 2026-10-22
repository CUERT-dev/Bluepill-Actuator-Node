#include "gp_types.h"
#include "command_map.h"
#include "motor_driver.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)
#define FLAGS (GP_FLAG_VALID | GP_FLAG_ARM | GP_FLAG_DEADMAN)

int headers_reverse(void);

static void output_is(MotorOutput out, int throttle, int steering)
{
    CHECK(out.throttle_duty_pm == throttle);
    CHECK(out.steering_rate_pm == steering);
}

static CommandMap new_map(uint16_t rate)
{
    CommandMap map;
    CommandMapConfig config = {1000, 1000, rate, rate, 1000};
    CHECK(CommandMap_Init(&map, &config) == GP_OK);
    return map;
}

static TargetSample command(int throttle, int steering)
{
    TargetSample cmd = {0, (uint16_t)throttle, (int16_t)steering, FLAGS};
    return cmd;
}

static void map_endpoints(void)
{
    CommandMap map = new_map(1000);
    MotorOutput out;
    TargetSample cmd = command(1000, 1000);
    CHECK(CommandMap_Update(&map, &cmd, 1000, true, &out) == GP_OK);
    output_is(out, 1000, 1000);
    cmd = command(0, 0);
    CHECK(CommandMap_Update(&map, &cmd, 1, true, &out) == GP_OK);
    output_is(out, 0, 0);
    cmd = command(1000, -1000);
    map.config.throttle_cap_pm = 350;
    map.config.steering_cap_pm = 600;
    CHECK(CommandMap_Update(&map, &cmd, 1000, true, &out) == GP_OK);
    output_is(out, 350, -600);
    cmd.throttle_pm = 0;
    CHECK(CommandMap_Update(&map, &cmd, 1, true, &out) == GP_OK);
    output_is(out, 0, -600);
    cmd.steering_pm = 0;
    cmd.throttle_pm = 1000;
    CHECK(CommandMap_Update(&map, &cmd, 100, true, &out) == GP_OK);
    output_is(out, 100, 0);
}

static void map_fractional(void)
{
    CommandMap map = new_map(1);
    MotorOutput out;
    TargetSample cmd = command(1000, -1000);
    map.config.throttle_slew_pm_s = 3;
    for (int i = 0; i < 999; ++i) {
        CHECK(CommandMap_Update(&map, &cmd, 1, true, &out) == GP_OK);
    }
    output_is(out, 2, 0);
    CHECK(map.throttle_remainder == 997 && map.steering_remainder == 999);
    CHECK(CommandMap_Update(&map, &cmd, 1, true, &out) == GP_OK);
    output_is(out, 3, -1);
    CHECK(map.throttle_remainder == 0 && map.steering_remainder == 0);
}

static void map_reversal(void)
{
    for (int sign = -1; sign <= 1; sign += 2) {
        CommandMap map = new_map(1);
        MotorOutput out;
        TargetSample cmd = command(1, sign);
        CHECK(CommandMap_Update(&map, &cmd, 1000, true, &out) == GP_OK);
        cmd.steering_pm = (int16_t)-sign;
        for (int i = 0; i < 999; ++i) {
            CHECK(CommandMap_Update(&map, &cmd, 1, true, &out) == GP_OK);
            output_is(out, 1, sign);
        }
        CHECK(CommandMap_Update(&map, &cmd, 1, true, &out) == GP_OK);
        output_is(out, 1, 0);
        CHECK(CommandMap_Update(&map, &cmd, 1000, true, &out) == GP_OK);
        output_is(out, 1, -sign);
    }
}

static void map_flags(void)
{
    /* Exhaust all flag combinations, including reserved bits. */
    for (unsigned flags = 0; flags <= UINT8_MAX; ++flags) {
        CommandMap map = new_map(1);
        MotorOutput out;
        TargetSample cmd = command(1000, 1000);
        CHECK(CommandMap_Update(&map, &cmd, 999, true, &out) == GP_OK);
        cmd.flags = (uint8_t)flags;
        GpResult expected = ((flags & GP_FLAG_VALID) == 0 ||
                            (flags & ~FLAGS) != 0) ? GP_INVALID : GP_OK;
        CHECK(CommandMap_Update(&map, &cmd, 1, true, &out) == expected);
        output_is(out, flags == FLAGS ? 1 : 0, flags == FLAGS ? 1 : 0);
        CHECK(map.throttle_remainder == 0 && map.steering_remainder == 0);
    }
}

static void map_invalid(void)
{
    for (int kind = 0; kind < 8; ++kind) {
        CommandMap map = new_map(1000);
        MotorOutput out;
        TargetSample cmd = command(1000, 1000);
        uint32_t dt = 1;
        CHECK(CommandMap_Update(&map, &cmd, 1000, true, &out) == GP_OK);
        switch (kind) {
        case 0: cmd.throttle_pm = 1001; break;
        case 1: cmd.steering_pm = -1001; break;
        case 2: cmd.steering_pm = 1001; break;
        case 3: dt = 0; break;
        case 4: dt = 1001; break;
        default: break;
        }
        CHECK(CommandMap_Update(&map, kind == 5 ? NULL : &cmd, dt, true,
                                kind == 6 ? NULL : &out) ==
              (kind == 5 || kind == 6 ? GP_BAD_ARG : kind == 7 ? GP_OK : GP_INVALID));
        if (kind != 7) {
            output_is(map.previous, 0, 0);
            if (kind != 6) output_is(out, 0, 0);
            cmd = command(1000, 1000);
            CHECK(CommandMap_Update(&map, &cmd, 1, true, &out) == GP_OK);
            output_is(out, 1, 1);
        }
    }
    MotorOutput out = {1, 1};
    TargetSample cmd = command(1, 1);
    CHECK(CommandMap_Update(NULL, &cmd, 1, true, &out) == GP_BAD_ARG);
    output_is(out, 0, 0);
    CommandMap map = {0};
    CHECK(CommandMap_Update(&map, &cmd, 1, true, &out) == GP_INVALID);
}

static void map_reset(void)
{
    CommandMap map = new_map(1);
    MotorOutput out;
    TargetSample cmd = command(1000, 1000);
    CHECK(CommandMap_Update(&map, &cmd, 999, true, &out) == GP_OK);
    CommandMap_Reset(&map);
    CHECK(map.initialized && map.config.steering_slew_pm_s == 1);
    CHECK(map.throttle_remainder == 0 && map.steering_remainder == 0);
    CHECK(CommandMap_Update(&map, &cmd, 1, true, &out) == GP_OK);
    output_is(out, 0, 0);
    cmd = command(0, 0);
    CHECK(CommandMap_Update(&map, &cmd, 1, true, &out) == GP_OK);
    CHECK(map.throttle_remainder == 0 && map.steering_remainder == 0);
    cmd = command(1000, 1000);
    CHECK(CommandMap_Update(&map, &cmd, 1000, true, &map.previous) == GP_OK);
    output_is(map.previous, 1, 1);
    CHECK(CommandMap_Update(&map, &cmd, 1000, true, &map.previous) == GP_OK);
    output_is(map.previous, 2, 2);
    cmd.flags = 0;
    CHECK(CommandMap_Update(&map, &cmd, 0, false, &out) == GP_OK);
    output_is(out, 0, 0);
    output_is(map.previous, 0, 0);
    CommandMap_Reset(NULL);
}

static void map_config(void)
{
    CommandMap map = new_map(1);
    CommandMapConfig good = map.config;
    CHECK(CommandMap_Init(NULL, &good) == GP_BAD_ARG);
    CHECK(CommandMap_Init(&map, NULL) == GP_BAD_ARG);
    CHECK(CommandMap_Init(&map, &map.config) == GP_OK);
    for (int kind = 0; kind < 5; ++kind) {
        CommandMapConfig bad = good;
        switch (kind) {
        case 0: bad.throttle_cap_pm = 1001; break;
        case 1: bad.steering_cap_pm = 1001; break;
        case 2: bad.throttle_slew_pm_s = 0; break;
        case 3: bad.steering_slew_pm_s = 0; break;
        case 4: bad.max_dt_ms = 0; break;
        }
        CHECK(CommandMap_Init(&map, &bad) == GP_INVALID);
        CHECK(!map.initialized);
        output_is(map.previous, 0, 0);
    }
    good.throttle_cap_pm = good.steering_cap_pm = 0;
    CHECK(CommandMap_Init(&map, &good) == GP_OK);
    TargetSample cmd = command(1000, 1000);
    MotorOutput out;
    CHECK(CommandMap_Update(&map, &cmd, 1000, true, &out) == GP_OK);
    output_is(out, 0, 0);
}

static void map_large_dt(void)
{
    CommandMap map = new_map(UINT16_MAX);
    map.config.max_dt_ms = UINT32_MAX;
    TargetSample cmd = command(1000, -1000);
    MotorOutput out;
    CHECK(CommandMap_Update(&map, &cmd, UINT32_MAX, true, &out) == GP_OK);
    output_is(out, 1000, -1000);
}

/* Hardware model only. Synthetic limits/timings are NOT vehicle approval.
 * Events: O=off, D=DIR+ENA established with STEP stopped, S=output active. */
typedef struct {
    MotorOutput active, staged;
    unsigned off_count, write_count;
    GpResult result;
    uint32_t now, setup_since, setup_ms;
    bool preparing;
    char events[1024];
    size_t event_count;
} Mock;

static void event(Mock *mock, char value)
{
    CHECK(mock->event_count < sizeof(mock->events));
    mock->events[mock->event_count++] = value;
}

static void mock_off(void *user)
{
    Mock *mock = user;
    ++mock->off_count;
    mock->active = (MotorOutput){0, 0};
    mock->preparing = false;
    event(mock, 'O');
}

static GpResult mock_write(void *user, const MotorOutput *out)
{
    Mock *mock = user;
    ++mock->write_count;
    if (mock->result != GP_OK) {
        /* Even a failing callback might partially write before failing. */
        mock->active = mock->result == GP_PENDING ? (MotorOutput){0, 0} : *out;
        return mock->result;
    }
    if (mock->setup_ms && out->steering_rate_pm != 0 &&
        (mock->active.steering_rate_pm == 0 ||
         (mock->active.steering_rate_pm < 0) != (out->steering_rate_pm < 0))) {
        if (!mock->preparing ||
            mock->staged.steering_rate_pm != out->steering_rate_pm ||
            mock->staged.throttle_duty_pm != out->throttle_duty_pm) {
            mock->active = (MotorOutput){0, 0};
            mock->staged = *out;
            mock->setup_since = mock->now;
            mock->preparing = true;
            event(mock, 'D');
        }
        if ((uint32_t)(mock->now - mock->setup_since) < mock->setup_ms) {
            return GP_PENDING;
        }
    }
    mock->active = *out;
    mock->preparing = false;
    event(mock, 'S');
    return GP_OK;
}

static MotorDriverConfig driver_config_for(Mock *mock, uint32_t deadtime)
{
    MotorDriverConfig config = {mock_write, mock_off, mock, 650, 20000,
                                5, 2000, 3000, deadtime};
    return config;
}

static MotorDriver new_driver(Mock *mock, uint32_t deadtime)
{
    MotorDriver driver;
    MotorDriverConfig config = driver_config_for(mock, deadtime);
    CHECK(MotorDriver_Init(&driver, &config) == GP_OK);
    CHECK(driver.initialized);
    output_is(mock->active, 0, 0);
    return driver;
}

static GpResult apply(MotorDriver *driver, Mock *mock, int throttle, int steering,
                      uint32_t now, MotorOutput *applied)
{
    MotorOutput request = {(int16_t)throttle, (int16_t)steering};
    MotorSafety safety = {true, true};
    mock->now = now;
    GpResult result = MotorDriver_Apply(driver, &request, &safety, now, applied);
    if (result == GP_PENDING) {
        output_is(*applied, 0, 0);
        output_is(driver->previous, 0, 0);
        output_is(mock->active, 0, 0);
    }
    return result;
}

static void driver_config(void)
{
    Mock mock = {0};
    MotorDriver driver = {0};
    MotorDriverConfig good = driver_config_for(&mock, 5);
    CHECK(MotorDriver_Init(NULL, &good) == GP_BAD_ARG);
    CHECK(MotorDriver_Init(&driver, NULL) == GP_BAD_ARG);
    for (int kind = 0; kind < 8; ++kind) {
        MotorDriverConfig bad = good;
        switch (kind) {
        case 0: bad.write = NULL; break;
        case 1: bad.off = NULL; break;
        case 2: bad.max_duty_pm = 1001; break;
        case 3: bad.max_step_hz = 0; break;
        case 4: bad.step_pulse_us = 0; break;
        case 5: bad.reversal_deadtime_ms = 0x80000000u; break;
        case 6: bad.max_step_hz = 200000; break;
        case 7: bad.max_step_hz = UINT32_MAX; bad.step_pulse_us = UINT16_MAX; break;
        }
        unsigned count = mock.off_count;
        CHECK(MotorDriver_Init(&driver, &bad) ==
              (kind < 2 ? GP_BAD_ARG : GP_INVALID));
        CHECK(!driver.initialized);
        CHECK(mock.off_count == count + (kind == 1 ? 0u : 1u));
    }
    good.max_duty_pm = 0;
    good.reversal_deadtime_ms = 0x7fffffffu;
    CHECK(MotorDriver_Init(&driver, &good) == GP_OK);
    CHECK(MotorDriver_Init(&driver, &driver.config) == GP_OK);
    CHECK(driver.config.user == &mock && driver.config.max_duty_pm == 0);
}

static void driver_stop(void)
{
    Mock mock = {0};
    MotorDriver driver = new_driver(&mock, 10);
    MotorOutput out;
    CHECK(apply(&driver, &mock, 600, 1000, 0, &out) == GP_PENDING);
    CHECK(mock.write_count == 0);
    CHECK(apply(&driver, &mock, 600, 1000, 10, &out) == GP_OK);
    for (int i = 0; i < 3; ++i) {
        unsigned count = mock.off_count;
        MotorDriver_Stop(&driver);
        CHECK(mock.off_count == count + 1);
        output_is(mock.active, 0, 0);
        output_is(driver.previous, 0, 0);
        CHECK(driver.last_steering_direction == 0 && driver.zero_since_ms == 0);
        CHECK(driver.initialized && driver.config.max_duty_pm == 650);
    }
    CHECK(apply(&driver, &mock, 0, 0, 20, &out) == GP_OK);
    CHECK(apply(&driver, &mock, 600, -1000, 21, &out) == GP_PENDING);
    CHECK(apply(&driver, &mock, 600, -1000, 30, &out) == GP_PENDING);
    CHECK(apply(&driver, &mock, 600, -1000, 31, &out) == GP_OK);
    MotorDriver_Stop(NULL);
    MotorDriver uninitialized = {0};
    MotorDriver_Stop(&uninitialized);
}

static void driver_safety(void)
{
    for (int kind = 0; kind < 2; ++kind) {
        Mock mock = {0};
        MotorDriver driver = new_driver(&mock, 0);
        MotorOutput request = {500, 1000}, out;
        MotorSafety safety = {kind != 0, kind == 0};
        CHECK(apply(&driver, &mock, 500, 1000, 0, &out) == GP_OK);
        CHECK(MotorDriver_Apply(&driver, &request, &safety, 1, &out) == GP_OK);
        output_is(out, 0, 0);
        output_is(mock.active, 0, 0);
        output_is(driver.previous, 0, 0);
        CHECK(driver.last_steering_direction == 0);
    }
}

static void driver_limits(void)
{
    Mock mock = {0};
    MotorDriver driver = new_driver(&mock, 0);
    MotorOutput out;
    CHECK(apply(&driver, &mock, 1000, -1000, 0, &out) == GP_OK);
    output_is(out, 650, -1000);
    const MotorOutput invalid[] = {{-1, 0}, {1001, 0}, {0, -1001}, {0, 1001}};
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        CHECK(apply(&driver, &mock, invalid[i].throttle_duty_pm,
                    invalid[i].steering_rate_pm, 0, &out) == GP_INVALID);
        output_is(mock.active, 0, 0);
        output_is(out, 0, 0);
    }
}

static void driver_errors(void)
{
    const GpResult errors[] = {GP_BAD_ARG, GP_INVALID, GP_STALE, GP_IO_ERROR, GP_NO_SPACE};
    Mock mock = {0};
    MotorDriver driver = new_driver(&mock, 0);
    MotorOutput out;
    for (size_t i = 0; i < sizeof(errors) / sizeof(errors[0]); ++i) {
        mock.result = errors[i];
        CHECK(apply(&driver, &mock, 500, 1000, 0, &out) == GP_IO_ERROR);
        output_is(out, 0, 0);
        output_is(mock.active, 0, 0);
        output_is(driver.previous, 0, 0);
        CHECK(driver.last_steering_direction == 0);
    }
    mock.result = GP_OK;
    for (int kind = 0; kind < 3; ++kind) {
        MotorOutput request = {500, 1000};
        MotorSafety safety = {true, true};
        CHECK(apply(&driver, &mock, 500, 1000, 0, &out) == GP_OK);
        CHECK(MotorDriver_Apply(&driver, kind == 0 ? NULL : &request,
                                kind == 1 ? NULL : &safety, 1,
                                kind == 2 ? NULL : &out) == GP_BAD_ARG);
        output_is(driver.previous, 0, 0);
        output_is(mock.active, 0, 0);
    }
    CHECK(MotorDriver_Apply(NULL, NULL, NULL, 0, &out) == GP_BAD_ARG);
    output_is(out, 0, 0);
    MotorDriver uninitialized = {0};
    CHECK(MotorDriver_Apply(&uninitialized, NULL, NULL, 0, &out) == GP_BAD_ARG);
}

static void driver_reversal(void)
{
    Mock mock = {0};
    MotorDriver driver = new_driver(&mock, 10);
    MotorOutput out;
    mock.setup_ms = 3; /* max(DIR=2ms, ENA=3ms), established together */
    CHECK(apply(&driver, &mock, 500, 1000, 100, &out) == GP_PENDING);
    CHECK(apply(&driver, &mock, 500, 1000, 109, &out) == GP_PENDING);
    CHECK(mock.write_count == 0);
    CHECK(apply(&driver, &mock, 500, 1000, 110, &out) == GP_PENDING);
    CHECK(mock.events[mock.event_count - 1] == 'D');
    unsigned off_count = mock.off_count;
    CHECK(apply(&driver, &mock, 500, 1000, 112, &out) == GP_PENDING);
    CHECK(mock.off_count == off_count);
    CHECK(apply(&driver, &mock, 500, 1000, 113, &out) == GP_OK);
    output_is(mock.active, 500, 1000);
    CHECK(apply(&driver, &mock, 500, -1000, 120, &out) == GP_PENDING);
    CHECK(mock.events[mock.event_count - 1] == 'O');
    unsigned writes = mock.write_count;
    CHECK(apply(&driver, &mock, 500, -1000, 129, &out) == GP_PENDING);
    CHECK(mock.write_count == writes);
    CHECK(apply(&driver, &mock, 500, -1000, 130, &out) == GP_PENDING);
    CHECK(mock.events[mock.event_count - 1] == 'D');
    CHECK(apply(&driver, &mock, 500, -1000, 132, &out) == GP_PENDING);
    CHECK(apply(&driver, &mock, 500, -1000, 133, &out) == GP_OK);
    CHECK(mock.events[mock.event_count - 1] == 'S');
    output_is(out, 500, -1000);

    /* Stop only steering, allow throttle, then reverse before deadtime ends. */
    CHECK(apply(&driver, &mock, 500, 0, 140, &out) == GP_OK);
    CHECK(apply(&driver, &mock, 500, 1000, 141, &out) == GP_PENDING);
    CHECK(driver.zero_since_ms == 140);
    CHECK(apply(&driver, &mock, 500, 1000, 150, &out) == GP_PENDING);
    CHECK(apply(&driver, &mock, 500, 1000, 153, &out) == GP_OK);
}

static void driver_pending(void)
{
    Mock mock = {0};
    MotorDriver driver = new_driver(&mock, 0);
    MotorOutput out;
    mock.result = GP_PENDING;
    CHECK(apply(&driver, &mock, 500, 1000, 0, &out) == GP_PENDING);
    unsigned count = mock.off_count;
    CHECK(apply(&driver, &mock, 500, 1000, 1, &out) == GP_PENDING);
    CHECK(mock.off_count == count);
    MotorDriver_Stop(&driver);
    CHECK(mock.off_count == count + 1);
    mock.result = GP_OK;
    mock.setup_ms = 3;
    CHECK(apply(&driver, &mock, 500, 1000, 2, &out) == GP_PENDING);
    CHECK(apply(&driver, &mock, 300, -500, 3, &out) == GP_PENDING);
    CHECK(apply(&driver, &mock, 300, -500, 5, &out) == GP_PENDING);
    CHECK(apply(&driver, &mock, 300, -500, 6, &out) == GP_OK);
    output_is(mock.active, 300, -500);
    CHECK(apply(&driver, &mock, 500, 1000, 7, &out) == GP_PENDING);
    CHECK(apply(&driver, &mock, 0, 0, 8, &out) == GP_OK);
    CHECK(!mock.preparing);
    output_is(mock.active, 0, 0);
    CHECK(apply(&driver, &mock, 500, 1000, 9, &out) == GP_PENDING);
    MotorSafety estop = {false, true};
    MotorOutput request = {500, 1000};
    CHECK(MotorDriver_Apply(&driver, &request, &estop, 10, &out) == GP_OK);
    CHECK(!mock.preparing);
    output_is(mock.active, 0, 0);
}

static void driver_rollover(void)
{
    Mock mock = {0};
    MotorDriver driver = new_driver(&mock, 10);
    MotorOutput out;
    CHECK(apply(&driver, &mock, 500, 1000, UINT32_MAX - 20, &out) == GP_PENDING);
    CHECK(apply(&driver, &mock, 500, 1000, UINT32_MAX - 10, &out) == GP_OK);
    CHECK(apply(&driver, &mock, 500, -1000, UINT32_MAX - 4, &out) == GP_PENDING);
    CHECK(apply(&driver, &mock, 500, -1000, 4, &out) == GP_PENDING);
    CHECK(apply(&driver, &mock, 500, -1000, 5, &out) == GP_OK);
    output_is(out, 500, -1000);
    driver = new_driver(&mock, 0);
    mock.setup_ms = 3;
    CHECK(apply(&driver, &mock, 500, 1000, UINT32_MAX - 1, &out) == GP_PENDING);
    CHECK(apply(&driver, &mock, 500, 1000, 0, &out) == GP_PENDING);
    CHECK(apply(&driver, &mock, 500, 1000, 1, &out) == GP_OK);
}

static void driver_alias(void)
{
    Mock mock = {0};
    MotorDriver driver = new_driver(&mock, 0);
    MotorSafety safety = {true, true};
    MotorOutput out = {1000, -1000};
    CHECK(MotorDriver_Apply(&driver, &out, &safety, 0, &out) == GP_OK);
    output_is(out, 650, -1000);
    out.steering_rate_pm = 1000;
    CHECK(MotorDriver_Apply(&driver, &out, &safety, 1, &driver.previous) == GP_OK);
    output_is(driver.previous, 650, 1000);
}

static void combined(void)
{
    CommandMap map = new_map(10);
    Mock mock = {0};
    MotorDriver driver = new_driver(&mock, 2);
    MotorOutput mapped, applied;
    MotorSafety safety = {true, true};
    TargetSample cmd = command(1000, 1000);
    mock.setup_ms = 1;
    for (uint32_t now = 0; now < 400; ++now) {
        if (now == 150) cmd.steering_pm = -1000;
        CHECK(CommandMap_Update(&map, &cmd, 1, true, &mapped) == GP_OK);
        mock.now = now;
        GpResult result = MotorDriver_Apply(&driver, &mapped, &safety, now, &applied);
        CHECK(result == GP_OK || result == GP_PENDING);
        if (result == GP_PENDING) {
            output_is(applied, 0, 0);
            output_is(mock.active, 0, 0);
        }
        if (now == 199) CHECK(mapped.steering_rate_pm == 0);
    }
    CHECK(mapped.steering_rate_pm < 0 && applied.steering_rate_pm < 0);
    cmd.flags = 0;
    CHECK(CommandMap_Update(&map, &cmd, 1, true, &mapped) == GP_INVALID);
    CHECK(MotorDriver_Apply(&driver, &mapped, &safety, 400, &applied) == GP_OK);
    output_is(mock.active, 0, 0);
    output_is(driver.previous, 0, 0);
}

int main(int argc, char **argv)
{
    const struct { const char *name; void (*run)(void); } cases[] = {
#define CASE(name) {#name, name}
        CASE(map_endpoints), CASE(map_fractional), CASE(map_reversal),
        CASE(map_flags), CASE(map_invalid), CASE(map_reset), CASE(map_config),
        CASE(map_large_dt), CASE(driver_config), CASE(driver_stop),
        CASE(driver_safety), CASE(driver_limits), CASE(driver_errors),
        CASE(driver_reversal), CASE(driver_pending), CASE(driver_rollover),
        CASE(driver_alias), CASE(combined)
#undef CASE
    };
    CHECK(headers_reverse() == 10);
    CHECK(argc == 2);
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        if (strcmp(argv[1], cases[i].name) == 0) {
            cases[i].run();
            printf("PASS %s\n", cases[i].name);
            return 0;
        }
    }
    fprintf(stderr, "Unknown test: %s\n", argv[1]);
    return 2;
}
