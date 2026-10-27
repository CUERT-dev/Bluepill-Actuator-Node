#include "gp_diag.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    GpDiagStatus s = {0};
    char buffer[640];
    s.state = GP_ARMED;
    s.last_rx_id = GP_CAN_COMMAND_ID;
    s.last_rx_dlc = 8;
    s.have_requested = true;
    s.requested = (TargetSample){65535, 1000, -1000, 7};
    s.decoded_command = (TargetSample){65535, 600, -278, 1};
    s.applied = (MotorOutput){1000, -1000};
    s.tim2 = (GpDiagTimer){0, 3599, 3600, 129, 16};
    s.tim3 = (GpDiagTimer){0, 432, 216, 129, 1};
    s.timer_clock_hz = 72000000;
    s.motor_ready = true;
    s.gpioa_odr = 1u << 7;
    int n = GpDiag_FormatMotion(buffer, sizeof(buffer), &s);
    assert(n > 0 && (size_t)n == strlen(buffer));
    assert(strstr(buffer, "id=105 dlc=8 have=1"));
    assert(strstr(buffer, "requested=1000,-1000 applied=1000,-1000"));
    assert(strstr(buffer, "received=600,-278 requested=1000,-1000"));
    assert(strstr(buffer, "t2=0,3599,3600,0081,0010"));
    assert(strstr(buffer, "ena=unwired\r\n"));
    assert(strstr(buffer, "armed=1 direct=0 motor=1 pwm_pm=1000 step_hz=166281 dir=1"));
    s.direct_commands = true;
    s.rx_waiting = true;
    s.rx_wait_count = UINT32_MAX;
    /* An axis stop disables its counter/channel without necessarily clearing CCR. */
    s.tim2.ccer = 0;
    s.tim3.cr1 = 128;
    s.gpioa_odr = 0;
    s.state = GP_WAIT_FOR_NEUTRAL;
    assert(GpDiag_FormatMotion(buffer, sizeof(buffer), &s) > 0);
    assert(strstr(buffer, "armed=0 direct=1 motor=1 pwm_pm=0 step_hz=0 dir=0"));
    assert(strstr(buffer, "rx_wait=1 rx_waits=4294967295"));
    s.tim2 = (GpDiagTimer){0, 3599, 360, 129, 16};
    s.tim3 = (GpDiagTimer){10, 65454, 33, 129, 1};
    assert(GpDiag_FormatMotion(buffer, sizeof(buffer), &s) > 0);
    assert(strstr(buffer, "pwm_pm=100 step_hz=99")); /* integer Hz truncates */
    /* Maximum field widths must still fit the actual firmware response buffer. */
    s.state = s.faults = s.shutdown_reason = s.shutdown_count = UINT32_MAX;
    s.motor_runs = s.motor_ms = s.requested_ms = UINT32_MAX;
    s.motor_result = INT32_MIN;
    s.timer_clock_hz = UINT32_MAX;
    s.last_rx_id = UINT16_MAX;
    s.last_rx_dlc = UINT8_MAX;
    s.tim2 = s.tim3 = (GpDiagTimer){UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX};
    s.gpioa_crl = s.gpiob_crl = s.gpioa_idr = s.gpiob_idr = UINT32_MAX;
    s.gpioa_odr = s.gpiob_odr = s.mapr = UINT32_MAX;
    n = GpDiag_FormatMotion(buffer, sizeof(buffer), &s);
    assert(n > 0 && (size_t)n < sizeof(buffer));
    char small[8];
    int truncated = GpDiag_FormatMotion(small, sizeof(small), &s);
    assert(truncated < 0 || (size_t)truncated >= sizeof(small));
    assert(small[sizeof(small) - 1] == '\0');
    assert(GpDiag_FormatMotion(NULL, sizeof(buffer), &s) == -1);
    assert(GpDiag_FormatMotion(buffer, sizeof(buffer), NULL) == -1);
    GpDiagStatus rx = {0};
    assert(GpDiag_FormatRx(buffer, sizeof(buffer), &rx) > 0);
    assert(strstr(buffer, "CAN RX have=0"));
    assert(strstr(buffer, "last_decoded=NO_VALID_SAMPLE"));
    rx.captured = 1;
    rx.last_rx_id = GP_CAN_COMMAND_ID;
    rx.last_rx_dlc = 8;
    rx.last_rx_ms = 123;
    const uint8_t bytes[] = {1, 0, 0x58, 2, 0x38, 0xff, 7, 0x62};
    memcpy(rx.last_rx_data, bytes, sizeof(bytes));
    assert(GpDiag_FormatRx(buffer, sizeof(buffer), &rx) > 0);
    assert(strstr(buffer, "have=1 id=105 dlc=8 ide=0 rtr=0 data=0100580238FF0762 rx_ms=123"));
    assert(strstr(buffer, "NO_VALID_SAMPLE")); /* captured does not imply decoded */
    rx.decoded = 1;
    rx.decoded_command = (TargetSample){1, 600, -200, 7};
    assert(GpDiag_FormatRx(buffer, sizeof(buffer), &rx) > 0);
    assert(strstr(buffer, "last_decoded=DEMAND"));
    rx.decoded_command = (TargetSample){2, 0, 0, GP_FLAG_VALID};
    assert(GpDiag_FormatRx(buffer, sizeof(buffer), &rx) > 0);
    assert(strstr(buffer, "last_decoded=NEUTRAL"));
    rx.decoded_command.flags = 0;
    assert(GpDiag_FormatRx(buffer, sizeof(buffer), &rx) > 0);
    assert(strstr(buffer, "last_decoded=STOP"));
    /* Neither a shutdown nor a rejected latest raw packet invents a sample. */
    rx.state = GP_FAULT_SHUTDOWN;
    rx.last_rx_extended = rx.last_rx_remote = true;
    rx.last_rx_data[7] ^= 1;
    assert(GpDiag_FormatRx(buffer, sizeof(buffer), &rx) > 0);
    assert(strstr(buffer, "ide=1 rtr=1 data=0100580238FF0763"));
    assert(strstr(buffer, "last_decoded=STOP"));
    assert(GpDiag_FormatRx(small, sizeof(small), &rx) != 0);
    assert(small[sizeof(small) - 1] == '\0');
    assert(GpDiag_FormatRx(NULL, sizeof(buffer), &rx) == -1);
    assert(GpDiag_FormatRx(buffer, sizeof(buffer), NULL) == -1);
    printf("PASS motion report, max length %d, truncation and unwired enable\n", n);
    return 0;
}
