#ifndef GP_STM32_BSP_H
#define GP_STM32_BSP_H
#include "gp_control.h"
#include "gp_hardware.h"
#include "stm32f1xx_hal.h"

typedef struct {
    uint32_t blockers, reset_flags, panic_reason;
    bool can_ready, motor_ready, watchdog_started;
} GpBootStatus;
enum {
    GP_BLOCK_CAN = 1u << 0, GP_BLOCK_DECODER = 1u << 1,
    GP_BLOCK_MOTOR_SETTINGS = 1u << 2, GP_BLOCK_ACTUATION = 1u << 3,
    GP_BLOCK_WATCHDOG = 1u << 4
};
extern volatile GpBootStatus gp_boot_status;
void GpBsp_Init(void);
void GpBsp_EmergencyOff(void); /* register-only waveform stop, not power cutoff */
bool GpBsp_ControlInputs(void *user, GpControlInputs *inputs);
bool GpBsp_DiscardRx(void *user);
GpResult GpBsp_CanStart(void);
GpResult GpBsp_CanRestart(void); /* motor task, outputs already off */
void GpBsp_CanInterrupt(void);
GpResult GpBsp_MotorConfigure(CommandMapConfig *, MotorDriverConfig *);
GpResult GpBsp_MotorWrite(void *, const MotorOutput *);
void GpBsp_MotorOff(void *);
uint32_t GpBsp_Micros(void);
bool GpBsp_WatchdogStart(void);
void GpBsp_WatchdogFeed(void);
#endif
