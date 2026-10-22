#ifndef GP_APP_H
#define GP_APP_H
#include "gp_types.h"
/* Starts the scheduler; any startup/scheduler failure enters GpApp_Fatal. */
void GpApp_Create(void) __attribute__((noreturn));
void GpApp_KernelTick(void);
void GpApp_LatchCanFault(uint32_t reason);
GpResult GpApp_CaptureFrame(const GpCanFrame *frame, bool extended, bool remote);
/* Local acknowledgement; notification only. Supervisor runs in motor task. */
void GpApp_RequestRecovery(void);
void GpApp_AssertFailed(const char *file, int line);
void GpApp_Fatal(uint32_t reason) __attribute__((noreturn));
#endif
