#ifndef GP_SERVICE_H
#define GP_SERVICE_H
#include <stdbool.h>
#include <stdint.h>
typedef enum { GP_SERVICE_NONE, GP_SERVICE_STATUS, GP_SERVICE_RECOVER,
               GP_SERVICE_BAD_COMMAND } GpServiceCommand;
typedef struct { char line[16]; uint8_t length; bool discard; } GpServiceParser;
/* Errors/overlength discard through the next newline: a damaged command must
 * never become a recovery acknowledgement. No motor commands are supported. */
GpServiceCommand GpService_Byte(GpServiceParser *, uint8_t, bool error);
#endif
