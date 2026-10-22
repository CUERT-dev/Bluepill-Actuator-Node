#include "gp_service.h"
#include <assert.h>
#include <string.h>
static GpServiceCommand line(GpServiceParser *p, const char *s)
{
    GpServiceCommand result = GP_SERVICE_NONE;
    while (*s) result = GpService_Byte(p, (uint8_t)*s++, false);
    return result;
}
int main(void)
{
    GpServiceParser p = {0};
    assert(line(&p, "status\n") == GP_SERVICE_STATUS);
    assert(line(&p, "recover\r\n") == GP_SERVICE_RECOVER);
    assert(line(&p, "arm\n") == GP_SERVICE_BAD_COMMAND);
    assert(line(&p, "motor 1000\n") == GP_SERVICE_BAD_COMMAND);
    assert(line(&p, "xxxxxxxxxxxxxxxxrecover\n") == GP_SERVICE_BAD_COMMAND);
    assert(line(&p, "reco") == GP_SERVICE_NONE);
    assert(GpService_Byte(&p, 'v', true) == GP_SERVICE_NONE);
    assert(line(&p, "er\n") == GP_SERVICE_BAD_COMMAND);
    assert(line(&p, "recover\n") == GP_SERVICE_RECOVER);
    assert(line(&p, "recover\001\n") == GP_SERVICE_BAD_COMMAND);
    assert(line(&p, "status\n") == GP_SERVICE_STATUS);
    /* CR is accepted only as the final byte of CRLF, not inside a command. */
    assert(line(&p, "reco\rver\n") == GP_SERVICE_BAD_COMMAND);
    assert(line(&p, "\rrecover\n") == GP_SERVICE_BAD_COMMAND);
    assert(line(&p, "recover\r\r\n") == GP_SERVICE_BAD_COMMAND);
    assert(line(&p, "recover\rX\n") == GP_SERVICE_BAD_COMMAND);
    assert(line(&p, "status\r\n") == GP_SERVICE_STATUS);
    assert(GpService_Byte(NULL, 0, false) == GP_SERVICE_NONE);
    return 0;
}
