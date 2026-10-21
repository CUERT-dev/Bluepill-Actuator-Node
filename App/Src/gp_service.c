#include "gp_service.h"
#include <string.h>
GpServiceCommand GpService_Byte(GpServiceParser *p, uint8_t byte, bool error)
{
    if (!p) return GP_SERVICE_NONE;
    if (error) p->discard = true;
    if (byte == '\n') {
        GpServiceCommand result = GP_SERVICE_BAD_COMMAND;
        /* Permit CRLF without deleting embedded CR bytes: malformed input
         * such as reco\rver must never become a recovery acknowledgement. */
        if (p->length && p->line[p->length - 1u] == '\r') --p->length;
        p->line[p->length] = '\0';
        if (!p->discard) {
            if (!strcmp(p->line, "status")) result = GP_SERVICE_STATUS;
            else if (!strcmp(p->line, "recover")) result = GP_SERVICE_RECOVER;
        }
        p->length = 0;
        p->discard = false;
        return result;
    }
    if ((byte < 32 && byte != '\r') || byte > 126 || p->length == sizeof(p->line) - 1u)
        p->discard = true;
    if (!p->discard) p->line[p->length++] = (char)byte;
    return GP_SERVICE_NONE;
}
