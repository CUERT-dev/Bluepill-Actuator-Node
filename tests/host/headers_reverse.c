/* Verify the other include order in a separately compiled translation unit. */
#include "motor_driver.h"
#include "command_map.h"
#include "gp_types.h"

_Static_assert(GP_API_REVISION == 10u, "Tests target GP API revision 10");

int headers_reverse(void)
{
    return GP_API_REVISION;
}
