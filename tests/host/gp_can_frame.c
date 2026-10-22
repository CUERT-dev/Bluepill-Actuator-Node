/* Offline bench vector generator: calls the actual production encoder.
 * It has no CAN/serial transport and cannot transmit a motor command. */
#include "gp_types.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
int main(int argc, char **argv)
{
    const long minimum[] = {0, 0, -1000, 0};
    const long maximum[] = {65535, 1000, 1000, 7};
    long v[4];
    if (argc != 5) {
        fprintf(stderr, "usage: gp_can_frame sequence throttle_pm steering_pm flags\n");
        return 2;
    }
    for (int i = 0; i < 4; ++i) {
        char *end;
        errno = 0;
        v[i] = strtol(argv[i + 1], &end, 0);
        if (errno || *end || end == argv[i + 1] || v[i] < minimum[i] || v[i] > maximum[i]) return 2;
    }
    TargetSample command = {(uint16_t)v[0], (uint16_t)v[1], (int16_t)v[2], (uint8_t)v[3]};
    GpCanFrame frame;
    if (GP_EncodeCan(&command, &frame) != GP_OK) return 2;
    printf("%03X#", frame.id);
    for (unsigned i = 0; i < frame.dlc; ++i) printf("%02X", frame.data[i]);
    putchar('\n');
    return 0;
}
