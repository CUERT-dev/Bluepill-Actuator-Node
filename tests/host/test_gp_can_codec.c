#include "gp_types.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    const uint8_t golden_motion[8] = {0x01,0x00,0x58,0x02,0x38,0xFF,0x07,0x62};
    const uint8_t golden_stop[8]   = {0x02,0x00,0x00,0x00,0x00,0x00,0x00,0xB9};
    TargetSample motion = {1,600,-200,GP_FLAG_VALID|GP_FLAG_ARM|GP_FLAG_DEADMAN};
    TargetSample stop = {2,0,0,0};
    TargetSample out = {0};
    GpCanFrame frame;

    assert(GP_EncodeCan(&motion, &frame) == GP_OK);
    assert(frame.id == GP_CAN_COMMAND_ID && frame.dlc == 8);
    assert(memcmp(frame.data, golden_motion, 8) == 0);
    assert(GP_DecodeCan(&frame, &out) == GP_OK);
    assert(out.sequence == 1 && out.throttle_pm == 600 && out.steering_pm == -200 && out.flags == 7);

    assert(GP_EncodeCan(&stop, &frame) == GP_OK);
    assert(memcmp(frame.data, golden_stop, 8) == 0);
    assert(GP_DecodeCan(&frame, &out) == GP_OK);
    assert(out.sequence == 2 && out.throttle_pm == 0 && out.steering_pm == 0 && out.flags == 0);

    /* Invalid results must not overwrite the prior successful output. */
    TargetSample prior = out;
    frame.id = 0x106;
    assert(GP_DecodeCan(&frame,&out) == GP_INVALID);
    assert(out.sequence == prior.sequence && out.throttle_pm == prior.throttle_pm &&
           out.steering_pm == prior.steering_pm && out.flags == prior.flags);
    frame.id = GP_CAN_COMMAND_ID;
    frame.dlc = 7;
    assert(GP_DecodeCan(&frame,&out) == GP_INVALID);
    frame.dlc = 8;
    frame.data[0] ^= 0x01;
    assert(GP_DecodeCan(&frame,&out) == GP_INVALID);
    assert(out.sequence == prior.sequence && out.throttle_pm == prior.throttle_pm &&
           out.steering_pm == prior.steering_pm && out.flags == prior.flags);

    assert(GP_DecodeCan(NULL,&out) == GP_BAD_ARG);
    assert(GP_DecodeCan(&frame,NULL) == GP_BAD_ARG);
    assert(GP_EncodeCan(NULL,&frame) == GP_BAD_ARG);
    assert(GP_EncodeCan(&stop,NULL) == GP_BAD_ARG);

    /* Valid bit clear requires a fully neutral packet with all flags clear. */
    stop.flags = GP_FLAG_ARM;
    assert(GP_EncodeCan(&stop,&frame) == GP_INVALID);
    stop.flags = 0;
    stop.throttle_pm = 1;
    assert(GP_EncodeCan(&stop,&frame) == GP_INVALID);

    /* Value limits and reserved bits are rejected. */
    motion.throttle_pm=1001;
    assert(GP_EncodeCan(&motion,&frame) == GP_INVALID);
    motion.throttle_pm=1000;
    motion.steering_pm=-1001;
    assert(GP_EncodeCan(&motion,&frame) == GP_INVALID);
    motion.steering_pm=1000;
    motion.flags=GP_FLAG_VALID|0x80;
    assert(GP_EncodeCan(&motion,&frame) == GP_INVALID);

    /* Sequence rollover preserves wire representation. */
    motion.flags=GP_FLAG_VALID;
    motion.sequence=65535;
    assert(GP_EncodeCan(&motion,&frame) == GP_OK);
    assert(GP_DecodeCan(&frame,&out) == GP_OK && out.sequence == 65535);

    puts("PASS: canonical-header CAN codec golden vectors, stop, invalid frames, limits, rollover");
    return 0;
}
