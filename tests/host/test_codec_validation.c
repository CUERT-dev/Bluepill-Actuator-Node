#include "gp_types.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); \
} } while (0)

/* Independent polynomial long division for CRC-8/SMBUS (poly 0x07,
 * init/xorout 0, non-reflected). Not a call into the production CRC helper. */
static uint8_t reference_crc(const uint8_t data[8])
{
    unsigned remainder = 0;
    for (unsigned i = 0; i < 7; ++i) {
        remainder ^= (unsigned)data[i] << 8;
        for (unsigned bit = 0; bit < 8; ++bit) {
            if (remainder & (0x8000u >> bit)) remainder ^= 0x107u << (7u - bit);
        }
        remainder <<= 8;
        remainder &= 0xffffu;
    }
    return (uint8_t)(remainder >> 8);
}

static GpCanFrame encoded(TargetSample sample)
{
    GpCanFrame frame;
    CHECK(GP_EncodeCan(&sample, &frame) == GP_OK);
    return frame;
}

static void unchanged(const GpCanFrame *frame, GpResult expected)
{
    TargetSample out = {0x1234, 234, -567, GP_FLAG_VALID};
    CHECK(GP_DecodeCan(frame, &out) == expected);
    CHECK(out.sequence == 0x1234 && out.throttle_pm == 234 &&
          out.steering_pm == -567 && out.flags == GP_FLAG_VALID);
}

static void layout(void)
{
    CHECK(GP_API_REVISION == 10 && GP_CAN_COMMAND_ID == 0x105);
    const TargetSample input = {0xbeef, 1000, -1000, 7};
    GpCanFrame frame = encoded(input);
    const uint8_t prefix[] = {0xef, 0xbe, 0xe8, 0x03, 0x18, 0xfc, 7};
    CHECK(frame.id == 0x105 && frame.dlc == 8);
    CHECK(memcmp(prefix, frame.data, sizeof(prefix)) == 0);
    CHECK(frame.data[7] == reference_crc(frame.data));
    TargetSample out;
    CHECK(GP_DecodeCan(&frame, &out) == GP_OK);
    CHECK(out.sequence == input.sequence && out.throttle_pm == 1000 &&
          out.steering_pm == -1000 && out.flags == 7);
}

static void crc(void)
{
    GpCanFrame original = encoded((TargetSample){1, 600, -200, 7});
    CHECK(original.data[7] == 0x62 && reference_crc(original.data) == 0x62);
    for (unsigned bit = 0; bit < 64; ++bit) {
        GpCanFrame changed = original;
        changed.data[bit / 8] ^= (uint8_t)(1u << (bit % 8));
        unchanged(&changed, GP_INVALID);
    }
    for (unsigned seq = 0; seq <= UINT16_MAX; ++seq) {
        GpCanFrame frame = encoded((TargetSample){(uint16_t)seq, 1000, -1000, 7});
        CHECK(frame.data[7] == reference_crc(frame.data));
    }
}

static void malformed(void)
{
    GpCanFrame frame = encoded((TargetSample){2, 0, 0, 0});
    for (unsigned dlc = 0; dlc <= UINT8_MAX; ++dlc) {
        if (dlc == 8) continue;
        frame.dlc = (uint8_t)dlc;
        unchanged(&frame, GP_INVALID);
    }
    frame.dlc = 8;
    for (unsigned id = 0; id <= UINT16_MAX; ++id) {
        if (id == 0x105) continue;
        frame.id = (uint16_t)id;
        unchanged(&frame, GP_INVALID);
    }
    unchanged(NULL, GP_BAD_ARG);
    CHECK(GP_DecodeCan(&frame, NULL) == GP_BAD_ARG);
}

static void limits(void)
{
    /* Recompute CRC so these exercise decoded semantic checks, not CRC rejection. */
    GpCanFrame frame = encoded((TargetSample){0xffff, 0, 0, 7});
    for (unsigned raw = 0; raw <= UINT16_MAX; ++raw) {
        frame.data[2] = (uint8_t)raw;
        frame.data[3] = (uint8_t)(raw >> 8);
        frame.data[7] = reference_crc(frame.data);
        TargetSample out;
        CHECK(GP_DecodeCan(&frame, &out) == (raw <= 1000 ? GP_OK : GP_INVALID));
    }
    frame.data[2] = frame.data[3] = 0;
    for (unsigned raw = 0; raw <= UINT16_MAX; ++raw) {
        int value = raw <= 32767u ? (int)raw : (int)raw - 65536;
        frame.data[4] = (uint8_t)raw;
        frame.data[5] = (uint8_t)(raw >> 8);
        frame.data[7] = reference_crc(frame.data);
        TargetSample out;
        CHECK(GP_DecodeCan(&frame, &out) ==
              (value >= -1000 && value <= 1000 ? GP_OK : GP_INVALID));
    }
}

static void flags(void)
{
    for (unsigned flags = 0; flags <= UINT8_MAX; ++flags) {
        for (unsigned moving = 0; moving < 2; ++moving) {
            TargetSample sample = {1, (uint16_t)moving, 0, (uint8_t)flags};
            bool valid = (flags & ~7u) == 0 && ((flags & 1u) || (!flags && !moving));
            GpCanFrame frame = {0x105, 8, {1, 0, (uint8_t)moving, 0, 0, 0, (uint8_t)flags, 0}};
            frame.data[7] = reference_crc(frame.data);
            TargetSample out;
            CHECK(GP_DecodeCan(&frame, &out) == (valid ? GP_OK : GP_INVALID));
            CHECK(GP_EncodeCan(&sample, &frame) == (valid ? GP_OK : GP_INVALID));
        }
    }
    GpCanFrame invalid_stop = encoded((TargetSample){1, 0, 1, GP_FLAG_VALID});
    invalid_stop.data[6] = 0;
    invalid_stop.data[7] = reference_crc(invalid_stop.data);
    unchanged(&invalid_stop, GP_INVALID);
}

static void output_preserved(void)
{
    GpCanFrame out = encoded((TargetSample){1, 600, -200, 7});
    GpCanFrame before = out;
    const TargetSample invalid[] = {
        {2, 1001, 0, 7}, {2, 0, -1001, 7}, {2, 0, 1001, 7},
        {2, 0, 0, 0x80}, {2, 1, 0, 0}, {2, 0, 0, GP_FLAG_ARM}
    };
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        CHECK(GP_EncodeCan(&invalid[i], &out) == GP_INVALID);
        CHECK(out.id == before.id && out.dlc == before.dlc &&
              memcmp(out.data, before.data, sizeof(out.data)) == 0);
    }
    CHECK(GP_EncodeCan(NULL, &out) == GP_BAD_ARG);
    CHECK(GP_EncodeCan(&invalid[0], NULL) == GP_BAD_ARG);
    GpCanFrame bad = before;
    bad.data[6] = 0x80;
    bad.data[7] = reference_crc(bad.data);
    unchanged(&bad, GP_INVALID);
}

int main(int argc, char **argv)
{
    const struct { const char *name; void (*run)(void); } cases[] = {
        {"layout", layout}, {"crc", crc}, {"malformed", malformed},
        {"limits", limits}, {"flags", flags}, {"output_preserved", output_preserved}
    };
    CHECK(argc == 2);
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        if (strcmp(argv[1], cases[i].name) == 0) {
            cases[i].run();
            printf("PASS codec %s\n", cases[i].name);
            return 0;
        }
    }
    return 2;
}
