/*
 * CUERT GP CAN command codec, API revision 10.
 * Adapted from ESP32-Sensing-Node/main/lib/gp-protocol/src/gp-protocol.c.
 * Uses the authoritative gp_types.h instead of redefining shared types.
 * Pure C; no HAL, ESP-IDF or RTOS dependencies.
 */
#include "gp_types.h"
#include <limits.h>

#define CAN_DLC 8u
#define CAN_CRC_POSITION 7u

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t read_i16_le(const uint8_t *p)
{
    uint16_t raw = read_u16_le(p);
    if (raw <= (uint16_t)INT16_MAX) {
        return (int16_t)raw;
    }
    return (int16_t)((int32_t)raw - 65536);
}

static void write_u16_le(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static uint8_t crc8(const uint8_t *bytes, size_t length)
{
    uint8_t crc = 0u;
    for (size_t i = 0u; i < length; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0u; bit < 8u; ++bit) {
            crc = (crc & 0x80u) != 0u
                ? (uint8_t)((crc << 1) ^ 0x07u)
                : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static bool valid_sample(const TargetSample *sample)
{
    if (sample->throttle_pm > 1000u ||
        sample->steering_pm < -1000 || sample->steering_pm > 1000 ||
        (sample->flags & (uint8_t)~(GP_FLAG_VALID | GP_FLAG_ARM | GP_FLAG_DEADMAN)) != 0u) {
        return false;
    }
    if ((sample->flags & GP_FLAG_VALID) == 0u) {
        return sample->flags == 0u &&
               sample->throttle_pm == 0u && sample->steering_pm == 0;
    }
    return true;
}

GpResult GP_EncodeCan(const TargetSample *command, GpCanFrame *out)
{
    GpCanFrame encoded = {0};
    if (command == NULL || out == NULL) {
        return GP_BAD_ARG;
    }
    if (!valid_sample(command)) {
        return GP_INVALID;
    }
    encoded.id = (uint16_t)GP_CAN_COMMAND_ID;
    encoded.dlc = CAN_DLC;
    write_u16_le(&encoded.data[0], command->sequence);
    write_u16_le(&encoded.data[2], command->throttle_pm);
    write_u16_le(&encoded.data[4], (uint16_t)command->steering_pm);
    encoded.data[6] = command->flags;
    encoded.data[CAN_CRC_POSITION] = crc8(encoded.data, CAN_CRC_POSITION);
    *out = encoded;
    return GP_OK;
}

GpResult GP_DecodeCan(const GpCanFrame *frame, TargetSample *out)
{
    TargetSample decoded;
    if (frame == NULL || out == NULL) {
        return GP_BAD_ARG;
    }
    if (frame->id != GP_CAN_COMMAND_ID || frame->dlc != CAN_DLC ||
        frame->data[CAN_CRC_POSITION] != crc8(frame->data, CAN_CRC_POSITION)) {
        return GP_INVALID;
    }
    decoded.sequence = read_u16_le(&frame->data[0]);
    decoded.throttle_pm = read_u16_le(&frame->data[2]);
    decoded.steering_pm = read_i16_le(&frame->data[4]);
    decoded.flags = frame->data[6];
    if (!valid_sample(&decoded)) {
        return GP_INVALID;
    }
    *out = decoded;
    return GP_OK;
}
