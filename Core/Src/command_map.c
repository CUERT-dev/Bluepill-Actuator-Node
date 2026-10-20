#include "command_map.h"

#include <stddef.h>
#include <string.h>

/* Internal helper: clear output history without changing configuration. */
static void clear_history(CommandMap *map)
{
    if (map == NULL)
    {
        return;
    }

    map->previous.throttle_duty_pm = 0;
    map->previous.steering_rate_pm = 0;

    map->throttle_remainder = 0U;
    map->steering_remainder = 0U;
}

/* Move one value toward its target using a rate limit in pm/s. */
static int32_t slew_toward(int32_t current,
                           int32_t target,
                           uint16_t slew_rate_pm_s,
                           uint32_t dt_ms,
                           uint32_t *remainder)
{
    int32_t effective_target;
    int32_t distance;
    uint64_t accumulated;
    uint64_t max_step;

    /* A neutral request stops this axis immediately. */
    if (target == 0)
    {
        *remainder = 0U;
        return 0;
    }

    effective_target = target;

    /*
     * If steering changes direction, reach zero first.
     * The new direction will be handled on the next update.
     */
    if ((current < 0 && target > 0) ||
        (current > 0 && target < 0))
    {
        effective_target = 0;
        *remainder = 0U;
    }

    if (current == effective_target)
    {
        *remainder = 0U;
        return current;
    }

    distance = effective_target - current;

    /*
     * Keep fractional progress between updates.
     * Divide by 1000 because dt_ms is measured in milliseconds.
     */
    accumulated = ((uint64_t)slew_rate_pm_s * dt_ms)
                  + *remainder;

    max_step = accumulated / 1000U;

    if (max_step == 0U)
    {
        *remainder = (uint32_t)(accumulated % 1000U);
        return current;
    }

    /* Do not overshoot the target. */
    if (max_step >= (uint64_t)(
            (distance < 0) ? -distance : distance))
    {
        *remainder = 0U;
        return effective_target;
    }

    *remainder = (uint32_t)(accumulated % 1000U);

    if (distance > 0)
    {
        return current + (int32_t)max_step;
    }

    return current - (int32_t)max_step;
}

GpResult CommandMap_Init(CommandMap *map,
                         const CommandMapConfig *config)
{
    CommandMapConfig config_copy;

    if (map == NULL || config == NULL)
    {
        return GP_BAD_ARG;
    }

    /* Copy first in case config refers to memory inside map. */
    config_copy = *config;

    /* Leave the module uninitialized unless validation succeeds. */
    memset(map, 0, sizeof(*map));

    if (config_copy.throttle_cap_pm > 1000U ||
        config_copy.steering_cap_pm > 1000U ||
        config_copy.throttle_slew_pm_s == 0U ||
        config_copy.steering_slew_pm_s == 0U ||
        config_copy.max_dt_ms == 0U)
    {
        return GP_INVALID;
    }

    map->config = config_copy;
    clear_history(map);
    map->initialized = true;

    return GP_OK;
}

GpResult CommandMap_Update(CommandMap *map,
                           const TargetSample *command,
                           uint32_t dt_ms,
                           bool enabled,
                           MotorOutput *out)
{
    int32_t throttle_target;
    int32_t steering_target;
    int32_t throttle_result;
    int32_t steering_result;

    /* If possible, make the output safe before checking arguments. */
    if (out != NULL)
    {
        out->throttle_duty_pm = 0;
        out->steering_rate_pm = 0;
    }

    if (map == NULL || command == NULL || out == NULL)
    {
        return GP_BAD_ARG;
    }

    if (!map->initialized)
    {
        clear_history(map);
        return GP_INVALID;
    }

    /* Disabled means immediate stop, not a gradual ramp-down. */
    if (!enabled)
    {
        clear_history(map);
        return GP_OK;
    }

    /* Validate the command's numerical ranges and update timing. */
    if (command->throttle_pm > 1000U ||
        command->steering_pm < -1000 ||
        command->steering_pm > 1000 ||
        dt_ms == 0U ||
        dt_ms > map->config.max_dt_ms)
    {
        clear_history(map);
        return GP_INVALID;
    }

    /* Apply the configured output caps. */
    throttle_target = (int32_t)command->throttle_pm;

    if (throttle_target > (int32_t)map->config.throttle_cap_pm)
    {
        throttle_target = (int32_t)map->config.throttle_cap_pm;
    }

    steering_target = (int32_t)command->steering_pm;

    if (steering_target > (int32_t)map->config.steering_cap_pm)
    {
        steering_target = (int32_t)map->config.steering_cap_pm;
    }
    else if (steering_target < -(int32_t)map->config.steering_cap_pm)
    {
        steering_target = -(int32_t)map->config.steering_cap_pm;
    }

    /* Calculate throttle and steering independently. */
    throttle_result = slew_toward(
        (int32_t)map->previous.throttle_duty_pm,
        throttle_target,
        map->config.throttle_slew_pm_s,
        dt_ms,
        &map->throttle_remainder);

    steering_result = slew_toward(
        (int32_t)map->previous.steering_rate_pm,
        steering_target,
        map->config.steering_slew_pm_s,
        dt_ms,
        &map->steering_remainder);

    /* Save the results so the next update can continue from them. */
    map->previous.throttle_duty_pm = (int16_t)throttle_result;
    map->previous.steering_rate_pm = (int16_t)steering_result;

    *out = map->previous;

    return GP_OK;
}

void CommandMap_Reset(CommandMap *map)
{
    if (map == NULL)
    {
        return;
    }

    /* Keep the configuration and initialized state intact. */
    clear_history(map);
}