#ifndef COMMAND_MAP_H
#define COMMAND_MAP_H

#include <stdint.h>
#include <stdbool.h>

#include "gp_types.h"

/**
 * @brief Initialize the command mapping module.
 *
 * @param map Pointer to the command map instance.
 * @param config Pointer to the mapping configuration.
 *
 * @return GP_OK on success.
 *         GP_BAD_ARG if a required pointer is NULL.
 *         GP_INVALID if the configuration is invalid.
 */
GpResult CommandMap_Init(
    CommandMap *map,
    const CommandMapConfig *config
);

/**
 * @brief Map a target command to motor output with slew-rate limiting.
 *
 * @param map Pointer to the initialized command map.
 * @param command Command in per-mille, with VALID and no unknown flags.
 * @param dt_ms Elapsed update time in milliseconds.
 * @param enabled Whether motion is currently permitted.
 * @param out Pointer to the resulting motor output.
 *
 * @return GP_OK on success.
 *         GP_BAD_ARG if a required pointer is NULL.
 *         GP_INVALID if the command or timing is invalid.
 * Missing ARM or DEADMAN stops immediately with GP_OK. Disabled updates
 * stop without validating values/timing. Every error clears available output
 * and history. out may refer to map->previous.
 */
GpResult CommandMap_Update(
    CommandMap *map,
    const TargetSample *command,
    uint32_t dt_ms,
    bool enabled,
    MotorOutput *out
);

/**
 * @brief Reset the mapping state while preserving its configuration.
 *
 * @param map Pointer to the command map instance.
 */
void CommandMap_Reset(CommandMap *map);

#endif /* COMMAND_MAP_H */
