#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "live_analysis_core.h"

/* Starts the sole analysis consumer. Call before acquisition/optical start.
 * Producers own separate zero-wait queues, unaffected by serial consumers.
 * Current sample time is a nominal-rate estimate anchored to FIFO COUNT
 * observations, not a measured sample instant. No verified phase or weight.
 */
esp_err_t live_analysis_start(void);
void live_analysis_get_status(live_analysis_status_t *status);
const char *live_analysis_state_name(live_analysis_state_t state);
