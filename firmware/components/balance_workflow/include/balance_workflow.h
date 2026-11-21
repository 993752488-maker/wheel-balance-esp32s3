#pragma once
#include "balance_core.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    balance_config_t config;
    unsigned axis;
    int direction;
    uint32_t generation, revision;
    bool original_captured, trial_captured, simulated;
    bool can_capture, live_qualified;
    balance_measurement_t original, trial;
    balance_result_t result;
    const char *state;
} balance_workflow_status_t;

/* Owned by the sole synchronous HTTP server task. Configuration/captures are
 * RAM-only and reset on reboot. Never called by acquisition/ISR/timer tasks. */
void balance_workflow_init(void);
bool balance_workflow_configure(double mass, double trial_radius, double angle,
    double correction_radius, unsigned axis, int direction);
bool balance_workflow_command(const char *command);
void balance_workflow_get_status(balance_workflow_status_t *status);
