#pragma once
#include <stddef.h>
#include "live_analysis_core.h"

/* Formats one bounded diagnostic line. Zero and an empty output on invalid
 * input/truncation. Only current ESTIMATED hardware snapshots are accepted;
 * this never grants phase qualification or produces a balancing suggestion. */
size_t live_analysis_record(char *out, size_t capacity,
                            const live_analysis_status_t *status, unsigned axis);
size_t live_coast_record(char *out, size_t capacity,
                         const live_analysis_status_t *status, unsigned axis);
