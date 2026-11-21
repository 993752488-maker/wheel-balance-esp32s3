#pragma once
#include "product_engine.h"
typedef struct {
    bool valid;
    unsigned axis;
    double score[3];
    product_reason_t reason;
} product_axis_choice_t;
/* Baseline ONLY: median per-speed-band directional coherence, then usable
 * turn count and fixed XYZ order. No trial/retest/desired mass is an input.
 * This selects a channel, never qualifies a balance estimate. */
bool product_choose_axis(const product_run_t baseline[3],product_axis_choice_t *);
