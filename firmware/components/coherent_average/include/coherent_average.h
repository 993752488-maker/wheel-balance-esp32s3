#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "sync_analysis.h"

enum { COHERENT_MAX_TURNS=128 };
typedef enum { COHERENT_ADDED,COHERENT_SKIPPED,COHERENT_ARGUMENT,
    COHERENT_CONTEXT,COHERENT_ORDER,COHERENT_CAPACITY,COHERENT_NUMERIC } coherent_status_t;
typedef struct {
    double lower_rpm,upper_rpm;
    unsigned min_turns;
    uint32_t epoch;
    sync_time_quality_t time_quality;
} coherent_config_t;
typedef struct {
    uint64_t reference_sequence,time_us;
    uint32_t epoch,faults;
    double rpm,real,imag;
} coherent_input_t;
typedef struct {
    coherent_config_t config;
    bool initialized;
    coherent_status_t failure;
    size_t count;
    uint64_t last_reference,last_time_us;
    double mean_real,mean_imag,m2,sum_magnitudes,min_rpm,max_rpm;
} coherent_average_t;
typedef struct {
    coherent_status_t status;
    size_t count;
    uint32_t epoch;
    bool mathematical_valid,phase_defined,diagnostic_only;
    bool measurement_qualified,recommendation_valid;
    double real,imag,peak,phase_deg,direction_coherence,scalar_peak_mean;
    double vector_scatter_rms,min_rpm,max_rpm;
} coherent_result_t;

/* A speed-band vector mean, NOT a single-RPM calibration. Inputs must be
 * ordered, full-turn complex fits from one unchanged acquisition context.
 * Weak single-turn SNR is permitted; caller must reject faulty/transient
 * windows and establish matching RPM before supplying trial differences.
 * Coherence/scatter have no certified statistical coverage. No heap/RTOS.
 * Errors latch and suppress old results until explicit reinitialization. */
bool coherent_average_init(coherent_average_t *,const coherent_config_t *);
coherent_status_t coherent_average_push(coherent_average_t *,const coherent_input_t *);
void coherent_average_result(const coherent_average_t *,coherent_result_t *);
