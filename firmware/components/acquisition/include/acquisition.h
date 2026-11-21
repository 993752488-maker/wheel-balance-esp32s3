#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "mpu6050.h"
#include "timing_trace_core.h"

typedef struct {
    uint64_t sequence;
    uint32_t epoch; /* changes after sensor restart/loss */
    int64_t readout_us; /* host FIFO read completion; NOT sample timestamp */
    int64_t fifo_count_begin_us, fifo_count_end_us; /* host COUNT observation interval */
    uint16_t fifo_pending_frames, batch_index; /* full count, not limited read batch */
    mpu6050_raw_t raw;
} acceleration_sample_t;

typedef struct {
    bool online;
    uint8_t who_am_i;
    uint8_t identity_reads;
    bool identity_matches_label;
    const char *profile_name; /* driver-owned static string; not proof of authenticity */
    uint16_t fifo_capacity_bytes;
    uint32_t epoch;
    uint32_t init_errors;
    uint32_t i2c_errors;
    uint32_t fifo_overflows;
    uint32_t fifo_loss_resets; /* hardware overflow OR conservative near-full reset */
    uint32_t fifo_count_faults,frame_mismatches;
    uint32_t fifo_count_confirm_recoveries;
    bool frame_consistent;
    float frame_difference_lsb;
    uint32_t stream_drops;
    uint32_t analysis_drops; /* independent nonblocking analysis queue */
    uint32_t restarts;
    uint32_t observed_drdy;
    int64_t last_drdy_us;
    uint64_t total_samples;
    int64_t last_data_us;
    esp_err_t last_error;
    bool storage_paused; /* Intentional idle-only gap; never inside a capture. */
    uint32_t storage_pauses;
    mpu6050_raw_t direct_raw; /* diagnostic register read; not synchronized to FIFO sample */
    int64_t direct_readout_us;
    acceleration_sample_t latest;
} acquisition_status_t;

/* gpio_install_isr_service(0) must be called once by the application. */
esp_err_t acquisition_start(void);
void acquisition_get_status(acquisition_status_t *status);
bool acquisition_pop_sample(acceleration_sample_t *sample);
bool acquisition_pop_analysis_sample(acceleration_sample_t *sample);
/* Caller must hold raw_capture_maintenance_begin for the whole operation.
 * FIFO operations execute on the existing I2C owner task. Bounded handshake. */
esp_err_t acquisition_maintenance_begin(void);
esp_err_t acquisition_maintenance_end(void);

/* Optional 3-second GPIO-entry trace; not used to retime live measurements. */
esp_err_t acquisition_timing_begin(uint32_t duration_ms);
void acquisition_timing_status(timing_trace_status_t *);
esp_err_t acquisition_timing_lease(uint32_t id,timing_trace_status_t *);
void acquisition_timing_release(uint32_t id);
size_t acquisition_timing_read(uint32_t id,size_t offset,uint64_t *,size_t capacity);

