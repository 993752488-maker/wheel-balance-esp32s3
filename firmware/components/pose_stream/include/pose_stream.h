#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
typedef struct {
    uint64_t fifo_sequence,read_begin_us,read_end_us;
    uint32_t epoch;
    int16_t accel[3],temperature,gyro[3];
    bool frame_consistent;
} pose_observation_t;
esp_err_t pose_stream_start(const char *boot_id);
/* Task producer only, zero-wait queue fork. Not a correction pipeline. */
void pose_stream_publish(const pose_observation_t *);
