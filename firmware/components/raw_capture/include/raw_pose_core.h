#pragma once
#include "raw_capture_core.h"

/* Optional, sparse register observations. These are NOT FIFO/ADC timestamps,
 * nor an input to production balancing. Acceleration recording remains valid
 * independently of optional diagnostic loss. */
typedef struct {
    uint64_t ingest_us, fifo_sequence, read_begin_us, read_end_us;
    uint32_t epoch;
    int16_t accel[3], temperature, gyro[3];
    bool frame_consistent;
} raw_pose_t;
enum {
    RAW_POSE_METADATA=1U<<0, RAW_POSE_EPOCH=1U<<1, RAW_POSE_TIME=1U<<2,
    RAW_POSE_FRAME=1U<<3, RAW_POSE_CAPACITY=1U<<4, RAW_POSE_SATURATION=1U<<5
};
typedef struct {
    bool enabled, frozen;
    uint32_t capture_id, faults, queue_drops, boundary_excluded, stale_events;
    uint32_t first_epoch, last_epoch;
    size_t count, capacity;
} raw_pose_status_t;
typedef struct {
    raw_pose_status_t status;
    raw_pose_t *storage;
} raw_pose_core_t;
void raw_pose_init(raw_pose_core_t *, raw_pose_t *, size_t capacity);
void raw_pose_begin(raw_pose_core_t *, uint32_t capture_id);
bool raw_pose_add(raw_pose_core_t *, const raw_status_t *, uint32_t capture_id, const raw_pose_t *);
void raw_pose_finish(raw_pose_core_t *, const raw_status_t *, uint32_t drops);
size_t raw_pose_csv(char *, size_t, uint32_t capture_id, size_t index, const raw_pose_t *);
