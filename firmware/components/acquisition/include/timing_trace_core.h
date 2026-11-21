#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* A bounded observation of GPIO ISR entry times, never calibrated sample times.
 * Caller serializes task/ISR access; storage must be internal RAM on device. */
typedef enum { TIMING_DISABLED, TIMING_IDLE, TIMING_RECORDING, TIMING_READY } timing_state_t;
enum { TIMING_FAULT_OVERFLOW=1u, TIMING_FAULT_CLOCK=2u, TIMING_FAULT_EPOCH=4u };
typedef struct {
    timing_state_t state;
    uint32_t id, epoch, faults;
    uint64_t start_us, deadline_us;
    size_t events;
} timing_trace_status_t;
typedef struct {
    timing_trace_status_t status;
    uint64_t *storage;
    size_t capacity;
    unsigned leases;
} timing_trace_core_t;

void timing_trace_init(timing_trace_core_t *,uint64_t *,size_t);
bool timing_trace_begin(timing_trace_core_t *,uint32_t epoch,uint64_t now_us,uint32_t duration_ms);
void timing_trace_tick(timing_trace_core_t *,uint64_t now_us);
void timing_trace_event(timing_trace_core_t *,uint32_t epoch,uint64_t time_us);
bool timing_trace_lease(timing_trace_core_t *,uint32_t id);
void timing_trace_release(timing_trace_core_t *,uint32_t id);
size_t timing_trace_read(const timing_trace_core_t *,uint32_t id,size_t offset,uint64_t *,size_t);
