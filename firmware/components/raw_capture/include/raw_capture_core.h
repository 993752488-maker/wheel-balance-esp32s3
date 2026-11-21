#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum { RAW_IDLE, RAW_RECORDING, RAW_CLOSING, RAW_READY } raw_state_t;
typedef enum { RAW_CONTEXT_USER, RAW_CONTEXT_BOOT_CHECK, RAW_CONTEXT_USB } raw_context_t;
typedef enum { RAW_END_NONE, RAW_END_DURATION, RAW_END_MANUAL, RAW_END_CAPACITY } raw_end_t;
enum {
    RAW_FAULT_SENSOR=1U<<0, RAW_FAULT_FIFO=1U<<1, RAW_FAULT_QUEUE=1U<<2,
    RAW_FAULT_SAMPLE_SEQUENCE=1U<<3, RAW_FAULT_EPOCH=1U<<4, RAW_FAULT_TIME=1U<<5,
    RAW_FAULT_CAPACITY=1U<<6, RAW_FAULT_REFERENCE=1U<<7,
    RAW_FAULT_SATURATION=1U<<8, RAW_FAULT_EMPTY=1U<<9, RAW_FAULT_METADATA=1U<<10,
    RAW_FAULT_ALIGNMENT=1U<<11
};

typedef struct {
    uint64_t sequence, ingest_us;
    int64_t readout_us, count_begin_us, count_end_us;
    uint32_t epoch;
    uint16_t pending_frames, batch_index;
    int16_t raw[3];
} raw_sample_t;

typedef struct {
    uint64_t sequence, ingest_us, reference_us;
    bool has_period, timestamp_resynced;
} raw_reference_t;

typedef struct {
    bool enabled;
    raw_state_t state;
    raw_context_t context;
    raw_end_t end_reason;
    uint32_t id, faults, queue_drops, stale_events;
    size_t samples, references, sample_capacity, reference_capacity;
    uint64_t start_us, deadline_us, stop_requested_us, end_us;
    uint32_t first_epoch, last_epoch;
    uint64_t first_sequence, last_sequence, first_reference, last_reference;
    bool integrity_ok; /* recorded-window continuity only; NOT phase qualification */
    bool measurement_qualified; /* always false */
} raw_status_t;

typedef struct {
    raw_status_t status;
    raw_sample_t *sample_storage;
    raw_reference_t *reference_storage;
} raw_core_t;

void raw_core_init(raw_core_t *, raw_sample_t *, size_t, raw_reference_t *, size_t);
bool raw_core_begin(raw_core_t *, uint32_t id, uint64_t now_us, uint64_t duration_us, raw_context_t);
bool raw_core_sample(raw_core_t *, uint32_t id, const raw_sample_t *);
bool raw_core_reference(raw_core_t *, uint32_t id, const raw_reference_t *);
void raw_core_close(raw_core_t *, uint64_t now_us, raw_end_t);
void raw_core_finish(raw_core_t *, uint64_t now_us, uint32_t faults, uint32_t queue_drops);
const char *raw_state_name(raw_state_t);
const char *raw_context_name(raw_context_t);
const char *raw_end_name(raw_end_t);
size_t raw_csv_sample(char *, size_t, uint32_t, const raw_sample_t *);
size_t raw_csv_reference(char *, size_t, uint32_t, const raw_reference_t *);

/* Small reservation fence. Every call is protected by the adapter's internal
 * RAM lock. Closing prevents new reservations; existing ones must enqueue or
 * report loss and release BEFORE the worker seals the drained queue. */
typedef struct {
    uint32_t generation, active_id, inflight, faults, queue_drops;
} raw_gate_t;
bool raw_gate_open(raw_gate_t *, uint32_t id);
uint32_t raw_gate_reserve(raw_gate_t *);
void raw_gate_release(raw_gate_t *, uint32_t token, bool queued);
void raw_gate_close(raw_gate_t *);
void raw_gate_fault(raw_gate_t *, uint32_t faults);
bool raw_gate_quiet(const raw_gate_t *);
