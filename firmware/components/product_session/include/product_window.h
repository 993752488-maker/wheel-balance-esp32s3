#pragma once
#include "product_job.h"

#define SELECT_SETTLING 512U
#define SELECT_WINDOW_LIMIT 1024U
#define PRODUCT_WINDOW_KEEP PRODUCT_MAX_TURNS
typedef enum { WINDOW_WAITING,WINDOW_SETTLING,WINDOW_COLLECTING,WINDOW_COMPLETE } product_window_phase_t;
typedef enum { WINDOW_UNDETERMINED,WINDOW_STEADY,WINDOW_COAST,WINDOW_MIXED } product_window_mode_t;
typedef struct {
    uint32_t capture_id,epoch,total_turns,accelerating_turns,mode,phase,collecting_mask,common_pairs;
    uint32_t usable[3],skipped[3],transient[3];
    uint32_t end_reason; /* Schema 2 uses the former 4-byte alignment padding. */
    uint64_t first_usable_us[3],last_usable_us[3];
    uint64_t start_us,end_us,updated_us;
} product_window_stage_t;
/* Optional sidecar: released snapshot/trial/retest/speed ABIs stay readable. */
typedef struct {
    uint32_t schema,session_id,snapshot_revision,generation;
    bool simulated;char boot_id[17];
    product_capture_config_t capture; /* Former header padding; ABI stays 472 bytes. */
    product_window_stage_t stages[3];
} product_window_t;

/* Mutates only selection reason bits. Same deterministic rule on prefix and
 * final replay, all three axes, independent of trial response or improvement.
 * Detected startup/transients are excluded; hand release is not proven. */
void product_window_select(selection_turn_t *,size_t,product_window_stage_t *);
void product_window_bind(product_window_t *,const product_snapshot_t *);
void product_window_reset(product_window_t *,const product_snapshot_t *);
void product_window_clear(product_window_t *,unsigned);
bool product_window_valid(const product_window_t *,const product_snapshot_t *,bool stored);
