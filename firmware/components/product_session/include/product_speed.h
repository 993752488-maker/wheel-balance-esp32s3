#pragma once
#include "product_engine.h"

#define PRODUCT_SPEED_BINS 14U
#define PRODUCT_SPEED_WIDTH 20U
typedef enum { SPEED_EMPTY, SPEED_PREVIEW, SPEED_FROZEN, SPEED_INVALID } product_speed_mode_t;
typedef struct { float rms,minimum,maximum;uint16_t count,reserved; } product_speed_bin_t;
typedef struct {
    uint32_t mode,capture_id,epoch,used_turns,rejected_turns,outside_turns;
    uint64_t start_us,end_us,updated_us,last_reference;
    product_speed_bin_t bins[PRODUCT_SPEED_BINS];
} product_speed_stage_t;
/* Optional summary only. Never consumed by the balancing solver. */
typedef struct {
    uint32_t schema,session_id,snapshot_revision,generation,axis;
    bool simulated;char boot_id[17];
    product_speed_stage_t stages[3];
} product_speed_t;
void product_speed_reset(product_speed_t *,const product_snapshot_t *);
void product_speed_bind(product_speed_t *,const product_snapshot_t *);
bool product_speed_set(product_speed_t *,unsigned,const product_run_t *,bool preview,uint64_t updated_us);
void product_speed_clear(product_speed_t *,unsigned);
bool product_speed_valid(const product_speed_t *,const product_snapshot_t *,bool stored);
