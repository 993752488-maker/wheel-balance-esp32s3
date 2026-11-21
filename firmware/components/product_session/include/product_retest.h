#pragma once
#include "product_engine.h"
#define PRODUCT_RETEST_HISTORY 6U
typedef enum { RETEST_CONDITION_NONE,RETEST_CONDITION_FIRST,RETEST_CONDITION_SAME,
    RETEST_CONDITION_CHANGED,RETEST_CONDITION_LEGACY } product_retest_condition_t;
typedef struct {
    uint32_t ordinal,correction_group;
    product_retest_condition_t condition;
    bool legacy;
    product_stage_summary_t stage;
    product_comparison_t comparison;
    product_repeat_observation_t repeat;
} product_retest_entry_t;
typedef struct {
    uint32_t schema,session_id,snapshot_revision;
    char boot_id[17];
    uint32_t count,total_count,dropped_count;
    bool legacy_count_unknown,current_group_variable;
    uint32_t original_capture_id,trial_capture_id;
    product_config_t config;
    product_retest_entry_t entries[PRODUCT_RETEST_HISTORY];
} product_retest_log_t;
void product_retest_init(product_retest_log_t *,const product_snapshot_t *);
bool product_retest_valid(const product_retest_log_t *,const product_snapshot_t *);
/* Legacy snapshots contain only their latest retest; never invent lost rows. */
void product_retest_import_latest(product_retest_log_t *,const product_snapshot_t *);
bool product_retest_can_start(const product_retest_log_t *,product_retest_condition_t);
bool product_retest_available(const product_snapshot_t *);
bool product_retest_record(product_retest_log_t *,const product_snapshot_t *,product_retest_condition_t,
    const product_repeat_observation_t *);
const char *product_retest_condition_name(product_retest_condition_t);
