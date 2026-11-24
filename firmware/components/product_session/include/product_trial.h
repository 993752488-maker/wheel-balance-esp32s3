#pragma once
#include <stddef.h>
#include "product_engine.h"

/* Separate, versioned sidecar: schema-1 product_snapshot_t stays readable. */
#define PRODUCT_TRIAL_ATTEMPTS 3U
#define PRODUCT_GUIDED_TRIAL_MAX_G 40.0
typedef enum { TRIAL_PLAN_STARTER,TRIAL_PLAN_MANUAL,TRIAL_PLAN_RESPONSE } product_trial_source_t;
typedef struct {
    double mass_g;
    product_trial_source_t source;
    uint32_t axis; /* Schema 3 uses the former alignment padding. */
    product_stage_summary_t stage;
    product_solution_t solution;
} product_trial_attempt_t;
typedef struct {
    uint32_t schema,session_id,snapshot_revision;
    char boot_id[17];
    double piece_mass_g,initial_mass_g;
    product_trial_source_t source;
    uint32_t count;
    product_trial_attempt_t attempts[PRODUCT_TRIAL_ATTEMPTS];
    /* Appended after the entire 920-byte v1 payload. */
    bool automatic_axis,axis_resolved;
    double axis_score[3];
    /* Schema 4 keeps the physical angle of each attempt as the plan moves. */
    double initial_angle_deg,attempt_angle_deg[PRODUCT_TRIAL_ATTEMPTS];
} product_trial_guide_t;
typedef enum { TRIAL_NEXT_READY,TRIAL_NEXT_NO_BASELINE,TRIAL_NEXT_BUSY,TRIAL_NEXT_FINISHED,
    TRIAL_NEXT_HARD_FAULT,TRIAL_NEXT_LIMIT,TRIAL_NEXT_NO_PREDICTION,TRIAL_NEXT_RESPONSE,
    TRIAL_NEXT_LIGHTER_COMPARISON,TRIAL_NEXT_OPPOSITE,TRIAL_NEXT_STEP_UP,
    TRIAL_NEXT_MATCH_SPEED,TRIAL_NEXT_CLEAN_RECORD } product_trial_next_t;
typedef struct {
    bool editable,retry_available,suggested;
    double mass_g,angle_deg;
    product_trial_next_t reason;
} product_trial_advice_t;

const char *product_trial_source_name(product_trial_source_t);
const char *product_trial_next_name(product_trial_next_t);
bool product_trial_init(product_trial_guide_t *,product_snapshot_t *,double piece_g,bool guided);
bool product_trial_plan_original(product_trial_guide_t *,product_snapshot_t *,double rpm);
bool product_trial_valid(const product_trial_guide_t *,const product_snapshot_t *);
/* Accept the original v1 sidecar as an explicitly manual historical channel. */
bool product_trial_restore(product_trial_guide_t *,const product_snapshot_t *,size_t stored_bytes);
void product_trial_advice(const product_trial_guide_t *,const product_snapshot_t *,product_trial_advice_t *);
void product_trial_apply_advice(product_trial_guide_t *,product_snapshot_t *);
bool product_trial_contradicts_previous(const product_trial_guide_t *,const product_snapshot_t *);
/* Only the trial mass changes here. Radius, direction and the original
 * capture stay bound to the session. Installation changes use PRODUCT_NEW. */
bool product_trial_set_plan(product_trial_guide_t *,product_snapshot_t *,double,product_trial_source_t);
bool product_trial_can_start(const product_trial_guide_t *,const product_snapshot_t *,double confirmed_mass_g);
/* Append once, including failed/cancelled captures. Never rewrite an attempt. */
bool product_trial_record(product_trial_guide_t *,const product_snapshot_t *);
