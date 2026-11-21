#include "sdkconfig.h"
#include "live_analysis.h"
#include <string.h>
#include "acquisition.h"
#include "optical_ref.h"
#include "esp_timer.h"

const char *live_analysis_state_name(live_analysis_state_t state)
{
    switch (state) {
    case LIVE_ANALYSIS_DISABLED: return "DISABLED";
    case LIVE_ANALYSIS_WAIT_SENSOR: return "WAIT_SENSOR";
    case LIVE_ANALYSIS_WAIT_REFERENCE: return "WAIT_REFERENCE";
    case LIVE_ANALYSIS_COLLECTING: return "COLLECTING";
    case LIVE_ANALYSIS_READY_ESTIMATED: return "READY_ESTIMATED";
    case LIVE_ANALYSIS_UNSTABLE_RPM: return "UNSTABLE_RPM";
    case LIVE_ANALYSIS_INPUT_LOSS: return "INPUT_LOSS";
    case LIVE_ANALYSIS_TIMING_ERROR: return "TIMING_ERROR";
    case LIVE_ANALYSIS_INSUFFICIENT: return "INSUFFICIENT";
    case LIVE_ANALYSIS_SATURATION: return "SATURATION";
    default: return "DISABLED";
    }
}

#if CONFIG_LIVE_ANALYSIS_ENABLED
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SAMPLE_CAPACITY 8192U
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static live_analysis_core_t s_core; /* worker-owned */
static live_analysis_status_t s_status;
static uint32_t s_epoch, s_i2c_seen, s_fifo_seen;
static bool s_started;

static void clear_result(live_analysis_status_t *status, live_analysis_state_t reason)
{
    memset(&status->result, 0, sizeof(status->result));
    status->state = reason;
    status->updated_us = 0;
    status->window_ref_start_sequence = status->window_ref_end_sequence = 0;
    status->window_start_us = status->window_end_us = 0;
    status->window_epoch = 0;
    memset(&status->coast,0,sizeof(status->coast));
    status->coast_updated_us=0;
}

static void publish(const acquisition_status_t *imu, const optical_ref_status_t *ref)
{
    s_core.status.sample_queue_drops = imu->analysis_drops;
    s_core.status.reference_queue_drops = ref->analysis_queue_drops;
    portENTER_CRITICAL(&s_lock);
    s_status = s_core.status;
    s_epoch = imu->epoch;
    s_i2c_seen = imu->i2c_errors;
    s_fifo_seen = imu->fifo_loss_resets;
    portEXIT_CRITICAL(&s_lock);
}

static void discard_inputs(void)
{
    acceleration_sample_t sample;
    optical_ref_event_t ref;
    /* Queues have fixed capacity; bounded drains cannot wait on producers. */
    for (unsigned i=0; i<256 && acquisition_pop_analysis_sample(&sample); ++i) {}
    for (unsigned i=0; i<64 && optical_ref_pop_analysis_event(&ref); ++i) {}
}

static void analysis_task(void *arg)
{
    (void)arg;
    uint32_t samples_dropped=0, references_dropped=0, last_i2c=0, last_fifo=0;
    while (true) {
        acquisition_status_t imu;
        optical_ref_status_t ref;
        acquisition_get_status(&imu);
        optical_ref_get_status(&ref);
        const bool loss = imu.analysis_drops != samples_dropped ||
            ref.analysis_queue_drops != references_dropped || imu.i2c_errors != last_i2c ||
            imu.fifo_loss_resets != last_fifo;
        samples_dropped = imu.analysis_drops;
        references_dropped = ref.analysis_queue_drops;
        last_i2c = imu.i2c_errors;
        last_fifo = imu.fifo_loss_resets;
        if (!imu.online || loss || ref.input_queue_drops || ref.clock_faults || ref.sample_gaps ||
            !ref.enabled || ref.timed_out) {
            live_core_reset(&s_core, !imu.online ? LIVE_ANALYSIS_WAIT_SENSOR :
                (loss || ref.input_queue_drops || ref.clock_faults || ref.sample_gaps) ?
                LIVE_ANALYSIS_INPUT_LOSS : LIVE_ANALYSIS_WAIT_REFERENCE);
            discard_inputs();
            publish(&imu, &ref);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        acceleration_sample_t sample;
        for (unsigned i=0; i<256 && acquisition_pop_analysis_sample(&sample); ++i) {
            const live_input_sample_t input = {
                .sequence=sample.sequence, .epoch=sample.epoch,
                .count_begin_us=sample.fifo_count_begin_us, .count_end_us=sample.fifo_count_end_us,
                .pending_frames=sample.fifo_pending_frames, .batch_index=sample.batch_index,
                .raw={sample.raw.x, sample.raw.y, sample.raw.z},
            };
            live_core_sample(&s_core, &input);
        }
        optical_ref_event_t event;
        for (unsigned i=0; i<64 && optical_ref_pop_analysis_event(&event); ++i)
            live_core_reference(&s_core, event.accepted_sequence, event.timestamp_us);
        live_core_analyze(&s_core, esp_timer_get_time());
        publish(&imu, &ref);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t live_analysis_start(void)
{
    if (s_started) return ESP_ERR_INVALID_STATE;
    if (CONFIG_V0_PULSES_PER_REV != 1) return ESP_ERR_NOT_SUPPORTED;
    /* No giant internal-RAM fallback: failure disables only analysis, leaving
     * sensor transport and the diagnostic webpage available. */
    sync_sample_t *samples = heap_caps_calloc(SAMPLE_CAPACITY, sizeof(*samples),
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!samples) return ESP_ERR_NO_MEM;
    live_core_init(&s_core, samples, SAMPLE_CAPACITY);
    s_status = s_core.status;
    if (xTaskCreatePinnedToCore(analysis_task, "live_1x", 6144, NULL, 5, NULL, 1) != pdPASS) {
        heap_caps_free(samples);
        memset(&s_core, 0, sizeof(s_core));
        memset(&s_status, 0, sizeof(s_status));
        return ESP_ERR_NO_MEM;
    }
    s_started = true;
    return ESP_OK;
}

void live_analysis_get_status(live_analysis_status_t *status)
{
    if (!status) return;
    uint32_t epoch, i2c, fifo;
    portENTER_CRITICAL(&s_lock);
    *status = s_status;
    epoch = s_epoch; i2c = s_i2c_seen; fifo = s_fifo_seen;
    portEXIT_CRITICAL(&s_lock);
    status->timing_estimated = true;
    status->measurement_qualified = false;
    if (!status->enabled) { clear_result(status, LIVE_ANALYSIS_DISABLED); return; }
    acquisition_status_t imu;
    optical_ref_status_t ref;
    acquisition_get_status(&imu);
    optical_ref_get_status(&ref);
    if (!imu.online) { clear_result(status, LIVE_ANALYSIS_WAIT_SENSOR); return; }
    if (imu.epoch != epoch || (status->result.valid && status->window_epoch != imu.epoch) ||
        (status->coast.valid && status->coast.epoch != imu.epoch) ||
        imu.i2c_errors != i2c || imu.fifo_loss_resets != fifo ||
        imu.analysis_drops != status->sample_queue_drops ||
        ref.analysis_queue_drops != status->reference_queue_drops ||
        ref.input_queue_drops || ref.clock_faults || ref.sample_gaps) {
        clear_result(status, LIVE_ANALYSIS_INPUT_LOSS);
        return;
    }
    if (!ref.enabled || !ref.rpm.has_accepted_edge || ref.timed_out) {
        clear_result(status, LIVE_ANALYSIS_WAIT_REFERENCE);
        return;
    }
    if (!ref.rpm.rpm_valid) { clear_result(status, LIVE_ANALYSIS_COLLECTING); return; }
    const uint64_t now = (uint64_t)esp_timer_get_time();
    /* A fresh HTTP response must never make an old rotating result live.
     * Two measured periods allows long marks/trailing-edge reporting. */
    uint64_t fresh_us = ref.rpm.period_us < UINT64_MAX/2U-100000U ?
        ref.rpm.period_us*2U+100000U : UINT64_MAX;
    const uint64_t stop_us = (uint64_t)CONFIG_V0_STOP_TIMEOUT_MS*1000U;
    if (fresh_us > stop_us) fresh_us = stop_us;
    if (now < ref.rpm.last_accepted_edge_us || now-ref.rpm.last_accepted_edge_us >= fresh_us ||
        (status->result.valid && (status->updated_us <= 0 || now < (uint64_t)status->updated_us ||
         now-(uint64_t)status->updated_us >= fresh_us)))
        clear_result(status, LIVE_ANALYSIS_WAIT_REFERENCE);
    if(status->coast.valid && (status->coast_updated_us<=0 ||
        now<(uint64_t)status->coast_updated_us || now-(uint64_t)status->coast_updated_us>=fresh_us)) {
        memset(&status->coast,0,sizeof(status->coast));
        status->coast_updated_us=0;
    }
}

#else
esp_err_t live_analysis_start(void) { return ESP_ERR_NOT_SUPPORTED; }
void live_analysis_get_status(live_analysis_status_t *status)
{
    if (!status) return;
    memset(status, 0, sizeof(*status));
    status->timing_estimated = true;
    status->state = LIVE_ANALYSIS_DISABLED;
}
#endif
