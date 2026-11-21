#include "optical_ref.h"
#include "raw_capture.h"
#include "rpm_edge_gate.h"
#include "ref_pulse_guard.h"
#include "board_config.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static optical_ref_status_t s_status;
static QueueHandle_t s_reports;
static QueueHandle_t s_analysis_reports;
static bool s_started;

#if CONFIG_V0_OPTICAL_ENABLED
static QueueHandle_t s_edges;
typedef struct {
    uint64_t timestamp_us;
    bool timestamp_resynced;
    bool restart;
} edge_record_t;
/* Timer-task-owned: rejected edges never extend the exclusion interval. */
static bool s_have_queued_edge;
static uint64_t s_last_queued_us;
static ref_pulse_guard_t s_guard;
static esp_timer_handle_t s_poll_timer;
static bool s_poll_paused,s_poll_running;

static void poll_level(void *arg)
{
    (void)arg;
    portENTER_CRITICAL(&s_lock);
    if(s_poll_paused){portEXIT_CRITICAL(&s_lock);return;}
    s_poll_running=true;
    portEXIT_CRITICAL(&s_lock);
    const uint64_t observed_us = (uint64_t)esp_timer_get_time();
    uint64_t now = 0;
    const bool level = gpio_get_level(BOARD_OPTICAL_GPIO) != 0;
    const bool changed = level != s_guard.level;
    const uint32_t previous_clock=s_guard.clock_faults,previous_gaps=s_guard.sample_gaps;
    const ref_guard_result_t qualified = ref_pulse_guard_sample(
        &s_guard, level, observed_us, &now);
    portENTER_CRITICAL(&s_lock);
    if (changed && s_status.raw_edges < UINT32_MAX) ++s_status.raw_edges;
    s_status.pulse_rejected = s_guard.rejected_pulses;
    s_status.level_resyncs = s_guard.level_resyncs;
    s_status.clock_faults = s_guard.clock_faults;
    s_status.resynced_pulses = s_guard.resynced_pulses;
    s_status.sample_gaps = s_guard.sample_gaps;
    s_status.max_sample_gap_us = s_guard.max_sample_gap_us;
    portEXIT_CRITICAL(&s_lock);
    if(previous_clock!=s_guard.clock_faults || previous_gaps!=s_guard.sample_gaps)
        raw_capture_signal_fault(RAW_FAULT_REFERENCE);
    if (qualified != REF_GUARD_PULSE) goto done;
    const bool too_close = rpm_edge_too_close(s_have_queued_edge, s_last_queued_us,
                                             now, CONFIG_V0_MIN_PULSE_INTERVAL_US);
    portENTER_CRITICAL(&s_lock);
    if (too_close && s_status.glitch_rejected < UINT32_MAX) ++s_status.glitch_rejected;
    portEXIT_CRITICAL(&s_lock);
    if (too_close) goto done;
    const edge_record_t edge = {.timestamp_us = now,
        .timestamp_resynced = s_guard.last_pulse_resynced};
    if (xQueueSend(s_edges, &edge, 0) != pdTRUE) {
        portENTER_CRITICAL(&s_lock);
        if (s_status.input_queue_drops < UINT32_MAX) ++s_status.input_queue_drops;
        portEXIT_CRITICAL(&s_lock);
        raw_capture_signal_fault(RAW_FAULT_REFERENCE);
    } else {
        s_last_queued_us = now;
        s_have_queued_edge = true;
    }
done:
    portENTER_CRITICAL(&s_lock);s_poll_running=false;portEXIT_CRITICAL(&s_lock);
}

static void reference_task(void *arg)
{
    (void)arg;
    rpm_core_t core;
    const rpm_core_config_t cfg = {
        .pulses_per_revolution = CONFIG_V0_PULSES_PER_REV,
        .min_interval_us = CONFIG_V0_MIN_PULSE_INTERVAL_US,
        .stop_timeout_us = (uint64_t)CONFIG_V0_STOP_TIMEOUT_MS * 1000,
    };
    if (rpm_core_init(&core, &cfg) != RPM_CORE_OK) {
        portENTER_CRITICAL(&s_lock);
        s_status.enabled = false;
        portEXIT_CRITICAL(&s_lock);
        vTaskDelete(NULL);
        return;
    }
    while (true) {
        edge_record_t edge;
        if (xQueueReceive(s_edges, &edge, portMAX_DELAY) != pdTRUE) continue;
        if(edge.restart){rpm_core_snapshot_t snapshot;(void)rpm_core_break_reference(&core);
            (void)rpm_core_get_snapshot(&core,edge.timestamp_us,&snapshot);
            portENTER_CRITICAL(&s_lock);s_status.rpm=snapshot;portEXIT_CRITICAL(&s_lock);continue;}
        const uint64_t timestamp = edge.timestamp_us;
        const rpm_core_result_t result = rpm_core_on_pulse(&core, timestamp);
        rpm_core_snapshot_t snapshot;
        /* Snapshot at the processed observation, not wall-clock now. */
        (void)rpm_core_get_snapshot(&core, timestamp, &snapshot);
        portENTER_CRITICAL(&s_lock);
        s_status.rpm = snapshot;
        portEXIT_CRITICAL(&s_lock);
        if (result == RPM_CORE_REFERENCE || result == RPM_CORE_PERIOD) {
            const optical_ref_event_t report = {
                .timestamp_us = timestamp, .accepted_sequence = snapshot.accepted_edges,
                .has_period = result == RPM_CORE_PERIOD,
                .timestamp_resynced = edge.timestamp_resynced,
            };
            const raw_reference_t raw={.sequence=report.accepted_sequence,
                .reference_us=report.timestamp_us,.has_period=report.has_period,
                .timestamp_resynced=report.timestamp_resynced};
            raw_capture_publish_reference(&raw);
            if (xQueueSend(s_reports, &report, 0) != pdTRUE) {
                portENTER_CRITICAL(&s_lock);
                if (s_status.report_queue_drops < UINT32_MAX) ++s_status.report_queue_drops;
                portEXIT_CRITICAL(&s_lock);
            }
#if CONFIG_LIVE_ANALYSIS_ENABLED
            if (s_analysis_reports && xQueueSend(s_analysis_reports, &report, 0) != pdTRUE) {
                portENTER_CRITICAL(&s_lock);
                if (s_status.analysis_queue_drops < UINT32_MAX) ++s_status.analysis_queue_drops;
                portEXIT_CRITICAL(&s_lock);
            }
#endif
        }
    }
}
#endif

esp_err_t optical_ref_start(void)
{
    if (s_started) return ESP_ERR_INVALID_STATE;
#if CONFIG_V0_OPTICAL_ENABLED
    if (CONFIG_V0_PULSES_PER_REV < 1 || CONFIG_V0_MIN_PULSE_INTERVAL_US < 1 ||
        CONFIG_V0_OPTICAL_POLL_US >= CONFIG_V0_REF_STABLE_US ||
        (uint64_t)CONFIG_V0_STOP_TIMEOUT_MS * 1000 <= CONFIG_V0_MIN_PULSE_INTERVAL_US)
        return ESP_ERR_INVALID_ARG;
    s_edges = xQueueCreate(128, sizeof(edge_record_t));
    s_reports = xQueueCreate(64, sizeof(optical_ref_event_t));
#if CONFIG_LIVE_ANALYSIS_ENABLED
    s_analysis_reports = xQueueCreate(64, sizeof(optical_ref_event_t));
    if (!s_analysis_reports) s_status.analysis_queue_drops = 1;
#endif
    if (!s_edges || !s_reports) {
        if (s_edges) vQueueDelete(s_edges);
        if (s_reports) vQueueDelete(s_reports);
        if (s_analysis_reports) vQueueDelete(s_analysis_reports);
        s_analysis_reports = NULL;
        s_edges = s_reports = NULL;
        return ESP_ERR_NO_MEM;
    }
    const gpio_config_t input = {
        .pin_bit_mask = 1ULL << BOARD_OPTICAL_GPIO, .mode = GPIO_MODE_INPUT,
        /* Do not load an open-collector comparator output with a pulldown. */
        .pull_up_en = GPIO_PULLUP_ENABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&input);
    if (err != ESP_OK) goto fail;
    ref_pulse_guard_init(&s_guard,
#if CONFIG_V0_OPTICAL_RISING
                         true,
#else
                         false,
#endif
                         gpio_get_level(BOARD_OPTICAL_GPIO) != 0,
                         (uint64_t)esp_timer_get_time(), CONFIG_V0_REF_STABLE_US);
    TaskHandle_t worker = NULL;
    if (xTaskCreate(reference_task, "optical_ref", 3072, NULL, 11, &worker) != pdPASS) {
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    const esp_timer_create_args_t timer_cfg = {
        .callback = poll_level, .dispatch_method = ESP_TIMER_TASK,
        .name = "optical_poll", .skip_unhandled_events = true,
    };
    err = esp_timer_create(&timer_cfg, &s_poll_timer);
    if (err == ESP_OK) err = esp_timer_start_periodic(s_poll_timer, CONFIG_V0_OPTICAL_POLL_US);
    if (err != ESP_OK) {
        vTaskDelete(worker);
        goto fail;
    }
    portENTER_CRITICAL(&s_lock);
    s_status.enabled = true;
    portEXIT_CRITICAL(&s_lock);
    s_started = true;
    return ESP_OK;
fail:
    if (s_poll_timer) {
        (void)esp_timer_stop(s_poll_timer);
        (void)esp_timer_delete(s_poll_timer);
        s_poll_timer = NULL;
    }
    gpio_intr_disable(BOARD_OPTICAL_GPIO);
    vQueueDelete(s_edges);
    vQueueDelete(s_reports);
    if (s_analysis_reports) vQueueDelete(s_analysis_reports);
    s_analysis_reports = NULL;
    s_edges = s_reports = NULL;
    return err;
#else
    s_started = true;
    return ESP_OK;
#endif
}

void optical_ref_get_status(optical_ref_status_t *status)
{
    if (!status) return;
    portENTER_CRITICAL(&s_lock);
    *status = s_status;
    portEXIT_CRITICAL(&s_lock);
    status->input_level = status->enabled ? gpio_get_level(BOARD_OPTICAL_GPIO) : -1;
    const uint64_t now = (uint64_t)esp_timer_get_time();
    if (status->rpm.has_accepted_edge &&
        now - status->rpm.last_accepted_edge_us >= (uint64_t)CONFIG_V0_STOP_TIMEOUT_MS * 1000) {
        status->timed_out = true;
        status->rpm.has_reference = false;
        status->rpm.rpm_valid = false;
    }
    /* Input loss is sticky until reboot: missing a reference can produce a
     * plausible but false subharmonic RPM. Never silently reuse such a value. */
    if (status->input_queue_drops || status->clock_faults || status->sample_gaps)
        status->rpm.rpm_valid = false;
    if (!status->rpm.rpm_valid) {
        status->rpm.rpm = 0;
        status->rpm.period_us = 0;
    }
}

bool optical_ref_pop_event(optical_ref_event_t *event)
{
    return event && s_reports && xQueueReceive(s_reports, event, 0) == pdTRUE;
}

bool optical_ref_pop_analysis_event(optical_ref_event_t *event)
{
    return event && s_analysis_reports && xQueueReceive(s_analysis_reports, event, 0) == pdTRUE;
}
esp_err_t optical_ref_maintenance_begin(void)
{
#if CONFIG_V0_OPTICAL_ENABLED
    if(!s_started)return ESP_OK;
    portENTER_CRITICAL(&s_lock);
    if(s_poll_paused){portEXIT_CRITICAL(&s_lock);return ESP_ERR_INVALID_STATE;}
    s_poll_paused=true;portEXIT_CRITICAL(&s_lock);
    esp_err_t err=esp_timer_stop(s_poll_timer);
    if(err!=ESP_OK){portENTER_CRITICAL(&s_lock);s_poll_paused=false;portEXIT_CRITICAL(&s_lock);return err;}
    for(;;){portENTER_CRITICAL(&s_lock);bool running=s_poll_running;portEXIT_CRITICAL(&s_lock);if(!running)break;vTaskDelay(1);}
#endif
    return ESP_OK;
}
esp_err_t optical_ref_maintenance_end(void)
{
#if CONFIG_V0_OPTICAL_ENABLED
    if(!s_started)return ESP_OK;
    const uint64_t now=(uint64_t)esp_timer_get_time();
    ref_pulse_guard_resume(&s_guard,gpio_get_level(BOARD_OPTICAL_GPIO)!=0,now);
    s_have_queued_edge=false;s_last_queued_us=0;
    edge_record_t reset={.timestamp_us=now,.restart=true};
    if(xQueueSend(s_edges,&reset,pdMS_TO_TICKS(20))!=pdTRUE){portENTER_CRITICAL(&s_lock);++s_status.input_queue_drops;portEXIT_CRITICAL(&s_lock);}
    portENTER_CRITICAL(&s_lock);
    s_status.clock_faults=s_guard.clock_faults;s_status.rpm.rpm_valid=false;s_status.rpm.has_reference=false;s_status.rpm.has_accepted_edge=false;
    s_status.rpm.rpm=0;s_status.rpm.period_us=0;s_poll_paused=false;
    portEXIT_CRITICAL(&s_lock);
    return esp_timer_start_periodic(s_poll_timer,CONFIG_V0_OPTICAL_POLL_US);
#else
    return ESP_OK;
#endif
}

