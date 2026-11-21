#include <inttypes.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "board_config.h"
#include "acquisition.h"
#include "raw_capture.h"
#include "pose_stream.h"
#include "raw_console.h"
#include "optical_ref.h"
#include "link_probe.h"
#include "fifo_probe.h"
#include "frame_probe.h"
#include "serial_console.h"
#include "diagnostic_web.h"
#include "live_analysis.h"
#include "live_analysis_record.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_app_desc.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_V0_MPU_DRDY_ENABLED
#define CAPTURE_MODE "DRDY"
#else
#define CAPTURE_MODE "FIFO_POLL"
#endif

static const char *ref_state(const optical_ref_status_t *r)
{
    if (!r->enabled) return "DISABLED";
    if (r->input_queue_drops) return "INPUT_LOSS_REBOOT_REQUIRED";
    if (r->clock_faults) return "CLOCK_FAULT_REBOOT_REQUIRED";
    if (r->sample_gaps) return "SAMPLE_GAP_REBOOT_REQUIRED";
    if (r->timed_out) return "NO_REFERENCE_TIMEOUT";
    if (!r->rpm.has_accepted_edge) return "NO_REFERENCE";
    if (!r->rpm.rpm_valid) return "WAIT_SECOND_EDGE";
    return "PERIOD_OK";
}

static void print_sample(const acceleration_sample_t *s)
{
    const float scale = BOARD_GRAVITY_M_S2 / BOARD_MPU_ACCEL_LSB_G;
    serial_console_printf("ACC,%" PRIu64 ",%" PRIu32 ",%" PRId64 ",%d,%d,%d,%.6f,%.6f,%.6f\n",
        s->sequence, s->epoch, s->readout_us, s->raw.x, s->raw.y, s->raw.z,
        (double)(s->raw.x * scale), (double)(s->raw.y * scale), (double)(s->raw.z * scale));
}

void app_main(void)
{
    /* USB Serial/JTAG uses GPIO19/20 internally; no UART adapter needed. */
    setvbuf(stdout, NULL, _IOLBF, 0);
    ESP_ERROR_CHECK(serial_console_start());
    ESP_LOGI("v0", "wheel_balance_v0 %s; IDF=%s; reset=%d",
             esp_app_get_description()->version, esp_get_idf_version(),
             (int)esp_reset_reason());
#if CONFIG_V0_FRAME_PROBE_ONLY
    frame_probe_run();
    return;
#endif
#if CONFIG_V0_FIFO_PROBE_ONLY
    fifo_probe_run();
    return;
#endif
#if CONFIG_V0_LINK_PROBE_ONLY
    link_probe_run();
    return;
#endif
    ESP_LOGI("v0", "XIAO ESP32S3 Sense; SDA=D4/GPIO5 SCL=D5/GPIO6 INT=D1/GPIO2");
#if !CONFIG_V0_MPU_DRDY_ENABLED
    ESP_LOGI("v0", "V0 capture=FIFO_POLL; MPU INT unused; timestamps=host batch readout");
#endif
#if CONFIG_V0_MPU_INT_OPEN_DRAIN
    ESP_LOGI("v0", "MPU INT: active-low open-drain; GPIO2 internal pull-up, falling edge");
#endif
#if CONFIG_V0_DIAG_DISABLE_DRDY_ISR
    ESP_LOGW("v0", "DIAGNOSTIC: GPIO2 DRDY ISR disabled; sensor INT output and FIFO remain enabled");
#endif
    ESP_LOGI("v0", "IMU addr=0x68 (AD0=GND), I2C=%dHz, target=500Hz +/-4g; accel-only FIFO",
             BOARD_I2C_SPEED_HZ);
    ESP_LOGW("v0", "WHO=0x70 selects an MPU6500 candidate register profile; label=6050 remains unverified");
    ESP_LOGW("v0", "No balance mass/angle or calibrated phase/precision claim");
#if CONFIG_DIAGNOSTIC_WEB_ENABLED
    ESP_LOGI("v0", "USB records: asynchronous whole-record queue; disconnected host cannot block acquisition/web");
#else
    ESP_LOGI("v0", "USB records: 8192B TX buffer; blocking enqueue, tx_err and queue losses counted");
#endif
#if CONFIG_V0_DIAG_DISABLE_SENSOR_INT
    ESP_LOGW("v0", "DIAGNOSTIC: sensor INT output disabled; FIFO transport only, no DRDY timing qualification");
#endif
    ESP_LOGW("v0", "ACC readout_us is FIFO read completion, NOT sample time; gravity included");
    ESP_LOGI("v0", "CSV: ACC,seq,epoch,readout_us,raw_x,raw_y,raw_z,ax_m_s2,ay_m_s2,az_m_s2");
    ESP_LOGI("v0", "CSV: REF,accepted_seq,reference_observed_us,has_period,timestamp_resynced");
#if CONFIG_V0_OPTICAL_ENABLED
    ESP_LOGI("v0", "REF mode=GPIO_POLL; poll_us=%d stable_us=%d; raw_edges=observed transitions; all timestamps unqualified for phase",
             CONFIG_V0_OPTICAL_POLL_US, CONFIG_V0_REF_STABLE_US);
#else
    ESP_LOGI("v0", "REF disabled");
#endif
    ESP_LOGW("v0", "Diagnostic: FIFO reads use 6B transactions; direct register comparison is NOT synchronized");
    ESP_LOGI("v0", "CSV: DIAG,seq,epoch,fifo_readout_us,fifo_x,fifo_y,fifo_z,direct_readout_us,direct_x,direct_y,direct_z");
#if CONFIG_V0_STREAM_ALL_SAMPLES
    ESP_LOGI("v0", "ACC stream=all acquired samples; bounded queue, drops counted");
#else
    ESP_LOGI("v0", "ACC stream=latest at 10Hz; acquisition=500Hz (display decimated)");
#endif
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    const esp_err_t raw_start=raw_capture_init();
    if(raw_start!=ESP_OK && raw_start!=ESP_ERR_NOT_SUPPORTED)
        ESP_LOGE("raw", "raw_capture_init=%s",esp_err_to_name(raw_start));
    /* Complete Wi-Fi initialization/calibration before the optical sampler
     * starts; phone telemetry is independent of this serial display loop. */
    const esp_err_t web_start=diagnostic_web_start();
    if (web_start!=ESP_OK && web_start!=ESP_ERR_NOT_SUPPORTED)
        ESP_LOGE("web", "dashboard_start_failed=%s", esp_err_to_name(web_start));
#if CONFIG_MPU_POSE_DIAG_ENABLED
    const esp_err_t pose_start=pose_stream_start(diagnostic_web_boot_id());
    if(pose_start!=ESP_OK)ESP_LOGE("pose","pose_stream_start=%s",esp_err_to_name(pose_start));
#endif
    const esp_err_t analysis_start=live_analysis_start();
    if (analysis_start!=ESP_OK && analysis_start!=ESP_ERR_NOT_SUPPORTED)
        ESP_LOGE("analysis", "live_analysis_start_failed=%s", esp_err_to_name(analysis_start));
    ESP_ERROR_CHECK(optical_ref_start());
    ESP_ERROR_CHECK(acquisition_start());
#if CONFIG_DIAGNOSTIC_WEB_ENABLED
    if (web_start==ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        (void)diagnostic_web_run_selfcheck();
    }
#endif
    int64_t last_status_us = 0;
#if !CONFIG_V0_STREAM_ALL_SAMPLES
    int64_t last_acc_us = 0;
    uint64_t last_displayed_sequence = 0;
#endif
    int64_t last_rate_us = esp_timer_get_time();
    uint64_t last_rate_samples = 0;
    uint32_t last_rate_epoch = 0;
    float readout_hz = 0;
#if CONFIG_LIVE_ANALYSIS_ENABLED
    uint64_t last_recorded_analysis = 0;
    uint64_t last_recorded_coast = 0;
    /* Owned only by this task. Keep bounded diagnostic scratch outside the
     * main stack: startup HTTP self-checks run on the same task before this
     * loop, and the compiler reserves the whole app_main frame at entry. */
    static live_analysis_status_t analysis;
    static char record[768];
#endif
    while (true) {
        raw_console_poll();
        const int64_t now = esp_timer_get_time();
        acquisition_status_t imu;
        acquisition_get_status(&imu);
#if CONFIG_V0_STREAM_ALL_SAMPLES
        acceleration_sample_t sample;
        /* Bound each drain so status/REF cannot be starved by serial output. */
        for (int i = 0; i < 64 && acquisition_pop_sample(&sample); ++i) print_sample(&sample);
#else
        if (imu.online && imu.latest.sequence != last_displayed_sequence && now - last_acc_us >= 100000) {
            print_sample(&imu.latest);
            last_displayed_sequence = imu.latest.sequence;
            last_acc_us = now;
        }
#endif
        optical_ref_event_t event;
        for (int i = 0; i < 64 && optical_ref_pop_event(&event); ++i) {
            serial_console_printf("REF,%" PRIu64 ",%" PRIu64 ",%d,%d\n", event.accepted_sequence,
                   event.timestamp_us, event.has_period, event.timestamp_resynced);
        }
        if (now - last_status_us >= 1000000) {
            optical_ref_status_t ref;
            optical_ref_get_status(&ref);
            if (imu.online && imu.epoch == last_rate_epoch) {
                readout_hz = (float)((double)(imu.total_samples - last_rate_samples) *
                                        1000000.0 / (double)(now - last_rate_us));
            } else readout_hz = 0;
            last_rate_us = now;
            last_rate_samples = imu.total_samples;
            last_rate_epoch = imu.epoch;
            last_status_us = now;
            char rpm_text[32] = "NA";
            if (ref.rpm.rpm_valid) snprintf(rpm_text, sizeof(rpm_text), "%.3f", (double)ref.rpm.rpm);
            serial_console_printf("RPM=%s mode=%s uptime_us=%" PRId64 " ref=%s edges=%" PRIu64 " rejected=%" PRIu64 " ref_q_drop=%" PRIu32
                   " ref_log_drop=%" PRIu32 " ref_raw_edges=%" PRIu32 " ref_glitch_rejected=%" PRIu32
                   " ref_pulse_rejected=%" PRIu32 " ref_level_resync=%" PRIu32 " ref_clock_faults=%" PRIu32
                   " ref_resynced_pulses=%" PRIu32
                   " ref_sample_gaps=%" PRIu32 " ref_max_sample_gap_us=%" PRIu64
                   " ref_level=%d imu=%s who=0x%02x profile=%s id_reads=%u"
                   " identity_matches_label=%d fifo_capacity=%u samples=%" PRIu64
                   " fifo_readout_hz=%.1f irq_seen=%" PRIu32 " irq_ok=%d init_err=%" PRIu32 " i2c_err=%" PRIu32
                   " fifo_overflow=%" PRIu32 " fifo_loss_resets=%" PRIu32
                   " acc_log_drop=%" PRIu32 " tx_err=%" PRIu32 " epoch=%" PRIu32
                   " err=%s phase_ready=0\n",
                   rpm_text, CAPTURE_MODE,
                   now, ref_state(&ref), ref.rpm.accepted_edges, ref.rpm.rejected_edges,
                   ref.input_queue_drops, ref.report_queue_drops, ref.raw_edges, ref.glitch_rejected,
                   ref.pulse_rejected, ref.level_resyncs, ref.clock_faults,
                   ref.resynced_pulses,
                   ref.sample_gaps, ref.max_sample_gap_us,
                   ref.input_level, imu.online ? "ONLINE" : "NO_FRESH_DATA",
                   imu.who_am_i, imu.profile_name ? imu.profile_name : "UNIDENTIFIED",
                   (unsigned)imu.identity_reads, imu.identity_matches_label,
                   (unsigned)imu.fifo_capacity_bytes, imu.total_samples, (double)readout_hz, imu.observed_drdy,
                   imu.observed_drdy > 0 && now - imu.last_drdy_us < 500000,
                   imu.init_errors, imu.i2c_errors, imu.fifo_overflows, imu.fifo_loss_resets, imu.stream_drops,
                   serial_console_errors(), imu.epoch,
                   esp_err_to_name(imu.last_error));
            if (imu.online) {
                serial_console_printf("DIAG,%" PRIu64 ",%" PRIu32 ",%" PRId64 ",%d,%d,%d,%" PRId64 ",%d,%d,%d\n",
                    imu.latest.sequence, imu.latest.epoch, imu.latest.readout_us,
                    imu.latest.raw.x, imu.latest.raw.y, imu.latest.raw.z, imu.direct_readout_us,
                    imu.direct_raw.x, imu.direct_raw.y, imu.direct_raw.z);
            }
            serial_console_printf("FRAME source=live_diagnostic consistent=%d difference_lsb=%.1f count_faults=%" PRIu32 " mismatches=%" PRIu32 " epoch=%" PRIu32 " count_confirm_recoveries=%" PRIu32 " qualified=0\n",
                imu.frame_consistent,(double)imu.frame_difference_lsb,imu.fifo_count_faults,imu.frame_mismatches,imu.epoch,imu.fifo_count_confirm_recoveries);
#if CONFIG_LIVE_ANALYSIS_ENABLED
            live_analysis_get_status(&analysis);
            serial_console_printf("ANALYSIS state=%s valid=%d seq=%" PRIu64
                " turns=%u buffered_samples=%u sample_drops=%" PRIu32
                " ref_drops=%" PRIu32 " timing_resets=%" PRIu32
                " timing_error_us=%" PRId64 " stack_min_free_bytes=%u measurement_qualified=0\n",
                live_analysis_state_name(analysis.state), analysis.result.valid,
                analysis.result_sequence, (unsigned)analysis.result.turns,
                (unsigned)analysis.buffered_samples, analysis.sample_queue_drops,
                analysis.reference_queue_drops, analysis.timing_resets,
                analysis.last_time_error_us,(unsigned)uxTaskGetStackHighWaterMark(NULL));
            /* Decimated diagnostics only. The analysis task and its input
             * queues remain independent; no USB backpressure reaches them.
             * Use one snapshot for all axes and identify every window. */
            if (analysis.result_sequence != last_recorded_analysis) {
                bool recorded=false;
                for (unsigned axis=0; axis<3; ++axis) {
                    if (live_analysis_record(record,sizeof(record),&analysis,axis)) {
                        (void)serial_console_printf("%s",record);
                        recorded=true;
                    }
                }
                if (recorded) last_recorded_analysis=analysis.result_sequence;
            }
            serial_console_printf("COAST_STATE state=%s valid=%d seq=%" PRIu64
                " diagnostic_only=1 measurement_qualified=0\n",
                coast_status_name(analysis.coast.status),analysis.coast.valid,analysis.coast_sequence);
            if(analysis.coast_sequence!=last_recorded_coast) {
                bool recorded=false;
                for(unsigned axis=0;axis<3;++axis)if(live_coast_record(record,sizeof(record),&analysis,axis)) {
                    (void)serial_console_printf("%s",record);recorded=true;
                }
                if(recorded)last_recorded_coast=analysis.coast_sequence;
            }
#endif
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
