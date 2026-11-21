#include "acquisition.h"
#include "raw_capture.h"
#include "fifo_guard.h"
#include "pose_stream.h"
#include "esp_log.h"
#include "board_config.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <string.h>

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static acquisition_status_t s_status;
static QueueHandle_t s_samples;
static QueueHandle_t s_analysis_samples;
static bool s_started;
static bool s_storage_requested,s_storage_acknowledged;
static esp_err_t s_storage_result;

#if CONFIG_MPU_TIMING_TRACE_ENABLED
/* ISR writes internal RAM only. No PSRAM, queue, allocation or logging here.
 * The existing GPIO service is non-IRAM; entry latency remains uncalibrated. */
static uint64_t s_timing_events[2048];
static timing_trace_core_t s_timing;
#endif

#if CONFIG_V0_MPU_DRDY_ENABLED && !CONFIG_V0_DIAG_DISABLE_DRDY_ISR
static void drdy_isr(void *arg)
{
    (void)arg;
    const int64_t now = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&s_lock);
    ++s_status.observed_drdy;
    s_status.last_drdy_us = now;
#if CONFIG_MPU_TIMING_TRACE_ENABLED
    timing_trace_event(&s_timing,s_status.epoch,(uint64_t)now);
#endif
    portEXIT_CRITICAL_ISR(&s_lock);
}
#endif

static void offline(esp_err_t err, bool initializing)
{
    portENTER_CRITICAL(&s_lock);
    s_status.online = false;
    s_status.frame_consistent = false;
    s_status.last_error = err;
    if (initializing) ++s_status.init_errors;
    else ++s_status.i2c_errors;
    portEXIT_CRITICAL(&s_lock);
    raw_capture_signal_fault(RAW_FAULT_SENSOR);
}

/* Flash erases can suspend non-IRAM tasks longer than the FIFO budget. This
 * handshake is only permitted with the recorder exclusively idle. No sample
 * is generated across the intentional gap, and the sensor itself is not
 * reset/reconfigured: separate measured windows retain their sensor epoch. */
static bool storage_requested(void)
{portENTER_CRITICAL(&s_lock);bool v=s_storage_requested;portEXIT_CRITICAL(&s_lock);return v;}
static bool service_storage(mpu6050_t *sensor,bool *configured,frame_guard_t *guard,int64_t *last_nonempty)
{
    if(!storage_requested())return false;
    esp_err_t err=*configured?mpu6050_fifo_pause(sensor):ESP_OK;
    if(err!=ESP_OK){offline(err,false);*configured=false;}
    portENTER_CRITICAL(&s_lock);
    s_storage_result=err;s_storage_acknowledged=true;
    s_status.storage_paused=true;s_status.online=false;++s_status.storage_pauses;
    portEXIT_CRITICAL(&s_lock);
    while(storage_requested())vTaskDelay(1);
    if(*configured){
        err=mpu6050_fifo_reset(sensor);
        if(err!=ESP_OK){offline(err,false);*configured=false;}
    }
    /* Drop the statistical window crossing the unrecorded gap. A checked FIFO
     * reset preserves an already established direct/FIFO agreement; any real
     * I2C/reset failure above still takes the normal offline/restart path. */
    const bool agreement=*configured&&guard->consistent;
    frame_guard_init(guard);guard->consistent=agreement;
    *last_nonempty=esp_timer_get_time();
    portENTER_CRITICAL(&s_lock);
    s_storage_result=err;s_storage_acknowledged=false;s_status.storage_paused=false;
    portEXIT_CRITICAL(&s_lock);
    return true;
}

static void sampling_task(void *arg)
{
    (void)arg;
    mpu6050_t sensor = {0};
    esp_err_t err = mpu6050_create(&sensor, BOARD_MPU_SDA_GPIO,
        BOARD_MPU_SCL_GPIO, BOARD_MPU_ADDRESS, BOARD_I2C_SPEED_HZ);
    if (err != ESP_OK) {
        offline(err, true);
        vTaskDelete(NULL);
        return;
    }
    bool configured = false;
    frame_guard_t frame_guard;frame_guard_init(&frame_guard);
    int64_t last_nonempty_us = esp_timer_get_time();
    while (true) {
        if(service_storage(&sensor,&configured,&frame_guard,&last_nonempty_us))continue;
        if (!configured) {
            err = mpu6050_configure(&sensor, BOARD_MPU_SAMPLE_HZ);
            portENTER_CRITICAL(&s_lock);
            s_status.who_am_i = sensor.who_am_i;
            s_status.identity_reads = sensor.identity_reads;
            s_status.profile_name = sensor.profile_name;
            s_status.fifo_capacity_bytes = sensor.fifo_capacity_bytes;
            s_status.identity_matches_label = sensor.expected_identity;
            portEXIT_CRITICAL(&s_lock);
            if (err != ESP_OK) {
                offline(err, true);
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            portENTER_CRITICAL(&s_lock);
            ++s_status.epoch;
            ++s_status.restarts;
            s_status.who_am_i = sensor.who_am_i;
            s_status.online = false; /* only valid after fresh FIFO data */
            s_status.frame_consistent=false;
            s_status.last_error = ESP_OK;
            portEXIT_CRITICAL(&s_lock);
            configured = true;
            frame_guard_init(&frame_guard);
            last_nonempty_us = esp_timer_get_time();
        }
        uint16_t bytes = 0;
        uint8_t irq_status = 0;
        const int64_t count_begin_us = esp_timer_get_time();
        const uint32_t old_count_anomalies=sensor.fifo_count_anomalies;
        err = mpu6050_fifo_count(&sensor, &bytes, &irq_status);
        const int64_t count_end_us = esp_timer_get_time();
        portENTER_CRITICAL(&s_lock);
        s_status.fifo_count_confirm_recoveries=sensor.fifo_count_confirm_recoveries;
        portEXIT_CRITICAL(&s_lock);
        if (err != ESP_OK) {
            if(sensor.fifo_count_anomalies!=old_count_anomalies) {
                portENTER_CRITICAL(&s_lock);++s_status.fifo_count_faults;s_status.online=false;s_status.frame_consistent=false;s_status.last_error=err;portEXIT_CRITICAL(&s_lock);
                raw_capture_signal_fault(RAW_FAULT_FIFO|RAW_FAULT_ALIGNMENT);
            }else offline(err, false);
            configured = false; /* partial FIFO transactions invalidate alignment */
            continue;
        }
        /* Six-byte accel frames: 1020 B for MPU6050, 510 B for MPU6500.
         * Reaching a near-full threshold is an intentional loss reset,
         * not by itself evidence of a hardware overflow interrupt. */
        const uint16_t near_full_bytes = sensor.fifo_capacity_bytes>192 ? (sensor.fifo_capacity_bytes-192)/6*6 : 0;
        if (near_full_bytes < 6 || (irq_status & 0x10) || bytes >= near_full_bytes) {
            portENTER_CRITICAL(&s_lock);
            if (irq_status & 0x10) ++s_status.fifo_overflows;
            ++s_status.fifo_loss_resets;
            s_status.online = false;
            s_status.frame_consistent = false;
            s_status.last_error = ESP_ERR_INVALID_SIZE;
            portEXIT_CRITICAL(&s_lock);
            raw_capture_signal_fault(RAW_FAULT_FIFO);
            configured = false; /* actual number of lost samples is unknown */
            continue;
        }
        size_t frames = bytes / 6;
        if (frames == 0) {
            if (esp_timer_get_time() - last_nonempty_us > 500000) {
                portENTER_CRITICAL(&s_lock);
                s_status.online = false;
                s_status.last_error = ESP_ERR_TIMEOUT;
                portEXIT_CRITICAL(&s_lock);
                raw_capture_signal_fault(RAW_FAULT_SENSOR);
                configured = false;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        if (frames > 32) frames = 32;
        mpu6050_raw_t batch[32];
        const uint32_t old_read_overflows=sensor.fifo_read_overflows;
        err = mpu6050_fifo_read(&sensor, batch, frames);
        if (err != ESP_OK) {
            if(sensor.fifo_read_overflows!=old_read_overflows) {
                portENTER_CRITICAL(&s_lock);
                ++s_status.fifo_overflows;++s_status.fifo_loss_resets;
                s_status.online=false;s_status.frame_consistent=false;s_status.last_error=err;
                portEXIT_CRITICAL(&s_lock);
                raw_capture_signal_fault(RAW_FAULT_FIFO|RAW_FAULT_ALIGNMENT);
            } else offline(err, false);
            configured = false;
            continue;
        }
        const int64_t readout_us = esp_timer_get_time();
        mpu6050_raw_t direct_raw;
#if CONFIG_MPU_POSE_DIAG_ENABLED
        motion_snapshot_t motion;
        const int64_t motion_begin_us=esp_timer_get_time();
        err=mpu6050_read_motion_registers(&sensor,&motion);
        direct_raw=(mpu6050_raw_t){motion.accel[0],motion.accel[1],motion.accel[2]};
#else
        err = mpu6050_read_accel_registers(&sensor, &direct_raw);
#endif
        if (err != ESP_OK) {
            offline(err, false);
            configured = false;
            continue; /* a diagnostic transfer failed; discard the whole FIFO batch */
        }
        const int64_t direct_readout_us = esp_timer_get_time();
        last_nonempty_us = readout_us;
        bool alignment_reset=false;
        for (size_t i = 0; i < frames; ++i) {
            const int16_t f[3]={batch[i].x,batch[i].y,batch[i].z},d[3]={direct_raw.x,direct_raw.y,direct_raw.z};
            const frame_check_t check=frame_guard_feed(&frame_guard,f,d,(uint64_t)direct_readout_us,bytes/6);
            if(check==FRAME_MISMATCH || check==FRAME_TIME_ERROR) {
                raw_capture_signal_fault(RAW_FAULT_ALIGNMENT);
                portENTER_CRITICAL(&s_lock);s_status.online=false;s_status.frame_consistent=false;s_status.frame_difference_lsb=(float)frame_guard.difference_lsb;portEXIT_CRITICAL(&s_lock);
                if(check==FRAME_TIME_ERROR || frame_guard.bad_windows>=2) {
                    portENTER_CRITICAL(&s_lock);++s_status.frame_mismatches;s_status.last_error=ESP_ERR_INVALID_RESPONSE;portEXIT_CRITICAL(&s_lock);
                    ESP_LOGE("imu_guard","fifo_direct_mismatch diff_lsb=%.1f; discard, restart epoch",frame_guard.difference_lsb);
                    configured=false;alignment_reset=true;break;
                }
            }
            acceleration_sample_t sample = {.readout_us = readout_us, .raw = batch[i],
                .fifo_count_begin_us = count_begin_us, .fifo_count_end_us = count_end_us,
                .fifo_pending_frames = bytes / 6, .batch_index = (uint16_t)i};
            portENTER_CRITICAL(&s_lock);
            sample.sequence = ++s_status.total_samples;
            sample.epoch = s_status.epoch;
            s_status.latest = sample;
            s_status.last_data_us = readout_us;
            s_status.direct_raw = direct_raw;
            s_status.direct_readout_us = direct_readout_us;
            s_status.frame_consistent=frame_guard.consistent;
            s_status.frame_difference_lsb=(float)frame_guard.difference_lsb;
            s_status.online = frame_guard.consistent;
            s_status.last_error = ESP_OK;
            portEXIT_CRITICAL(&s_lock);
            const raw_sample_t raw={.sequence=sample.sequence,.epoch=sample.epoch,
                .readout_us=sample.readout_us,.count_begin_us=sample.fifo_count_begin_us,
                .count_end_us=sample.fifo_count_end_us,.pending_frames=sample.fifo_pending_frames,
                .batch_index=sample.batch_index,.raw={sample.raw.x,sample.raw.y,sample.raw.z}};
            raw_capture_publish_sample(&raw);
#if CONFIG_LIVE_ANALYSIS_ENABLED
            if (s_analysis_samples && xQueueSend(s_analysis_samples, &sample, 0) != pdTRUE) {
                portENTER_CRITICAL(&s_lock);
                ++s_status.analysis_drops;
                portEXIT_CRITICAL(&s_lock);
            }
#endif
#if CONFIG_V0_STREAM_ALL_SAMPLES
            if (xQueueSend(s_samples, &sample, 0) != pdTRUE) {
                portENTER_CRITICAL(&s_lock);
                ++s_status.stream_drops;
                portEXIT_CRITICAL(&s_lock);
            }
#endif
        }
        if(alignment_reset)continue;
#if CONFIG_MPU_POSE_DIAG_ENABLED
        pose_observation_t pose={.read_begin_us=(uint64_t)motion_begin_us,
            .read_end_us=(uint64_t)direct_readout_us,.temperature=motion.temperature,
            .frame_consistent=frame_guard.consistent};
        for(unsigned axis=0;axis<3;++axis){pose.accel[axis]=motion.accel[axis];pose.gyro[axis]=motion.gyro[axis];}
        portENTER_CRITICAL(&s_lock);pose.fifo_sequence=s_status.total_samples;pose.epoch=s_status.epoch;portEXIT_CRITICAL(&s_lock);
        pose_stream_publish(&pose);
        raw_pose_t saved_pose={.fifo_sequence=pose.fifo_sequence,.epoch=pose.epoch,
            .read_begin_us=pose.read_begin_us,.read_end_us=pose.read_end_us,
            .temperature=pose.temperature,.frame_consistent=pose.frame_consistent};
        memcpy(saved_pose.accel,pose.accel,sizeof(saved_pose.accel));
        memcpy(saved_pose.gyro,pose.gyro,sizeof(saved_pose.gyro));
        raw_capture_publish_pose(&saved_pose);
#endif
        /* Catch up when a batch remains; otherwise let ~5 frames accumulate. */
        vTaskDelay(pdMS_TO_TICKS(bytes / 6 > 32 ? 1 : 10));
    }
}

esp_err_t acquisition_maintenance_begin(void)
{
    if(!s_started)return ESP_ERR_INVALID_STATE;
    raw_status_t raw;raw_capture_get_status(&raw);
    if(raw.state==RAW_RECORDING||raw.state==RAW_CLOSING)return ESP_ERR_INVALID_STATE;
    portENTER_CRITICAL(&s_lock);
    if(s_storage_requested||s_storage_acknowledged){portEXIT_CRITICAL(&s_lock);return ESP_ERR_INVALID_STATE;}
    s_storage_result=ESP_ERR_INVALID_STATE;s_storage_requested=true;
    portEXIT_CRITICAL(&s_lock);
    const int64_t deadline=esp_timer_get_time()+1000000;
    while(esp_timer_get_time()<deadline){
        portENTER_CRITICAL(&s_lock);bool ack=s_storage_acknowledged;esp_err_t err=s_storage_result;portEXIT_CRITICAL(&s_lock);
        if(ack){if(err!=ESP_OK)(void)acquisition_maintenance_end();return err;}
        vTaskDelay(1);
    }
    (void)acquisition_maintenance_end();return ESP_ERR_TIMEOUT;
}
esp_err_t acquisition_maintenance_end(void)
{
    portENTER_CRITICAL(&s_lock);s_storage_requested=false;portEXIT_CRITICAL(&s_lock);
    const int64_t deadline=esp_timer_get_time()+1000000;
    while(esp_timer_get_time()<deadline){
        portENTER_CRITICAL(&s_lock);bool ack=s_storage_acknowledged;esp_err_t err=s_storage_result;bool fresh=s_status.online;portEXIT_CRITICAL(&s_lock);
        if(!ack&&(fresh||err!=ESP_OK))return err;
        vTaskDelay(1);
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t acquisition_start(void)
{
    if (s_started) return ESP_ERR_INVALID_STATE;
#if CONFIG_MPU_TIMING_TRACE_ENABLED
    timing_trace_init(&s_timing,s_timing_events,sizeof(s_timing_events)/sizeof(s_timing_events[0]));
#endif
#if CONFIG_V0_MPU_DRDY_ENABLED
    const gpio_config_t irq = {
        .pin_bit_mask = 1ULL << BOARD_MPU_INT_GPIO, .mode = GPIO_MODE_INPUT,
#if CONFIG_V0_MPU_INT_OPEN_DRAIN
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
#else
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
#endif
#if CONFIG_V0_DIAG_DISABLE_DRDY_ISR
        .intr_type = GPIO_INTR_DISABLE,
#else
#if CONFIG_V0_MPU_INT_OPEN_DRAIN
        .intr_type = GPIO_INTR_NEGEDGE,
#else
        .intr_type = GPIO_INTR_POSEDGE,
#endif
#endif
    };
    esp_err_t err = gpio_config(&irq);
    if (err != ESP_OK) return err;
#endif
#if !CONFIG_V0_DIAG_DISABLE_DRDY_ISR
#if CONFIG_V0_MPU_DRDY_ENABLED
    err = gpio_isr_handler_add(BOARD_MPU_INT_GPIO, drdy_isr, NULL);
    if (err != ESP_OK) return err;
#endif
#endif
#if CONFIG_V0_STREAM_ALL_SAMPLES
    s_samples = xQueueCreate(512, sizeof(acceleration_sample_t));
    if (!s_samples) {
#if CONFIG_V0_MPU_DRDY_ENABLED && !CONFIG_V0_DIAG_DISABLE_DRDY_ISR
        gpio_isr_handler_remove(BOARD_MPU_INT_GPIO);
#endif
        return ESP_ERR_NO_MEM;
    }
#endif
#if CONFIG_LIVE_ANALYSIS_ENABLED
    s_analysis_samples = xQueueCreate(256, sizeof(acceleration_sample_t));
    if (!s_analysis_samples) {
        /* Analysis exhaustion must never abort the primary sensor capture. */
        portENTER_CRITICAL(&s_lock);
        ++s_status.analysis_drops;
        portEXIT_CRITICAL(&s_lock);
    }
#endif
    /* Create the bus and perform all synchronous transfers on the same CPU
     * as its allocated peripheral ISR. See the V0 I2C affinity investigation. */
    if (xTaskCreatePinnedToCore(sampling_task, "imu_fifo", 4096, NULL, 10, NULL, 0) != pdPASS) {
        if (s_samples) vQueueDelete(s_samples);
        s_samples = NULL;
        if (s_analysis_samples) vQueueDelete(s_analysis_samples);
        s_analysis_samples = NULL;
#if CONFIG_V0_MPU_DRDY_ENABLED && !CONFIG_V0_DIAG_DISABLE_DRDY_ISR
        gpio_isr_handler_remove(BOARD_MPU_INT_GPIO);
#endif
        return ESP_ERR_NO_MEM;
    }
    s_started = true;
    return ESP_OK;
}

esp_err_t acquisition_timing_begin(uint32_t ms)
{
#if CONFIG_MPU_TIMING_TRACE_ENABLED
    const uint64_t now=(uint64_t)esp_timer_get_time();
    portENTER_CRITICAL(&s_lock);
    const bool ok=s_started && s_status.online && timing_trace_begin(&s_timing,s_status.epoch,now,ms);
    portEXIT_CRITICAL(&s_lock);return ok?ESP_OK:ESP_ERR_INVALID_STATE;
#else
    (void)ms;return ESP_ERR_NOT_SUPPORTED;
#endif
}
void acquisition_timing_status(timing_trace_status_t *out)
{
    if(!out)return;
#if CONFIG_MPU_TIMING_TRACE_ENABLED
    const uint64_t now=(uint64_t)esp_timer_get_time();
    portENTER_CRITICAL(&s_lock);timing_trace_tick(&s_timing,now);*out=s_timing.status;portEXIT_CRITICAL(&s_lock);
#else
    *out=(timing_trace_status_t){0};
#endif
}
esp_err_t acquisition_timing_lease(uint32_t id,timing_trace_status_t *out)
{
    if(!out)return ESP_ERR_INVALID_ARG;
#if CONFIG_MPU_TIMING_TRACE_ENABLED
    portENTER_CRITICAL(&s_lock);const bool ok=timing_trace_lease(&s_timing,id);
    if(ok)*out=s_timing.status;
    portEXIT_CRITICAL(&s_lock);return ok?ESP_OK:ESP_ERR_INVALID_STATE;
#else
    (void)id;return ESP_ERR_NOT_SUPPORTED;
#endif
}
void acquisition_timing_release(uint32_t id)
{
#if CONFIG_MPU_TIMING_TRACE_ENABLED
    portENTER_CRITICAL(&s_lock);timing_trace_release(&s_timing,id);portEXIT_CRITICAL(&s_lock);
#else
    (void)id;
#endif
}
size_t acquisition_timing_read(uint32_t id,size_t offset,uint64_t *out,size_t capacity)
{
#if CONFIG_MPU_TIMING_TRACE_ENABLED
    if(capacity>32)capacity=32;
    portENTER_CRITICAL(&s_lock);const size_t n=timing_trace_read(&s_timing,id,offset,out,capacity);portEXIT_CRITICAL(&s_lock);return n;
#else
    (void)id;(void)offset;(void)out;(void)capacity;return 0;
#endif
}

void acquisition_get_status(acquisition_status_t *status)
{
    if (!status) return;
    portENTER_CRITICAL(&s_lock);
    *status = s_status;
    portEXIT_CRITICAL(&s_lock);
    if (status->online && esp_timer_get_time() - status->last_data_us > 500000) {
        status->online = false;
        status->last_error = ESP_ERR_TIMEOUT;
    }
}

bool acquisition_pop_sample(acceleration_sample_t *sample)
{
    return sample && s_samples && xQueueReceive(s_samples, sample, 0) == pdTRUE;
}

bool acquisition_pop_analysis_sample(acceleration_sample_t *sample)
{
    return sample && s_analysis_samples && xQueueReceive(s_analysis_samples, sample, 0) == pdTRUE;
}
