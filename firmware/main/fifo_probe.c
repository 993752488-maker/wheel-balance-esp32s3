#include "fifo_probe.h"
#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include "serial_console.h"
#include "board_config.h"
#include "mpu6050.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

enum { WINDOW_US = 10000000, CONTROL_READS = 5 };

typedef struct {
    const char *mode;
    bool fifo_enabled;
    bool complete;
    uint8_t who;
    uint32_t setup_attempts, setups_ok, setup_errors;
    uint32_t polls, batches, read_errors, discarded_batches;
    uint32_t overflow_flags, loss_resets, empty_polls;
    uint16_t max_count;
    uint64_t payload_reads, direct_reads;
    uint32_t delta_max;
    int16_t payload_low[3], payload_high[3], direct_low[3], direct_high[3];
    int64_t elapsed_us;
    esp_err_t last_error;
} fifo_probe_result_t;

static void update_range(const mpu6050_raw_t *s, int16_t low[3], int16_t high[3])
{
    const int16_t value[] = {s->x, s->y, s->z};
    for (int axis = 0; axis < 3; ++axis) {
        if (value[axis] < low[axis]) low[axis] = value[axis];
        if (value[axis] > high[axis]) high[axis] = value[axis];
    }
}

static uint32_t axis_delta(int16_t a, int16_t b)
{
    const int32_t difference = (int32_t)a - b;
    return (uint32_t)(difference < 0 ? -difference : difference);
}

static esp_err_t probe_write(mpu6050_t *sensor, uint8_t reg, uint8_t value)
{
    const uint8_t packet[] = {reg, value};
    return i2c_master_transmit(sensor->device, packet, sizeof(packet), 50);
}

static esp_err_t probe_read(mpu6050_t *sensor, uint8_t reg, uint8_t *value)
{
    return i2c_master_transmit_receive(sensor->device, &reg, 1, value, 1, 50);
}

static esp_err_t configure_probe(mpu6050_t *sensor, bool enable_fifo)
{
    esp_err_t err = mpu6050_configure(sensor, BOARD_MPU_SAMPLE_HZ);
    if (err != ESP_OK) return err;
    err = probe_write(sensor, 0x38, 0x00); /* Disable sensor interrupt output. */
    if (err != ESP_OK) return err;
    uint8_t value = 0xff;
    err = probe_read(sensor, 0x38, &value);
    if (err != ESP_OK) return err;
    if (value & 0x59) return ESP_ERR_INVALID_RESPONSE;
    if (enable_fifo) return ESP_OK;

    err = probe_write(sensor, 0x23, 0x00);
    if (err != ESP_OK) return err;
    err = probe_write(sensor, 0x6a, 0x00);
    if (err != ESP_OK) return err;
    err = probe_write(sensor, 0x6a, 0x04); /* Reset the disabled FIFO. */
    if (err != ESP_OK) return err;
    bool reset_finished = false;
    for (int i = 0; i < 10; ++i) {
        vTaskDelay(pdMS_TO_TICKS(2));
        err = probe_read(sensor, 0x6a, &value);
        if (err != ESP_OK) return err;
        if ((value & 0x77) == 0) { reset_finished = true; break; }
    }
    if (!reset_finished) return ESP_ERR_INVALID_RESPONSE;
    err = probe_read(sensor, 0x23, &value);
    if (err != ESP_OK) return err;
    if (value != 0) return ESP_ERR_INVALID_RESPONSE;
    uint16_t count = 0;
    uint8_t status = 0;
    err = mpu6050_fifo_count(sensor, &count, &status);
    if (err != ESP_OK) return err;
    return count == 0 ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

static void print_result(const fifo_probe_result_t *r)
{
    serial_console_printf("FIFO_RESULT mode=%s fifo_enabled=%d complete=%d bus_hz=%d who=0x%02x"
           " elapsed_us=%" PRId64 " setup_attempts=%" PRIu32 " setups_ok=%" PRIu32
           " setup_errors=%" PRIu32 " polls=%" PRIu32 " batches=%" PRIu32
           " read_errors=%" PRIu32 " discarded_batches=%" PRIu32
           " overflow_flags=%" PRIu32 " loss_resets=%" PRIu32 " empty_polls=%" PRIu32
           " max_count=%u payload_reads=%" PRIu64 " direct_reads=%" PRIu64
           " delta_max=%" PRIu32 " payload_min=%d:%d:%d payload_max=%d:%d:%d"
           " direct_min=%d:%d:%d direct_max=%d:%d:%d last_err=%s phase_ready=0\n",
           r->mode, r->fifo_enabled, r->complete, BOARD_I2C_SPEED_HZ, r->who,
           r->elapsed_us, r->setup_attempts, r->setups_ok, r->setup_errors,
           r->polls, r->batches, r->read_errors, r->discarded_batches,
           r->overflow_flags, r->loss_resets, r->empty_polls,
           (unsigned)r->max_count, r->payload_reads, r->direct_reads, r->delta_max,
           r->payload_low[0], r->payload_low[1], r->payload_low[2],
           r->payload_high[0], r->payload_high[1], r->payload_high[2],
           r->direct_low[0], r->direct_low[1], r->direct_low[2],
           r->direct_high[0], r->direct_high[1], r->direct_high[2], esp_err_to_name(r->last_error));
}

static void run_window(fifo_probe_result_t *r)
{
    for (int axis = 0; axis < 3; ++axis) {
        r->payload_low[axis] = r->direct_low[axis] = INT16_MAX;
        r->payload_high[axis] = r->direct_high[axis] = INT16_MIN;
    }
    const int64_t start = esp_timer_get_time();
    mpu6050_t sensor = {0};
    esp_err_t err = mpu6050_create(&sensor, BOARD_MPU_SDA_GPIO, BOARD_MPU_SCL_GPIO,
                                  BOARD_MPU_ADDRESS, BOARD_I2C_SPEED_HZ);
    bool configured = false;
    int64_t last_row = start;
    if (err != ESP_OK) {
        ++r->setup_attempts;
        ++r->setup_errors;
        r->last_error = err;
    } else while (esp_timer_get_time() - start < WINDOW_US) {
        if (!configured) {
            ++r->setup_attempts;
            err = configure_probe(&sensor, r->fifo_enabled);
            r->who = sensor.who_am_i;
            if (err != ESP_OK) {
                ++r->setup_errors;
                r->last_error = err;
                serial_console_printf("FIFO_SETUP_FAILED mode=%s err=%s\n", r->mode, esp_err_to_name(err));
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            ++r->setups_ok;
            configured = true;
            serial_console_printf("FIFO_SETUP mode=%s who=0x%02x id_reads=%u fifo_enabled=%d int_enabled=0\n",
                   r->mode, sensor.who_am_i, (unsigned)sensor.identity_reads, r->fifo_enabled);
        }
        ++r->polls;
        uint16_t bytes = 0;
        uint8_t irq_status = 0;
        const char *operation = "status_count";
        err = mpu6050_fifo_count(&sensor, &bytes, &irq_status);
        if (err == ESP_OK) {
            if (bytes > r->max_count) r->max_count = bytes;
            if (r->fifo_enabled && ((irq_status & 0x10) ||
                bytes >= (sensor.fifo_capacity_bytes / 6) * 6)) {
                if (irq_status & 0x10) ++r->overflow_flags;
                ++r->loss_resets;
                configured = false;
                continue;
            }
            if (!r->fifo_enabled && bytes != 0) err = ESP_ERR_INVALID_RESPONSE;
        }
        size_t frames = r->fifo_enabled ? bytes / 6 : CONTROL_READS;
        if (frames > 32) frames = 32;
        if (err == ESP_OK && frames == 0) {
            ++r->empty_polls;
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        mpu6050_raw_t batch[32];
        if (err == ESP_OK) {
            operation = r->fifo_enabled ? "fifo_payload" : "direct_payload";
            if (r->fifo_enabled) err = mpu6050_fifo_read(&sensor, batch, frames);
            else for (size_t i = 0; i < frames && err == ESP_OK; ++i)
                err = mpu6050_read_accel_registers(&sensor, &batch[i]);
        }
        mpu6050_raw_t direct;
        if (err == ESP_OK) {
            operation = "direct_comparison";
            err = mpu6050_read_accel_registers(&sensor, &direct);
        }
        if (err != ESP_OK) {
            ++r->read_errors;
            ++r->discarded_batches;
            r->last_error = err;
            configured = false;
            serial_console_printf("FIFO_FAIL mode=%s op=%s err=%s\n", r->mode, operation, esp_err_to_name(err));
            continue; /* Discard even successfully read parts; reset before reuse. */
        }
        ++r->batches;
        r->payload_reads += frames;
        ++r->direct_reads;
        update_range(&direct, r->direct_low, r->direct_high);
        for (size_t i = 0; i < frames; ++i) {
            update_range(&batch[i], r->payload_low, r->payload_high);
            const uint32_t delta[] = {axis_delta(batch[i].x, direct.x),
                axis_delta(batch[i].y, direct.y), axis_delta(batch[i].z, direct.z)};
            for (int axis = 0; axis < 3; ++axis)
                if (delta[axis] > r->delta_max) r->delta_max = delta[axis];
        }
        const int64_t now = esp_timer_get_time();
        if (now - last_row >= 1000000) {
            const mpu6050_raw_t *last = &batch[frames - 1];
            serial_console_printf("FIFO_ROW mode=%s elapsed_us=%" PRId64 " count=%u frames=%u"
                   " payload=%d:%d:%d direct=%d:%d:%d delta_max=%" PRIu32 "\n",
                   r->mode, now - start, (unsigned)bytes, (unsigned)frames,
                   last->x, last->y, last->z, direct.x, direct.y, direct.z, r->delta_max);
            last_row = now;
        }
        vTaskDelay(pdMS_TO_TICKS(r->fifo_enabled && bytes / 6 > 32 ? 1 : 10));
    }
    r->elapsed_us = esp_timer_get_time() - start;
    r->complete = sensor.device != NULL && r->elapsed_us >= WINDOW_US;
    mpu6050_destroy(&sensor);
    for (int axis = 0; axis < 3; ++axis) {
        if (!r->payload_reads) r->payload_low[axis] = r->payload_high[axis] = 0;
        if (!r->direct_reads) r->direct_low[axis] = r->direct_high[axis] = 0;
    }
    print_result(r);
}

void fifo_probe_run(void)
{
    esp_log_level_set("i2c.master", ESP_LOG_DEBUG);
    ESP_LOGW("fifo_probe", "V0 static bench only: DIRECT_BEFORE -> FIFO -> DIRECT_AFTER, 10s each");
    ESP_LOGW("fifo_probe", "Single main task, no DRDY ISR, no optical, no synchronized phase or balance output");
    ESP_LOGW("fifo_probe", "Control repeatedly reads direct registers; counts do not prove independent new samples");
    fifo_probe_result_t results[] = {
        {.mode = "DIRECT_BEFORE", .fifo_enabled = false},
        {.mode = "FIFO", .fifo_enabled = true},
        {.mode = "DIRECT_AFTER", .fifo_enabled = false},
    };
    for (int i = 0; i < 3; ++i) run_window(&results[i]);
    while (true) {
        serial_console_printf("FIFO_DONE optical=DISABLED acquisition=STOPPED phase_ready=0\n");
        for (int i = 0; i < 3; ++i) print_result(&results[i]);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
