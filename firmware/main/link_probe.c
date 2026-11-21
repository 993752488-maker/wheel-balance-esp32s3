#include "link_probe.h"
#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include "serial_console.h"
#include "sdkconfig.h"
#include "board_config.h"
#include "mpu6050.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

enum { PROBE_READS = 3000 };
#if CONFIG_V0_LINK_PROBE_BACK_TO_BACK
enum { PROBE_PERIOD_MS = 0 };
#else
enum { PROBE_PERIOD_MS = 2 };
#endif

typedef struct {
    uint32_t bus_hz;
    uint8_t reg;
    bool setup_ok;
    uint32_t attempted, good, failed, wrong_identity, late;
    uint32_t max_call_us;
    int64_t elapsed_us;
    esp_err_t last_error;
    bool deadline_reached;
    int16_t low[3], high[3];
} probe_result_t;

static int16_t signed_be(const uint8_t *p)
{
    const uint16_t bits = ((uint16_t)p[0] << 8) | p[1];
    return bits < 0x8000 ? (int16_t)bits : (int16_t)((int32_t)bits - 65536);
}

static void print_result(const probe_result_t *r)
{
    serial_console_printf("LINK_RESULT bus_hz=%" PRIu32 " reg=0x%02x period_ms=%d setup_ok=%d attempted=%" PRIu32
           " good=%" PRIu32 " failed=%" PRIu32 " wrong_identity=%" PRIu32
           " elapsed_us=%" PRId64 " late=%" PRIu32 " max_call_us=%" PRIu32
           " deadline=%d last_err=%s xyz_min=%d:%d:%d xyz_max=%d:%d:%d phase_ready=0\n",
           r->bus_hz, r->reg, PROBE_PERIOD_MS, r->setup_ok, r->attempted, r->good, r->failed,
           r->wrong_identity, r->elapsed_us, r->late, r->max_call_us,
           r->deadline_reached, esp_err_to_name(r->last_error),
           r->low[0], r->low[1], r->low[2], r->high[0], r->high[1], r->high[2]);
}

static esp_err_t disable_fifo_and_interrupts(mpu6050_t *d)
{
    const uint8_t settings[][3] = {{0x38, 0x00, 0x59}, {0x23, 0x00, 0xff}, {0x6a, 0x00, 0x70}};
    for (size_t i = 0; i < sizeof(settings) / sizeof(settings[0]); ++i) {
        const uint8_t packet[] = {settings[i][0], settings[i][1]};
        esp_err_t err = i2c_master_transmit(d->device, packet, sizeof(packet), 50);
        if (err != ESP_OK) return err;
        uint8_t actual = 0;
        err = i2c_master_transmit_receive(d->device, packet, 1, &actual, 1, 50);
        if (err != ESP_OK) return err;
        if ((actual & settings[i][2]) != 0) return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static void read_probe(mpu6050_t *d, probe_result_t *r)
{
    r->setup_ok = true;
    for (int axis = 0; axis < 3; ++axis) {
        r->low[axis] = INT16_MAX;
        r->high[axis] = INT16_MIN;
    }
    const int64_t start_us = esp_timer_get_time();
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PROBE_PERIOD_MS);
    for (unsigned i = 0; i < PROBE_READS; ++i) {
        if (esp_timer_get_time() - start_us >= 10000000) {
            r->deadline_reached = true;
            break;
        }
        uint8_t bytes[6] = {0};
        const size_t length = r->reg == 0x75 ? 1 : sizeof(bytes);
        const int64_t before_us = esp_timer_get_time();
        const esp_err_t err = i2c_master_transmit_receive(d->device, &r->reg, 1, bytes, length, 50);
        const uint32_t call_us = (uint32_t)(esp_timer_get_time() - before_us);
        if (call_us > r->max_call_us) r->max_call_us = call_us;
        ++r->attempted;
        if (err != ESP_OK) {
            ++r->failed;
            r->last_error = err;
            serial_console_printf("LINK_FAIL bus_hz=%" PRIu32 " reg=0x%02x attempt=%" PRIu32 " err=%s call_us=%" PRIu32 "\n",
                   r->bus_hz, r->reg, r->attempted, esp_err_to_name(err), call_us);
        } else {
            ++r->good;
            if (r->reg == 0x75) {
                if (bytes[0] != d->who_am_i) ++r->wrong_identity;
            } else {
                for (int axis = 0; axis < 3; ++axis) {
                    const int16_t value = signed_be(bytes + 2 * axis);
                    if (value < r->low[axis]) r->low[axis] = value;
                    if (value > r->high[axis]) r->high[axis] = value;
                }
            }
        }
        /* No retries and no reconstruction. This is a non-consuming register
         * experiment, not a continuous, synchronized sampling stream. */
        if (period) {
            if (xTaskGetTickCount() - wake >= period) ++r->late;
            (void)xTaskDelayUntil(&wake, period);
        }
    }
    r->elapsed_us = esp_timer_get_time() - start_us;
    if (r->reg == 0x75 || r->good == 0) {
        for (int axis = 0; axis < 3; ++axis) r->low[axis] = r->high[axis] = 0;
    }
    print_result(r);
}

void link_probe_run(void)
{
    esp_log_level_set("i2c.master", ESP_LOG_DEBUG);
    ESP_LOGW("link_probe", "V0 link-only experiment: no FIFO reads, no optical, no balance output");
    ESP_LOGW("link_probe", "3000 WHO + 3000 direct XYZ reads at each bus rate, period_ms=%d; max 10s per phase", PROBE_PERIOD_MS);
    probe_result_t results[4] = {
        {.bus_hz = 100000, .reg = 0x75}, {.bus_hz = 100000, .reg = 0x3b},
        {.bus_hz = 50000, .reg = 0x75}, {.bus_hz = 50000, .reg = 0x3b},
    };
    for (int rate = 0; rate < 2; ++rate) {
        mpu6050_t sensor = {0};
        esp_err_t err = mpu6050_create(&sensor, BOARD_MPU_SDA_GPIO, BOARD_MPU_SCL_GPIO,
                                      BOARD_MPU_ADDRESS, results[rate * 2].bus_hz);
        if (err == ESP_OK) err = mpu6050_configure(&sensor, BOARD_MPU_SAMPLE_HZ);
        if (err == ESP_OK) err = disable_fifo_and_interrupts(&sensor);
        if (err == ESP_OK) {
            serial_console_printf("LINK_SETUP bus_hz=%" PRIu32 " who=0x%02x id_reads=%u identity_matches_label=%d fifo_enabled=0 int_enabled=0\n",
                   sensor.i2c_speed_hz, sensor.who_am_i, (unsigned)sensor.identity_reads, sensor.expected_identity);
            read_probe(&sensor, &results[rate * 2]);
            read_probe(&sensor, &results[rate * 2 + 1]);
        } else {
            results[rate * 2].last_error = results[rate * 2 + 1].last_error = err;
            serial_console_printf("LINK_SETUP_FAILED bus_hz=%" PRIu32 " err=%s\n", results[rate * 2].bus_hz, esp_err_to_name(err));
        }
        mpu6050_destroy(&sensor);
    }
    /* Preserve full aggregate evidence even if the host opened its port late. */
    while (true) {
        serial_console_printf("LINK_DONE optical=DISABLED fifo=DISABLED phase_ready=0\n");
        for (int i = 0; i < 4; ++i) print_result(&results[i]);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
