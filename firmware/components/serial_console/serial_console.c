#include "serial_console.h"
#include <stdatomic.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#if CONFIG_DIAGNOSTIC_WEB_ENABLED
#include "freertos/message_buffer.h"
#endif

static SemaphoreHandle_t s_tx_lock;
static atomic_uint s_errors;
static atomic_uint s_skipped;
#if CONFIG_DIAGNOSTIC_WEB_ENABLED
static MessageBufferHandle_t s_records;

/* Only this low-priority task can wait for USB. Web/sampling/main tasks
 * enqueue whole records without waiting for the host to open its port. */
static void usb_logger(void *arg)
{
    (void)arg;
    char record[1024];
    unsigned records=0;
    while (true) {
        const size_t length=xMessageBufferReceive(s_records, record, sizeof(record), portMAX_DELAY);
        if (!length) continue;
        if (!usb_serial_jtag_is_connected()) {
            atomic_fetch_add(&s_skipped, 1);
            continue;
        }
        size_t sent=0;
        while (sent<length) {
            const int written=usb_serial_jtag_write_bytes(record+sent, length-sent, portMAX_DELAY);
            if (written>0) sent+=(size_t)written;
            else { atomic_fetch_add(&s_errors, 1); vTaskDelay(1); }
        }
        if(!(++records%16))vTaskDelay(1);
    }
}
#endif

static int console_vprintf(const char *format, va_list args)
{
    char record[1024];
    const int length = vsnprintf(record, sizeof(record), format, args);
    if (length < 0 || (size_t)length >= sizeof(record) || !s_tx_lock) {
        atomic_fetch_add(&s_errors, 1);
        return -1; /* Do not emit a truncated measurement record. */
    }
    if (length == 0) return 0;
#if CONFIG_DIAGNOSTIC_WEB_ENABLED
    if (!usb_serial_jtag_is_connected()) {
        atomic_fetch_add(&s_skipped, 1);
        return length;
    }
    xSemaphoreTakeRecursive(s_tx_lock, portMAX_DELAY);
    const size_t queued=xMessageBufferSend(s_records, record, (size_t)length, 0);
    xSemaphoreGiveRecursive(s_tx_lock);
    if (queued!=(size_t)length) {
        atomic_fetch_add(&s_errors, 1);
        return -1; /* Whole diagnostic record dropped; never block web/main. */
    }
    return length;
#else
    xSemaphoreTakeRecursive(s_tx_lock, portMAX_DELAY);
    size_t sent = 0;
    while (sent < (size_t)length) {
        /* Bypass VFS's character-at-a-time timeout/drop policy. This copies
         * the complete remaining record into the driver ring buffer, or
         * waits for space. Successful enqueue is not a host acknowledgement. */
        const int written = usb_serial_jtag_write_bytes(record + sent,
            (size_t)length - sent, portMAX_DELAY);
        if (written <= 0) {
            atomic_fetch_add(&s_errors, 1);
            vTaskDelay(1);
            continue;
        }
        sent += (size_t)written;
    }
    xSemaphoreGiveRecursive(s_tx_lock);
    return length;
#endif
}

esp_err_t serial_console_start(void)
{
    if (s_tx_lock) return ESP_ERR_INVALID_STATE;
    s_tx_lock = xSemaphoreCreateRecursiveMutex();
    if (!s_tx_lock) return ESP_ERR_NO_MEM;
    usb_serial_jtag_driver_config_t config = {
        .tx_buffer_size = 8192, .rx_buffer_size = 256,
    };
    const esp_err_t err = usb_serial_jtag_driver_install(&config);
    if (err != ESP_OK) {
        vSemaphoreDelete(s_tx_lock);
        s_tx_lock = NULL;
        return err;
    }
    usb_serial_jtag_vfs_use_driver();
#if CONFIG_DIAGNOSTIC_WEB_ENABLED
    s_records=xMessageBufferCreate(8192);
    if (!s_records || xTaskCreate(usb_logger, "usb_logger", 3072, NULL, 1, NULL)!=pdPASS) {
        if (s_records) vMessageBufferDelete(s_records);
        s_records=NULL;
        (void)usb_serial_jtag_driver_uninstall();
        vSemaphoreDelete(s_tx_lock);
        s_tx_lock=NULL;
        return ESP_ERR_NO_MEM;
    }
#endif
    esp_log_set_vprintf(console_vprintf);
    return ESP_OK;
}

int serial_console_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    const int result = console_vprintf(format, args);
    va_end(args);
    return result;
}

uint32_t serial_console_errors(void)
{
    return atomic_load(&s_errors);
}

uint32_t serial_console_skipped_records(void)
{
    return atomic_load(&s_skipped);
}

size_t serial_console_read(char *out,size_t capacity)
{
    if(!s_tx_lock || !out || !capacity)return 0;
    const int got=usb_serial_jtag_read_bytes(out,capacity,0);return got>0?(size_t)got:0;
}
bool serial_console_try_record(const char *data,size_t length)
{
#if CONFIG_DIAGNOSTIC_WEB_ENABLED
    if(!s_records || !data || !length || length>1024 || !usb_serial_jtag_is_connected())return false;
    if(xSemaphoreTakeRecursive(s_tx_lock,0)!=pdTRUE)return false;
    const size_t sent=xMessageBufferSend(s_records,data,length,0);
    xSemaphoreGiveRecursive(s_tx_lock);return sent==length;
#else
    (void)data;(void)length;return false;
#endif
}
