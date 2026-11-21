#pragma once

#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/* Task context only. Web builds enqueue whole records into a bounded
 * diagnostic queue so an unread USB port cannot block main/web operation.
 * Legacy non-web builds retain the original blocking, whole-record path. */
esp_err_t serial_console_start(void);
int serial_console_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));
uint32_t serial_console_errors(void);
uint32_t serial_console_skipped_records(void); /* no USB host: not a sensor fault */
size_t serial_console_read(char *out,size_t capacity); /* nonblocking, task only */
bool serial_console_try_record(const char *data,size_t length); /* retry is not data loss */
