#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    bool ready,busy;
    uint32_t completed,failed,stack_min_free_bytes;
} capture_export_status_t;
/* One permanent task/queue, allocated before clients can submit work.
 * No per-export task allocation. The recorder lease covers the entire job. */
esp_err_t capture_export_start(const char *boot_id,const char *firmware);
esp_err_t capture_export_usb(uint32_t capture_id,bool pose);
void capture_export_get_status(capture_export_status_t *);
