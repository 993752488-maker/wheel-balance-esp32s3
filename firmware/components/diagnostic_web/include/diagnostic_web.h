#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Call before optical_ref_start()/acquisition_start(): WiFi startup/calibration
 * must finish before timing-sensitive capture. Returns failures, never aborts
 * and never erases NVS. A failure must not prevent the application capture.
 * Owns the WiFi AP interface; only call from the application's startup task.
 * Disabled builds return ESP_ERR_NOT_SUPPORTED without creating tasks.
 */
esp_err_t diagnostic_web_start(void);

/* Optional, bounded local GET of / and /api/live at 192.168.4.1.
 * Call from an ordinary application task, never a timer callback/ISR. Logs
 * HTTP_SELFTEST through serial_console; success does not prove a phone joined.
 */
esp_err_t diagnostic_web_run_selfcheck(void);
/* Bounded read-only 5x live + 1x session GET load, at most 1200 requests.
 * Uses the caller task; no recorder commands, Flash writes or wheel action. */
esp_err_t diagnostic_web_load_selfcheck(uint32_t requests);
/* Read-only live HTTP latency/size counters; no network request is generated. */
esp_err_t diagnostic_web_link_status_to_serial(void);
/* Queue a frozen CSV in the persistent exporter. USB reads the same writer
 * directly (indexed/CRC checked), independently of HTTP sockets. Task context,
 * no per-request allocation; ESP_OK means queued, not already transmitted. */
esp_err_t diagnostic_web_raw_export_to_serial(uint32_t capture_id);
esp_err_t diagnostic_web_pose_export_to_serial(uint32_t capture_id);
/* Bounded read-only HTTP CSV probe on the calling task. Reports bytes/CRC;
 * does not start, replace or qualify a recording. For static software checks. */
esp_err_t diagnostic_web_csv_selfcheck(uint32_t capture_id,int pose);
/* Read-only bounded snapshot of the real web-display ring for USB diagnosis.
 * Does not consume acquisition/analysis queues or change a product session. */
esp_err_t diagnostic_web_trace_to_serial(void);
/* Stable for this boot; NULL unless web initialization succeeded. */
const char *diagnostic_web_boot_id(void);

#ifdef __cplusplus
}
#endif
