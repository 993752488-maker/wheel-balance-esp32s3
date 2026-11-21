#pragma once
#include "raw_capture_core.h"
#include "raw_pose_core.h"
#include "esp_err.h"

esp_err_t raw_capture_init(void);
/* Producers never wait for storage, web or USB. Calls are task-context only. */
void raw_capture_publish_sample(const raw_sample_t *);
void raw_capture_publish_reference(const raw_reference_t *);
void raw_capture_publish_pose(const raw_pose_t *);
void raw_capture_signal_fault(uint32_t faults);
void raw_capture_get_status(raw_status_t *);
/* Matching capture ID required; unavailable/old data is never substituted. */
bool raw_capture_get_pose_status(uint32_t id,raw_pose_status_t *);
esp_err_t raw_capture_begin(uint32_t duration_ms,raw_context_t context,bool only_if_unused);
esp_err_t raw_capture_stop(uint32_t id);
/* A lease protects this immutable dataset from replacement while exporting. */
esp_err_t raw_capture_export_begin(uint32_t id,raw_status_t *);
/* Nonblocking recording-prefix lease. Existing rows stay immutable while
 * producers append. Bound reads by returned counts; release with export_end. */
esp_err_t raw_capture_preview_begin(uint32_t id,raw_status_t *);
void raw_capture_export_end(uint32_t id);
/* Exclusively reserve an idle recorder for storage maintenance. This prevents
 * another client starting a capture during an intentional observation pause. */
esp_err_t raw_capture_maintenance_begin(void);
void raw_capture_maintenance_end(void);
size_t raw_capture_read_samples(uint32_t id,size_t offset,raw_sample_t *,size_t capacity);
size_t raw_capture_read_references(uint32_t id,size_t offset,raw_reference_t *,size_t capacity);
size_t raw_capture_read_poses(uint32_t id,size_t offset,raw_pose_t *,size_t capacity);
typedef bool (*raw_csv_sink_t)(const char *,size_t,void *);
/* Synchronous, task-only bounded streaming. The export lease is held until
 * completion/error. A failed sink stops output and always releases the lease. */
esp_err_t raw_capture_write_csv(uint32_t id,const char *boot_id,const char *firmware,
    raw_csv_sink_t sink,void *arg);
esp_err_t raw_capture_write_pose_csv(uint32_t id,const char *boot_id,const char *firmware,
    raw_csv_sink_t sink,void *arg);
