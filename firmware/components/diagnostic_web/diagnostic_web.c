#include "sdkconfig.h"
#include "diagnostic_web.h"

#if CONFIG_DIAGNOSTIC_WEB_ENABLED

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "acquisition.h"
#include "optical_ref.h"
#include "product_http.h"
#include "raw_capture.h"
#include "capture_export.h"
#include "capture_export_http.h"
#include "cJSON.h"
#include "serial_console.h"
#include "board_config.h"
#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define HISTORY_POINTS 512U
#define RESPONSE_POINTS 128U
#define DISPLAY_HZ 100U
#define TELEMETRY_STACK_BYTES 3072U
#define HTTP_STACK_BYTES 6144U
#define LIVE_JSON_CAPACITY 24576U
#define MAX_SAMPLE_AGE_US INT64_C(500000)
#define AP_SSID CONFIG_DIAGNOSTIC_WEB_AP_SSID
#define AP_PASSWORD CONFIG_DIAGNOSTIC_WEB_AP_PASSWORD

_Static_assert(sizeof(AP_SSID) >= 2 && sizeof(AP_SSID) <= 33,
               "Wi-Fi SSID must contain 1 to 32 bytes");
_Static_assert(sizeof(AP_PASSWORD) >= 9 && sizeof(AP_PASSWORD) <= 64,
               "Wi-Fi WPA2 password must contain 8 to 63 bytes");
#define AP_IP "192.168.4.1"

/* Raw storage keeps the fixed ring small. Convert to SI after snapshotting.
 * t_us is the sensor driver's FIFO batch READOUT time, never a reconstructed
 * sample timestamp. Missing source_seq values reflect display decimation.
 */
typedef struct {
    uint64_t id;
    int64_t t_us;
    uint64_t source_seq;
    uint32_t epoch;
    int16_t x, y, z;
    int8_t level;
    bool gap_before; /* Display pause/input event, not intentional decimation. */
} display_point_t;

typedef struct {
    acquisition_status_t imu;
    optical_ref_status_t ref;
    double readout_hz;
    uint64_t history_overwrites;
    uint32_t serial_drops;
} telemetry_status_t;

typedef struct {
    telemetry_status_t status;
    display_point_t points[RESPONSE_POINTS];
    size_t count;
    uint64_t last_id;
    bool truncated;
} response_snapshot_t;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static display_point_t s_history[HISTORY_POINTS];
static size_t s_next, s_count;
static uint64_t s_last_id;
static telemetry_status_t s_status;
/* The synchronous HTTP server invokes one handler at a time on its sole
 * server task. These scratch buffers are therefore shared by handlers, not
 * allocated for each phone/request. No async handler may use them.
 */
static response_snapshot_t s_response;
static char s_json_chunk[2048];
static char s_boot_id[17];
static char s_firmware_json[512];
static httpd_handle_t s_server;
static esp_netif_t *s_ap;
static TaskHandle_t s_telemetry_task;
static bool s_wifi_initialized, s_wifi_started, s_created_event_loop;
static bool s_started;
static char *s_live_json;
static size_t s_live_json_used;
static bool s_live_buffering;
static portMUX_TYPE s_link_lock=portMUX_INITIALIZER_UNLOCKED;
static struct {
    uint32_t requests,errors;
    uint64_t last_us,max_us,last_gap_us,max_gap_us,last_display_id;
    int64_t previous_begin_us;
    uint32_t last_bytes,last_points;
} s_link;

extern const unsigned char index_html_start[] asm("_binary_index_html_start");
extern const unsigned char index_html_end[] asm("_binary_index_html_end");

static const char *reference_state(const optical_ref_status_t *ref)
{
    if (!ref->enabled) return "DISABLED";
    if (ref->input_queue_drops) return "INPUT_LOSS_REBOOT_REQUIRED";
    if (ref->clock_faults) return "CLOCK_FAULT_REBOOT_REQUIRED";
    if (ref->sample_gaps) return "SAMPLE_GAP_REBOOT_REQUIRED";
    if (ref->timed_out) return "NO_REFERENCE_TIMEOUT";
    if (!ref->rpm.has_accepted_edge) return "NO_REFERENCE";
    if (!ref->rpm.rpm_valid) return "WAIT_SECOND_EDGE";
    return "PERIOD_OK";
}

static void telemetry_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    uint64_t displayed_sequence = 0, rate_samples = 0;
    uint32_t displayed_epoch = 0, rate_epoch = 0;
    uint32_t previous_epoch=0,previous_i2c=0,previous_loss=0,previous_overflow=0,previous_restarts=0,previous_frame_faults=0;
    int64_t previous_point_us=0;
    bool gap_pending=true;
    int64_t rate_us = esp_timer_get_time();
    double readout_hz = 0;
    while (true) {
        telemetry_status_t status = {0};
        acquisition_get_status(&status.imu);
        optical_ref_get_status(&status.ref);
        status.serial_drops = serial_console_errors();
        if(!status.imu.online || (previous_epoch&&status.imu.epoch!=previous_epoch) ||
            status.imu.i2c_errors!=previous_i2c || status.imu.fifo_loss_resets!=previous_loss ||
            status.imu.fifo_overflows!=previous_overflow || status.imu.restarts!=previous_restarts ||
            status.imu.frame_mismatches!=previous_frame_faults)gap_pending=true;
        previous_epoch=status.imu.epoch;previous_i2c=status.imu.i2c_errors;
        previous_loss=status.imu.fifo_loss_resets;previous_overflow=status.imu.fifo_overflows;
        previous_restarts=status.imu.restarts;previous_frame_faults=status.imu.frame_mismatches;
        const int64_t now = esp_timer_get_time();
        if (!status.imu.online || status.imu.epoch != rate_epoch) {
            readout_hz = 0;
            rate_samples = status.imu.total_samples;
            rate_epoch = status.imu.epoch;
            rate_us = now;
        } else if (now - rate_us >= INT64_C(1000000)) {
            readout_hz = (double)(status.imu.total_samples - rate_samples) *
                         1000000.0 / (double)(now - rate_us);
            rate_samples = status.imu.total_samples;
            rate_us = now;
        }
        status.readout_hz = readout_hz;
        const acceleration_sample_t *sample = &status.imu.latest;
        const bool append = status.imu.online && sample->sequence != 0 &&
            (sample->sequence != displayed_sequence || sample->epoch != displayed_epoch);
        display_point_t point = {0};
        if (append) {
            point.t_us = sample->readout_us;
            point.source_seq = sample->sequence;
            point.epoch = sample->epoch;
            point.x = sample->raw.x;
            point.y = sample->raw.y;
            point.z = sample->raw.z;
            point.level = (int8_t)status.ref.input_level;
            point.gap_before=gap_pending||(previous_point_us&&
                (point.t_us<previous_point_us||point.t_us-previous_point_us>INT64_C(250000)));
            gap_pending=false;previous_point_us=point.t_us;
            displayed_sequence = sample->sequence;
            displayed_epoch = sample->epoch;
        }
        portENTER_CRITICAL(&s_lock);
        status.history_overwrites = s_status.history_overwrites;
        if (append) {
            point.id = ++s_last_id;
            s_history[s_next] = point;
            s_next = (s_next + 1U) % HISTORY_POINTS;
            if (s_count < HISTORY_POINTS) ++s_count;
            else ++status.history_overwrites;
        }
        s_status = status;
        portEXIT_CRITICAL(&s_lock);
        /* This task only observes published status; it never drains either
         * acquisition or optical reference queues used by the USB logger.
         */
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000U / DISPLAY_HZ));
    }
}

static void snapshot_after(uint64_t after)
{
    s_response.count = 0;
    s_response.truncated = false;
    s_response.last_id = after;
    portENTER_CRITICAL(&s_lock);
    s_response.status = s_status;
    if (s_count == 0) {
        s_response.last_id = 0;
    } else {
        const size_t oldest = (s_next + HISTORY_POINTS - s_count) % HISTORY_POINTS;
        const uint64_t oldest_id = s_history[oldest].id;
        size_t offset;
        if (after == 0) {
            /* First page intentionally starts with the latest short window. */
            offset = s_count > RESPONSE_POINTS ? s_count - RESPONSE_POINTS : 0;
        } else if (after > s_last_id) {
            /* An old boot's cursor must not strand a client after reboot. */
            s_response.truncated = true;
            offset = s_count > RESPONSE_POINTS ? s_count - RESPONSE_POINTS : 0;
        } else if (after < oldest_id - 1U) {
            s_response.truncated = true;
            offset = 0;
        } else {
            offset = (size_t)(after - oldest_id + 1U);
        }
        size_t count = s_count - offset;
        if (count > RESPONSE_POINTS) count = RESPONSE_POINTS;
        /* At most two bounded copies, and no formatting/socket work under
         * the short lock. At most 128 raw points (~5 KiB) are copied here.
         */
        const size_t begin = (oldest + offset) % HISTORY_POINTS;
        const size_t first = count < HISTORY_POINTS - begin ? count : HISTORY_POINTS - begin;
        memcpy(s_response.points, s_history + begin, first * sizeof(display_point_t));
        if (count > first)
            memcpy(s_response.points + first, s_history, (count - first) * sizeof(display_point_t));
        s_response.count = count;
        if (count) s_response.last_id = s_response.points[count - 1U].id;
    }
    portEXIT_CRITICAL(&s_lock);
}

static bool parse_after(httpd_req_t *req, uint64_t *after)
{
    char query[64], value[24];
    *after = 0;
    const size_t len = httpd_req_get_url_query_len(req);
    if (!len) return true;
    if (len >= sizeof(query) || httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK)
        return false;
    const esp_err_t err = httpd_query_key_value(query, "after", value, sizeof(value));
    if (err == ESP_ERR_NOT_FOUND) return true;
    if (err != ESP_OK || !value[0]) return false;
    for (const char *p = value; *p; ++p) {
        if (*p < '0' || *p > '9') return false;
        const unsigned digit = (unsigned)(*p - '0');
        if (*after > (UINT64_MAX - digit) / 10U) return false;
        *after = *after * 10U + digit;
    }
    return true;
}

static esp_err_t common_headers(httpd_req_t *req, const char *type)
{
    esp_err_t err = httpd_resp_set_type(req, type);
    if (err == ESP_OK) err = httpd_resp_set_hdr(req, "Cache-Control", "no-store, max-age=0");
    if (err == ESP_OK) err = httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
    return err;
}

static esp_err_t index_get(httpd_req_t *req)
{
    esp_err_t err = common_headers(req, "text/html; charset=utf-8");
    if (err != ESP_OK) return err;
    size_t size = (size_t)(index_html_end - index_html_start);
    if (size && index_html_start[size - 1U] == '\0') --size;
    return httpd_resp_send(req, (const char *)index_html_start, (ssize_t)size);
}

/* JSON must never contain NaN/Inf, or old invalid analysis values. Keep this
 * formatting outside all capture/status locks and bound each response chunk. */
// BEGIN LIVE JSON BUFFER
/* Only the sole HTTP task uses this buffer. CSV is streamed separately by its
 * own worker. Preserve the existing JSON bytes, send live data with one length-
 * delimited response, and avoid many small chunk headers/writes per poll. */
static esp_err_t send_json_piece(httpd_req_t *req,const char *data,ssize_t len)
{
    if(!s_live_buffering)return httpd_resp_send_chunk(req,data,len);
    if(!s_live_json)return ESP_ERR_INVALID_STATE;
    if(!data){if(len)return ESP_ERR_INVALID_ARG;return httpd_resp_send(req,s_live_json,(ssize_t)s_live_json_used);}
    if(len==HTTPD_RESP_USE_STRLEN)len=(ssize_t)strlen(data);
    if(len<0||(size_t)len>LIVE_JSON_CAPACITY-s_live_json_used)return ESP_ERR_INVALID_SIZE;
    memcpy(s_live_json+s_live_json_used,data,(size_t)len);s_live_json_used+=(size_t)len;
    return ESP_OK;
}
// END LIVE JSON BUFFER

static esp_err_t send_raw(httpd_req_t *req,bool comma)
{
    raw_status_t s;raw_capture_get_status(&s);
    raw_pose_status_t pose={0};(void)raw_capture_get_pose_status(s.id,&pose);
    capture_export_status_t exporter;capture_export_get_status(&exporter);
    const int n=snprintf(s_json_chunk,sizeof(s_json_chunk),
        "\"raw_capture\":{\"enabled\":%s,\"state\":\"%s\",\"context\":\"%s\","
        "\"id\":%" PRIu32 ",\"samples\":%u,\"references\":%u,\"faults\":%" PRIu32
        ",\"queue_drops\":%" PRIu32 ",\"start_us\":%" PRIu64 ",\"deadline_us\":%" PRIu64
        ",\"end_us\":%" PRIu64 ",\"integrity_ok\":%s,\"measurement_qualified\":false,"
        "\"pose\":{\"enabled\":%s,\"count\":%u,\"faults\":%" PRIu32 ",\"queue_drops\":%" PRIu32 ",\"frozen\":%s},\"export_busy\":%s}%s",
        s.enabled?"true":"false",raw_state_name(s.state),raw_context_name(s.context),s.id,
        (unsigned)s.samples,(unsigned)s.references,s.faults,s.queue_drops,s.start_us,s.deadline_us,s.end_us,
        s.integrity_ok?"true":"false",pose.enabled?"true":"false",(unsigned)pose.count,pose.faults,pose.queue_drops,
        pose.frozen?"true":"false",exporter.busy?"true":"false",comma?",":"");
    return n>0 && (size_t)n<sizeof(s_json_chunk)?send_json_piece(req,s_json_chunk,n):ESP_FAIL;
}
static esp_err_t raw_reply(httpd_req_t *req,bool ok)
{
    esp_err_t err=common_headers(req,"application/json; charset=utf-8");
    const int n=snprintf(s_json_chunk,sizeof(s_json_chunk),"{\"schema_version\":1,\"source\":\"live_device\",\"boot_id\":\"%s\",\"request_ok\":%s,",s_boot_id,ok?"true":"false");
    if(err==ESP_OK)err=send_json_piece(req,s_json_chunk,n);
    if(err==ESP_OK)err=send_raw(req,false);
    if(err==ESP_OK)err=httpd_resp_sendstr_chunk(req,"}");
    if(err==ESP_OK)err=send_json_piece(req,NULL,0);
    return err;
}
static esp_err_t raw_get(httpd_req_t *req){return raw_reply(req,true);}
static esp_err_t raw_post(httpd_req_t *req)
{
    char body[256];
    if(req->content_len<2 || req->content_len>=sizeof(body))return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"bounded JSON required");
    size_t used=0;
    while(used<req->content_len){int n=httpd_req_recv(req,body+used,req->content_len-used);if(n<=0)return ESP_FAIL;used+=(size_t)n;}
    body[used]=0;cJSON *root=cJSON_ParseWithLengthOpts(body,used+1,NULL,true);
    const cJSON *action=root?cJSON_GetObjectItemCaseSensitive(root,"action"):NULL;
    const cJSON *boot=root?cJSON_GetObjectItemCaseSensitive(root,"boot_id"):NULL;
    if(!cJSON_IsString(action) || !cJSON_IsString(boot) || strcmp(boot->valuestring,s_boot_id)){
        cJSON_Delete(root);return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"current boot_id required");}
    bool ok=false;
    if(!strcmp(action->valuestring,"start")) {
        const cJSON *ms=cJSON_GetObjectItemCaseSensitive(root,"duration_ms");
        acquisition_status_t imu;acquisition_get_status(&imu);
        optical_ref_status_t ref;optical_ref_get_status(&ref);
        if(imu.online && ref.enabled && !ref.input_queue_drops && !ref.clock_faults && !ref.sample_gaps &&
           cJSON_IsNumber(ms) && isfinite(ms->valuedouble) && ms->valuedouble>=250 && ms->valuedouble<=60000 &&
           floor(ms->valuedouble)==ms->valuedouble)
            ok=raw_capture_begin((uint32_t)ms->valuedouble,RAW_CONTEXT_USER,false)==ESP_OK;
    } else if(!strcmp(action->valuestring,"stop")) {
        const cJSON *id=cJSON_GetObjectItemCaseSensitive(root,"id");
        if(cJSON_IsNumber(id) && isfinite(id->valuedouble) && id->valuedouble>=1 &&
           id->valuedouble<=UINT32_MAX && floor(id->valuedouble)==id->valuedouble)
            ok=raw_capture_stop((uint32_t)id->valuedouble)==ESP_OK;
    }
    cJSON_Delete(root);return raw_reply(req,ok);
}
static esp_err_t live_render(httpd_req_t *req)
{
    uint64_t after;
    if (!parse_after(req, &after))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "after must be an unsigned decimal integer");
    snapshot_after(after);
    const int64_t now = esp_timer_get_time();
    acquisition_status_t *imu = &s_response.status.imu;
    optical_ref_status_t *ref = &s_response.status.ref;
    if (!ref->enabled) ref->input_level = -1;
    const int64_t age = imu->last_data_us > 0 && now >= imu->last_data_us ?
                        now - imu->last_data_us : now;
    const bool online = imu->online && imu->latest.sequence != 0 && age <= MAX_SAMPLE_AGE_US;
    if (ref->rpm.has_accepted_edge && (uint64_t)now >= ref->rpm.last_accepted_edge_us &&
        (uint64_t)now - ref->rpm.last_accepted_edge_us >= (uint64_t)CONFIG_V0_STOP_TIMEOUT_MS * 1000U) {
        ref->timed_out = true;
        ref->rpm.rpm_valid = false;
    }
    const bool rpm_valid = ref->enabled && !ref->timed_out && !ref->input_queue_drops &&
        !ref->clock_faults && !ref->sample_gaps && ref->rpm.rpm_valid &&
        isfinite(ref->rpm.rpm) && ref->rpm.rpm > 0;
    ref->rpm.rpm_valid = rpm_valid;
    char rpm[32] = "null";
    if (rpm_valid) snprintf(rpm, sizeof(rpm), "%.6f", ref->rpm.rpm);
    const uint64_t filtered = (uint64_t)ref->glitch_rejected + ref->pulse_rejected + ref->rpm.rejected_edges;
    esp_err_t err = common_headers(req, "application/json; charset=utf-8");
    if (err != ESP_OK) return err;
    int len = snprintf(s_json_chunk, sizeof(s_json_chunk),
        "{\"schema_version\":1,\"source\":\"live_device\",\"boot_id\":\"%s\","
        "\"firmware\":%s,\"uptime_us\":%" PRId64 ",\"rpm\":%s,\"rpm_valid\":%s,\"ref_state\":\"%s\","
        "\"imu\":{\"online\":%s,\"who\":\"0x%02x\",\"epoch\":%" PRIu32 ",\"samples\":%" PRIu64 ","
        "\"fifo_readout_hz\":%.3f,\"last_sample_age_us\":%" PRId64 ",\"frame_consistent\":%s,\"frame_difference_lsb\":%.2f},"
        "\"optical\":{\"level\":%d,\"accepted\":%" PRIu64 ",\"observed_edges\":%" PRIu32 ","
        "\"period_us\":%" PRIu64 ",\"last_accepted_edge_us\":%" PRIu64 ","
        "\"filtered\":%" PRIu64 ",\"sample_gaps\":%" PRIu32 ",\"max_sample_gap_us\":%" PRIu64 ","
        "\"input_queue_drops\":%" PRIu32 ",\"report_queue_drops\":%" PRIu32 "},"
        "\"faults\":{\"init_err\":%" PRIu32 ",\"i2c_err\":%" PRIu32 ",\"fifo_overflow\":%" PRIu32 ","
        "\"fifo_loss_resets\":%" PRIu32 ",\"acc_log_drop\":%" PRIu32 ",\"serial_drops\":%" PRIu32 ","
        "\"web_history_overwrites\":%" PRIu64 ",\"fifo_count_faults\":%" PRIu32 ",\"frame_mismatch\":%" PRIu32 "},"
        "\"waveform\":{\"display_hz\":100,\"acquisition_hz\":%d,\"gravity_included\":true,"
        "\"timestamp_kind\":\"fifo_batch_readout_us\",\"display_mode\":\"latest_fifo_observation\","
        "\"gap_policy\":\"display_id_epoch_input_event_or_250ms_pause\",\"points\":[",
        s_boot_id, s_firmware_json, now, rpm, rpm_valid ? "true" : "false", reference_state(ref),
        online ? "true" : "false", (unsigned)imu->who_am_i, imu->epoch, imu->total_samples,
        online ? s_response.status.readout_hz : 0.0, age,imu->frame_consistent?"true":"false",(double)imu->frame_difference_lsb,ref->input_level, ref->rpm.accepted_edges,
        ref->raw_edges, rpm_valid ? ref->rpm.period_us : 0,
        ref->rpm.has_accepted_edge ? ref->rpm.last_accepted_edge_us : 0,
        filtered, ref->sample_gaps, ref->max_sample_gap_us,
        ref->input_queue_drops, ref->report_queue_drops, imu->init_errors, imu->i2c_errors,
        imu->fifo_overflows, imu->fifo_loss_resets, imu->stream_drops,
        s_response.status.serial_drops, s_response.status.history_overwrites,imu->fifo_count_faults,imu->frame_mismatches, BOARD_MPU_SAMPLE_HZ);
    if (len < 0 || (size_t)len >= sizeof(s_json_chunk))
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON header overflow");
    err = send_json_piece(req, s_json_chunk, len);
    if (err != ESP_OK) return err;
    size_t used = 0;
    const double scale = 9.80665 / 8192.0;
    for (size_t i = 0; i < s_response.count; ++i) {
        const display_point_t *p = &s_response.points[i];
        if (sizeof(s_json_chunk) - used < 256U) {
            err = send_json_piece(req, s_json_chunk, (ssize_t)used);
            if (err != ESP_OK) return err;
            used = 0;
        }
        len = snprintf(s_json_chunk + used, sizeof(s_json_chunk) - used,
            "%s{\"id\":%" PRIu64 ",\"t_us\":%" PRId64 ",\"source_seq\":%" PRIu64 ",\"epoch\":%" PRIu32 ","
            "\"x\":%.6f,\"y\":%.6f,\"z\":%.6f,\"level\":%d,\"gap_before\":%s}",
            i ? "," : "", p->id, p->t_us, p->source_seq, p->epoch,
            (double)p->x * scale, (double)p->y * scale, (double)p->z * scale, (int)p->level,p->gap_before?"true":"false");
        if (len < 0 || (size_t)len >= sizeof(s_json_chunk) - used) return ESP_FAIL;
        used += (size_t)len;
    }
    if (used) {
        err = send_json_piece(req, s_json_chunk, (ssize_t)used);
        if (err != ESP_OK) return err;
    }
    len = snprintf(s_json_chunk, sizeof(s_json_chunk),
        "],\"last_id\":%" PRIu64 ",\"truncated\":%s},",
        s_response.last_id, s_response.truncated ? "true" : "false");
    if (len < 0 || (size_t)len >= sizeof(s_json_chunk)) return ESP_FAIL;
    err = send_json_piece(req, s_json_chunk, len);
    if (err == ESP_OK) err = send_raw(req, true);
    if (err == ESP_OK) {
        const char *features="\"features\":{\"sync_1x_live\":false,\"legacy_diagnostics\":false,\"product_session\":true}}";
        err = send_json_piece(req, features, HTTPD_RESP_USE_STRLEN);
    }
    if (err == ESP_OK) err = send_json_piece(req, NULL, 0);
    return err;
}

static esp_err_t live_get(httpd_req_t *req)
{
    const int64_t begin=esp_timer_get_time();
    s_live_json_used=0;s_live_buffering=true;
    const esp_err_t err=live_render(req);
    s_live_buffering=false;
    const uint64_t elapsed=(uint64_t)(esp_timer_get_time()-begin);
    portENTER_CRITICAL(&s_link_lock);
    ++s_link.requests;if(err!=ESP_OK)++s_link.errors;
    s_link.last_us=elapsed;if(elapsed>s_link.max_us)s_link.max_us=elapsed;
    s_link.last_gap_us=s_link.previous_begin_us>0?(uint64_t)(begin-s_link.previous_begin_us):0;
    if(s_link.last_gap_us>s_link.max_gap_us)s_link.max_gap_us=s_link.last_gap_us;
    s_link.previous_begin_us=begin;s_link.last_bytes=(uint32_t)s_live_json_used;
    s_link.last_points=(uint32_t)s_response.count;s_link.last_display_id=s_response.last_id;
    portEXIT_CRITICAL(&s_link_lock);
    if(err==ESP_ERR_INVALID_SIZE)return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"live JSON exceeds bounded buffer");
    return err;
}

esp_err_t diagnostic_web_link_status_to_serial(void)
{
    typeof(s_link) status;
    portENTER_CRITICAL(&s_link_lock);status=s_link;portEXIT_CRITICAL(&s_link_lock);
    serial_console_printf("WEB_LINK_STATUS requests=%" PRIu32 " errors=%" PRIu32 " last_us=%" PRIu64 " max_us=%" PRIu64 " last_gap_us=%" PRIu64 " max_gap_us=%" PRIu64 " last_bytes=%" PRIu32 " last_points=%" PRIu32 " last_display_id=%" PRIu64 " payload_mode=single_content_length\n",
        status.requests,status.errors,status.last_us,status.max_us,status.last_gap_us,status.max_gap_us,status.last_bytes,status.last_points,status.last_display_id);
    return ESP_OK;
}

static void make_firmware_json(void)
{
    char version[80];
    const esp_app_desc_t *app = esp_app_get_description();
    snprintf(version, sizeof(version), "%s@%s", app->project_name, app->version);
    size_t n = 0;
    s_firmware_json[n++] = '"';
    for (const unsigned char *p = (const unsigned char *)version; *p; ++p) {
        if (*p == '"' || *p == '\\') {
            s_firmware_json[n++] = '\\';
            s_firmware_json[n++] = (char)*p;
        } else if (*p < 0x20U) {
            n += (size_t)snprintf(s_firmware_json + n, sizeof(s_firmware_json) - n, "\\u%04x", (unsigned)*p);
        } else {
            s_firmware_json[n++] = (char)*p;
        }
    }
    s_firmware_json[n++] = '"';
    s_firmware_json[n] = '\0';
}

static void cleanup_start_failure(void)
{
    if (s_server) {
        (void)httpd_stop(s_server);
        s_server = NULL;
    }
    if (s_telemetry_task) {
        vTaskDelete(s_telemetry_task);
        s_telemetry_task = NULL;
    }
    if (s_wifi_started) (void)esp_wifi_stop();
    s_wifi_started = false;
    if (s_wifi_initialized) (void)esp_wifi_deinit();
    s_wifi_initialized = false;
    if (s_ap) {
        esp_netif_destroy_default_wifi(s_ap);
        s_ap = NULL;
    }
    if (s_created_event_loop) (void)esp_event_loop_delete_default();
    s_created_event_loop = false;
    /* NVS and TCP/IP are global facilities. Do not erase or deinitialize
     * storage or tear down a global TCP/IP stack owned by the application.
     */
}

/* Large waveform JSON trees must not consume the internal heap reserved for
 * Wi-Fi/task stacks. Hooks are installed once, before any HTTP task exists. */
static void *web_json_allocate(size_t size)
{
    void *memory=heap_caps_malloc(size,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if(!memory)memory=heap_caps_malloc(size,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    return memory;
}
esp_err_t diagnostic_web_start(void)
{
    if (s_started) return ESP_ERR_INVALID_STATE;
    cJSON_Hooks hooks={.malloc_fn=web_json_allocate,.free_fn=heap_caps_free};
    cJSON_InitHooks(&hooks);
    /* In particular, do NOT erase NVS on NO_FREE_PAGES/NEW_VERSION_FOUND. */
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) return err;
    /* Separate bounded calibration metadata; preserve the original NVS and app
     * offsets so older histories/configuration and rollback stay readable. */
    esp_err_t metadata_err=nvs_flash_init_partition("wb_meta");
    if(metadata_err!=ESP_OK)ESP_LOGW("web","assessment storage not ready: %s",esp_err_to_name(metadata_err));
    err = esp_netif_init();
    if (err != ESP_OK) return err;
    err = esp_event_loop_create_default();
    if (err == ESP_OK) s_created_event_loop = true;
    else if (err != ESP_ERR_INVALID_STATE) return err;

    /* Avoid esp_netif_create_default_wifi_ap(): that convenience helper
     * asserts/aborts on failure instead of returning an acquisition-safe error.
     */
    const esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_WIFI_AP();
    s_ap = esp_netif_new(&netif_config);
    if (!s_ap) { err = ESP_ERR_NO_MEM; goto fail; }
    err = esp_netif_attach_wifi_ap(s_ap);
    if (err != ESP_OK) goto fail;
    err = esp_wifi_set_default_wifi_ap_handlers();
    if (err != ESP_OK) goto fail;
    err = esp_netif_dhcps_stop(s_ap);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) goto fail;
    const esp_netif_ip_info_t ip = {
        .ip = {.addr = ESP_IP4TOADDR(192, 168, 4, 1)},
        .gw = {.addr = ESP_IP4TOADDR(192, 168, 4, 1)},
        .netmask = {.addr = ESP_IP4TOADDR(255, 255, 255, 0)},
    };
    err = esp_netif_set_ip_info(s_ap, &ip);
    if (err != ESP_OK) goto fail;
    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&wifi_init);
    if (err != ESP_OK) goto fail;
    s_wifi_initialized = true;
    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) goto fail;
    wifi_config_t wifi = {.ap = {
        .ssid = AP_SSID,
        .password = AP_PASSWORD,
        .ssid_len = sizeof(AP_SSID) - 1U,
        .channel = 1,
        .authmode = WIFI_AUTH_WPA2_PSK,
        .max_connection = 3,
        .pmf_cfg = {.required = false},
    }};
    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) goto fail;
    err = esp_wifi_set_config(WIFI_IF_AP, &wifi);
    if (err != ESP_OK) goto fail;
    err = esp_wifi_start();
    if (err != ESP_OK) goto fail;
    s_wifi_started = true;
    err = esp_netif_dhcps_start(s_ap);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) goto fail;

    if(!s_boot_id[0])snprintf(s_boot_id, sizeof(s_boot_id), "%08" PRIx32 "%08" PRIx32, esp_random(), esp_random());
    make_firmware_json();
    err=capture_export_start(s_boot_id,esp_app_get_description()->version);if(err!=ESP_OK)goto fail;
    if (xTaskCreatePinnedToCore(telemetry_task, "web_telemetry", TELEMETRY_STACK_BYTES,
                              NULL, 4, &s_telemetry_task, 1) != pdPASS) {
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    if(!s_live_json)s_live_json=heap_caps_malloc(LIVE_JSON_CAPACITY,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if(!s_live_json){err=ESP_ERR_NO_MEM;goto fail;}
    httpd_config_t http = HTTPD_DEFAULT_CONFIG();
    http.task_priority = 3;
    http.core_id = 1;
    http.stack_size = HTTP_STACK_BYTES + 2048U;
    http.max_open_sockets = 4; /* bounded: two polls + one async CSV + spare */
    http.max_uri_handlers = 12;
    http.backlog_conn = 4;
    http.lru_purge_enable = true;
    http.recv_wait_timeout = 2;
    http.send_wait_timeout = 2;
    err = httpd_start(&s_server, &http);
    if (err != ESP_OK) goto fail;
    const httpd_uri_t root_uri = {.uri = "/", .method = HTTP_GET, .handler = index_get};
    const httpd_uri_t live_uri = {.uri = "/api/live", .method = HTTP_GET, .handler = live_get};
    err = httpd_register_uri_handler(s_server, &root_uri);
    if (err != ESP_OK) goto fail;
    err = httpd_register_uri_handler(s_server, &live_uri);
    if (err != ESP_OK) goto fail;
    const httpd_uri_t raw_read={.uri="/api/raw",.method=HTTP_GET,.handler=raw_get};
    const httpd_uri_t raw_write={.uri="/api/raw",.method=HTTP_POST,.handler=raw_post};
    err=httpd_register_uri_handler(s_server,&raw_read);if(err!=ESP_OK)goto fail;
    err=httpd_register_uri_handler(s_server,&raw_write);if(err!=ESP_OK)goto fail;
    err=capture_export_register(s_server);if(err!=ESP_OK)goto fail;
    err=product_http_register(s_server,s_boot_id);if(err!=ESP_OK)goto fail;
    s_started = true;
    serial_console_printf("WEB_AP ssid=%s ip=%s auth=WPA2 display_hz=%u history=%u page_max=%u\n",
                          AP_SSID, AP_IP, DISPLAY_HZ, HISTORY_POINTS, RESPONSE_POINTS);
    return ESP_OK;
fail:
    cleanup_start_failure();
    return err;
}

static bool usb_record(const char *line,size_t n)
{
    const int64_t deadline=esp_timer_get_time()+2000000;
    do{if(serial_console_try_record(line,n))return true;vTaskDelay(1);}while(esp_timer_get_time()<deadline);
    return false;
}
const char *diagnostic_web_boot_id(void){return s_started?s_boot_id:NULL;}
esp_err_t diagnostic_web_trace_to_serial(void)
{
    if(!s_started)return ESP_ERR_INVALID_STATE;
    display_point_t *points=heap_caps_malloc(RESPONSE_POINTS*sizeof(*points),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if(!points)return ESP_ERR_NO_MEM;
    telemetry_status_t status;size_t count;
    portENTER_CRITICAL(&s_lock);
    status=s_status;count=s_count<RESPONSE_POINTS?s_count:RESPONSE_POINTS;
    const size_t begin=(s_next+HISTORY_POINTS-count)%HISTORY_POINTS;
    const size_t first=count<HISTORY_POINTS-begin?count:HISTORY_POINTS-begin;
    memcpy(points,s_history+begin,first*sizeof(*points));
    if(count>first)memcpy(points+first,s_history,(count-first)*sizeof(*points));
    portEXIT_CRITICAL(&s_lock);
    char record[384];esp_err_t err=ESP_OK;
    int n=snprintf(record,sizeof(record),"WEB_TRACE_BEGIN source=live_device boot_id=%s count=%u i2c_err=%" PRIu32 " fifo_loss=%" PRIu32 " epoch=%" PRIu32 "\n",s_boot_id,(unsigned)count,status.imu.i2c_errors,status.imu.fifo_loss_resets,status.imu.epoch);
    if(n<0||(size_t)n>=sizeof(record)||!usb_record(record,(size_t)n))err=ESP_ERR_TIMEOUT;
    const int64_t deadline=esp_timer_get_time()+INT64_C(3000000);size_t emitted=0;
    while(err==ESP_OK&&emitted<count){const display_point_t *p=&points[emitted];
        if(esp_timer_get_time()>deadline){err=ESP_ERR_TIMEOUT;break;}
        n=snprintf(record,sizeof(record),"WEB_POINT index=%u id=%" PRIu64 " t_us=%" PRId64 " source_seq=%" PRIu64 " epoch=%" PRIu32 " gap_before=%d x=%d y=%d z=%d level=%d\n",(unsigned)emitted,p->id,p->t_us,p->source_seq,p->epoch,p->gap_before,p->x,p->y,p->z,p->level);
        if(n<0||(size_t)n>=sizeof(record)||!usb_record(record,(size_t)n)){err=ESP_ERR_TIMEOUT;break;}
        ++emitted;if(!(emitted%8))vTaskDelay(1);
    }
    heap_caps_free(points);
    n=snprintf(record,sizeof(record),"WEB_TRACE_END count=%u result=%s\n",(unsigned)emitted,esp_err_to_name(err));
    if(!usb_record(record,(size_t)n))return ESP_ERR_TIMEOUT;
    return err;
}
esp_err_t diagnostic_web_raw_export_to_serial(uint32_t id){return capture_export_usb(id,false);}
esp_err_t diagnostic_web_pose_export_to_serial(uint32_t id){return capture_export_usb(id,true);}
esp_err_t diagnostic_web_csv_selfcheck(uint32_t id,int pose)
{
    raw_status_t raw;esp_err_t err=raw_capture_export_begin(id,&raw);if(err!=ESP_OK)return err;
    char url[128],buffer[512],first[64]={0},tail[8]={0};size_t used=0,bytes=0;uint32_t crc=UINT32_MAX;
    snprintf(url,sizeof(url),"http://" AP_IP "/api/%s.csv?id=%" PRIu32 "&boot_id=%s",pose?"raw-pose":"raw",id,s_boot_id);
    const esp_http_client_config_t config={.url=url,.timeout_ms=3000,.buffer_size=512,.buffer_size_tx=512};
    esp_http_client_handle_t client=esp_http_client_init(&config);
    if(!client){raw_capture_export_end(id);return ESP_ERR_NO_MEM;}
    serial_console_printf("CSV_HTTP_CHECK_BEGIN id=%" PRIu32 " pose=%d\n",id,!!pose);
    err=esp_http_client_open(client,0);
    if(err==ESP_OK&&(esp_http_client_fetch_headers(client)<0||esp_http_client_get_status_code(client)!=200))err=ESP_ERR_INVALID_RESPONSE;
    const int64_t deadline=esp_timer_get_time()+30000000;
    while(err==ESP_OK){
        if(esp_timer_get_time()>deadline){err=ESP_ERR_TIMEOUT;break;}
        int got=esp_http_client_read(client,buffer,sizeof(buffer));if(got<0){err=ESP_FAIL;break;}if(!got)break;
        size_t n=(size_t)got;if(used<sizeof(first)-1){size_t take=sizeof(first)-1-used;if(take>n)take=n;memcpy(first+used,buffer,take);used+=take;}
        if(n>=sizeof(tail))memcpy(tail,buffer+n-sizeof(tail),sizeof(tail));
        else{memmove(tail,tail+n,sizeof(tail)-n);memcpy(tail+sizeof(tail)-n,buffer,n);}
        for(size_t i=0;i<n;++i){crc^=(unsigned char)buffer[i];for(unsigned b=0;b<8;++b)crc=(crc>>1)^((crc&1)?UINT32_C(0xedb88320):0);}
        bytes+=n;vTaskDelay(1);
    }
    const char *prefix=pose?"# schema=wheelbalance_pose_v1\n":"# schema=wheelbalance_raw_v1\n";
    if(err==ESP_OK&&(!esp_http_client_is_complete_data_received(client)||bytes<8||strncmp(first,prefix,strlen(prefix))||memcmp(tail,"# end=1\n",8)))err=ESP_ERR_INVALID_RESPONSE;
    esp_http_client_close(client);esp_http_client_cleanup(client);raw_capture_export_end(id);
    serial_console_printf("CSV_HTTP_CHECK_END id=%" PRIu32 " pose=%d bytes=%u crc32=%08" PRIx32 " result=%s\n",id,!!pose,(unsigned)bytes,crc^UINT32_MAX,esp_err_to_name(err));
    return err;
}

#if CONFIG_DIAGNOSTIC_WEB_SELFCHECK
static bool selfcheck_json(const char *body,const char *path)
{
    cJSON *root=cJSON_ParseWithOpts(body,NULL,true);
    if(!root)return false;
    const cJSON *schema=cJSON_GetObjectItemCaseSensitive(root,"schema_version");
    if(!strcmp(path,"/api/session")||!strcmp(path,"/api/session/report")) {
        const cJSON *s=cJSON_GetObjectItemCaseSensitive(root,"session");
        bool valid=cJSON_IsNumber(schema)&&schema->valueint==2&&cJSON_IsObject(s)&&
            cJSON_IsNumber(cJSON_GetObjectItemCaseSensitive(s,"id"))&&
            cJSON_IsFalse(cJSON_GetObjectItemCaseSensitive(s,"accuracy_validated"))&&
            cJSON_IsObject(cJSON_GetObjectItemCaseSensitive(s,"result"));
        const cJSON *g=cJSON_GetObjectItemCaseSensitive(s,"trial_guide");
        const cJSON *source=cJSON_GetObjectItemCaseSensitive(s,"source");
        valid=valid&&cJSON_IsObject(g)&&cJSON_IsBool(cJSON_GetObjectItemCaseSensitive(g,"available"))&&cJSON_IsString(source);
        if(valid&&strcmp(source->valuestring,"synthetic")){
            const cJSON *planned=cJSON_GetObjectItemCaseSensitive(g,"planned_mass_g"),*piece=cJSON_GetObjectItemCaseSensitive(g,"piece_mass_g");
            const cJSON *attempts=cJSON_GetObjectItemCaseSensitive(g,"attempt_count"),*revision=cJSON_GetObjectItemCaseSensitive(g,"snapshot_revision");
            const cJSON *current_revision=cJSON_GetObjectItemCaseSensitive(s,"revision");
            const cJSON *config=cJSON_GetObjectItemCaseSensitive(s,"config"),*mass=cJSON_GetObjectItemCaseSensitive(config,"trial_mass_g");
            const cJSON *plan_source=cJSON_GetObjectItemCaseSensitive(g,"plan_source");
            valid=cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(g,"available"))&&cJSON_IsNumber(planned)&&cJSON_IsNumber(mass)&&
                planned->valuedouble==mass->valuedouble&&cJSON_IsNumber(piece)&&piece->valuedouble>0&&
                cJSON_IsNumber(attempts)&&attempts->valueint>=0&&attempts->valueint<=3&&cJSON_IsString(plan_source)&&
                cJSON_IsNumber(revision)&&cJSON_IsNumber(current_revision)&&revision->valuedouble==current_revision->valuedouble;
            if(valid)serial_console_printf("TRIAL_GUIDE_SELFTEST scope=local_server plan_source=%s planned_mass_g=%.3f piece_mass_g=%.3f attempts=%d revision_bound=1 session_unchanged=1\n",
                plan_source->valuestring,planned->valuedouble,piece->valuedouble,attempts->valueint);
        }
        const cJSON *retests=cJSON_GetObjectItemCaseSensitive(s,"retest_history");
        if(valid&&cJSON_IsObject(retests)){
            const cJSON *entries=cJSON_GetObjectItemCaseSensitive(retests,"entries");
            const cJSON *retained=cJSON_GetObjectItemCaseSensitive(retests,"retained_count");
            const cJSON *variable=cJSON_GetObjectItemCaseSensitive(retests,"group_variable_observed");
            valid=cJSON_IsArray(entries)&&cJSON_IsNumber(retained)&&retained->valueint>=0&&retained->valueint<=6&&
                cJSON_GetArraySize(entries)==retained->valueint&&cJSON_IsBool(variable);
            if(valid)serial_console_printf("RETEST_HISTORY_SELFTEST scope=local_server retained=%d group_variable=%d schema=1\n",
                retained->valueint,cJSON_IsTrue(variable));
        }
        const cJSON *speed=cJSON_GetObjectItemCaseSensitive(s,"speed_profile");
        const cJSON *available=cJSON_GetObjectItemCaseSensitive(speed,"available");
        valid=valid&&cJSON_IsObject(speed)&&cJSON_IsBool(available)&&
            cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(speed,"display_only"))&&
            cJSON_IsFalse(cJSON_GetObjectItemCaseSensitive(speed,"accuracy_validated"));
        if(valid&&cJSON_IsTrue(available)){
            const cJSON *stages=cJSON_GetObjectItemCaseSensitive(speed,"stages");
            valid=cJSON_IsArray(stages)&&cJSON_GetArraySize(stages)==3;
            for(unsigned i=0;valid&&i<3;++i){const cJSON *a=cJSON_GetArrayItem(stages,(int)i);
                const cJSON *points=cJSON_GetObjectItemCaseSensitive(a,"points");
                const cJSON *state=cJSON_GetObjectItemCaseSensitive(a,"state");
                valid=cJSON_IsString(state)&&cJSON_IsArray(points)&&cJSON_GetArraySize(points)<=14;
                if(valid)serial_console_printf("SPEED_PROFILE_SELFTEST scope=local_server stage=%u state=%s bins=%d display_only=1\n",
                    i,state->valuestring,cJSON_GetArraySize(points));
            }
        }
        const cJSON *flow=cJSON_GetObjectItemCaseSensitive(s,"capture_flow");
        valid=valid&&cJSON_IsObject(flow)&&cJSON_IsBool(cJSON_GetObjectItemCaseSensitive(flow,"available"))&&
            cJSON_IsFalse(cJSON_GetObjectItemCaseSensitive(flow,"hand_release_confirmed"));
        if(valid&&cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(flow,"available"))){
            const cJSON *stages=cJSON_GetObjectItemCaseSensitive(flow,"stages");valid=cJSON_IsArray(stages)&&cJSON_GetArraySize(stages)==3;
            for(unsigned i=0;valid&&i<3;++i){const cJSON *a=cJSON_GetArrayItem(stages,(int)i),*phase=cJSON_GetObjectItemCaseSensitive(a,"phase");
                const cJSON *used=cJSON_GetObjectItemCaseSensitive(a,"usable_turns"),*skipped=cJSON_GetObjectItemCaseSensitive(a,"skipped_turns");
                valid=cJSON_IsString(phase)&&cJSON_IsNumber(used)&&used->valueint>=0&&used->valueint<=64&&cJSON_IsNumber(skipped);
                if(valid)serial_console_printf("CAPTURE_FLOW_SELFTEST scope=local_server stage=%u phase=%s used=%d skipped=%d\n",i,phase->valuestring,used->valueint,skipped->valueint);
            }
        }
        cJSON_Delete(root);return valid;
    }
    bool ok=cJSON_IsNumber(schema) && schema->valueint==1;
    if(!strncmp(path,"/api/live",9)) {
        const cJSON *source=cJSON_GetObjectItemCaseSensitive(root,"source");
        const cJSON *features=cJSON_GetObjectItemCaseSensitive(root,"features");
        ok=ok&&cJSON_IsString(source)&&!strcmp(source->valuestring,"live_device")&&
            cJSON_IsFalse(cJSON_GetObjectItemCaseSensitive(features,"sync_1x_live"))&&
            cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(features,"product_session"));
    } else if(!strcmp(path,"/api/raw")) {
        const cJSON *r=cJSON_GetObjectItemCaseSensitive(root,"raw_capture");
        ok=ok && cJSON_IsObject(r) && cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(r,"enabled")) &&
            cJSON_IsFalse(cJSON_GetObjectItemCaseSensitive(r,"measurement_qualified"));
    } else ok=false;
    cJSON_Delete(root);return ok;
}

static esp_err_t selfcheck_get(const char *path, bool json)
{
    enum { JSON_LIMIT=32768 };
    char *body=json ? heap_caps_malloc(JSON_LIMIT+1,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT) : NULL;
    if(json && !body)body=heap_caps_malloc(JSON_LIMIT+1,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    if(json && !body)return ESP_ERR_NO_MEM;
    char url[80];
    snprintf(url, sizeof(url), "http://" AP_IP "%s", path);
    const esp_http_client_config_t config = {
        .url = url, .timeout_ms = 2000, .buffer_size = 512, .buffer_size_tx = 512,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {heap_caps_free(body);return ESP_ERR_NO_MEM;}
    esp_err_t err = esp_http_client_open(client, 0);
    int status = 0;
    size_t bytes = 0;
    char buf[512];
    if (err == ESP_OK && esp_http_client_fetch_headers(client) < 0) err = ESP_FAIL;
    if (err == ESP_OK) {
        status = esp_http_client_get_status_code(client);
        if (status != 200) err = ESP_ERR_INVALID_RESPONSE;
    }
    const int64_t deadline = esp_timer_get_time() + INT64_C(5000000);
    while (err == ESP_OK) {
        if (esp_timer_get_time() > deadline) { err = ESP_ERR_TIMEOUT; break; }
        const int got = esp_http_client_read(client, buf, sizeof(buf));
        if (got < 0) { err = ESP_FAIL; break; }
        if (!got) break;
        if(json) {
            if((size_t)got>JSON_LIMIT-bytes){err=ESP_ERR_INVALID_SIZE;break;}
            memcpy(body+bytes,buf,(size_t)got);
        }
        bytes += (size_t)got;
    }
    if (err == ESP_OK && (!bytes || !esp_http_client_is_complete_data_received(client)))
        err = ESP_ERR_INVALID_RESPONSE;
    if(err==ESP_OK && json) {
        body[bytes]='\0';
        if(!selfcheck_json(body,path))err=ESP_ERR_INVALID_RESPONSE;
    }
    (void)esp_http_client_close(client);
    (void)esp_http_client_cleanup(client);
    heap_caps_free(body);
    serial_console_printf("HTTP_SELFTEST scope=local_server path=%s status=%d bytes=%u result=%s json_checked=%d phone_verified=0\n",
                          path, status, (unsigned)bytes, esp_err_to_name(err),json&&err==ESP_OK);
    return err;
}
#endif

esp_err_t diagnostic_web_run_selfcheck(void)
{
#if CONFIG_DIAGNOSTIC_WEB_SELFCHECK
    if (!s_started) return ESP_ERR_INVALID_STATE;
    esp_err_t html = selfcheck_get("/", false);
    esp_err_t api = selfcheck_get("/api/live?after=0", true);
    esp_err_t raw_api = selfcheck_get("/api/raw", true);
    esp_err_t product_api=selfcheck_get("/api/session",true);
    esp_err_t product_report=selfcheck_get("/api/session/report",true);
    esp_err_t product_js=selfcheck_get("/product.js",false);
    esp_err_t product_css=selfcheck_get("/product.css",false);
    esp_err_t speed_js=selfcheck_get("/speed.js",false);
    const esp_err_t results[]={html,api,raw_api,product_api,product_report,product_js,product_css,speed_js};
    esp_err_t result=ESP_OK;for(unsigned i=0;i<8;++i)if(results[i]!=ESP_OK)result=results[i];
    serial_console_printf("WEB_SELFCHECK_END routes=8 result=%s session_unchanged=1\n",esp_err_to_name(result));
    return result;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t diagnostic_web_load_selfcheck(uint32_t requests)
{
#if CONFIG_DIAGNOSTIC_WEB_SELFCHECK
    if(!s_started||!requests||requests>1200)return ESP_ERR_INVALID_ARG;
    unsigned passed=0,failed=0,min_free=UINT32_MAX,min_largest=UINT32_MAX;
    int64_t max_elapsed=0;
    serial_console_printf("WEB_LOAD_BEGIN requests=%" PRIu32 " source=local_server phone_verified=0 session_unchanged=1\n",requests);
    for(uint32_t i=0;i<requests;++i){
        const int64_t begin=esp_timer_get_time();
        const esp_err_t result=selfcheck_get(i%6==5?"/api/session":"/api/live?after=0",true);
        const int64_t elapsed=esp_timer_get_time()-begin;
        if(result==ESP_OK)++passed;else ++failed;
        if(elapsed>max_elapsed)max_elapsed=elapsed;
        const unsigned free=heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
        const unsigned largest=heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
        if(free<min_free)min_free=free;
        if(largest<min_largest)min_largest=largest;
        if((i+1)%60==0||result!=ESP_OK)serial_console_printf(
            "WEB_LOAD_PROGRESS done=%" PRIu32 " passed=%u failed=%u internal_free=%u internal_largest=%u max_request_us=%" PRIi64 "\n",
            i+1,passed,failed,free,largest,max_elapsed);
        if(failed>=3)break;
        if(elapsed<200000)vTaskDelay(pdMS_TO_TICKS((200000-elapsed+999)/1000));
    }
    serial_console_printf("WEB_LOAD_END requested=%" PRIu32 " passed=%u failed=%u min_after_request_free=%u min_after_request_largest=%u max_request_us=%" PRIi64 " source=local_server phone_verified=0 session_unchanged=1\n",
        requests,passed,failed,min_free,min_largest,max_elapsed);
    return passed==requests?ESP_OK:ESP_FAIL;
#else
    (void)requests;return ESP_ERR_NOT_SUPPORTED;
#endif
}

#else

esp_err_t diagnostic_web_start(void) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t diagnostic_web_run_selfcheck(void) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t diagnostic_web_link_status_to_serial(void){return ESP_ERR_NOT_SUPPORTED;}
esp_err_t diagnostic_web_load_selfcheck(uint32_t requests){(void)requests;return ESP_ERR_NOT_SUPPORTED;}
esp_err_t diagnostic_web_raw_export_to_serial(uint32_t id) { (void)id;return ESP_ERR_NOT_SUPPORTED; }
esp_err_t diagnostic_web_pose_export_to_serial(uint32_t id) { (void)id;return ESP_ERR_NOT_SUPPORTED; }
esp_err_t diagnostic_web_csv_selfcheck(uint32_t id,int pose){(void)id;(void)pose;return ESP_ERR_NOT_SUPPORTED;}
esp_err_t diagnostic_web_trace_to_serial(void){return ESP_ERR_NOT_SUPPORTED;}
const char *diagnostic_web_boot_id(void){return NULL;}

#endif
