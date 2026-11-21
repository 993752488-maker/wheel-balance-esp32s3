#include "mpu6050.h"
#include "fifo_guard.h"
#include "esp_timer.h"
#include <inttypes.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

enum {
    REG_SMPLRT_DIV = 0x19, REG_CONFIG = 0x1a, REG_GYRO_CONFIG = 0x1b,
    REG_ACCEL_CONFIG = 0x1c, REG_ACCEL_CONFIG2 = 0x1d,
    REG_FIFO_EN = 0x23, REG_INT_PIN_CFG = 0x37,
    REG_INT_ENABLE = 0x38, REG_INT_STATUS = 0x3a,
    REG_ACCEL_XOUT_H = 0x3b, REG_USER_CTRL = 0x6a,
    REG_PWR_MGMT_1 = 0x6b, REG_PWR_MGMT_2 = 0x6c, REG_FIFO_COUNTH = 0x72,
    REG_FIFO_RW = 0x74, REG_WHO_AM_I = 0x75,
};

enum { IDENTITY_READ_COUNT = 16, QUALIFICATION_I2C_MAX_HZ = 100000 };

#if CONFIG_V0_MPU_INT_OPEN_DRAIN
#define MPU_INT_PIN_SETTING 0xc0 /* INT_LEVEL=1, INT_OPEN=1, pulse mode. */
#else
#define MPU_INT_PIN_SETTING 0x00
#endif

static const char *TAG = "imu_profile";

#if CONFIG_MPU_POSE_DIAG_ENABLED
#define MPU6050_DLPF 0x06
#define MPU6050_PROFILE_NAME "mpu6050_pose_diag_5hz"
#define MPU6050_FILTER_NAME "CONFIG.DLPF_CFG=6/5Hz_pose_diag"
#else
#define MPU6050_DLPF 0x02
#define MPU6050_PROFILE_NAME "mpu6050"
#define MPU6050_FILTER_NAME "CONFIG.DLPF_CFG=2/94Hz"
#endif

typedef struct { uint8_t reg, value; } register_setting_t;

typedef struct {
    uint8_t who_am_i;
    uint16_t fifo_capacity_bytes;
    bool expected_identity;
    const char *name;
    const char *accel_filter;
    const register_setting_t *settings;
    size_t settings_count;
} register_profile_t;

/* MPU-6000/MPU-6050 register map: CONFIG controls accel and gyro DLPF.
 * No write to 0x1d: it is not ACCEL_CONFIG2 in this register model. */
static const register_setting_t mpu6050_settings[] = {
    {REG_INT_ENABLE, 0x00}, {REG_FIFO_EN, 0x00}, {REG_USER_CTRL, 0x00},
    {REG_PWR_MGMT_1, 0x01}, /* CLKSEL=1: X gyro PLL reference on MPU6050. */
    {REG_PWR_MGMT_2, 0x00}, {REG_CONFIG, MPU6050_DLPF},
    {REG_GYRO_CONFIG, 0x00}, {REG_ACCEL_CONFIG, 0x08},
    {REG_INT_PIN_CFG, MPU_INT_PIN_SETTING},
};

/* Candidate MPU6500 model, RM-MPU-6500A-00 rev. 2.1.
 * GYRO_CONFIG.FCHOICE_B=00 enables the CONFIG DLPF/divider path.
 * ACCEL_CONFIG2.ACCEL_FCHOICE_B=0 enables its independent accel DLPF.
 * A_DLPF_CFG=2 is specified as 92 Hz, 7.80 ms delay, 1 kHz internal rate.
 * WHO=0x70 alone does not prove the physical chip is an authentic MPU6500. */
static const register_setting_t mpu6500_candidate_settings[] = {
    {REG_INT_ENABLE, 0x00}, {REG_FIFO_EN, 0x00}, {REG_USER_CTRL, 0x00},
    {REG_PWR_MGMT_1, 0x01}, /* CLKSEL=1: auto-select best clock/PLL on MPU6500. */
    {REG_PWR_MGMT_2, 0x00}, {REG_CONFIG, 0x02},
    {REG_GYRO_CONFIG, 0x00}, {REG_ACCEL_CONFIG, 0x08},
    {REG_ACCEL_CONFIG2, 0x02}, {REG_INT_PIN_CFG, MPU_INT_PIN_SETTING},
};

static const register_profile_t profiles[] = {
    {0x68, 1024, true, MPU6050_PROFILE_NAME, MPU6050_FILTER_NAME,
     mpu6050_settings, sizeof(mpu6050_settings) / sizeof(mpu6050_settings[0])},
    {0x70, 512, false, "mpu6500_register_model_candidate",
     "ACCEL_CONFIG2.A_DLPF_CFG=2/92Hz_RM2.1", mpu6500_candidate_settings,
     sizeof(mpu6500_candidate_settings) / sizeof(mpu6500_candidate_settings[0])},
};

static const register_profile_t *find_profile(uint8_t who_am_i)
{
    for (size_t i = 0; i < sizeof(profiles) / sizeof(profiles[0]); ++i) {
        if (profiles[i].who_am_i == who_am_i) return &profiles[i];
    }
    return NULL;
}

static bool fully_configured(const mpu6050_t *d)
{
    if (!d || !d->device || !d->sample_hz ||
        d->identity_reads != IDENTITY_READ_COUNT) return false;
    const register_profile_t *profile = find_profile(d->who_am_i);
    return profile && d->fifo_capacity_bytes == profile->fifo_capacity_bytes;
}

static void delay_ms(uint32_t milliseconds)
{
    TickType_t ticks = pdMS_TO_TICKS(milliseconds);
    vTaskDelay(ticks ? ticks : 1);
}

static esp_err_t reg_read(mpu6050_t *d, uint8_t reg, uint8_t *data, size_t n)
{
    const esp_err_t err = i2c_master_transmit_receive(d->device, &reg, 1, data, n, 50);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_read_failed reg=0x%02x nbytes=%u err=%s(0x%x) bus_hz=%" PRIu32,
                 (unsigned)reg, (unsigned)n, esp_err_to_name(err), (unsigned)err,
                 d->i2c_speed_hz);
    }
    return err;
}

static esp_err_t reg_write(mpu6050_t *d, uint8_t reg, uint8_t value)
{
    const uint8_t data[] = {reg, value};
    const esp_err_t err = i2c_master_transmit(d->device, data, sizeof(data), 50);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_write_failed reg=0x%02x nbytes=%u value=0x%02x err=%s(0x%x) bus_hz=%" PRIu32,
                 (unsigned)reg, (unsigned)sizeof(data), (unsigned)value,
                 esp_err_to_name(err), (unsigned)err, d->i2c_speed_hz);
    }
    return err;
}

static uint8_t persistent_mask(const mpu6050_t *d, uint8_t reg)
{
    const bool is_mpu6050 = d->who_am_i == 0x68;
    switch (reg) {
    case REG_CONFIG: return is_mpu6050 ? 0x3f : 0x7f;
    case REG_GYRO_CONFIG: return is_mpu6050 ? 0xf8 : 0xfb;
    case REG_ACCEL_CONFIG: return is_mpu6050 ? 0xff : 0xf8;
    case REG_ACCEL_CONFIG2: return 0x0f;
    case REG_PWR_MGMT_1: return 0x7f; /* DEVICE_RESET self-clears separately. */
    case REG_PWR_MGMT_2: return is_mpu6050 ? 0xff : 0x3f;
    case REG_INT_PIN_CFG: return 0xfe;
    case REG_INT_ENABLE: return is_mpu6050 ? 0x19 : 0x59;
    case REG_USER_CTRL: return 0x70; /* Reset bits are checked separately. */
    default: return 0xff;
    }
}

static esp_err_t write_checked(mpu6050_t *d, uint8_t reg, uint8_t value)
{
    esp_err_t err = reg_write(d, reg, value);
    if (err != ESP_OK) return err;
    uint8_t actual = 0;
    err = reg_read(d, reg, &actual, 1);
    if (err != ESP_OK) return err;
    const uint8_t mask = persistent_mask(d, reg);
    if ((actual & mask) != (value & mask)) {
        ESP_LOGE(TAG, "register_readback_mismatch reg=0x%02x expected=0x%02x actual=0x%02x mask=0x%02x profile=%s",
                 (unsigned)reg, (unsigned)value, (unsigned)actual, (unsigned)mask,
                 d->profile_name);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

esp_err_t mpu6050_create(mpu6050_t *d, int sda, int scl,
                        uint8_t address, uint32_t speed)
{
    if (!d || sda == scl || (address != 0x68 && address != 0x69) ||
        speed == 0 || speed > 400000) return ESP_ERR_INVALID_ARG;
    memset(d, 0, sizeof(*d));
    d->i2c_speed_hz = speed;
    d->profile_name = "unprobed";
    const i2c_master_bus_config_t bus = {
        .i2c_port = I2C_NUM_0, .sda_io_num = sda, .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus, &d->bus);
    if (err != ESP_OK) return err;
    const i2c_device_config_t device = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = address,
        .scl_speed_hz = speed,
    };
    err = i2c_master_bus_add_device(d->bus, &device, &d->device);
    if (err != ESP_OK) {
        i2c_del_master_bus(d->bus);
        d->bus = NULL;
        d->device = NULL;
    }
    return err;
}

static esp_err_t fifo_reset_identified(mpu6050_t *d)
{
    esp_err_t err = write_checked(d, REG_FIFO_EN, 0x00);
    if (err != ESP_OK) return err;
    err = write_checked(d, REG_USER_CTRL, 0x00);
    if (err != ESP_OK) return err;
    err = reg_write(d, REG_USER_CTRL, 0x04); /* FIFO reset; self-clearing */
    if (err != ESP_OK) return err;
    uint8_t user_ctrl = 0;
    bool reset_finished = false;
    for (unsigned i = 0; i < 10; ++i) {
        delay_ms(2);
        err = reg_read(d, REG_USER_CTRL, &user_ctrl, 1);
        if (err != ESP_OK) return err;
        if ((user_ctrl & 0x77) == 0x00) {
            reset_finished = true;
            break;
        }
    }
    if (!reset_finished) {
        ESP_LOGE(TAG, "fifo_reset_incomplete user_ctrl=0x%02x profile=%s",
                 (unsigned)user_ctrl, d->profile_name);
        return ESP_ERR_INVALID_RESPONSE;
    }
    uint8_t count[2] = {0};
    err = reg_read(d, REG_FIFO_COUNTH, count, sizeof(count));
    if (err != ESP_OK) return err;
    if (count[0] != 0 || count[1] != 0) return ESP_ERR_INVALID_RESPONSE;
    err = write_checked(d, REG_USER_CTRL, 0x40); /* FIFO enabled */
    if (err != ESP_OK) return err;
    return write_checked(d, REG_FIFO_EN, 0x08); /* XYZ accel only, 6 B/frame */
}

esp_err_t mpu6050_fifo_reset(mpu6050_t *d)
{
    if (!d || !d->device) return ESP_ERR_INVALID_ARG;
    if (!fully_configured(d)) return ESP_ERR_INVALID_STATE;
    const esp_err_t err = fifo_reset_identified(d);
    if (err != ESP_OK) d->sample_hz = 0;
    return err;
}

esp_err_t mpu6050_fifo_pause(mpu6050_t *d)
{
    if(!d||!d->device)return ESP_ERR_INVALID_ARG;
    if(!fully_configured(d))return ESP_ERR_INVALID_STATE;
    const esp_err_t err=write_checked(d,REG_FIFO_EN,0x00);
    if(err!=ESP_OK)d->sample_hz=0;
    return err;
}

esp_err_t mpu6050_configure(mpu6050_t *d, uint16_t sample_hz)
{
    if (!d || !d->device) return ESP_ERR_INVALID_ARG;
    d->sample_hz = 0;
    d->fifo_capacity_bytes = 0;
    d->profile_name = "identity_probe_incomplete";
    d->expected_identity = false;
    d->identity_reads = 0;
    if (!sample_hz || sample_hz > 1000 ||
        1000 % sample_hz != 0 || 1000 / sample_hz > 256)
        return ESP_ERR_INVALID_ARG;
    if (d->i2c_speed_hz == 0 || d->i2c_speed_hz > QUALIFICATION_I2C_MAX_HZ)
        return ESP_ERR_INVALID_ARG;

    uint8_t stable_id = 0;
    esp_err_t err;
    /* These are separate I2C transactions. No reset/configuration writes are
     * allowed until every pre-configuration identity read is identical. */
    for (uint8_t i = 0; i < IDENTITY_READ_COUNT; ++i) {
        uint8_t actual_id = 0;
        err = reg_read(d, REG_WHO_AM_I, &actual_id, 1);
        if (err != ESP_OK) return err;
        d->who_am_i = actual_id;
        ++d->identity_reads;
        if (i == 0) stable_id = actual_id;
        else if (actual_id != stable_id) {
            d->profile_name = "inconsistent_identity";
            ESP_LOGE(TAG, "identity_inconsistent first=0x%02x actual=0x%02x reads=%u bus_hz=%" PRIu32,
                     (unsigned)stable_id, (unsigned)actual_id,
                     (unsigned)d->identity_reads, d->i2c_speed_hz);
            return ESP_ERR_INVALID_RESPONSE;
        }
        if (i + 1 < IDENTITY_READ_COUNT) delay_ms(2);
    }
    const register_profile_t *profile = find_profile(stable_id);
    if (!profile) {
        d->profile_name = "unsupported_identity";
        ESP_LOGE(TAG, "identity_unsupported who=0x%02x reads=%u bus_hz=%" PRIu32,
                 (unsigned)d->who_am_i, (unsigned)d->identity_reads, d->i2c_speed_hz);
        return ESP_ERR_INVALID_RESPONSE;
    }
#if CONFIG_MPU_POSE_DIAG_ENABLED
    /* This diagnostic's filter and raw scaling are MPU6050-specific.
     * Reject the historical 0x70 candidate before any configuration write. */
    if (stable_id != 0x68) {
        d->profile_name = "pose_diag_requires_mpu6050";
        ESP_LOGE(TAG, "pose diagnostic requires WHO=0x68; actual=0x%02x", (unsigned)stable_id);
        return ESP_ERR_NOT_SUPPORTED;
    }
#endif
    d->fifo_capacity_bytes = profile->fifo_capacity_bytes;
    d->profile_name = profile->name;
    d->expected_identity = profile->expected_identity;
    ESP_LOGI(TAG, "identity_stable who=0x%02x reads=%u profile=%s identity_matches_label=%u fifo_bytes=%u accel_filter=%s bus_hz=%" PRIu32,
             (unsigned)d->who_am_i, (unsigned)d->identity_reads, d->profile_name,
             d->expected_identity ? 1u : 0u, (unsigned)d->fifo_capacity_bytes,
             profile->accel_filter, d->i2c_speed_hz);

    err = reg_write(d, REG_PWR_MGMT_1, 0x80);
    if (err != ESP_OK) return err;
    delay_ms(100);
    uint8_t reset_power = 0;
    err = reg_read(d, REG_PWR_MGMT_1, &reset_power, 1);
    if (err != ESP_OK) return err;
    if (reset_power & 0x80) return ESP_ERR_INVALID_RESPONSE;
    uint8_t reset_id = 0;
    err = reg_read(d, REG_WHO_AM_I, &reset_id, 1);
    if (err != ESP_OK) return err;
    if (reset_id != stable_id) {
        d->who_am_i = reset_id;
        d->profile_name = "identity_changed_after_reset";
        d->fifo_capacity_bytes = 0;
        d->expected_identity = false;
        return ESP_ERR_INVALID_RESPONSE;
    }
    for (size_t i = 0; i < profile->settings_count; ++i) {
        err = write_checked(d, profile->settings[i].reg, profile->settings[i].value);
        if (err != ESP_OK) return err;
    }
    err = write_checked(d, REG_SMPLRT_DIV, (uint8_t)(1000 / sample_hz - 1));
    if (err != ESP_OK) return err;
    delay_ms(100); /* allow gyro/PLL and both models' accel filters to settle */
    err = fifo_reset_identified(d);
    if (err != ESP_OK) return err;
    uint8_t ignored_status = 0;
    err = reg_read(d, REG_INT_STATUS, &ignored_status, 1); /* clear old status */
    if (err != ESP_OK) return err;
#if !CONFIG_V0_MPU_DRDY_ENABLED
    const uint8_t int_enable = 0x10; /* overflow enabled; no DRDY wire required */
    const char *drdy_mode = "FIFO_POLL_OVERFLOW_STATUS";
#elif CONFIG_V0_DIAG_DISABLE_SENSOR_INT
    const uint8_t int_enable = 0x00; /* Diagnostic: sensor output disabled. */
    const char *drdy_mode = "DISABLED_DIAG";
#else
    const uint8_t int_enable = 0x11; /* DATA_RDY and FIFO overflow. */
#if CONFIG_V0_MPU_INT_OPEN_DRAIN
    const char *drdy_mode = "low_open_drain_50us";
#else
    const char *drdy_mode = "high_50us";
#endif
#endif
    err = write_checked(d, REG_INT_ENABLE, int_enable);
    if (err != ESP_OK) return err;
    d->sample_hz = sample_hz;
    ESP_LOGI(TAG, "configuration_readback_ok who=0x%02x profile=%s identity_matches_label=%u sample_hz=%u accel_lsb_g=8192 fifo_bytes=%u drdy=%s int_enable=0x%02x int_pin_cfg=0x%02x bus_hz=%" PRIu32,
             (unsigned)d->who_am_i, d->profile_name, d->expected_identity ? 1u : 0u,
             (unsigned)d->sample_hz, (unsigned)d->fifo_capacity_bytes,
             drdy_mode, (unsigned)int_enable, (unsigned)MPU_INT_PIN_SETTING, d->i2c_speed_hz);
    return ESP_OK;
}

esp_err_t mpu6050_fifo_count(mpu6050_t *d, uint16_t *bytes, uint8_t *status)
{
    if (!d || !d->device || !bytes || !status) return ESP_ERR_INVALID_ARG;
    *bytes = 0;
    *status = 0;
    if (!fully_configured(d)) return ESP_ERR_INVALID_STATE;
    esp_err_t err = reg_read(d, REG_INT_STATUS, status, 1);
    if (err != ESP_OK) {
        d->sample_hz = 0;
        return err;
    }
    if(*status&0x10){*bytes=d->fifo_capacity_bytes;return ESP_OK;}
    uint16_t observations[FIFO_COUNT_MAX_OBSERVATIONS]={0};
    uint64_t times[FIFO_COUNT_MAX_OBSERVATIONS]={0};size_t n=0;
    fifo_count_state_t confirmed=FIFO_COUNT_WAIT;
    while(n<FIFO_COUNT_MAX_OBSERVATIONS) {
        uint8_t count[2];err=reg_read(d,REG_FIFO_COUNTH,count,sizeof(count));
        if(err!=ESP_OK){d->sample_hz=0;return err;}
        observations[n]=((uint16_t)count[0]<<8)|count[1];times[n]=(uint64_t)esp_timer_get_time();
        ++n;confirmed=fifo_count_confirm(observations,times,n,d->fifo_capacity_bytes,d->sample_hz,bytes);
        if(confirmed!=FIFO_COUNT_WAIT)break;
    }
    uint8_t after=0;err=reg_read(d,REG_INT_STATUS,&after,1);*status|=after;
    if(err!=ESP_OK){d->sample_hz=0;return err;}
    if(*status&0x10){*bytes=d->fifo_capacity_bytes;return ESP_OK;}
    if(confirmed!=FIFO_COUNT_READY) {
        ++d->fifo_count_anomalies;d->sample_hz=0;
        ESP_LOGE(TAG,"fifo_count_incoherent reads=%u counts=%u:%u:%u:%u:%u:%u:%u:%u"
            " times_us=%" PRIu64 ":%" PRIu64 ":%" PRIu64 ":%" PRIu64 ":%" PRIu64 ":%" PRIu64 ":%" PRIu64 ":%" PRIu64
            "; discard and reconfigure",(unsigned)n,(unsigned)observations[0],(unsigned)observations[1],
            (unsigned)observations[2],(unsigned)observations[3],(unsigned)observations[4],(unsigned)observations[5],
            (unsigned)observations[6],(unsigned)observations[7],times[0],times[1],times[2],times[3],times[4],times[5],times[6],times[7]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if(n>3)++d->fifo_count_confirm_recoveries;
    return ESP_OK;
}

static int16_t signed_be(const uint8_t *p)
{
    const uint16_t bits = ((uint16_t)p[0] << 8) | p[1];
    return bits < 0x8000 ? (int16_t)bits : (int16_t)((int32_t)bits - 65536);
}

esp_err_t mpu6050_fifo_read(mpu6050_t *d, mpu6050_raw_t *samples, size_t count)
{
    if (!d || !d->device || !samples || count == 0 || count > 32)
        return ESP_ERR_INVALID_ARG;
    if (!fully_configured(d)) return ESP_ERR_INVALID_STATE;
    /* Diagnostic transport mode: keep the caller's batch, but issue one
     * complete six-byte FIFO transaction per frame. Never retry a consuming
     * read or infer/reorder byte alignment. Caller discards the entire batch
     * if any frame fails, including frames already decoded below. */
    for (size_t i = 0; i < count; ++i) {
        uint8_t data[6];
        const esp_err_t err = reg_read(d, REG_FIFO_RW, data, sizeof(data));
        if (err != ESP_OK) {
            d->sample_hz = 0; /* A partial read can lose FIFO frame alignment. */
            return err;
        }
        samples[i].x = signed_be(data);
        samples[i].y = signed_be(data + 2);
        samples[i].z = signed_be(data + 4);
    }
    /* A pre-read COUNT/status snapshot cannot certify bytes consumed later.
     * Scheduler/bus delays may let FIFO overflow while this batch is being
     * consumed. Never publish even its successfully transferred prefix. */
    uint8_t status=0;
    const esp_err_t err=reg_read(d,REG_INT_STATUS,&status,1);
    if(err!=ESP_OK){d->sample_hz=0;return err;}
    if(status&0x10) {
        ++d->fifo_read_overflows;
        d->sample_hz=0;
        ESP_LOGE(TAG,"fifo_overflow_during_read frames=%u; discard entire batch",(unsigned)count);
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

esp_err_t mpu6050_read_accel_registers(mpu6050_t *d, mpu6050_raw_t *sample)
{
    if (!d || !d->device || !sample) return ESP_ERR_INVALID_ARG;
    if (!fully_configured(d)) return ESP_ERR_INVALID_STATE;
    uint8_t data[6];
    const esp_err_t err = reg_read(d, REG_ACCEL_XOUT_H, data, sizeof(data));
    if (err != ESP_OK) {
        d->sample_hz = 0;
        return err;
    }
    sample->x = signed_be(data);
    sample->y = signed_be(data + 2);
    sample->z = signed_be(data + 4);
    return ESP_OK;
}

esp_err_t mpu6050_read_motion_registers(mpu6050_t *d,motion_snapshot_t *sample)
{
    if(!sample)return ESP_ERR_INVALID_ARG;
    memset(sample,0,sizeof(*sample));
    if(!d || !d->device)return ESP_ERR_INVALID_ARG;
    if(!fully_configured(d))return ESP_ERR_INVALID_STATE;
    if(d->who_am_i!=0x68)return ESP_ERR_NOT_SUPPORTED;
    uint8_t data[14];const esp_err_t err=reg_read(d,REG_ACCEL_XOUT_H,data,sizeof(data));
    if(err!=ESP_OK){d->sample_hz=0;return err;}
    return motion_decode(data,sizeof(data),sample)?ESP_OK:ESP_ERR_INVALID_RESPONSE;
}

void mpu6050_destroy(mpu6050_t *d)
{
    if (!d) return;
    if (d->device) i2c_master_bus_rm_device(d->device);
    if (d->bus) i2c_del_master_bus(d->bus);
    memset(d, 0, sizeof(*d));
}

