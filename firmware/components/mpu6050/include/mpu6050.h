#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "motion_decode.h"

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t device;
    uint8_t who_am_i;
    uint16_t sample_hz;
    uint16_t fifo_capacity_bytes;
    const char *profile_name;
    bool expected_identity; /* WHO matches the supplied MPU6050 label only. */
    uint32_t i2c_speed_hz;
    uint8_t identity_reads; /* Successful consecutive pre-configuration reads. */
    uint32_t fifo_count_anomalies;
    uint32_t fifo_count_confirm_recoveries; /* >3 non-consuming observations confirmed alignment */
    uint32_t fifo_read_overflows; /* overflow discovered after consuming a batch */
} mpu6050_t;

typedef struct { int16_t x, y, z; } mpu6050_raw_t;

esp_err_t mpu6050_create(mpu6050_t *dev, int sda, int scl,
                         uint8_t address, uint32_t i2c_speed_hz);
/* Before any write, require 16 identical WHO reads at <=100 kHz.
 * Stable 0x68 selects MPU6050: 1024-byte FIFO, CONFIG.DLPF_CFG=2 (94 Hz accel).
 * MPU_POSE_DIAG_ENABLED instead selects CFG=6 (nominal 5 Hz accel/gyro)
 * for diagnostic snapshots and rejects all other identities before writes.
 * Stable 0x70 selects a CANDIDATE MPU6500 register model: 512-byte FIFO,
 * ACCEL_CONFIG2.A_DLPF_CFG=2 (92 Hz in RM-MPU-6500A-00 rev. 2.1).
 * The latter is a register compatibility experiment, not chip authentication;
 * expected_identity remains false. Other or inconsistent IDs are rejected.
 * Both profiles use +/-4 g (8192 LSB/g), 1 kHz base / integer divider,
 * pulsed DRDY and accel-only FIFO. Default DRDY is active-high push-pull;
 * V0_MPU_INT_OPEN_DRAIN selects active-low open-drain instead.
 * sample_hz=500 sets divider=1.
 * sample_hz stays zero until all defined persistent configuration bits pass
 * readback; reset bits are checked for self-clear. Diagnostic
 * identity/profile fields remain available when configuration fails.
 */
esp_err_t mpu6050_configure(mpu6050_t *dev, uint16_t sample_hz);
/* Read COUNT high then low without consuming FIFO. Up to 8 observations / a
 * 6ms observation interval may confirm two consecutive complete frame counts.
 * No incomplete count is rounded into readable data. Recheck overflow before
 * returning; persistent/incoherent observations still invalidate sample_hz. */
esp_err_t mpu6050_fifo_count(mpu6050_t *dev, uint16_t *bytes, uint8_t *status);
/* Diagnostic transport: count consecutive 6-byte FIFO transactions. A failure
 * invalidates sample_hz and the ENTIRE output batch, even decoded frames.
 * INT_STATUS is checked again after the batch: overflow during consumption
 * invalidates the entire batch with ESP_ERR_INVALID_SIZE and increments
 * fifo_read_overflows. No read retry, alignment inference or repair is used.
 */
esp_err_t mpu6050_fifo_read(mpu6050_t *dev, mpu6050_raw_t *samples, size_t count);
/* Read 0x3b..0x40 directly, without consuming FIFO. This is a later, separate
 * observation, not synchronized with FIFO samples; intended for static
 * transport diagnosis only. Requires complete configuration and invalidates
 * sample_hz on I2C error. Output is valid only when ESP_OK is returned.
 */
esp_err_t mpu6050_read_accel_registers(mpu6050_t *dev, mpu6050_raw_t *sample);
/* Optional MPU6050-only register snapshot, not a FIFO sample or measured
 * sample timestamp. One 14-byte burst; zero output on failure. */
esp_err_t mpu6050_read_motion_registers(mpu6050_t *dev,motion_snapshot_t *sample);
/* Public FIFO operations require a fully configured, identified device.
 * A FIFO I2C failure invalidates sample_hz; call configure to restore a known
 * complete configuration and FIFO frame alignment before subsequent reads.
 */
esp_err_t mpu6050_fifo_reset(mpu6050_t *dev);
/* Owner-task only. Stop FIFO writes for an explicitly unrecorded idle gap;
 * keep sensor rate/filter/range running. Resume with checked fifo_reset. */
esp_err_t mpu6050_fifo_pause(mpu6050_t *dev);
void mpu6050_destroy(mpu6050_t *dev);

