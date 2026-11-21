#pragma once

/* Seeed XIAO ESP32-S3 Sense. D numbers are NOT GPIO numbers. */
#define BOARD_MPU_SDA_GPIO       5  /* D4 */
#define BOARD_MPU_SCL_GPIO       6  /* D5 */
#define BOARD_MPU_INT_GPIO       2  /* D1 */
#define BOARD_OPTICAL_GPIO       1  /* D0, disabled by default */
#define BOARD_MPU_ADDRESS        0x68 /* AD0 tied to GND */
#define BOARD_I2C_SPEED_HZ       100000 /* V0 identity/register qualification */
#define BOARD_MPU_SAMPLE_HZ      500
#define BOARD_MPU_ACCEL_LSB_G    8192.0f /* +/-4 g */
#define BOARD_GRAVITY_M_S2       9.80665f /* convention, not calibration */

