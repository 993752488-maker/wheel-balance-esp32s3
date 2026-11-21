#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef struct { int16_t accel[3],temperature,gyro[3]; } motion_snapshot_t;
/* Decode exactly one 0x3b..0x48 register burst; no FIFO interpretation. */
bool motion_decode(const uint8_t *,size_t,motion_snapshot_t *);
