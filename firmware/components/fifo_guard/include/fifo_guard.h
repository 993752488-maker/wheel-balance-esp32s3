#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
enum { FIFO_COUNT_MAX_OBSERVATIONS=8, FIFO_COUNT_CONFIRM_BUDGET_US=6000 };
typedef enum { FIFO_COUNT_WAIT,FIFO_COUNT_READY,FIFO_COUNT_INVALID } fifo_count_state_t;
/* Non-consuming observations only. Require two consecutive complete 6-byte
 * counts, monotonic plausible growth, and a bounded observation interval.
 * WAIT never permits a payload read, rounding or a change of byte alignment. */
fifo_count_state_t fifo_count_confirm(const uint16_t *,const uint64_t *,size_t,uint16_t,unsigned,uint16_t *);
bool fifo_count_select(const uint16_t *,const uint64_t *,size_t,uint16_t,unsigned,uint16_t *);
typedef enum { FRAME_COLLECTING,FRAME_AGREEMENT,FRAME_DYNAMIC,FRAME_MISMATCH,FRAME_TIME_ERROR } frame_check_t;
typedef struct {
    uint64_t start_us,last_us;
    unsigned samples,good_windows,bad_windows;
    double fifo_sum[3],direct_sum[3],fifo_square[3],direct_square[3];
    double difference_lsb,max_fifo_sigma,max_direct_sigma;
    bool consistent;
} frame_guard_t;
void frame_guard_init(frame_guard_t *);
/* Separate low-variation observations, not synchronized samples. Cannot
 * prove dynamic alignment or detect equal-axis gravity permutations. */
frame_check_t frame_guard_feed(frame_guard_t *,const int16_t [3],const int16_t [3],uint64_t,unsigned);
