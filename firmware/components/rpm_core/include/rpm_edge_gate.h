#ifndef RPM_EDGE_GATE_H
#define RPM_EDGE_GATE_H

#include <stdbool.h>
#include <stdint.h>

/* Reject only forward, too-close edges before the ISR queue. Equal or
 * backward timestamps still reach rpm_core and invalidate its reference.
 * The caller advances previous only after a successful queue insertion. */
static inline bool rpm_edge_too_close(bool have_previous, uint64_t previous,
                                     uint64_t current, uint64_t minimum_us)
{
    return have_previous && current > previous && current - previous < minimum_us;
}

#endif
