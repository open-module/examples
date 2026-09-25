#include "cybergear_twai_timing.h"

#include <stddef.h>

bool cybergear_twai_timing_1mbps_80_percent(cybergear_twai_timing_spec_t *timing)
{
    if (timing == NULL) {
        return false;
    }

    *timing = (cybergear_twai_timing_spec_t) {
        .quanta_resolution_hz = 20000000U,
        .propagation_segment = 0U,
        .time_segment_1 = 15U,
        .time_segment_2 = 4U,
        .sync_jump_width = 3U,
        .triple_sampling = false,
    };
    return true;
}
