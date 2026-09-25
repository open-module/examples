#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t quanta_resolution_hz;
    uint8_t propagation_segment;
    uint8_t time_segment_1;
    uint8_t time_segment_2;
    uint8_t sync_jump_width;
    bool triple_sampling;
} cybergear_twai_timing_spec_t;

// Return the explicit Classical CAN timing used by this experiment: 1 Mbit/s,
// 20 time quanta per bit, and an 80% sample point. Null output is rejected.
bool cybergear_twai_timing_1mbps_80_percent(cybergear_twai_timing_spec_t *timing);

#ifdef __cplusplus
}
#endif
