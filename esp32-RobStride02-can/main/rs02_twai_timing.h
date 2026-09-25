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
} rs02_twai_timing_spec_t;

// Return the explicit Classical CAN timing used by this example: 1 Mbit/s,
// 20 time quanta per bit, and an 80% controller sample point. The RS02 manual
// specifies the bit rate but not the sample point. Null output is rejected.
bool rs02_twai_timing_1mbps_80_percent(rs02_twai_timing_spec_t *timing);

#ifdef __cplusplus
}
#endif
