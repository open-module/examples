#include "rs02_twai_timing.h"

#include <stdio.h>
#include <stdlib.h>

static unsigned int failures;

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
        failures++; \
    } \
} while (0)

// Keep the chosen controller-side timing stable. The RS02 manual only specifies 1 Mbit/s.
static void test_one_mbit_80_percent_timing(void)
{
    rs02_twai_timing_spec_t timing = {0};
    CHECK(rs02_twai_timing_1mbps_80_percent(&timing));

    const unsigned int total_quanta =
        1U + timing.propagation_segment + timing.time_segment_1 + timing.time_segment_2;
    const unsigned int sample_quanta =
        1U + timing.propagation_segment + timing.time_segment_1;

    CHECK(total_quanta == 20U);
    CHECK(timing.quanta_resolution_hz / total_quanta == 1000000U);
    CHECK(sample_quanta * 100U / total_quanta == 80U);
    CHECK(timing.sync_jump_width == 3U);
    CHECK(!timing.triple_sampling);
    CHECK(!rs02_twai_timing_1mbps_80_percent(NULL));
}

int main(void)
{
    test_one_mbit_80_percent_timing();
    if (failures != 0U) {
        fprintf(stderr, "%u TWAI timing checks failed\n", failures);
        return EXIT_FAILURE;
    }
    puts("all RobStride RS02 TWAI timing tests passed");
    return EXIT_SUCCESS;
}
