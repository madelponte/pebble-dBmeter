#pragma once

#include <pebble.h>

// A-weighting filter (IEC 61672) for 16 kHz audio, in fixed point: the watch has no FPU.
// Three biquads; within 0.1 dB of the standard curve from 10 Hz to 7 kHz.

typedef struct {
  int32_t state[3][4];  // per section: x1, x2, y1, y2
} AWeightFilter;

void aweight_reset(AWeightFilter *filter);

// Filters `count` samples and returns the sum of squares of the output, in units of
// (16-bit sample * AWEIGHT_OUT_SCALE)^2.
uint64_t aweight_sum_squares(AWeightFilter *filter, const int16_t *samples, size_t count);

#define AWEIGHT_OUT_SCALE 64
