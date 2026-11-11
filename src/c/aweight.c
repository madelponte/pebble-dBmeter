#include "aweight.h"

// Designed for fs = 16 kHz: the low-frequency poles (20.6 Hz x2, 107.7 Hz, 737.9 Hz) by bilinear
// transform, and the high-frequency roll-off (whose 12.2 kHz poles lie above Nyquist) as a biquad
// fitted to the standard curve. Gain is 0.0 dB at 1 kHz. Generated and verified, bit-exact, by
// tools/aweight_design.py.
//
// Three direct form I biquads, cascaded. The first two are high-pass with a double zero at DC, so
// their numerator is 1 - 2z^-1 + z^-2: just adds, with their gains folded into the third section.
// Each section's output history is the next one's input history, so the state is 4 nodes x 2.
//
// Coefficients are Q30, feedback stored negated so everything accumulates:
// y = b0*x + b1*x1 + b2*x2 + na1*y1 + na2*y2, where na = -a.
#define Q 30
#define ROUND (1LL << (Q - 1))
#define IN_SHIFT 12   // samples are scaled up by 2^12 for precision in the low-frequency sections
#define OUT_SHIFT (IN_SHIFT - 6)  // leaves the output scaled by 2^6 = AWEIGHT_OUT_SCALE

#define S1_NA1 2130182185
#define S1_NA2 -1056510057
#define S2_NA1 1831277025
#define S2_NA2 -768785805
#define S3_B0 1004168681
#define S3_B1 641406423
#define S3_B2 53590938
#define S3_NA1 -525994875
#define S3_NA2 2183315

#define MAC(c, v) ((int64_t)(c) * (v))   // one SMULL/SMLAL: 1 cycle on Cortex-M4/M33

void aweight_reset(AWeightFilter *filter) {
  memset(filter, 0, sizeof(*filter));
}

uint64_t aweight_sum_squares(AWeightFilter *filter, const int16_t *samples, size_t count) {
  // Work on locals so the compiler can keep the state in registers across the frame.
  int32_t x1 = filter->z[0][0], x2 = filter->z[0][1];   // input
  int32_t u1 = filter->z[1][0], u2 = filter->z[1][1];   // after section 1
  int32_t v1 = filter->z[2][0], v2 = filter->z[2][1];   // after section 2
  int32_t y1 = filter->z[3][0], y2 = filter->z[3][1];   // output
  uint64_t sum_sq = 0;

  for (size_t n = 0; n < count; n++) {
    const int32_t x = samples[n] * (1 << IN_SHIFT);  // not <<: undefined for negatives
    const int32_t u = x - 2 * x1 + x2 + (int32_t)((ROUND + MAC(S1_NA1, u1) + MAC(S1_NA2, u2)) >> Q);
    const int32_t v = u - 2 * u1 + u2 + (int32_t)((ROUND + MAC(S2_NA1, v1) + MAC(S2_NA2, v2)) >> Q);
    const int32_t y = (int32_t)((ROUND + MAC(S3_B0, v) + MAC(S3_B1, v1) + MAC(S3_B2, v2)
                                 + MAC(S3_NA1, y1) + MAC(S3_NA2, y2)) >> Q);
    x2 = x1; x1 = x;
    u2 = u1; u1 = u;
    v2 = v1; v1 = v;
    y2 = y1; y1 = y;

    const int32_t out = y >> OUT_SHIFT;
    sum_sq += (uint64_t)MAC(out, out);
  }

  filter->z[0][0] = x1; filter->z[0][1] = x2;
  filter->z[1][0] = u1; filter->z[1][1] = u2;
  filter->z[2][0] = v1; filter->z[2][1] = v2;
  filter->z[3][0] = y1; filter->z[3][1] = y2;
  return sum_sq;
}
