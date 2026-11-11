#include "aweight.h"

// Designed for fs = 16 kHz: the low-frequency poles (20.6 Hz x2, 107.7 Hz, 737.9 Hz) by bilinear
// transform, and the high-frequency roll-off (whose 12.2 kHz poles lie above Nyquist) as a biquad
// fitted to the standard curve. Gain is 0.0 dB at 1 kHz. Coefficients are Q30:
// y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2. Generated and verified by tools/aweight_design.py.
#define Q 30
#define IN_SHIFT 12   // samples are scaled up by 2^12 for precision in the low-frequency sections
#define OUT_SHIFT (IN_SHIFT - 6)  // leaves the output scaled by 2^6 = AWEIGHT_OUT_SCALE

static const int32_t s_coeffs[3][5] = {
  { 1065108517, -2130217033, 1065108517, -2130182185, 1056510057 },
  {  918451164, -1836902327,  918451164, -1831277025,  768785805 },
  { 1183467909,   755932676,   63159862,   525994875,   -2183315 },
};

void aweight_reset(AWeightFilter *filter) {
  memset(filter, 0, sizeof(*filter));
}

uint64_t aweight_sum_squares(AWeightFilter *filter, const int16_t *samples, size_t count) {
  uint64_t sum_sq = 0;
  for (size_t n = 0; n < count; n++) {
    int32_t v = (int32_t)samples[n] << IN_SHIFT;
    for (int s = 0; s < 3; s++) {
      const int32_t *c = s_coeffs[s];
      int32_t *st = filter->state[s];
      int64_t acc = (int64_t)c[0] * v
                  + (int64_t)c[1] * st[0]
                  + (int64_t)c[2] * st[1]
                  - (int64_t)c[3] * st[2]
                  - (int64_t)c[4] * st[3];
      const int32_t y = (int32_t)((acc + (1LL << (Q - 1))) >> Q);
      st[1] = st[0];
      st[0] = v;
      st[3] = st[2];
      st[2] = y;
      v = y;
    }
    const int32_t out = v >> OUT_SHIFT;
    sum_sq += (uint64_t)((int64_t)out * out);
  }
  return sum_sq;
}
