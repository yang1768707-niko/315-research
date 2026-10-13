#ifndef VIBRATION_FEATURES_H
#define VIBRATION_FEATURES_H
#include "vibration_wire.h"
#include <math.h>
#include <limits.h>

/* One accumulator per batch. Replayed windows never contribute twice. */
typedef struct {
  uint32_t batch, target, sample_count, raw_crc;
  int64_t sum[3], scaled_sum[3], scaled_sum_cube[3];
  uint64_t sum_square[3], scaled_sum_square[3], scaled_sum_fourth[3];
  int16_t minimum[3], maximum[3];
  int32_t dc_q8[3], previous_centered_q8[3];
  uint32_t crossings[3];
} VibFeatures;

static inline void vib_features_begin(VibFeatures *s, uint32_t batch, uint32_t target) {
  memset(s, 0, sizeof(*s)); s->batch=batch; s->target=target;
  s->raw_crc=UINT32_MAX;
  for(unsigned axis=0;axis<3;axis++) {
    s->minimum[axis]=INT16_MAX; s->maximum[axis]=INT16_MIN;
  }
}
static inline int vib_features_add(VibFeatures *s, uint32_t batch,
                                    uint32_t first, unsigned points, const uint8_t *raw_bytes) {
  if(!raw_bytes || !points || batch!=s->batch || first>s->target ||
     points>s->target-first) return 0;
  if(first<s->sample_count) return first+points<=s->sample_count;
  if(first!=s->sample_count) return 0;
  s->raw_crc=v2_crc_update(s->raw_crc,raw_bytes,(size_t)points*6U);
  for(unsigned sample=0;sample<points;sample++) {
    int16_t raw[3];
    for(unsigned axis=0;axis<3;axis++) {
      raw[axis]=(int16_t)v2_u16(raw_bytes+(size_t)sample*6U+axis*2U);
    }
      for (uint32_t axis = 0U; axis < 3U; axis++)
      {
        int32_t value = raw[axis];
        int32_t scaled_value = value >> 4U;
        uint64_t square = (uint64_t)((int64_t)value * value);
        uint64_t scaled_square =
            (uint64_t)((int64_t)scaled_value * scaled_value);
        int32_t sample_q8 = (int32_t)raw[axis] * 256;
        int32_t centered_q8;

        s->sum[axis] += value;
        s->sum_square[axis] += square;
        s->scaled_sum[axis] += scaled_value;
        s->scaled_sum_square[axis] += scaled_square;
        s->scaled_sum_cube[axis] +=
            (int64_t)scaled_square * scaled_value;
        s->scaled_sum_fourth[axis] +=
            scaled_square * scaled_square;
        if (raw[axis] < s->minimum[axis])
        {
          s->minimum[axis] = raw[axis];
        }
        if (raw[axis] > s->maximum[axis])
        {
          s->maximum[axis] = raw[axis];
        }

        if (s->sample_count == 0U)
        {
          s->dc_q8[axis] = sample_q8;
        }
        else
        {
          s->dc_q8[axis] +=
              (sample_q8 - s->dc_q8[axis]) >> 8U;
        }
        centered_q8 = sample_q8 - s->dc_q8[axis];
        if ((s->sample_count >= 256U) &&
            (s->previous_centered_q8[axis] <= 0) &&
            (centered_q8 > 0))
        {
          s->crossings[axis]++;
        }
        s->previous_centered_q8[axis] = centered_q8;
      }

    s->sample_count++;
  }
  return 1;
}
static inline int32_t vib_feature_scale(double value) {
  value*=1000.0;
  if(value>=(double)INT32_MAX) return INT32_MAX;
  if(value<=(double)INT32_MIN) return INT32_MIN;
  return (int32_t)(value+((value>=0.0)?0.5:-0.5));
}
static inline int vib_features_finish(const VibFeatures *s, uint16_t rate, int32_t out[3][5]) {
  if(!out || s->sample_count!=s->target || s->sample_count<256U) return 0;
    double count = (double)s->sample_count;

    for (uint32_t axis = 0U; axis < 3U; axis++)
    {
      double mean = (double)s->sum[axis] / count;
      double raw_second = (double)s->sum_square[axis] / count;
      double scaled_mean = (double)s->scaled_sum[axis] / count;
      double scaled_second =
          (double)s->scaled_sum_square[axis] / count;
      double scaled_third =
          (double)s->scaled_sum_cube[axis] / count;
      double scaled_fourth =
          (double)s->scaled_sum_fourth[axis] / count;
      double variance = raw_second - mean * mean;
      double scaled_variance = scaled_second - scaled_mean * scaled_mean;
      double central_fourth;
      double peak_raw;

      if (variance < 0.0)
      {
        variance = 0.0;
      }
      central_fourth = scaled_fourth - 4.0 * scaled_mean * scaled_third +
                       6.0 * scaled_mean * scaled_mean * scaled_second -
                       3.0 * scaled_mean * scaled_mean * scaled_mean * scaled_mean;
      if (central_fourth < 0.0)
      {
        central_fourth = 0.0;
      }
      peak_raw = fmax(fabs((double)s->maximum[axis] - mean),
                      fabs((double)s->minimum[axis] - mean));
      out[axis][0] =
          vib_feature_scale(sqrt(variance) * 0.000488);
      out[axis][1] =
          vib_feature_scale(peak_raw * 0.000488);
      out[axis][2] =
          vib_feature_scale(
              ((double)s->maximum[axis] -
               (double)s->minimum[axis]) *
              0.000488);
      out[axis][3] =
          vib_feature_scale((scaled_variance > 0.0) ?
              (central_fourth / (scaled_variance * scaled_variance)) : 0.0);
      out[axis][4] =
          vib_feature_scale(
              ((double)s->crossings[axis] *
               rate) / count);
    }

  return 1;
}
#endif
