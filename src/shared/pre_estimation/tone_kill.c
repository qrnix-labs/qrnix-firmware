/*
libspecbleach - A spectral processing library

Copyright 2022 Luciano Dato <lucianodato@gmail.com>

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "tone_kill.h"
#include "../configurations.h"
#include <math.h>
#include <stdlib.h>

// Band is 600-800 Hz -> 11 bins at 2048 points / 44.1 kHz. Generous bound in
// case the FFT geometry changes; the module allocates no per-bin arrays.
#define TONE_KILL_MAX_BAND_BINS 32U

typedef struct ToneKill {
  uint32_t fft_size;
  uint32_t hop;
  uint32_t sample_rate;

  int32_t k_lo;
  int32_t k_hi;

  float dominance_ratio;
  float min_power;
  uint32_t null_width;

  uint32_t continuity_frames;
  uint32_t release_frames;

  int32_t tracked_bin;
  uint32_t dominance_counter;
  int32_t nulled_bin;
  uint32_t release_counter;
} ToneKill;

static void insertion_sort(float *values, const uint32_t count) {
  for (uint32_t i = 1U; i < count; i++) {
    const float key = values[i];
    uint32_t j = i;
    while (j > 0U && values[j - 1U] > key) {
      values[j] = values[j - 1U];
      j--;
    }
    values[j] = key;
  }
}

ToneKill *tone_kill_initialize(const uint32_t sample_rate,
                               const uint32_t fft_size, const uint32_t hop) {
  ToneKill *self = (ToneKill *)calloc(1U, sizeof(ToneKill));
  if (!self) {
    return NULL;
  }

  self->fft_size = fft_size;
  self->hop = hop;
  self->sample_rate = sample_rate;

  // Explicit bin math: bin = freq * fft_size / sample_rate. Do not reuse
  // freq_to_fft_bin(): it divides by half the real bin width, returning bins
  // about 2x too high.
  self->k_lo =
      (int32_t)(TONE_KILL_MIN_HZ * (float)fft_size / (float)sample_rate);
  self->k_hi =
      (int32_t)(TONE_KILL_MAX_HZ * (float)fft_size / (float)sample_rate);
  if (self->k_lo < 1) {
    self->k_lo = 1;
  }
  if (self->k_hi >= (int32_t)(fft_size / 2U)) {
    self->k_hi = (int32_t)(fft_size / 2U) - 1;
  }
  if (self->k_hi - self->k_lo + 1 > (int32_t)TONE_KILL_MAX_BAND_BINS) {
    self->k_hi = self->k_lo + (int32_t)TONE_KILL_MAX_BAND_BINS - 1;
  }

  self->dominance_ratio = powf(10.F, TONE_KILL_DOMINANCE_DB / 10.F);
  self->min_power = TONE_KILL_MIN_POWER;
  self->null_width = TONE_KILL_NULL_WIDTH_BINS;

  // Frame counts are derived from wall time so behavior is identical in NR1,
  // NR2, and the bypass notch STFT (frames = ms * sample_rate / (1000 * hop)).
  self->continuity_frames =
      (uint32_t)((uint64_t)TONE_KILL_CONTINUITY_MS * sample_rate /
                 (1000UL * hop));
  self->release_frames =
      (uint32_t)((uint64_t)TONE_KILL_RELEASE_MS * sample_rate /
                 (1000UL * hop));
  if (self->continuity_frames == 0U) {
    self->continuity_frames = 1U;
  }
  if (self->release_frames == 0U) {
    self->release_frames = 1U;
  }

  self->tracked_bin = -1;
  self->nulled_bin = -1;

  return self;
}

void tone_kill_free(ToneKill *self) { free(self); }

bool tone_kill_run(ToneKill *self, float *fft_spectrum, const bool enabled) {
  if (!self || !fft_spectrum) {
    return false;
  }
  if (!enabled) {
    return true; // pass-through; state is preserved so arming is instant
  }

  // 1. Band powers from the packed CMSIS spectrum. Real part of bin k lives
  //    at index k, imaginary part at fft_size - k (the convention already
  //    used by compute_power_spectrum() and denoise_mixer_run()).
  float powers[TONE_KILL_MAX_BAND_BINS];
  const uint32_t band_bins = (uint32_t)(self->k_hi - self->k_lo + 1);
  float peak_power = -1.F;
  int32_t peak_k = -1;
  for (uint32_t i = 0U; i < band_bins; i++) {
    const int32_t k = self->k_lo + (int32_t)i;
    const float real_bin = fft_spectrum[k];
    const float imag_bin = fft_spectrum[self->fft_size - (uint32_t)k];
    const float power = real_bin * real_bin + imag_bin * imag_bin;
    powers[i] = power;
    if (power > peak_power) {
      peak_power = power;
      peak_k = k;
    }
  }

  // 2. Band median.
  float sorted[TONE_KILL_MAX_BAND_BINS];
  for (uint32_t i = 0U; i < band_bins; i++) {
    sorted[i] = powers[i];
  }
  insertion_sort(sorted, band_bins);
  const float band_median = sorted[band_bins / 2U];

  // 3/4. Dominance: relative peak plus an absolute floor so a dead-quiet
  //      band can never hold the counter via numerical noise.
  const bool dominant = peak_power > self->dominance_ratio * band_median &&
                        peak_power > self->min_power;

  // 5. Continuity tracking. Any non-dominant frame resets the counter, so
  //    keyed CW (element gaps) can never accumulate the continuity gate.
  if (dominant && peak_k == self->tracked_bin) {
    self->dominance_counter++;
  } else {
    self->dominance_counter = 0U;
    self->tracked_bin = dominant ? peak_k : -1;
  }
  if (self->dominance_counter >= self->continuity_frames) {
    self->nulled_bin = self->tracked_bin;
    self->release_counter = 0U;
  }

  // 6. Release after the nulled bin stops being dominant for the release
  //    window. The null persists through brief dropouts (hysteresis).
  if (self->nulled_bin >= 0) {
    if (dominant && peak_k == self->nulled_bin) {
      self->release_counter = 0U;
    } else {
      self->release_counter++;
      if (self->release_counter >= self->release_frames) {
        self->nulled_bin = -1;
        self->release_counter = 0U;
      }
    }
  }

  // 7. Null the bin and its mirror positions in the input spectrum, before
  //    the reference spectrum, estimator, gain, or mixer see the tone.
  if (self->nulled_bin >= 0) {
    int32_t k = self->nulled_bin - (int32_t)self->null_width;
    const int32_t k_max = self->nulled_bin + (int32_t)self->null_width;
    const int32_t nyquist = (int32_t)(self->fft_size / 2U);
    if (k < 1) {
      k = 1;
    }
    for (; k <= k_max && k <= nyquist; k++) {
      fft_spectrum[k] = 0.F;
      fft_spectrum[self->fft_size - (uint32_t)k] = 0.F;
    }
  }

  return true;
}
