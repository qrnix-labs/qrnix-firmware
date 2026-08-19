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

#include "stft_windows.h"
#include "../configurations.h"
#include <stdlib.h>

static float get_windows_scale_factor(StftWindows *self,
                                      uint32_t active_frame_size,
                                      uint32_t overlap_factor);

struct StftWindows {
  float *input_window;
  float *output_window;

  uint32_t stft_frame_size;
  float scale_factor;
};

StftWindows *stft_window_initialize(const uint32_t stft_frame_size,
                                    const uint32_t active_frame_size,
                                    const uint32_t overlap_factor,
                                    const WindowTypes input_window,
                                    const WindowTypes output_window) {
  StftWindows *self = (StftWindows *)calloc(1U, sizeof(StftWindows));

  self->stft_frame_size = stft_frame_size;

  self->input_window = (float *)calloc(self->stft_frame_size, sizeof(float));
  self->output_window = (float *)calloc(self->stft_frame_size, sizeof(float));

  get_fft_window(self->input_window, self->stft_frame_size, input_window);
  get_fft_window(self->output_window, self->stft_frame_size, output_window);

  self->scale_factor =
      get_windows_scale_factor(self, active_frame_size, overlap_factor);

  return self;
}

void stft_window_free(StftWindows *self) {
  free(self->input_window);
  free(self->output_window);

  free(self);
}

static float get_windows_scale_factor(StftWindows *self,
                                      const uint32_t active_frame_size,
                                      const uint32_t overlap_factor) {
  if (overlap_factor < 2 || active_frame_size == 0U) {
    return 0.F;
  }

  const uint32_t hop = active_frame_size / overlap_factor;
  const uint32_t copy_position =
      (self->stft_frame_size / 2U) - (active_frame_size / 2U);
  float sum = 0.F;
  for (uint32_t i = copy_position;
       i < copy_position + active_frame_size; i++) {
    sum += self->input_window[i] * self->output_window[i];
  }

  // CMSIS-DSP's inverse real FFT already applies the 1/N normalization.
  // Normalize the mean overlap-add gain over the active, centered portion of
  // the padded FFT frame. The previous calculation used the entire padded
  // window and boosted this 1102-sample/2048-FFT configuration by about 1.76x.
  return sum / (float)hop;
}

bool stft_window_apply(StftWindows *self, float *frame,
                       const WindowPlace place) {
  if (!self || !frame) {
    return false;
  }

  for (uint32_t i = 0U; i < self->stft_frame_size; i++) {
    switch (place) {
    case INPUT_WINDOW:
      frame[i] *= self->input_window[i];
      break;
    case OUTPUT_WINDOW:
      frame[i] *= self->output_window[i] / self->scale_factor;
      break;
    default:
      break;
    }
  }

  return true;
}
