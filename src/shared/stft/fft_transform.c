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

#include "fft_transform.h"
#include "../configurations.h"
#include "../utils/general_utils.h"

#include <arm_math.h>
#include <stdlib.h>
#include <string.h>

static uint32_t calculate_fft_size(FftTransform *self);
static void allocate_fftw(FftTransform *self);

struct FftTransform {
  arm_rfft_fast_instance_f32 forward;
  arm_rfft_fast_instance_f32 backward;

  uint32_t fft_size;
  uint32_t frame_size;
  uint32_t zeropadding_amount;
  uint32_t copy_position;
  ZeroPaddingType padding_type;
  uint32_t padding_amount;
  float *input_fft_buffer;
  float *output_fft_buffer;
};

FftTransform *fft_transform_initialize(const uint32_t frame_size,
                                       const ZeroPaddingType padding_type,
                                       const uint32_t zeropadding_amount) {
  FftTransform *self = (FftTransform *)calloc(1U, sizeof(FftTransform));

  self->padding_type = padding_type;
  self->zeropadding_amount = zeropadding_amount;
  self->frame_size = frame_size;

  self->fft_size = calculate_fft_size(self);

  self->copy_position = (self->fft_size / 2U) - (self->frame_size / 2U);

  allocate_fftw(self);

  return self;
}

FftTransform *fft_transform_initialize_bins(const uint32_t fft_size) {
  FftTransform *self = (FftTransform *)calloc(1U, sizeof(FftTransform));

  self->fft_size = fft_size;
  self->frame_size = self->fft_size;

  allocate_fftw(self);

  return self;
}

static void allocate_fftw(FftTransform *self) {
  self->input_fft_buffer =
      (float *)calloc(self->fft_size, sizeof(float));
  self->output_fft_buffer =
      (float *)calloc(self->fft_size, sizeof(float));
  arm_rfft_fast_init_f32(&self->forward, self->fft_size);
  arm_rfft_fast_init_f32(&self->backward, self->fft_size);
}

static uint32_t calculate_fft_size(FftTransform *self) {
  switch (self->padding_type) {
  case NO_PADDING: {
    self->padding_amount = 0;
    return get_next_divisible_two((int)self->frame_size);
  }
  case NEXT_POWER_OF_TWO: {
    uint32_t next_power_of_two = get_next_power_two((int)self->frame_size);
    self->padding_amount = next_power_of_two - self->frame_size;
    return next_power_of_two;
  }
  case FIXED_AMOUNT: {
    self->padding_amount = self->zeropadding_amount;
    return get_next_divisible_two(
        (int)(self->frame_size + self->padding_amount));
  }
  default:
    return get_next_divisible_two((int)self->frame_size);
  }
}

void fft_transform_free(FftTransform *self) {
  free(self->input_fft_buffer);
  free(self->output_fft_buffer);

  free(self);
}

uint32_t get_fft_size(FftTransform *self) { return self->fft_size; }
uint32_t get_fft_real_spectrum_size(FftTransform *self) {
  return self->fft_size / 2U + 1U;
}

bool fft_load_input_samples(FftTransform *self, const float *input) {
  if (!self || !input) {
    return false;
  }

  // Copy centered values only
  for (uint32_t i = self->copy_position;
       i < (self->frame_size + self->copy_position); i++) {
    self->input_fft_buffer[i] = input[i - self->copy_position];
  }

  return true;
}

bool fft_get_output_samples(FftTransform *self, float *output) {
  if (!self || !output) {
    return false;
  }

  // Copy centered values only
  for (uint32_t i = self->copy_position;
       i < (self->frame_size + self->copy_position); i++) {
    output[i - self->copy_position] = self->input_fft_buffer[i];
  }

  return true;
}

bool compute_forward_fft(FftTransform *self) {
  if (!self) {
    return false;
  }

  arm_rfft_fast_f32(&self->forward, self->input_fft_buffer,
                    self->output_fft_buffer, 0);

  // libspecbleach uses FFTW's R2HC layout: DC at 0, real bin k at k,
  // Nyquist at N/2, and imaginary bin k at N-k. CMSIS-DSP returns packed
  // interleaved bins: DC, Nyquist, Re(1), Im(1), Re(2), Im(2), ...
  self->input_fft_buffer[0] = self->output_fft_buffer[0];
  self->input_fft_buffer[self->fft_size / 2U] = self->output_fft_buffer[1];
  for (uint32_t k = 1U; k < self->fft_size / 2U; k++) {
    self->input_fft_buffer[k] = self->output_fft_buffer[2U * k];
    self->input_fft_buffer[self->fft_size - k] =
        self->output_fft_buffer[2U * k + 1U];
  }
  memcpy(self->output_fft_buffer, self->input_fft_buffer,
         self->fft_size * sizeof(float));

  return true;
}

bool compute_backward_fft(FftTransform *self) {
  if (!self) {
    return false;
  }

  // Convert FFTW R2HC layout back to CMSIS packed interleaved layout.
  self->input_fft_buffer[0] = self->output_fft_buffer[0];
  self->input_fft_buffer[1] = self->output_fft_buffer[self->fft_size / 2U];
  for (uint32_t k = 1U; k < self->fft_size / 2U; k++) {
    self->input_fft_buffer[2U * k] = self->output_fft_buffer[k];
    self->input_fft_buffer[2U * k + 1U] =
        self->output_fft_buffer[self->fft_size - k];
  }
  arm_rfft_fast_f32(&self->backward, self->input_fft_buffer,
                    self->output_fft_buffer, 1);
  memcpy(self->input_fft_buffer, self->output_fft_buffer,
         self->fft_size * sizeof(float));

  return true;
}

float *get_fft_input_buffer(FftTransform *self) {
  return self->input_fft_buffer;
}

float *get_fft_output_buffer(FftTransform *self) {
  return self->output_fft_buffer;
}
