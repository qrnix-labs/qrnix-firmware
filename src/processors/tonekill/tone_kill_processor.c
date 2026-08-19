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

#include "tone_kill_processor.h"
#include "../../shared/configurations.h"
#include "../../shared/pre_estimation/tone_kill.h"
#include "../../shared/stft/stft_processor.h"
#include <stdlib.h>

typedef struct ToneKillProcessor {
  StftProcessor *stft;
  ToneKill *tone_kill;
} ToneKillProcessor;

// Notch-only spectral processing: tone-kill without any denoising. Used by
// the bypass path, which has no STFT of its own. Allocated lazily so the
// "only the selected NR handle is allocated" heap invariant still holds.
static bool tone_kill_only_run(SpectralProcessorHandle handle,
                               float *fft_spectrum) {
  ToneKillProcessor *self = (ToneKillProcessor *)handle;
  return tone_kill_run(self->tone_kill, fft_spectrum, true);
}

SpectralProcessorHandle tone_kill_processor_initialize(
    const uint32_t sample_rate) {
  ToneKillProcessor *self =
      (ToneKillProcessor *)calloc(1U, sizeof(ToneKillProcessor));
  if (!self) {
    return NULL;
  }

  self->stft = stft_processor_initialize(
      sample_rate, 25.0f, OVERLAP_FACTOR_GENERAL, PADDING_CONFIGURATION_GENERAL,
      ZEROPADDING_AMOUNT_GENERAL, INPUT_WINDOW_TYPE_GENERAL,
      OUTPUT_WINDOW_TYPE_GENERAL);
  if (!self->stft) {
    free(self);
    return NULL;
  }

  const uint32_t fft_size = get_stft_fft_size(self->stft);
  self->tone_kill = tone_kill_initialize(sample_rate, fft_size,
                                         fft_size / OVERLAP_FACTOR_GENERAL);
  if (!self->tone_kill) {
    stft_processor_free(self->stft);
    free(self);
    return NULL;
  }

  return self;
}

void tone_kill_processor_free(SpectralProcessorHandle instance) {
  ToneKillProcessor *self = (ToneKillProcessor *)instance;
  if (!self) {
    return;
  }
  if (self->tone_kill) {
    tone_kill_free(self->tone_kill);
  }
  if (self->stft) {
    stft_processor_free(self->stft);
  }
  free(self);
}

bool tone_kill_processor_process(SpectralProcessorHandle instance,
                                 const uint32_t number_of_samples,
                                 const float *input, float *output) {
  ToneKillProcessor *self = (ToneKillProcessor *)instance;
  if (!self) {
    return false;
  }
  return stft_processor_run(self->stft, number_of_samples, input, output,
                            &tone_kill_only_run, self);
}
