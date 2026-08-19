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

#ifndef TONE_KILL_PROCESSOR_H
#define TONE_KILL_PROCESSOR_H

#include "../../interfaces/spectral_processor.h"
#include <stdbool.h>
#include <stdint.h>

SpectralProcessorHandle tone_kill_processor_initialize(uint32_t sample_rate);
void tone_kill_processor_free(SpectralProcessorHandle instance);
bool tone_kill_processor_process(SpectralProcessorHandle instance,
                                 uint32_t number_of_samples,
                                 const float *input, float *output);

#endif
