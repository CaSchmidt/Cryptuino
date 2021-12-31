/****************************************************************************
** Copyright (c) 2021, Carsten Schmidt. All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions
** are met:
**
** 1. Redistributions of source code must retain the above copyright
**    notice, this list of conditions and the following disclaimer.
**
** 2. Redistributions in binary form must reproduce the above copyright
**    notice, this list of conditions and the following disclaimer in the
**    documentation and/or other materials provided with the distribution.
**
** 3. Neither the name of the copyright holder nor the names of its
**    contributors may be used to endorse or promote products derived from
**    this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************/

#ifndef INSTRUMENT_H
#define INSTRUMENT_H

#include <list>
#include <string>

#include <visa.h>

#include "buffer.h"

using RsrcString = std::basic_string<ViChar>;
using   RsrcList = std::list<RsrcString>;

class csILogger;

bool flush(const csILogger *logger, ViSession vi,
           const bool rd_discard = true, const bool wr_discard = false);

bool handleError(const csILogger *logger, const ViObject obj,
                 const ViStatus status, const char *reason);

RsrcList queryInstruments(const csILogger *logger, ViSession rm);

bool queryRecordLength(const csILogger *logger, ViSession vi,
                       ViUInt32 *length);

bool querySampleRate(const csILogger *logger, ViSession vi,
                     float *rate);

bool readWaveform(const csILogger *logger, ViSession vi,
                  const char ch, const ViUInt32 numSamplesWant, SampleBuffer& samples);

bool setBufferAttributes(const csILogger *logger, ViSession vi,
                         const bool rd_flush = false, const bool wr_flush = false);

bool setSingleShotAcquisition(csILogger *logger, ViSession vi);

#endif // INSTRUMENT_H
