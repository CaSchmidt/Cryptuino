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

#include <cstdio>
#include <cstdlib>

#include <array>

#include <csUtil/csLogger.h>
#include <matio.h>

#include "instrument.h"

bool writeMatVector(const csILogger *logger, mat_t *file,
                    const char *varname, const std::size_t numSamples, const double *data)
{
  std::array<std::size_t,2> dims;
  dims[0] = numSamples;
  dims[1] = 1;

  matvar_t *var = Mat_VarCreate(varname, MAT_C_DOUBLE, MAT_T_DOUBLE,
                                dims.size(), dims.data(), const_cast<double*>(data),
                                MAT_F_DONT_COPY_DATA);
  if( var == NULL ) {
    logger->logError(u8"Mat_VarCreate()");
    return false;
  }

  Mat_VarWrite(file, var, MAT_COMPRESSION_ZLIB);
  Mat_VarFree(var);

  return true;
}

bool writeMatSamples(const csILogger *logger, const char *filename, const SampleArray& samples)
{
  if( samples.empty() ) {
    logger->logError(u8"samples.empty()");
    return false;
  }

  const std::size_t numSamples = samples.size() - 2;

  // (1) Time vector /////////////////////////////////////////////////////////

  SampleArray time;
  try {
    time.resize(numSamples, 0);
  } catch (...) {
    logger->logError(u8"time.resize()");
    return false;
  }

  const double xIncr = samples[0];
  const double xZero = samples[1];
  for(SampleArray::size_type i = 0; i < time.size(); i++) {
    time[i] = double(i)*xIncr + xZero;
  }

  // (2) MAT file ////////////////////////////////////////////////////////////

  mat_t *file = Mat_CreateVer(filename, NULL, MAT_FT_DEFAULT);
  if( file == NULL ) {
    logger->logError(u8"Mat_CreateVer()");
    return false;
  }

  // (3) Write vectors ///////////////////////////////////////////////////////

  writeMatVector(logger, file, "sig", numSamples, samples.data() + 2);
  writeMatVector(logger, file, "tim", numSamples, time.data());

  // Done! ///////////////////////////////////////////////////////////////////

  Mat_Close(file);

  return true;
}

int main(int argc, char **argv)
{
  constexpr char     DEF_CHANNEL     = '1';
  constexpr ViUInt32 DEF_NUM_SAMPLES = 10000;

  char      *resource = nullptr;
  char        channel = DEF_CHANNEL;
  ViUInt32 numSamples = DEF_NUM_SAMPLES;

  const int numArgs = argc - 1;
  if( numArgs < 1 ) {
    fprintf(stderr, "Usage: %s <resource> [channel] [samples]\n", argv[0]);
    return EXIT_FAILURE;
  }

  resource = argv[1];

  if( numArgs > 1 ) {
    const char c = *argv[2];
    if( '1' <= c  &&  c <= '4' ) {
      channel = c;
    } else {
      fprintf(stderr, "ERROR: Invalid channel!\n");
      channel = DEF_CHANNEL;
    }
  }

  if( numArgs > 2 ) {
    errno = 0;
    const ViUInt32 ui = strtoul(argv[3], NULL, 10);
    if( errno == 0  &&  ui != 0 ) {
      numSamples = ui;
    } else {
      fprintf(stderr, "ERROR: Invalid number of samples!\n");
      numSamples = DEF_NUM_SAMPLES;
    }
  }

  printf("Resource = %s\n", resource);
  printf("Channel  = %c\n", channel);
  printf("Samples  = %lu\n", numSamples);

  /* ********************************************************************** */

  csLogger logger;
  ViStatus status;

  ViSession rm;
  status = viOpenDefaultRM(&rm);
  if( handleError(&logger, rm, status, "viOpenDefaultRM()") ) {
    return EXIT_FAILURE;
  }

  ViSession vi;
  status = viOpen(rm, resource, VI_NULL, VI_NULL, &vi);
  if( handleError(&logger, rm, status, "viOpen()") ) {
    viClose(rm);
    return EXIT_FAILURE;
  }

  SampleArray samples;
  readWaveform(&logger, vi, channel, numSamples, samples);
  writeMatSamples(&logger, "output.mat", samples);

  viClose(vi);
  viClose(rm);

  return EXIT_SUCCESS;
}
