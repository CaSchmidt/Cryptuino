/****************************************************************************
** Copyright (c) 2022, Carsten Schmidt. All rights reserved.
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

#include <array>

#include <matio.h>

#define HAVE_STD_FORMAT
#include <cs/Logging/ILogger.h>

#include "MatOutput.h"

#include "ScopeGuard.h"

////// Private ///////////////////////////////////////////////////////////////

namespace priv {

  bool writeMatVector(const cs::ILogger *logger, mat_t *file,
                      const std::string& varname, const std::size_t numSamples, const double *data)
  {
    std::array<std::size_t,2> dims;
    dims[0] = numSamples;
    dims[1] = 1;

    matvar_t *var = Mat_VarCreate(varname.data(), MAT_C_DOUBLE, MAT_T_DOUBLE,
                                  int(dims.size()), dims.data(), const_cast<double*>(data),
                                  MAT_F_DONT_COPY_DATA);
    if( var == NULL ) {
      logger->logErrorf(u8"Mat_VarCreate({})", varname);
      return false;
    }

    const int err = Mat_VarWrite(file, var, MAT_COMPRESSION_ZLIB);
    Mat_VarFree(var);

    return err == 0;
  }

  bool writeMatSamples(const cs::ILogger *logger, mat_t *matfile,
                       const std::string& varname, const SampleBuffer& samples)
  {
    // (1) Sanity check //////////////////////////////////////////////////////

    if( samples.size() < 3 ) {
      logger->logErrorf(u8"No samples for \"{}\"!", varname);
      return false;
    }
    const SampleBuffer::size_type numSamples = samples.size() - 2;

    // (2) Time vector ///////////////////////////////////////////////////////

    SampleBuffer time;
    try {
      time.resize(numSamples, 0);
    } catch(...) {
      logger->logErrorf(u8"Unable to create time data for \"{}\"!", varname);
      return false;
    }

    const double xIncr = samples[0];
    const double xZero = samples[1];
    for(SampleBuffer::size_type i = 0; i < time.size(); i++) {
      time[i] = double(i)*xIncr + xZero;
    }

    // (3) Output ////////////////////////////////////////////////////////////

    if( !writeMatVector(logger, matfile, varname, numSamples, samples.data() + 2) ) {
      logger->logErrorf(u8"Unable to write data for \"{}\"!", varname);
      return false;
    }
    if( !writeMatVector(logger, matfile, "t_" + varname, numSamples, time.data()) ) {
      logger->logErrorf(u8"Unable to write time data for \"{}\"!", varname);
      return false;
    }

    return true;
  }

} // namespace priv

////// Public ////////////////////////////////////////////////////////////////

bool writeMatOutput(const cs::ILogger *logger, const InstrumentPtr& instr,
                    const std::filesystem::path& filename, const std::string& channels)
{
  const int ch_trace   = channels[0] - '0';
  const int ch_trigger = channels[1] - '0';

  mat_t *matfile = Mat_CreateVer(cs::CSTR(filename.generic_u8string().data()), NULL, MAT_FT_DEFAULT);
  if( matfile == NULL ) {
    return false;
  }
  ScopeGuard guard([&](void) -> void {
    if( matfile != NULL ) {
      Mat_Close(matfile);
      matfile = NULL;
    }
  });

  SampleBuffer trace;
  if( !instr->readSamples(logger, ch_trace, &trace) ) {
    return false;
  }
  if( !priv::writeMatSamples(logger, matfile, "trace", trace) ) {
    return false;
  }

  SampleBuffer trigger;
  if( !instr->readSamples(logger, ch_trigger, &trigger) ) {
    return false;
  }
  if( !priv::writeMatSamples(logger, matfile, "trigger", trigger) ) {
    return false;
  }

  logger->logTextf(u8"Wrote file \"{}\".", cs::CSTR(filename.generic_u8string().data()));

  return true;
}
