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

#include <array>

#include <matio.h>

#define HAVE_STD_FORMAT
#include <csUtil/csILogger.h>
#include <csUtil/csSerial.h>
#include <csUtil/csStringUtil.h>
#include <csUtil/csTime.h>

#include "util.h"

#include "instrument.h"
#include "Randomizer.h"
#include "ScopeGuard.h"

////// Private ///////////////////////////////////////////////////////////////

constexpr std::size_t AES_DATA_BYTES = 16;

using  AesBuffer = std::array<uint8_t,AES_DATA_BYTES>;
using CharBuffer = std::array<char,1024>;

inline char toHexChar(const uint8_t in, const bool hi_nibble = false)
{
  const uint8_t nibble = hi_nibble
      ? in >>    4
      : in  & 0x0F;
  return nibble >= 10
      ? nibble - 10 + 'A'
      : nibble + '0';
}

void buildAesCmd(const char prefix, char *cmd, const uint8_t *aes_data)
{
  *cmd++ = prefix;
  for(std::size_t i = 0; i < AES_DATA_BYTES; i++) {
    *cmd++ = toHexChar(aes_data[i], true);
    *cmd++ = toHexChar(aes_data[i]);
  }
  *cmd = '\n';
}

bool writeMatVector(const csILogger *logger, mat_t *file,
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

bool writeMatSamples(const csILogger *logger, mat_t *matfile,
                     const std::string& varname, const SampleBuffer& samples)
{
  // (1) Sanity check ////////////////////////////////////////////////////////

  if( samples.size() < 3 ) {
    logger->logErrorf(u8"No samples for \"{}\"!", varname);
    return false;
  }
  const SampleBuffer::size_type numSamples = samples.size() - 2;

  // (2) Time vector /////////////////////////////////////////////////////////

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

  // (3) Output //////////////////////////////////////////////////////////////

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

////// Public ////////////////////////////////////////////////////////////////

bool armInstrument(const csILogger *logger, ViSession vi, const unsigned int tout)
{
  if( !setSingleShotAcquisition(logger, vi) ) {
    return false;
  }
  csSleep(tout);
  return true;
}

bool initializeInstrument(const csILogger *logger, ViSession *rm, ViSession *vi)
{
  ViStatus status;

  *rm = *vi = VI_NULL; // cf. VI_WARN_NULL_OBJECT

  status = viOpenDefaultRM(rm);
  if( handleError(logger, *rm, status, "viOpenDefaultRM()") ) {
    return false;
  }

  const RsrcList resources = queryInstruments(logger, *rm);
  if( resources.empty() ) {
    logger->logError(u8"No instruments found!");
    viClose(*rm);
    return false;
  }
  const std::string instrument = resources.front(); // pick one...

  status = viOpen(*rm, const_cast<char*>(instrument.data()), VI_NULL, VI_NULL, vi);
  if( handleError(logger, *rm, status, "viOpen()") ) {
    viClose(*rm);
    return false;
  }

  ViUInt32 length = 0;
  if( !queryRecordLength(logger, *vi, &length) ) {
    viClose(*rm);
    return false;
  }
  logger->logTextf(u8"HORizontal:RECOrdlength = {}", length);

  float rate = 0;
  if( !querySampleRate(logger, *vi, &rate) ) {
    viClose(*rm);
    return false;
  }
  logger->logTextf(u8"HORizontal:SAMPLERate = {}", rate);

  return true;
}

void rxAesCmd(const csILogger *logger, const csSerial& serial, const unsigned int tout)
{
  CharBuffer buffer;
  buffer.fill(0);

  csSleep(tout);
  serial.read(buffer.data(), buffer.size());
  const std::list<std::string> answer = cs::split<char>(buffer.data(), '\n', true, true);
  for(const std::string& s : answer) {
    logger->logText(cs::UTF8(s.data()));
  }
}

void txAesCmd(const char prefix, const csSerial& serial, const Randomizer& randomizer)
{
  AesBuffer  aesdata;
  CharBuffer buffer;
  buffer.fill(0);

  randomizer.generate(aesdata.data(), aesdata.size());
  buildAesCmd(prefix, buffer.data(), aesdata.data());
  serial.write(buffer.data(), cs::length(buffer.data()));
}

bool writeMatOutput(const csILogger *logger, ViSession vi,
                    const std::string& filename, const std::string& channels)
{
  const char ch_trace   = channels[0];
  const char ch_trigger = channels[1];

  mat_t *matfile = Mat_CreateVer(filename.data(), NULL, MAT_FT_DEFAULT);
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
  if( !readWaveform(logger, vi, ch_trace, 0, trace) ) {
    return false;
  }
  if( !writeMatSamples(logger, matfile, "trace", trace) ) {
    return false;
  }

  SampleBuffer trigger;
  if( !readWaveform(logger, vi, ch_trigger, 0, trigger) ) {
    return false;
  }
  if( !writeMatSamples(logger, matfile, "trigger", trigger) ) {
    return false;
  }

  logger->logTextf(u8"Wrote file \"{}\".", filename);

  return true;
}
