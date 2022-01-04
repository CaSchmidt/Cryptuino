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

#include <iostream>

#define HAVE_INSTRUMENT

#ifdef HAVE_INSTRUMENT
# include <matio.h>
#endif

#define HAVE_STD_FORMAT
#include <csUtil/csDualLogger.h>
#include <csUtil/csLogger.h>
#include <csUtil/csSerial.h>
#include <csUtil/csTime.h>

#include "CmdOptions.h"
#ifdef HAVE_INSTRUMENT
# include "instrument.h"
#endif
#include "util.h"

#ifdef HAVE_INSTRUMENT
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
  } catch (...) {
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

bool writeMatOutput(const csILogger *logger, ViSession vi,
                    const std::string& filename)
{
  constexpr char CH_TRACE   = '1';
  constexpr char CH_TRIGGER = '2';

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
  if( !readWaveform(logger, vi, CH_TRACE, 0, trace) ) {
    return false;
  }
  if( !writeMatSamples(logger, matfile, "trace", trace) ) {
    return false;
  }

  SampleBuffer trigger;
  if( !readWaveform(logger, vi, CH_TRIGGER, 0, trigger) ) {
    return false;
  }
  if( !writeMatSamples(logger, matfile, "trigger", trigger) ) {
    return false;
  }

  logger->logTextf(u8"Wrote file \"{}\".", filename);

  return true;
}
#endif

CmdOptionsPtr options()
{
  CmdOptionsPtr opts = CmdOptions::make();

  CmdOptionPtr ptr;

  ptr = CmdStringValueOption::make("ser-device", std::string(), true, true,
                                   [](const std::string& s) -> bool { return s.size() > 0; });
  opts->add(ptr);

  ptr = CmdStringValueOption::make("channels", std::string(), true, false,
                                   [](const std::string& s) -> bool {
    return s.size() == 2  &&  cs::isDigit(s[0])  &&  cs::isDigit(s[1]);
  }, "12");
  opts->add(ptr);

  ptr = CmdIntValueOption::make("ser-rate", std::string(), true, false,
                                [](const int i) -> bool { return i > 0; }, 9600);
  opts->add(ptr);

  ptr = CmdIntValueOption::make("count", std::string(), true, true,
                                [](const int i) -> bool { return i > 0; });
  opts->add(ptr);

  return opts;
}

int main(int argc, char **argv)
{
#ifdef HAVE_INSTRUMENT
  constexpr unsigned int TOUT_INSTRUMENT = 2;
#endif
  constexpr unsigned int TOUT_SERIAL = 2;

  // (1) Options /////////////////////////////////////////////////////////////

  CmdOptionsPtr opts = options();
  if( !opts->parse(std::cerr, argc, argv) ) {
    return EXIT_FAILURE;
  }

  const int              count = opts->value<int>("count");
  const std::string ser_device = opts->value<std::string>("ser-device");
  const int         ser_rate   = opts->value<int>("ser-rate");

  // (2) Logging /////////////////////////////////////////////////////////////

  constexpr const char *log_filename = "sequence.log.txt";
  FILE *file = fopen(log_filename, "wb");
  if( file == NULL ) {
    fprintf(stderr, "ERROR: Unable to open file \"%s\"!\n", log_filename);
    return EXIT_FAILURE;
  }

  const csLogger     log_con;
  const csLogger     log_file(file);
  const csDualLogger log_dual(&log_file, &log_con);

  const csILogger *logger = &log_dual;

  // (3) Setup ///////////////////////////////////////////////////////////////

#ifdef HAVE_INSTRUMENT
  ViStatus status;

  ViSession rm;
  status = viOpenDefaultRM(&rm);
  if( handleError(logger, rm, status, "viOpenDefaultRM()") ) {
    return EXIT_FAILURE;
  }
  ScopeGuard guard_rm([&](void) -> void {
    viClose(rm);
  });

  const RsrcList resources = queryInstruments(logger, rm);
  if( resources.empty() ) {
    logger->logError(u8"No instruments found!");
    return EXIT_FAILURE;
  }
  const std::string instrument = resources.front(); // pick one...

  ViSession vi;
  status = viOpen(rm, const_cast<char*>(instrument.data()), VI_NULL, VI_NULL, &vi);
  if( handleError(logger, rm, status, "viOpen()") ) {
    return EXIT_FAILURE;
  }

  ViUInt32 length = 0;
  if( !queryRecordLength(logger, vi, &length) ) {
    return EXIT_FAILURE;
  }
  logger->logTextf(u8"HORizontal:RECOrdlength = {}", length);

  float rate = 0;
  if( !querySampleRate(logger, vi, &rate) ) {
    return EXIT_FAILURE;
  }
  logger->logTextf(u8"HORizontal:SAMPLERate = {}", rate);
#endif

  Randomizer randomizer;

  csSerial serial;
  if( !serial.open(cs::UTF8(ser_device.data()), ser_rate) ) {
    logger->logErrorf(u8"Unable to open serial device \"{}\"!", ser_device);
    return EXIT_FAILURE;
  }

  txAesCmd('@', serial, randomizer);
  rxAesCmd(logger, serial, TOUT_SERIAL);

  // (4) Sequence ////////////////////////////////////////////////////////////

  const int maxIter       = count - 1;
  const int numIterDigits = countDigits(maxIter);
  for(int i = 0; i <= maxIter; i++) {
    logger->logTextf(u8"{:0{}}/{}", i, numIterDigits, maxIter);

    // (4.1) Arm instrument //////////////////////////////////////////////////

#ifdef HAVE_INSTRUMENT
    setSingleShotAcquisition(logger, vi);
    csSleep(TOUT_INSTRUMENT);
#endif

    // (4.2) Request encryption //////////////////////////////////////////////

    txAesCmd('#', serial, randomizer);

    // (4.3) Wait for response ///////////////////////////////////////////////

    rxAesCmd(logger, serial, TOUT_SERIAL);

    // (4.4) Store samples ///////////////////////////////////////////////////

#ifdef HAVE_INSTRUMENT
    const std::string filename = std::format("{:0{}}.mat", i, numIterDigits);
    writeMatOutput(logger, vi, filename);
#endif
  }

  // Done! ///////////////////////////////////////////////////////////////////

  return EXIT_SUCCESS;
}
