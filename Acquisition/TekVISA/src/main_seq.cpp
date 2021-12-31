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
#include <charconv>
#include <format>
#include <string>

#define HAVE_INSTRUMENT

#ifdef HAVE_INSTRUMENT
# include <matio.h>
#endif

#include <csUtil/csDualLogger.h>
#include <csUtil/csLogger.h>
#include <csUtil/csSerial.h>
#include <csUtil/csStringUtil.h>
#include <csUtil/csTime.h>

#ifdef HAVE_INSTRUMENT
# include "instrument.h"
#endif
#include "util.h"

template<typename T>
using if_int_bool = std::enable_if_t<std::is_integral_v<T>,bool>;

struct Options {
  bool isValid() const
  {
    return count > 0  &&  serialDevice.size() > 0  &&  serialRate > 0;
  }

  void print(FILE *file = stdout) const
  {
    fprintf(file, "count      = %d\n", count);
    fprintf(file, "ser-device = %s\n", serialDevice.data());
    fprintf(file, "ser-rate   = %d\n", serialRate);
  }

  int count{0};
  std::string serialDevice;
  int serialRate{0};
};

inline bool isOption(const char *arg, const char *opt)
{
  return cs::startsWith(arg, cs::length(arg), opt, cs::length(opt));
}

inline const char *getOptionString(const char *arg, const char *opt)
{
  return isOption(arg, opt)
      ? arg + cs::length(opt)
      : nullptr;
}

template<typename T>
inline if_int_bool<T> parseIntOption(const char *opt, T& value, const T defValue = T{0})
{
  value = defValue;
  const std::from_chars_result result =
      std::from_chars(opt, opt + cs::length(opt), value, 10);
  return result.ec == std::errc{};
}

bool parseOptions(Options& options, int argc, char **argv)
{
  options = Options();

  const int numArgs = argc - 1;
  if( numArgs < 1 ) {
    fprintf(stdout, "%s <OPTIONS>\n\n", argv[0]);
    fprintf(stdout, "OPTIONS:\n\n");
    fprintf(stdout, "--count=INT\n");
    fprintf(stdout, "--ser-device=STRING\n");
    fprintf(stdout, "--ser-rate=INT\n");
    return false;
  }

  for(int i = 0; i < numArgs; i++) {
    const char *arg = argv[i + 1];

    const char *opt = nullptr;
    if(        (opt = getOptionString(arg, "--count=")) != nullptr ) {
      parseIntOption(opt, options.count);
    } else if( (opt = getOptionString(arg, "--ser-device=")) != nullptr ) {
      options.serialDevice = opt;
    } else if( (opt = getOptionString(arg, "--ser-rate=")) != nullptr ) {
      parseIntOption(opt, options.serialRate);
    } else {
      fprintf(stderr, "ERROR: Unknown option \"%s\"!\n", arg);
      return false;
    }
  }

  if( !options.isValid() ) {
    fprintf(stderr, "ERROR: Invalid or missing option!\n");
    return false;
  }

  return options.isValid();
}

template<typename... Args>
inline void logText(const csILogger *logger, const char8_t *fmt, Args&&... args)
{
  const std::string msg = std::vformat(cs::CSTR(fmt), std::make_format_args(args...));
  logger->logText(cs::UTF8(msg.data()));
}

template<typename... Args>
inline void logError(const csILogger *logger, const char8_t *fmt, Args&&... args)
{
  const std::string msg = std::vformat(cs::CSTR(fmt), std::make_format_args(args...));
  logger->logError(cs::UTF8(msg.data()));
}

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
    logError(logger, u8"Mat_VarCreate({})", varname);
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
    logError(logger, u8"No samples for \"{}\"!", varname);
    return false;
  }
  const SampleBuffer::size_type numSamples = samples.size() - 2;

  // (2) Time vector /////////////////////////////////////////////////////////

  SampleBuffer time;
  try {
    time.resize(numSamples, 0);
  } catch (...) {
    logError(logger, u8"Unable to create time data for \"{}\"!", varname);
    return false;
  }

  const double xIncr = samples[0];
  const double xZero = samples[1];
  for(SampleBuffer::size_type i = 0; i < time.size(); i++) {
    time[i] = double(i)*xIncr + xZero;
  }

  // (3) Output //////////////////////////////////////////////////////////////

  if( !writeMatVector(logger, matfile, varname, numSamples, samples.data() + 2) ) {
    logError(logger, u8"Unable to write data for \"{}\"!", varname);
    return false;
  }
  if( !writeMatVector(logger, matfile, "t_" + varname, numSamples, time.data()) ) {
    logError(logger, u8"Unable to write time data for \"{}\"!", varname);
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

  logText(logger, u8"Wrote file \"{}\".", filename);

  return true;
}
#endif

int main(int argc, char **argv)
{
#ifdef HAVE_INSTRUMENT
  constexpr unsigned int TOUT_INSTRUMENT = 2;
#endif
  constexpr unsigned int TOUT_SERIAL = 2;

  // (1) Options /////////////////////////////////////////////////////////////

  Options options;
  if( !parseOptions(options, argc, argv) ) {
    return EXIT_FAILURE;
  }
  options.print();

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
  logText(logger, u8"HORizontal:RECOrdlength = {}", length);

  float rate = 0;
  if( !querySampleRate(logger, vi, &rate) ) {
    return EXIT_FAILURE;
  }
  logText(logger, u8"HORizontal:SAMPLERate = {}", rate);
#endif

  Randomizer randomizer;

  csSerial serial;
  if( !serial.open(cs::UTF8(options.serialDevice.data()), options.serialRate) ) {
    logError(logger, u8"Unable to open serial device \"{}\"!", options.serialDevice);
    return EXIT_FAILURE;
  }

  txAesCmd('@', serial, randomizer);
  rxAesCmd(logger, serial, TOUT_SERIAL);

  // (4) Sequence ////////////////////////////////////////////////////////////

  const int maxIter       = options.count - 1;
  const int numIterDigits = countDigits(maxIter);
  for(int i = 0; i <= maxIter; i++) {
    logText(logger, u8"{:0{}}/{}", i, numIterDigits, maxIter);

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
