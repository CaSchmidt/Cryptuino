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
#include <cstdint>

#include <array>
#include <vector>

#include <matio.h>
#include <visa.h>

using   ByteArray = std::vector<uint8_t>;
using SampleArray = std::vector<double>;

inline bool isDigit(const char c)
{
  return '0' <= c  &&  c <= '9';
}

bool handleError(const ViObject obj, const ViStatus status, const char *reason = nullptr)
{
  const bool is_error = status < VI_SUCCESS;
  if( is_error ) {
    ViChar buffer[1024];
    viStatusDesc(obj, status, buffer);
    if( reason != nullptr ) {
      fprintf(stderr, "%s: ", reason);
    }
    fprintf(stderr, "%s\n", buffer);
  }
  return is_error;
}

#define MAIN_ERROR(obj,reas)              \
  if( handleError(obj, status, reas) ) {  \
  return EXIT_FAILURE;                  \
  }

#define INSTR_ERROR(reas)                \
  if( handleError(vi, status, reas) ) {  \
  return false;                        \
  }

namespace instr {

  template<typename T>
  struct WaveformInfo_T {
    using value_type = T;

    T xIncr{0};
    T xZero{0};
    T yMult{0};
    T yOff{0};
    T yZero{0};
  };

  using WaveformInfo = WaveformInfo_T<float>;

  bool flush(ViSession vi)
  {
    ViStatus status;

    status = viFlush(vi, VI_WRITE_BUF | VI_READ_BUF_DISCARD);
    INSTR_ERROR("viFlush(WRITE_BUF|READ_BUF_DISCARD)");

    return true;
  }

  bool initializeData(ViSession vi, const char channel, const ViUInt32 numSamples)
  {
    ViStatus status;

    status = viPrintf(vi, (ViChar*)"DATa INIT\n");
    INSTR_ERROR("viPrintf(init)");

    status = viPrintf(vi, (ViChar*)"DATa:SOUrce CH%c\n", channel);
    INSTR_ERROR("viPrintf(source)");

    status = viPrintf(vi, (ViChar*)"DATa:STARt 1\n");
    INSTR_ERROR("viPrintf(start)");

    status = viPrintf(vi, (ViChar*)"DATa:STOP %d\n", numSamples);
    INSTR_ERROR("viPrintf(stop)");

    /*
     *  RIBinary := int16 (MSB)
     * SRIbinary := int16 (LSB)
     */
    status = viPrintf(vi, (ViChar*)"DATa:ENCdg SRIbinary\n");
    INSTR_ERROR("viPrintf(encdg)");

    status = viPrintf(vi, (ViChar*)"DATa:WIDth 2\n");
    INSTR_ERROR("viPrintf(width)");

    return true;
  }

  /*
   * NOTE: Instrument's read buffer MUST point to the binary data of the samples!
   */
  template<typename T>
  bool readSamples(ViSession vi, SampleArray& samples,
                   const ViUInt32 numSamples, const WaveformInfo& info)
  {
    constexpr std::size_t RAWVALUE_SIZE = sizeof(T);

    try {
      samples.resize(numSamples + 2, 0);
    } catch(...) {
      fprintf(stderr, "ERROR: samples.resize()\n");
      return false;
    }

    samples[0] = info.xIncr;
    samples[1] = info.xZero;

    ByteArray raw;
    try {
      raw.resize(numSamples*RAWVALUE_SIZE, 0);
    } catch(...) {
      fprintf(stderr, "ERROR: raw.resize()\n");
      return false;
    }
    const T *rawSamples = reinterpret_cast<const T*>(raw.data());

    ViUInt32 numBytesRead = 0;
    const ViStatus status = viRead(vi, raw.data(), raw.size(), &numBytesRead);
    INSTR_ERROR("viRead(raw)");

    const double yMult = info.yMult;
    const double yOff  = info.yOff;
    const double yZero = info.yZero;

    const ViUInt32 numSamplesRead = numBytesRead/RAWVALUE_SIZE;
    for(ViUInt32 i = 0; i < numSamplesRead; i++) {
      samples[2 + i] = (double(rawSamples[i]) - yOff)*yMult + yZero;
    }

    return true;
  }

  bool readWaveformInfo(ViSession vi, WaveformInfo& info)
  {
    static_assert( sizeof(WaveformInfo::value_type) == 4 );

    ViStatus status;

    status = viQueryf(vi, (ViChar*)"WFMOutpre:XINcr?", (ViChar*)"%f", &info.xIncr);
    INSTR_ERROR("viQueryf(xincr)");

    status = viQueryf(vi, (ViChar*)"WFMOutpre:XZEro?", (ViChar*)"%f", &info.xZero);
    INSTR_ERROR("viQueryf(xzero)");

    status = viQueryf(vi, (ViChar*)"WFMOutpre:YMUlt?\n", (ViChar*)"%f", &info.yMult);
    INSTR_ERROR("viQueryf(ymult)");

    status = viQueryf(vi, (ViChar*)"WFMOutpre:YOFf?\n", (ViChar*)"%f", &info.yOff);
    INSTR_ERROR("viQueryf(yoff)");

    status = viQueryf(vi, (ViChar*)"WFMOutpre:YZEro?\n", (ViChar*)"%f", &info.yZero);
    INSTR_ERROR("viQueryf(yzero)");

    return true;
  }

  bool seekSamples(ViSession vi)
  {
    ViStatus status;

    std::array<ViByte,16> temp;

    temp.fill(0);
    status = viRead(vi, temp.data(), 2, VI_NULL);
    INSTR_ERROR("viRead(binary block header)");
    if( temp[0] != '#'  ||  !isDigit(temp[1]) ) {
      fprintf(stderr, "ERROR: Invalid binary block header!\n");
      return false;
    }
    const ViUInt32 numBinaryBytes = temp[1] - '0';

    printf("binary bytes = %lu\n", numBinaryBytes);

    printf("%c%c...\n", temp[0], temp[1]);

    temp.fill(0);
    status = viRead(vi, temp.data(), numBinaryBytes, VI_NULL);
    INSTR_ERROR("viRead(binary block size)");
    for(ViUInt32 i = 0; i < numBinaryBytes; i++) {
      if( !isDigit(temp[i]) ) {
        fprintf(stderr, "ERROR: Invalid binary block size!\n");
        return false;
      }
    }

    printf("...");
    for(ViUInt32 i = 0; i < numBinaryBytes; i++) {
      printf("%c", temp[i]);
    }
    printf("\n");

    return true;
  }

} // namespace instr

bool readWaveform(ViSession vi, const char ch, const ViUInt32 numSamplesWant,
                  SampleArray& samples)
{
  printf("-------\n");

  ViStatus status;

  status = viSetAttribute(vi, VI_ATTR_WR_BUF_OPER_MODE, VI_FLUSH_ON_ACCESS);
  INSTR_ERROR("viSetAttribute(WR_BUF_OPER_MODE)");
  status = viSetAttribute(vi, VI_ATTR_RD_BUF_OPER_MODE, VI_FLUSH_ON_ACCESS);
  INSTR_ERROR("viSetAttribute(RD_BUF_OPER_MODE)");

  status = viPrintf(vi, (ViChar*)"HEADer OFF\n");
  INSTR_ERROR("viPrintf(header)");

  ViUInt32 numSamplesHave = 0;
  status = viQueryf(vi, (ViChar*)"HORizontal:RECOrdlength?", (ViChar*)"%ld", &numSamplesHave);
  INSTR_ERROR("viQueryf(length)");

  printf("have samples = %lu\n", numSamplesHave);

  const ViUInt32 numSamples = std::min<ViUInt32>(numSamplesHave, numSamplesWant);
  printf("samples = %lu\n", numSamples);

  if( !instr::initializeData(vi, ch, numSamples) ) {
    return false;
  }

  instr::WaveformInfo info;
  if( !instr::readWaveformInfo(vi, info) ) {
    return false;
  }

  printf("xIncr = %g\n", info.xIncr);
  printf("xZero = %g\n", info.xZero);
  printf("yMult = %g\n", info.yMult);
  printf("yOff  = %g\n", info.yOff);
  printf("yZero = %g\n", info.yZero);

  status = viPrintf(vi, (ViChar*)"CURVE?\n");
  INSTR_ERROR("viPrintf(curve)");

  if( !instr::flush(vi) ) {
    return false;
  }

  status = viSetAttribute(vi, VI_ATTR_RD_BUF_OPER_MODE, VI_FLUSH_DISABLE);
  INSTR_ERROR("viSetAttribute(RD_BUF_OPER_MODE)");

  status = viWrite(vi, (unsigned char*)"CURVE?", 6, VI_NULL);
  INSTR_ERROR("viWrite(curve)");

  if( !instr::seekSamples(vi) ) {
    return false;
  }

  if( !instr::readSamples<int16_t>(vi, samples, numSamples, info) ) {
    return false;
  }

  if( !instr::flush(vi) ) {
    return false;
  }

  return true;
}

void writeMatVector(mat_t *file, const char *varname,
                    const std::size_t numSamples, const double *data)
{
  std::array<std::size_t,2> dims;
  dims[0] = numSamples;
  dims[1] = 1;

  matvar_t *var = Mat_VarCreate(varname, MAT_C_DOUBLE, MAT_T_DOUBLE,
                                dims.size(), dims.data(), const_cast<double*>(data),
                                MAT_F_DONT_COPY_DATA);
  if( var != NULL ) {
    Mat_VarWrite(file, var, MAT_COMPRESSION_ZLIB);
    Mat_VarFree(var);
  }
}

bool writeMatSamples(const char *filename, const SampleArray& samples)
{
  const std::size_t numSamples = samples.size() - 2;

  // (1) Time vector /////////////////////////////////////////////////////////

  SampleArray time;
  try {
    time.resize(numSamples, 0);
  } catch (...) {
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
    return false;
  }

  // (3) Write vectors ///////////////////////////////////////////////////////

  writeMatVector(file, "sig", numSamples, samples.data() + 2);
  writeMatVector(file, "tim", numSamples, time.data());

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

  ViStatus status;

  ViSession rm;
  status = viOpenDefaultRM(&rm);
  MAIN_ERROR(rm, "viOpenDefaultRM()");

  ViSession vi;
  status = viOpen(rm, resource, VI_NULL, VI_NULL, &vi);
  MAIN_ERROR(rm, "viOpen()");

  SampleArray samples;
  readWaveform(vi, channel, numSamples, samples);
  writeMatSamples("output.mat", samples);

  viClose(vi);
  viClose(rm);

  return EXIT_SUCCESS;
}
