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

#include <csUtil/csCharUtil.h>
#include <csUtil/csILogger.h>

#include "instrument.h"

#include "util.h"

////// Private ///////////////////////////////////////////////////////////////

namespace priv {

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

  bool initializeData(const csILogger *logger, ViSession vi,
                      const char channel, const ViUInt32 numSamples)
  {
    ViStatus status;

    status = viPrintf(vi, (ViChar*)"DATa INIT\n");
    if( handleError(logger, vi, status, "DATa INIT") ) {
      return false;
    }

    status = viPrintf(vi, (ViChar*)"DATa:SOUrce CH%c\n", channel);
    if( handleError(logger, vi, status, "DATa:SOUrce") ) {
      return false;
    }

    status = viPrintf(vi, (ViChar*)"DATa:STARt 1\n");
    if( handleError(logger, vi, status, "DATa:STARt") ) {
      return false;
    }

    status = viPrintf(vi, (ViChar*)"DATa:STOP %d\n", numSamples);
    if( handleError(logger, vi, status, "DATa:STOP") ) {
      return false;
    }

    /*
     *  RIBinary := int16 (MSB)
     * SRIbinary := int16 (LSB)
     */
    status = viPrintf(vi, (ViChar*)"DATa:ENCdg SRIbinary\n");
    if( handleError(logger, vi, status, "DATa:ENCdg") ) {
      return false;
    }

    status = viPrintf(vi, (ViChar*)"DATa:WIDth 2\n");
    if( handleError(logger, vi, status, "DATa:WIDth") ) {
      return false;
    }

    return true;
  }

  /*
   * NOTE: Instrument's read buffer MUST already point to the binary data of the samples!
   */
  template<typename T>
  bool readSamples(const csILogger *logger, ViSession vi,
                   SampleBuffer& samples, const ViUInt32 numSamples, const WaveformInfo& info)
  {
    constexpr std::size_t RAWVALUE_SIZE = sizeof(T);

    try {
      samples.resize(numSamples + 2, 0);
    } catch(...) {
      logger->logError(u8"samples.resize()");
      return false;
    }

    samples[0] = info.xIncr;
    samples[1] = info.xZero;

    ByteBuffer raw;
    try {
      raw.resize(numSamples*RAWVALUE_SIZE, 0);
    } catch(...) {
      logger->logError(u8"raw.resize()");
      return false;
    }
    const T *rawSamples = reinterpret_cast<const T*>(raw.data());

    ViUInt32 numBytesRead = 0;
    const ViStatus status = viRead(vi, raw.data(), ViUInt32(raw.size()), &numBytesRead);
    if( handleError(logger, vi, status, "viRead(raw)") ) {
      return false;
    }

    const double yMult = info.yMult;
    const double yOff  = info.yOff;
    const double yZero = info.yZero;

    const ViUInt32 numSamplesRead = numBytesRead/RAWVALUE_SIZE;
    for(ViUInt32 i = 0; i < numSamplesRead; i++) {
      samples[2 + i] = (double(rawSamples[i]) - yOff)*yMult + yZero;
    }

    return true;
  }

  bool readWaveformInfo(const csILogger *logger, ViSession vi,
                        WaveformInfo& info)
  {
    static_assert( sizeof(WaveformInfo::value_type) == 4 );

    ViStatus status;

    status = viQueryf(vi, (ViChar*)"WFMOutpre:XINcr?\n", (ViChar*)"%f", &info.xIncr);
    if( handleError(logger, vi, status, "WFMOutpre:XINcr?") ) {
      return false;
    }

    status = viQueryf(vi, (ViChar*)"WFMOutpre:XZEro?\n", (ViChar*)"%f", &info.xZero);
    if( handleError(logger, vi, status, "WFMOutpre:XZEro?") ) {
      return false;
    }

    status = viQueryf(vi, (ViChar*)"WFMOutpre:YMUlt?\n", (ViChar*)"%f", &info.yMult);
    if( handleError(logger, vi, status, "WFMOutpre:YMUlt?") ) {
      return false;
    }

    status = viQueryf(vi, (ViChar*)"WFMOutpre:YOFf?\n", (ViChar*)"%f", &info.yOff);
    if( handleError(logger, vi, status, "WFMOutpre:YOFf?") ) {
      return false;
    }

    status = viQueryf(vi, (ViChar*)"WFMOutpre:YZEro?\n", (ViChar*)"%f", &info.yZero);
    if( handleError(logger, vi, status, "WFMOutpre:YZEro?") ) {
      return false;
    }

    return true;
  }

  bool seekSamples(const csILogger *logger, ViSession vi)
  {
    ViStatus status;

    std::array<ViByte,16> temp;

    temp.fill(0);
    status = viRead(vi, temp.data(), 2, VI_NULL);
    if( handleError(logger, vi, status, "viRead(binary block header)") ) {
      return false;
    }
    if( temp[0] != '#'  ||  !cs::isDigit<char>(temp[1]) ) {
      logger->logError(u8"Invalid binary block header!");
      return false;
    }
    const ViUInt32 numBinaryBytes = temp[1] - '0';

    temp.fill(0);
    status = viRead(vi, temp.data(), numBinaryBytes, VI_NULL);
    if( handleError(logger, vi, status, "viRead(binary block size)") ) {
      return false;
    }
    for(ViUInt32 i = 0; i < numBinaryBytes; i++) {
      if( !cs::isDigit<char>(temp[i]) ) {
        logger->logError(u8"Invalid binary block size!");
        return false;
      }
    }

    return true;
  }

} // namespace priv

////// Public ////////////////////////////////////////////////////////////////

bool flush(const csILogger *logger, ViSession vi,
           const bool rd_discard, const bool wr_discard)
{
  ViStatus status;

  ViUInt16 mask = 0;
  mask |= rd_discard
      ? VI_READ_BUF_DISCARD
      : VI_READ_BUF;
  mask |= wr_discard
      ? VI_WRITE_BUF_DISCARD
      : VI_WRITE_BUF;

  status = viFlush(vi, mask);
  if( handleError(logger, vi, status, "viFlush()") ) {
    return false;
  }

  return true;
}

bool handleError(const csILogger *logger, const ViObject obj,
                 const ViStatus status, const char *reason)
{
  const bool is_error = status < VI_SUCCESS;
  if( is_error ) {
    std::array<ViChar,1024> buffer;
    buffer.fill(0);
    viStatusDesc(obj, status, buffer.data());

    std::u8string msg;
    if( reason != nullptr ) {
      msg += cs::UTF8(reason);
      msg += u8": ";
    }
    msg += cs::UTF8(buffer.data());

    logger->logError(msg);
  }
  return is_error;
}

RsrcList queryInstruments(const csILogger *logger, ViSession rm)
{
  std::array<ViChar,1024> buffer;
  ViUInt32          numResources;
  ViFindList           resources;
  ViStatus                status;

  buffer.fill(0);
  status = viFindRsrc(rm, (ViChar*)"?*INSTR", &resources, &numResources, buffer.data());
  if( handleError(logger, rm, status, "viFindRsrc(?*INSTR)") ) {
    return RsrcList();
  }

  if( numResources < 1 ) {
    viClose(resources);
    return RsrcList();
  }

  RsrcList result;
  for(ViUInt32 i = 0; i < numResources; i++) {
    result.push_back(RsrcString(buffer.data()));

    if( i < numResources - 1 ) {
      buffer.fill(0);
      status = viFindNext(resources, buffer.data());
      if( handleError(logger, rm, status, "viFindNext()") ) {
        viClose(resources);
        return RsrcList();
      }
    }
  }

  viClose(resources);

  return result;
}

bool queryRecordLength(const csILogger *logger, ViSession vi,
                       ViUInt32 *length)
{
  ViStatus status;

  if( !setBufferAttributes(logger, vi, true, true) ) {
    return false;
  }

  status = viQueryf(vi, (ViChar*)"HORizontal:RECOrdlength?\n", (ViChar*)"%ld", length);
  if( handleError(logger, vi, status, "HORizontal:RECOrdlength?") ) {
    return false;
  }

  return true;
}

bool querySampleRate(const csILogger *logger, ViSession vi,
                     float *rate)
{
  ViStatus status;

  if( !setBufferAttributes(logger, vi, true, true) ) {
    return false;
  }

  status = viQueryf(vi, (ViChar*)"HORizontal:SAMPLERate?\n", (ViChar*)"%f", rate);
  if( handleError(logger, vi, status, "HORizontal:SAMPLERate?") ) {
    return false;
  }

  return true;
}

bool readWaveform(const csILogger *logger, ViSession vi,
                  const char ch, const ViUInt32 numSamplesWant, SampleBuffer& samples)
{
  ViStatus status;

  if( !setBufferAttributes(logger, vi, true, true) ) {
    return false;
  }

  status = viPrintf(vi, (ViChar*)"HEADer OFF\n");
  if( handleError(logger, vi, status, "HEADer OFF") ) {
    return false;
  }

  ViUInt32 numSamplesHave = 0;
  status = viQueryf(vi, (ViChar*)"HORizontal:RECOrdlength?\n", (ViChar*)"%ld", &numSamplesHave);
  if( handleError(logger, vi, status, "HORizontal:RECOrdlength?") ) {
    return false;
  }

  const ViUInt32 numSamples = numSamplesWant > 0
      ? std::min<ViUInt32>(numSamplesHave, numSamplesWant)
      : numSamplesHave;

  if( !priv::initializeData(logger, vi, ch, numSamples) ) {
    return false;
  }

  priv::WaveformInfo info;
  if( !priv::readWaveformInfo(logger, vi, info) ) {
    return false;
  }

  status = viPrintf(vi, (ViChar*)"CURVe?\n");
  if( handleError(logger, vi, status, "CURVe?") ) {
    return false;
  }

  if( !flush(logger, vi) ) {
    return false;
  }

  if( !setBufferAttributes(logger, vi, false, true) ) {
    return false;
  }

  if( !priv::seekSamples(logger, vi) ) {
    return false;
  }

  if( !priv::readSamples<int16_t>(logger, vi, samples, numSamples, info) ) {
    return false;
  }

  if( !flush(logger, vi) ) {
    return false;
  }

  return true;
}

bool setBufferAttributes(const csILogger *logger, ViSession vi,
                         const bool rd_flush, const bool wr_flush)
{
  ViStatus status;

  const ViAttrState rd_state = rd_flush
      ? VI_FLUSH_ON_ACCESS
      : VI_FLUSH_DISABLE; // default state
  status = viSetAttribute(vi, VI_ATTR_RD_BUF_OPER_MODE, rd_state);
  if( handleError(logger, vi, status, "viSetAttribute(VI_ATTR_RD_BUF_OPER_MODE)") ) {
    return false;
  }

  const ViAttrState wr_state = wr_flush
      ? VI_FLUSH_ON_ACCESS
      : VI_FLUSH_WHEN_FULL; // default state
  status = viSetAttribute(vi, VI_ATTR_WR_BUF_OPER_MODE, wr_state);
  if( handleError(logger, vi, status, "viSetAttribute(VI_ATTR_WR_BUF_OPER_MODE)") ) {
    return false;
  }

  return true;
}

bool setSingleShotAcquisition(const csILogger *logger, ViSession vi)
{
  ViStatus status;

  if( !setBufferAttributes(logger, vi, true, true) ) {
    return false;
  }

  status = viPrintf(vi, (ViChar*)"ACQuire:STATE STOP\n");
  if( handleError(logger, vi, status, "ACQuire:STATE STOP") ) {
    return false;
  }

  status = viPrintf(vi, (ViChar*)"ACQuire:MODe SAMple\n");
  if( handleError(logger, vi, status, "ACQuire:MODe SAMple") ) {
    return false;
  }

  status = viPrintf(vi, (ViChar*)"ACQuire:STOPAfter SEQuence\n");
  if( handleError(logger, vi, status, "ACQuire:STOPAfter SEQuence") ) {
    return false;
  }

  status = viPrintf(vi, (ViChar*)"ACQuire:STATE RUN\n");
  if( handleError(logger, vi, status, "ACQuire:STATE RUN") ) {
    return false;
  }

  return true;
}
