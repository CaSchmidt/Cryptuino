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

#define HAVE_STD_FORMAT
#include <cs/Logging/ILogger.h>
#include <cs/System/Time.h>

#include "TekVISA.h"

#include "Instrument.h"

////// public ////////////////////////////////////////////////////////////////

TekVISA::TekVISA(const ctor_tag&) noexcept
  : IInstrument()
{
}

TekVISA::~TekVISA() noexcept
{
  disconnect();
}

bool TekVISA::connect(const cs::ILogger *logger, const InstrumentOptions& options)
{
  ViStatus status;

  disconnect();

  status = viOpenDefaultRM(&_rm);
  if( handleError(logger, _rm, status, "viOpenDefaultRM()") ) {
    return false;
  }

  const RsrcList resources = queryInstruments(logger, _rm);
  if( resources.empty() ) {
    logger->logError(u8"No instruments found!");
    disconnect();
    return false;
  }
  const std::string instrument = resources.front(); // pick one...

  status = viOpen(_rm, const_cast<char*>(instrument.data()), VI_NULL, VI_NULL, &_vi);
  if( handleError(logger, _rm, status, "viOpen()") ) {
    disconnect();
    return false;
  }

  ViUInt32 length = 0;
  if( !queryRecordLength(logger, _vi, &length) ) {
    disconnect();
    return false;
  }
  logger->logTextf(u8"HORizontal:RECOrdlength = {}", length);

  float rate = 0;
  if( !querySampleRate(logger, _vi, &rate) ) {
    disconnect();
    return false;
  }
  logger->logTextf(u8"HORizontal:SAMPLERate = {}", rate);

  return true;
}

void TekVISA::disconnect()
{
  if( _rm != VI_NULL ) {
    viClose(_rm);
  }
  _rm = _vi = VI_NULL;
}

bool TekVISA::isConnected() const
{
  return _rm != VI_NULL  &&  _vi != VI_NULL;
}

bool TekVISA::readSamples(const cs::ILogger *logger, const int channel,
                          SampleBuffer *samples, const std::size_t numSamplesWant) const
{
  if( channel < 1  ||  channel > 4 ) {
    logger->logErrorf(u8"Invalid channel \"{}\"!", channel);
    return false;
  }
  return readWaveform(logger, _vi, char(channel + '0'), numSamplesWant, samples);
}

bool TekVISA::setupTrigger(const cs::ILogger *logger, const unsigned int tout) const
{
  if( !setSingleShotAcquisition(logger, _vi) ) {
    return false;
  }
  cs::sleep(tout);
  return true;
}
