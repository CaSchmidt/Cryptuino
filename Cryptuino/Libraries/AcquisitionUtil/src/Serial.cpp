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

#include <csUtil/csILogger.h>
#include <csUtil/csSerial.h>
#include <csUtil/csStringUtil.h>
#include <csUtil/csTime.h>

#include "Serial.h"

#include "HexChar.h"
#include "Randomizer.h"

////// Private ///////////////////////////////////////////////////////////////

namespace priv {

  constexpr std::size_t AES_DATA_BYTES = 16;

  using  AesBuffer = std::array<uint8_t,AES_DATA_BYTES>;
  using CharBuffer = std::array<char,1024>;

  void buildAesCmd(const char prefix, char *cmd, const uint8_t *aes_data)
  {
    *cmd++ = prefix;
    for(std::size_t i = 0; i < AES_DATA_BYTES; i++) {
      *cmd++ = toHexChar(aes_data[i], true);
      *cmd++ = toHexChar(aes_data[i]);
    }
    *cmd = '\n';
  }

} // namespace priv

////// Public ////////////////////////////////////////////////////////////////

void rxAesCmd(const csILogger *logger, const csSerial& serial, const unsigned int tout)
{
  priv::CharBuffer buffer;
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
  priv::AesBuffer  aesdata;
  priv::CharBuffer buffer;
  buffer.fill(0);

  randomizer.generate(aesdata.data(), aesdata.size());
  priv::buildAesCmd(prefix, buffer.data(), aesdata.data());
  serial.write(buffer.data(), cs::length(buffer.data()));
}
