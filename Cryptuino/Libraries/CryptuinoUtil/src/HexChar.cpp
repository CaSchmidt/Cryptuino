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

#include <csUtil/csStringUtil.h>
#include <csUtil/csNumeric.h>

#include "HexChar.h"

////// Public ////////////////////////////////////////////////////////////////

ByteBuffer extractHexBytes(std::string text, const char *prefix)
{
  const std::size_t numPrefix = cs::length(prefix);
  if( numPrefix > 0 ) {
    text.erase(0, numPrefix);
  }

  cs::removeAll(&text, cs::lambda_is_space<char>());
  if( cs::isOdd(text.size()) ) {
    return ByteBuffer();
  }

  ByteBuffer buffer;
  try {
    buffer.resize(text.size()/2);
  } catch(...) {
    return ByteBuffer();
  }

  for(std::size_t i = 0; i < buffer.size(); i++) {
    const uint8_t hi = fromHexChar(text[i*2]);
    const uint8_t lo = fromHexChar(text[i*2 + 1]);
    if( hi == 0xFF  ||  lo == 0xFF ) {
      return ByteBuffer();
    }
    buffer[i] = (hi << 4) | lo;
  }

  return buffer;
}
