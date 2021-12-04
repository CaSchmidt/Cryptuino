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

#include <Arduino.h>

#include "util.h"

#include "key.h"

bool compareI(const char *s1, const char *s2, const size_t len)
{
  if( s1 == nullptr  ||  s2 == nullptr ) {
    return false; 
  }
  const size_t l1 = len < 1
  ? length(s1)
  : len;
  const size_t l2 = len < 1
  ? length(s2)
  : len;
  if( l1 != l2  ||  l1 == 0 ) {
    return false;
  }
  for(size_t i = 0; i < l1; i++) {
    if( toLower(s1[i]) != toLower(s2[i]) ) {
      return false;
    }
  }
  return true;
}

uint8_t fromHexChar(const char c)
{
  if(        '0' <= c  &&  c <= '9' ) {
    return c - '0';
  } else if( 'a' <= c  &&  c <= 'f' ) {
    return c - 'a' + 10;
  } else if( 'A' <= c  &&  c <= 'F' ) {
    return c - 'A' + 10;
  }
  return 0xFF;
}

void outputAesData(const uint8_t *data)
{
  for(uint8_t i = 0; i < AES_KEY_BYTES; i++) {
    Serial.write(' ');
    Serial.write(toHexChar(data[i], true));
    Serial.write(toHexChar(data[i]));
  }
}

void readAesData(uint8_t *data, const uint8_t *ascii)
{
  for(uint8_t i = 0; i < AES_KEY_BYTES; i++) {
    const uint8_t hi = fromHexChar(ascii[i*2]);
    const uint8_t lo = fromHexChar(ascii[i*2 + 1]);

    data[i]  = hi << 4;
    data[i] |= lo & 0x0F;
  }
}

char toHexChar(const uint8_t in, const bool hi_nibble)
{
  const uint8_t nibble = hi_nibble
  ? in >>   4
  : in & 0x0F;
  return nibble >= 10
  ? nibble - 10 + 'A'
  : nibble + '0';
}

uint8_t toLower(const uint8_t in)
{
  return 'A' <= in  &&  in <= 'Z'
  ? in - 'A' + 'a'
  : in;
}

uint8_t toUpper(const uint8_t in)
{
  return 'a' <= in  &&  in <= 'z'
  ? in - 'a' + 'A'
  : in;
}
