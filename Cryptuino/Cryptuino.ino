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

/*****************************************************************************
 * Constants *****************************************************************
 *****************************************************************************/

constexpr uint8_t AES_KEY_BYTES = 16;

/*****************************************************************************
 * Utility *******************************************************************
 *****************************************************************************/

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

namespace impl {
  constexpr size_t length2(const char *s)
  {
    return *s == '\0'
    ? 0
    : 1 + length2(s + 1);
  }
}

constexpr size_t length(const char *s)
{
  return s == nullptr
  ? 0
  : impl::length2(s);
}

char toHexChar(const uint8_t in, const bool hi_nibble = false)
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

bool compareI(const char *s1, const char *s2, const size_t len = 0)
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

void outputKey(const uint8_t *key)
{
  for(uint8_t i = 0; i < AES_KEY_BYTES; i++) {
    Serial.write(' ');
    Serial.write(toHexChar(key[i], true));
    Serial.write(toHexChar(key[i]));
  }
}

/*****************************************************************************
 * RX Buffer *****************************************************************
 *****************************************************************************/

#define CMD_GENKEY   "genkey"
#define CMD_SHOWKEY  "showkey"

constexpr uint8_t RXBUF_SIZE = 128;
uint8_t rx_buffer[RXBUF_SIZE];

uint8_t rx_len = 0;

bool rx_have_cmd(const char *cmd)
{
  const uint8_t len = length(cmd);
  return rx_len == len  &&  compareI((const char*)rx_buffer, cmd, len);
}

bool rx_have_aes_data()
{
  bool result = true;
  if( rx_len != AES_KEY_BYTES*2 + 1  ||  rx_buffer[0] != '#' ) {
    result = false;
  }
  for(uint8_t i = 1; i <= AES_KEY_BYTES*2; i++) {
    if( fromHexChar(rx_buffer[i]) == 0xFF ) {
      result = false;
    }
  }
  if( !result ) {
    Serial.println("ERROR: Invalid AES data!");
  }
  return result;
}

/*****************************************************************************
 * Key ***********************************************************************
 *****************************************************************************/

uint8_t key[AES_KEY_BYTES];

void key_init()
{
  for(uint8_t i = 0; i < AES_KEY_BYTES; i++) {
    key[i] = 0;
  }
}

void key_show()
{
  Serial.print("Key =");
  outputKey(key);
  Serial.println("");
}

void key_generate()
{
  Serial.println("Generating random key...");
  randomSeed(analogRead(A0));
  for(uint8_t i = 0; i < AES_KEY_BYTES; i++) {
    const uint8_t r = random(0, 256);
    key[i] = r;
  }
}

/*****************************************************************************
 * Main **********************************************************************
 *****************************************************************************/

#if 0
void test_cmpi(const char *s1, const char *s2)
{
  Serial.print(s1);
  Serial.print(" ");
  compareI(s1, s2)
  ? Serial.print("==")
  : Serial.print("!=");
  Serial.print(" ");
  Serial.print(s2);
  Serial.println("");
}
#endif

void setup() {
  Serial.begin(9600, SERIAL_8N1);
  while( !Serial ) {
    ;
  }
  Serial.println("Welcome to Cryptuino!");
  Serial.println("---------------------");

  key_init();

  rx_len = 0;
}

void loop() {
  if( Serial.available() < 1 ) {
    return;
  }

  bool have_cmd = false;
  while( Serial.available() > 0 ) {
    const uint8_t c = Serial.read();
    if( c == '\n' ) {
      have_cmd = true;
      break;
    } else if( rx_len < RXBUF_SIZE ) {
      rx_buffer[rx_len++] = c;
    }
  }

  if( !have_cmd) {
    return;
  }

  if(        rx_have_cmd(CMD_GENKEY) ) {
    key_generate();
  } else if( rx_have_cmd(CMD_SHOWKEY) ) {
    key_show();

  } else if( rx_have_aes_data() ) {
  } else {
    Serial.println("ERROR: Invalid command!");
  }

  rx_len = 0;

  /*
  const uint8_t  in = Serial.read();
  const uint8_t out = toUpper(in);
  Serial.write(out);
  Serial.println("");
  */
}
