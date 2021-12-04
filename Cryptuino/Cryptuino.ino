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

#include "key.h"
#include "util.h"

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

inline bool rx_have_aes_cmd()
{
  return rx_buffer[0] == '#'  ||  rx_buffer[0] == '@';
}

bool rx_have_aes_data()
{
  if( rx_len != AES_KEY_BYTES*2 + 1  ||  !rx_have_aes_cmd() ) {
    return false;
  }
  bool result = true;
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
 * Main **********************************************************************
 *****************************************************************************/

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
    if(        rx_buffer[0] == '@' ) {
      key_set(rx_buffer + 1);
    } else if( rx_buffer[0] == '#' ) {
      /* ... */
    }
  } else {
    Serial.println("ERROR: Invalid command!");
  }

  rx_len = 0;
}
