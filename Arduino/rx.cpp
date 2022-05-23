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

#include "rx.h"

#include "I_ser.h"

#include "ardutil.h"
#include "key.h"

uint8_t rx_buffer[RXBUF_SIZE];

uint8_t rx_len = 0;

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
    I_ser_puts("ERROR: Invalid AES data!\n");
  }
  return result;
}

bool rx_have_cmd(const char *cmd)
{
  const uint8_t len = length(cmd);
  return rx_len == len  &&  compareI((const char*)rx_buffer, cmd, len);
}
