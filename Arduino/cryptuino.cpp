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

#include "cryptuino.h"

#include "I_pin.h"
#include "I_random.h"
#include "I_ser.h"

#include "crypto.h"
#include "key.h"
#include "rx.h"

extern "C" void cryptuino_init(void)
{
  I_pin_init();
  I_random_init();
  I_ser_init(9600);

  I_ser_puts("Welcome to Cryptuino!\n");
  I_ser_puts("---------------------\n");

  key_init();

  rx_len = 0;
}

extern "C" void cryptuino_shell(void)
{
  if( I_ser_available() < 1 ) {
    return;
  }

  bool have_cmd = false;
  while( I_ser_available() > 0 ) {
    const uint8_t c = I_ser_getc();
    if( c == '\n'  ||  c == '\r' ) {
      have_cmd = true;
      break;
    } else if( rx_len < RXBUF_SIZE ) {
      rx_buffer[rx_len++] = c;
    }
  }

  if( !have_cmd  ||  rx_len < 1 ) {
    return;
  }

  if(        rx_have_cmd("genkey") ) {
    key_generate();
  } else if( rx_have_cmd("showkey") ) {
    key_show();
  } else if( rx_have_aes_data() ) {
    if(        rx_buffer[0] == '@' ) {
      key_set(rx_buffer + 1);
    } else if( rx_buffer[0] == '#' ) {
      crypto_encrypt(rx_buffer + 1);
    }
  } else {
    I_ser_puts("ERROR: Invalid command!\n");
  }

  rx_len = 0;
}
