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

#include <cstdio>
#include <cstdlib>
#include <cstdint>

#include <aes.hpp>

#define HAVE_STD_FORMAT
#include <cs/Logging/Logger.h>

#include "Campaign.h"
#include "CampaignReader.h"
#include "Cryptuino.h"

void printHex(const ByteBuffer& buffer, const bool eol = true, FILE *file = stdout)
{
  for(const ByteBuffer::value_type b : buffer) {
    fprintf(file, " %02X", b);
  }
  if( eol ) {
    fprintf(file, "\n");
  }
}

inline void printHex(const ByteBuffer& buffer, FILE *file)
{
  printHex(buffer, true, file);
}

void print(const Campaign& campaign, const bool do_aes = true, FILE *file = stdout)
{
  const bool have_aes = do_aes  &&  campaign.key.size() == AES128_KEY_SIZE;

  AES_ctx ctx;
  if( have_aes ) {
    AES_init_ctx(&ctx, campaign.key.data());
  }

  // (1) Output key (if present) /////////////////////////////////////////////

  if( !campaign.key.empty() ) {
    fprintf(file, "Setting key"); printHex(campaign.key, file);
  }

  // (2) Output entries... ///////////////////////////////////////////////////

  const std::string maxIter = campaign.lastEntryName();
  for(const CampaignEntry& entry : campaign.entries) {
    ByteBuffer data;

    // (2.1) Output iteration ////////////////////////////////////////////////

    fprintf(file, "%s/%s\n", entry.name.data(), maxIter.data());

    // (2.2) Output plain text ///////////////////////////////////////////////

    if( have_aes  &&  entry.cipher.size() == AES_BLOCK_SIZE ) {
      data = entry.cipher;
      AES_ECB_decrypt(&ctx, data.data());
      fprintf(file, "<"); printHex(data, file);
    } else if( !entry.plain.empty() ) {
      fprintf(file, "<"); printHex(entry.plain, file);
    }

    // (2.3) Output cipher text //////////////////////////////////////////////

    if( have_aes  &&  entry.plain.size() == AES_BLOCK_SIZE ) {
      data = entry.plain;
      AES_ECB_encrypt(&ctx, data.data());
      fprintf(file, ">"); printHex(data, file);
    } else if( !entry.cipher.empty() ) {
      fprintf(file, ">"); printHex(entry.cipher, file);
    }

    // (2.4) Output filename /////////////////////////////////////////////////

    fprintf(file, "Wrote file \"%s.mat\".\n", entry.name.data());
  } // For each entry...
}

int main(int /*argc*/, char **argv)
{
  const cs::Logger con_logger;
  const cs::ILogger *logger = &con_logger;

  Campaign campaign;
  if( !readCampaign(&campaign, cs::UTF8(argv[1]), logger) ) {
    return EXIT_FAILURE;
  }

  if( !campaign.isValid(AES128_KEY_SIZE, AES_BLOCK_SIZE) ) {
    logger->logError(u8"Invalid AES key/data size!");
    return EXIT_FAILURE;
  }

  print(campaign);

  return EXIT_SUCCESS;
}
