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

#define HAVE_AES
#ifdef HAVE_AES
# include <aes.hpp>
#endif

#include <csUtil/csFileIO.h>
#define HAVE_STD_FORMAT
#include <csUtil/csLogger.h>
#include <csUtil/csStringUtil.h>

#include "Campaign.h"
#include "HexChar.h"

using      String         = std::string;
using      StringIter     = String::iterator;
using ConstStringIter     = String::const_iterator;
using      StringList     = std::list<String>;
using      StringListIter = StringList::iterator;
using ConstStringListIter = StringList::const_iterator;

void printHex(const ByteBuffer& buffer, const bool eol = true)
{
  for(const ByteBuffer::value_type b : buffer) {
    printf(" %02X", b);
  }
  if( eol ) {
    printf("\n");
  }
}

int main(int /*argc*/, char **argv)
{
  const char *STR_setkey = "Setting key ";

  const csLogger con_logger;
  const csILogger *logger = &con_logger;

  if( cs::length(argv[1]) < 1 ) {
    return EXIT_FAILURE;
  }

  const StringList lines = csReadLines(cs::UTF8(argv[1]), true);
  ConstStringListIter iter = std::find_if(lines.cbegin(), lines.cend(),
                                          [=](const String& s) -> bool {
    return cs::startsWith(s.data(), STR_setkey);
  });
  if( iter == lines.end() ) {
    logger->logError(u8"AES key not found!");
    return EXIT_FAILURE;
  }

  Campaign campaign;

  campaign.key = extractHexBytes(*iter, STR_setkey);

  CampaignEntry entry;
  for(++iter; iter != lines.end(); ++iter) {
    const std::string& line = *iter;

    if( cs::isDigit(line[0]) ) {
      if( !entry.isEmpty() ) {
        campaign.entries.push_back(std::move(entry));
      }
      entry = CampaignEntry(); // Reset

      entry.name.reserve(16);
      for(ConstStringIter chit = line.cbegin(); cs::isDigit(*chit); ++chit) {
        entry.name.push_back(*chit);
      }

    } else if( line.starts_with('<') ) {
      entry.plain = extractHexBytes(line, "< ");

    } else if( line.starts_with('>') ) {
      entry.cipher = extractHexBytes(line, "> ");

    }
  } // for each line
  if( !entry.isEmpty() ) {
    campaign.entries.push_back(std::move(entry));
  }

  if( campaign.isEmpty() ) {
    logger->logError(u8"No entries!");
    return EXIT_FAILURE;
  }

  if( !campaign.isValid() ) {
    logger->logError(u8"Invalid AES key/data size!");
    return EXIT_FAILURE;
  }

  const std::string maxIter = campaign.lastName();

#ifdef HAVE_AES
  AES_ctx ctx;
  AES_init_ctx(&ctx, campaign.key.data());
#endif

  printf("Setting key"); printHex(campaign.key);
  for(const CampaignEntry& entry : campaign.entries) {
    printf("%s/%s\n", entry.name.data(), maxIter.data());
    printf("<"); printHex(entry.plain);
#ifdef HAVE_AES
    {
      ByteBuffer aesdata = entry.plain;
      AES_ECB_encrypt(&ctx, aesdata.data());
      printf(">"); printHex(aesdata);
    }
#else
    printf(">"); printHex(entry.cipher);
#endif
    printf("Wrote file \"%s.mat\".\n", entry.name.data());
  }

  return EXIT_SUCCESS;
}
