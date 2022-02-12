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

#include <csUtil/csFileIO.h>
#define HAVE_STD_FORMAT
#include <csUtil/csILogger.h>
#include <csUtil/csStringUtil.h>

#include "CampaignReader.h"

#include "Campaign.h"
#include "HexChar.h"

////// Private ///////////////////////////////////////////////////////////////

using      String         = cs::String<char>;
using ConstStringIter     = cs::ConstStringIter<char>;
using      StringList     = cs::StringList<char>;
using ConstStringListIter = cs::ConstStringListIter<char>;

////// Public ////////////////////////////////////////////////////////////////

bool readCampaign(Campaign *campaign, const std::filesystem::path& path, const csILogger *logger)
{
  *campaign = Campaign();

  logger->logTextf(u8"Opening file \"{}\".", cs::CSTR(path.generic_u8string().data()));

  // (1) Open file ///////////////////////////////////////////////////////////

  const StringList lines = csReadLines(path);
  if( lines.empty() ) {
    logger->logError(u8"No input!");
    return false;
  }

  // (2) Search (optional) key ///////////////////////////////////////////////

  const char *STR_setkey = "Setting key ";
  ConstStringListIter hit = std::find_if(lines.cbegin(), lines.cend(),
                                         [=](const String& str) -> bool {
    return cs::startsWith(str, STR_setkey);
  });
  if( hit != lines.cend() ) {
    campaign->key = extractHexBytes(*hit, STR_setkey);
  }

  // (3) Read entries... /////////////////////////////////////////////////////

  CampaignEntry entry;
  for(const String& line : lines) {
    if( cs::isDigit(line[0]) ) {
      campaign->add(entry);
      entry = CampaignEntry();

      entry.name.reserve(16);
      for(ConstStringIter iter = line.cbegin(); cs::isDigit(*iter); ++iter) {
        entry.name.push_back(*iter);
      }

    } else if( line.starts_with('<') ) {
      entry.plain = extractHexBytes(line, "<");

    } else if( line.starts_with('>') ) {
      entry.cipher = extractHexBytes(line, ">");

    }
  } // For each line...
  campaign->add(entry);

  // Done! ///////////////////////////////////////////////////////////////////

  logger->logTextf(u8"File \"{}\" opened.", cs::CSTR(path.generic_u8string().data()));

  return true;
}
