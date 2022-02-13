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

#include <algorithm>

#include "Campaign.h"

////// Private ///////////////////////////////////////////////////////////////

namespace priv {

  bool isValidBuffer(const ByteBuffer& buffer, const std::size_t size)
  {
    return buffer.size() != 0
        ? buffer.size() == size
        : true;
  }

} // namespace priv

////// Entry /////////////////////////////////////////////////////////////////

CampaignEntry::CampaignEntry() noexcept = default;

bool CampaignEntry::isEmpty() const
{
  return name.empty();
}

bool CampaignEntry::exists(const std::filesystem::path& base) const
{
  std::error_code ec;
  return std::filesystem::exists(path(base), ec);
}

std::filesystem::path CampaignEntry::path(const std::filesystem::path& base) const
{
  std::error_code ec;
  std::filesystem::path p = std::filesystem::is_directory(base, ec)
      ? base
      : base.parent_path();

  p.append(name + ".mat");

  return p;
}

////// Campaign //////////////////////////////////////////////////////////////

Campaign::Campaign() noexcept = default;

void Campaign::add(CampaignEntry& entry)
{
  if( !entry.isEmpty() ) {
    entries.push_back(std::move(entry));
  }
}

void Campaign::clear()
{
  key.clear();
  entries.clear();
}

bool Campaign::isEmpty() const
{
  return entries.empty();
}

bool Campaign::isValid(const std::size_t keySize, const std::size_t blockSize) const
{
  if( !priv::isValidBuffer(key, keySize) ) {
    return false;
  }

  for(const CampaignEntry& entry : entries) {
    if( !priv::isValidBuffer(entry.plain, blockSize) ) {
      return false;
    }
    if( !priv::isValidBuffer(entry.cipher, blockSize) ) {
      return false;
    }
  }

  return true;
}

std::string Campaign::lastEntryName() const
{
  return !isEmpty()
      ? entries.back().name
      : std::string();
}

std::size_t Campaign::numEntries(const std::filesystem::path& base, const std::size_t numWant) const
{
  if( isEmpty()  || numWant < 1 ) {
    return 0;
  }

  const std::size_t numHave = std::min<std::size_t>(numWant, entries.size());

  CampaignEntries::const_iterator iter = entries.cbegin();
  for(std::size_t i = 0; i < numHave; i++, ++iter) {
    if( !iter->exists(base) ) {
      return i;
    }
  }

  return numHave;
}
