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

#ifndef CAMPAIGN_H
#define CAMPAIGN_H

#include <filesystem>
#include <list>

#include "Buffer.h"
#include "Cryptuino.h"

struct CampaignEntry {
  CampaignEntry() noexcept;

  bool isEmpty() const;

  bool exists(const std::filesystem::path& base) const;
  std::filesystem::path path(const std::filesystem::path& base) const;

  std::string  name{};
  ByteBuffer  plain{};
  ByteBuffer cipher{};
};

using CampaignEntries = std::list<CampaignEntry>;

struct Campaign {
  Campaign() noexcept;

  void add(CampaignEntry& entry);

  void clear();

  bool isEmpty() const;
  bool isValid(const std::size_t keySize = AES128_KEY_SIZE,
               const std::size_t blockSize = AES_BLOCK_SIZE) const;

  std::string lastEntryName() const;
  std::size_t numEntries(const std::filesystem::path& base, const std::size_t numWant) const;

  ByteBuffer          key{};
  CampaignEntries entries{};
};

#endif // CAMPAIGN_H
