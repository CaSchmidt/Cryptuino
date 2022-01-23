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

#include <array>
#include <algorithm>
#include <vector>

#define HAVE_AES
#ifdef HAVE_AES
# include <aes.hpp>
#endif

#include <csUtil/csFileIO.h>
#define HAVE_STD_FORMAT
#include <csUtil/csLogger.h>
#include <csUtil/csStringUtil.h>

#include "Buffer.h"
#include "HexChar.h"

template<typename T>
concept IsCharacter = cs::is_char_v<T>;

template<typename T> requires IsCharacter<T>
constexpr auto lambda_is_space()
{
  return [](const T &c) -> bool {
    return cs::isSpace<T>(c);
  };
}

template<typename  T, typename PredFunc>
inline T *removeAll(T *first, T *last, PredFunc func)
{
  T *end = std::remove_if(first, last, func);
  while( end != last ) {
    *end++ = cs::glyph<T>::null;
  }
  return first;
}

template<typename T, typename PredFunc>
inline std::basic_string<T> removeAll(std::basic_string<T> s, PredFunc func)
{
  using Iter = typename std::basic_string<T>::iterator;
  Iter end = std::remove_if(s.begin(), s.end(), func);
  s.erase(end, s.end());
  return s;
}

using      String         = std::string;
using      StringIter     = String::iterator;
using ConstStringIter     = String::const_iterator;
using      StringList     = std::list<String>;
using      StringListIter = StringList::iterator;
using ConstStringListIter = StringList::const_iterator;

ByteBuffer extractHexBytes(std::string text, const char *prefix = nullptr)
{
  const std::size_t numPrefix = cs::length(prefix);
  if( numPrefix > 0 ) {
    text.erase(0, numPrefix);
  }

  text = removeAll(text, lambda_is_space<char>());
  if( (text.size() & 1) != 0 ) {
    return ByteBuffer();
  }

  ByteBuffer buffer;
  try {
    buffer.resize(text.size()/2);
  } catch(...) {
    return ByteBuffer();
  }

  for(std::size_t i = 0; i < buffer.size(); i++) {
    const uint8_t hi = fromHexChar(text[i*2]);
    const uint8_t lo = fromHexChar(text[i*2 + 1]);
    if( hi == 0xFF  ||  lo == 0xFF ) {
      return ByteBuffer();
    }
    buffer[i] = (hi << 4) | lo;
  }

  return buffer;
}

void printHex(const ByteBuffer& buffer, const bool eol = true)
{
  for(const ByteBuffer::value_type b : buffer) {
    printf(" %02X", b);
  }
  if( eol ) {
    printf("\n");
  }
}

struct Entry {
  std::string  name{};
  ByteBuffer  plain{};
  ByteBuffer cipher{};

  Entry() noexcept = default;
};

using Entries = std::list<Entry>;

int main(int argc, char **argv)
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

  const ByteBuffer key = extractHexBytes(*iter, STR_setkey);

  Entries entries;

  Entry entry;
  for(; iter != lines.end(); ++iter) {
    const std::string line = *iter;

    if( cs::isDigit(line[0]) ) {
      entry = Entry(); // Reset

      entry.name.reserve(16);
      for(ConstStringIter chit = line.cbegin(); cs::isDigit(*chit); ++chit) {
        entry.name.push_back(*chit);
      }

    } else if( line.starts_with('<') ) {
      entry.plain = extractHexBytes(line, "< ");

    } else if( line.starts_with('>') ) {
      entry.cipher = extractHexBytes(line, "> ");

      entries.push_back(std::move(entry));

    }
  } // for each line

  if( entries.empty() ) {
    logger->logError(u8"No entries!");
    return EXIT_FAILURE;
  }

  const std::string maxIter = entries.back().name;

#ifdef HAVE_AES
  if( key.size() != 16 ) {
    logger->logError(u8"Invalid AES key!");
    return EXIT_FAILURE;
  }
  AES_ctx ctx;
  AES_init_ctx(&ctx, key.data());
#endif

  printf("Setting key"); printHex(key);
  for(const Entry& entry : entries) {
    printf("%s/%s\n", entry.name.data(), maxIter.data());
    printf("<"); printHex(entry.plain);
#ifdef HAVE_AES
    {
      ByteBuffer aesdata = entry.plain;
      if( aesdata.size() != 16 ) {
        logger->logError(u8"Invalid AES data!");
        return EXIT_FAILURE;
      }
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
