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

#ifndef UTIL_H
#define UTIL_H

#include <cstddef>
#include <cstdint>

#include <random>
#include <type_traits>

template<typename T>
inline std::enable_if_t<std::is_integral_v<T>,int> countDigits(T value)
{
  constexpr T  TEN = 10;
  constexpr T ZERO =  0;

  int cnt = 0;
  do {
    cnt++;
    value /= TEN;
  } while( value != ZERO );

  return cnt;
}

class Randomizer {
public:
  Randomizer() noexcept;
  ~Randomizer() noexcept;

  void generate(uint8_t *ptr, const std::size_t len) const;

private:
  Randomizer(const Randomizer&) noexcept = delete;
  Randomizer& operator=(const Randomizer&) noexcept = delete;
  Randomizer(Randomizer&&) noexcept = delete;
  Randomizer& operator=(Randomizer&&) noexcept = delete;

  std::uniform_int_distribution<unsigned> _dist;
  std::mt19937 _gen;
};

class csILogger;
class csSerial;

void rxAesCmd(const csILogger *logger, const csSerial& serial, const unsigned int tout);
void txAesCmd(const char prefix, const csSerial& serial, const Randomizer& randomizer);

#endif // UTIL_H