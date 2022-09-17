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
#include <iterator>

#include "TriggerSelect.h"

////// Private ///////////////////////////////////////////////////////////////

namespace priv {

  std::size_t relativeCount(const std::size_t have, const std::size_t _range)
  {
    constexpr std::size_t HUNDRED = 100;

    const std::size_t range = std::clamp<std::size_t>(_range, 1, 100);
    const std::size_t  want = range == HUNDRED
        ? have
        : (have*range)/HUNDRED;

    return have < 1
        ? 0
        : want;
  }

} // namespace priv

////// Public ////////////////////////////////////////////////////////////////

SampleBuffer copyRange(const SampleBuffer& signal, const std::size_t _range)
{
  // (1) Sanity Check ////////////////////////////////////////////////////////

  if( signal.empty() ) {
    return SampleBuffer();
  }

  // (2) Copy Range //////////////////////////////////////////////////////////

  const std::size_t want = priv::relativeCount(signal.size(), _range);

  SampleBuffer result;
  try {
    result.resize(want);
    std::copy_n(signal.cbegin(), want, result.begin());
  } catch(...) {
    return SampleBuffer();
  }

  return result;
}

SampleBuffer selectTrigger(const SampleBuffer& signal, const SampleBuffer& trigger,
                           const TriggerPtr& event, const std::size_t _range)
{
  using ConstIter = SampleBuffer::const_iterator;

  const auto trigger_cond = [&](const double x) -> bool {
    return event->eval(x);
  };

  // (1) Sanity Check ////////////////////////////////////////////////////////

  if( signal.empty()  ||  trigger.empty()  ||  signal.size() != trigger.size() ) {
    return SampleBuffer();
  }

  // (2) Find Range //////////////////////////////////////////////////////////

  ConstIter beg = std::find_if(trigger.cbegin(), trigger.cend(), trigger_cond);
  if( beg == trigger.cend() ) {
    return SampleBuffer();
  }

  ConstIter end = std::find_if_not(beg + 1, trigger.cend(), trigger_cond);

  const std::ptrdiff_t pos = std::distance(trigger.cbegin(), beg);
  const std::size_t   have = std::distance(beg, end);

  // (3) Copy Range //////////////////////////////////////////////////////////

  const std::size_t want = priv::relativeCount(have, _range);

  SampleBuffer result;
  try {
    result.resize(want);
    std::copy_n(std::next(signal.cbegin(), pos), want, result.begin());
  } catch(...) {
    return SampleBuffer();
  }

  return result;
}
