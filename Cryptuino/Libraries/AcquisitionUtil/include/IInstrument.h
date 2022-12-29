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

#ifndef IINSTRUMENT_H
#define IINSTRUMENT_H

#include <cstddef>

#include <memory>

#include "Buffer.h"

using InstrumentPtr = std::unique_ptr<class IInstrument>;

namespace cs {
  class ILogger;
} // namespace cs

struct InstrumentOptions {
  enum Coupling {
    AC = 0,
    DC,
    GND
  };

  InstrumentOptions() noexcept = default;

  Coupling coupling{AC};
};

class IInstrument {
public:
  virtual ~IInstrument() noexcept;

  virtual bool connect(const cs::ILogger *logger, const InstrumentOptions& options) = 0;
  virtual void disconnect() = 0;
  virtual bool isConnected() const = 0;

  virtual bool readSamples(const cs::ILogger *logger, const int channel,
                           SampleBuffer *samples, const std::size_t numSamplesWant = 0) const = 0;

  virtual bool setupTrigger(const cs::ILogger *logger, const unsigned int tout) const = 0;

protected:
  IInstrument() noexcept;

private:
  template<typename DerivedT, typename... Args>
  friend InstrumentPtr make_instrument(Args&&... args);
};

// cf. "C++ Core Guidelines", C.50
template<typename DerivedT, typename... Args>
InstrumentPtr make_instrument(Args&&... args)
{
  using ctor_tag = typename DerivedT::ctor_tag;
  InstrumentPtr ptr;
  try {
    ptr = std::make_unique<DerivedT>(ctor_tag{}, std::forward<Args>(args)...);
  } catch(...) {
    return InstrumentPtr{};
  }
  return ptr;
}

#endif // IINSTRUMENT_H
