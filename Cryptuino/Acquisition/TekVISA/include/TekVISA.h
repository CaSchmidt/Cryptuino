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

#ifndef TEKVISA_H
#define TEKVISA_H

#include <visa.h>

#include "IInstrument.h"

class TekVISA : public IInstrument {
private:
  struct ctor_tag {
    ctor_tag() noexcept
    {
    }
  };

public:
  TekVISA(const ctor_tag&) noexcept;
  ~TekVISA() noexcept;

  bool connect(const csILogger *logger) final;
  void disconnect() final;
  bool isConnected() const final;

  bool readSamples(const csILogger *logger, const int channel,
                   SampleBuffer *samples, const std::size_t numSamplesWant = 0) const final;

  bool setupTrigger(const csILogger *logger, const unsigned int tout) const final;

private:
  TekVISA() noexcept = delete;

  ViSession _rm{VI_NULL}; // cf. VI_WARN_NULL_OBJECT
  ViSession _vi{VI_NULL};

  template<typename DerivedT, typename... Args>
  friend InstrumentPtr make_instrument(Args&&... args);
};

#endif // TEKVISA_H
