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

#ifndef CMDSTRINGVALUEOPTION_H
#define CMDSTRINGVALUEOPTION_H

#include <functional>

#include "CmdOption.h"

class CmdStringValueOption : public CmdOption {
private:
  struct ctor_tag {
    ctor_tag() noexcept
    {
    }
  };

public:
  using Validator = std::function<bool(const std::string&)>;

  CmdStringValueOption(const std::string& name, const std::string& help,
                       const bool isLongFormat, const bool isRequired,
                       const Validator& validator, const std::string& defValue,
                       const ctor_tag&) noexcept;
  ~CmdStringValueOption() noexcept;

  std::string value() const;

  static CmdOptionPtr make(const std::string& name, const std::string& help,
                           const bool isLongFormat, const bool isRequired,
                           const Validator& validator, const std::string& defValue = std::string());

private:
  CmdStringValueOption() noexcept = delete;

  const char *impl_defaultValue() const final;
  bool impl_parse(const char *value) final;
  bool impl_isValid() const final;

  std::string _defValue;
  Validator   _validator;
  std::string _value;
};

#endif // CMDSTRINGVALUEOPTION_H
