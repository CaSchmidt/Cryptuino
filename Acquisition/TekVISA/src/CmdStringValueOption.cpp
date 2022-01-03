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

#include <csUtil/csStringUtil.h>

#include "CmdStringValueOption.h"

////// public ////////////////////////////////////////////////////////////////

CmdStringValueOption::CmdStringValueOption(const std::string& name, const std::string& help,
                                           const bool isLongFormat, const bool isRequired,
                                           const Validator& validator, const std::string& defValue,
                                           const ctor_tag&) noexcept
  : CmdValueOption(name, help, isLongFormat, isRequired)
  , _defValue(defValue)
  , _validator(validator)
  , _value(defValue)
{
}

CmdStringValueOption::~CmdStringValueOption() noexcept
{
}

std::string CmdStringValueOption::value() const
{
  return _value;
}

CmdOptionPtr CmdStringValueOption::make(const std::string& name, const std::string& help,
                                        const bool isLongFormat, const bool isRequired,
                                        const Validator& validator, const std::string& defValue)
{
  return std::make_unique<CmdStringValueOption>(name, help,
                                                isLongFormat, isRequired,
                                                validator, defValue,
                                                ctor_tag());
}

////// private ///////////////////////////////////////////////////////////////

const char *CmdStringValueOption::impl_defaultValue() const
{
  return _defValue.data();
}

bool CmdStringValueOption::impl_parse(const char *value)
{
  if( cs::length(value) < 1 ) {
    return false;
  }
  _value = value;
  return isValid();
}

bool CmdStringValueOption::impl_isValid() const
{
  return _validator(_value);
}
