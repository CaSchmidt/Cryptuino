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

#include "CmdOption.h"

#include <csUtil/csStringUtil.h>

////// public ////////////////////////////////////////////////////////////////

CmdOption::~CmdOption() noexcept
{
}

bool CmdOption::isLongFormat() const
{
  return _isLongFormat;
}

bool CmdOption::isRequired() const
{
  return _isRequired;
}

const char *CmdOption::name() const
{
  return _name.data();
}

bool CmdOption::isOption(const char *arg) const
{
  return cs::startsWith(arg, impl_argPrefix());
}

bool CmdOption::parse(const char *arg)
{
  return isOption(arg)  &&  impl_parse(arg + cs::length(impl_argPrefix()));
}

bool CmdOption::isValid() const
{
  return impl_isValid();
}

void CmdOption::printUsage(std::ostream& strm) const
{
  if( isRequired() ) {
    strm << "    ";
  } else {
    strm << "  ? ";
  }

  strm << impl_argPrefix();

  if( impl_defaultValue() != nullptr ) {
    strm << "<" << impl_defaultValue() << ">";
  }

  strm << "    " << _help;

  strm << std::endl;
}

bool CmdOption::isValidName(const char *s)
{
  const auto cnt = std::count_if(s, s + cs::length(s),
                                 [](const char c) -> bool { return cs::isSpace(c); });
  return static_cast<std::size_t>(cnt) < 1;
}

////// protected /////////////////////////////////////////////////////////////

CmdOption::CmdOption(const std::string& name, const std::string& help,
                     const bool isLongFormat, const bool isRequired) noexcept
  : _help(help)
  , _isLongFormat(isLongFormat)
  , _isRequired(isRequired)
  , _name(name)
{
}