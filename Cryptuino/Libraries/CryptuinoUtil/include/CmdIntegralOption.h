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

#ifndef CMDINTEGRALOPTION_H
#define CMDINTEGRALOPTION_H

#include <array>
#include <charconv>
#include <functional>

#include <cs/Core/StringUtil.h>

#include "CmdOption.h"

template<typename T>
class CmdIntegralOption : public CmdOption {
private:
  struct ctor_tag {
    ctor_tag() noexcept
    {
    }
  };

public:
  using value_type = std::enable_if_t<!std::is_same_v<T,bool>  &&  std::is_integral_v<T>,T>;
  using valid_func = std::function<bool(const value_type)>;

  CmdIntegralOption(const ctor_tag&,
                    const std::string& name,
                    const valid_func& validator,
                    const value_type defValue = value_type{0}) noexcept
    : CmdOption(name)
    , _defValue(defValue)
    , _validator(validator)
    , _value(defValue)
  {
    initializeDefaultValue();
  }

  ~CmdIntegralOption() noexcept
  {
  }

  value_type value() const
  {
    return _value;
  }

private:
  CmdIntegralOption() noexcept = delete;

  void initializeDefaultValue()
  {
    try {
      std::array<char,32> buffer;
      buffer.fill(0);

      char *first = buffer.data();
      char  *last = first + buffer.size();
      const std::to_chars_result result = std::to_chars(first, last, _defValue, 10);

      if( result.ec == std::errc{} ) {
        _defValueStr = buffer.data();
      }
    } catch(...) {
      _defValueStr.clear();
    }
  }

  const char *impl_defaultValue() const final
  {
    return _defValueStr.data();
  }

  bool impl_parse(const char *value) final
  {
    const std::size_t length0 = cs::length(value);
    if( length0 < 1 ) {
      return false;
    }

    int base = 10;
    if( length0 > 2  &&  value[0] == '0' ) {
      if(        value[1] == 'b'  ||  value[1] == 'B' ) {
        base = 2;
        value += 2;
      } else if( value[1] == 'x'  ||  value[1] == 'X' ) {
        base = 16;
        value += 2;
      }
    }

    if( length0 > 1  &&  base == 10  &&  value[0] == '+' ) {
      value += 1;
    }

    const char *first = value;
    const char  *last = first + cs::length(first);
    const std::from_chars_result result = std::from_chars(first, last, _value, base);
    const bool is_ok = result.ec == std::errc{}  &&  result.ptr == last;

    return is_ok  &&  isValid();
  }

  bool impl_isValid() const final
  {
    return _validator(_value);
  }

  value_type  _defValue{0};
  std::string _defValueStr;
  valid_func  _validator;
  value_type  _value{0};

  template<typename DerivedT, typename... Args>
  friend CmdOptionPtr make_option(const std::string& name, Args&&... args);
};

using CmdIntOption  = CmdIntegralOption<int>;
using CmdUIntOption = CmdIntegralOption<unsigned int>;

#endif // CMDINTEGRALOPTION_H
