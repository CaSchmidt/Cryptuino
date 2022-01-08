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

#ifndef CMDOPTIONS_H
#define CMDOPTIONS_H

#include <map>

#include "CmdIntegralValueOption.h"
#include "CmdStringValueOption.h"

using CmdOptionsPtr = std::unique_ptr<class CmdOptions>;

class CmdOptions {
private:
  struct ctor_tag {
    ctor_tag() noexcept
    {
    }
  };

public:
  using      Options    = std::map<std::string,CmdOptionPtr>;
  using      OptionIter = Options::iterator;
  using ConstOptionIter = Options::const_iterator;

  CmdOptions(const ctor_tag&) noexcept;
  ~CmdOptions() noexcept;

  void clear();

  bool add(CmdOptionPtr& ptr);

  bool isValid(std::ostream& strm) const;
  bool parse(std::ostream& strm, int argc, char **argv);

  void printUsage(std::ostream& strm, int argc, char **argv) const;

  void setLongFormat(const bool on);

  const CmdOption *get(const std::string& name) const;

  /*
  template<typename T>
  inline std::enable_if_t<std::is_same_v<T,bool>,T> value(const std::string& name) const
  {
  }

  template<typename T>
  inline std::enable_if_t<std::is_floating_point_v<T>,T> value(const std::string& name) const
  {
  }
  */

  template<typename T>
  inline std::enable_if_t<!std::is_same_v<T,bool>  &&  std::is_integral_v<T>,T> value(const std::string& name) const
  {
    return dynamic_cast<const CmdIntegralValueOption<T>*>(get(name))->value();
  }

  template<typename T>
  inline std::enable_if_t<std::is_same_v<T,std::string>,T> value(const std::string& name) const
  {
    return dynamic_cast<const CmdStringValueOption*>(get(name))->value();
  }

  static CmdOptionsPtr make();

private:
  Options _options;
};

#endif // CMDOPTIONS_H
