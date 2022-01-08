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

#include <iomanip>

#include "CmdOptions.h"

////// public ////////////////////////////////////////////////////////////////

CmdOptions::CmdOptions(const ctor_tag&) noexcept
{
}

CmdOptions::~CmdOptions() noexcept
{
}

void CmdOptions::clear()
{
  _options.clear();
}

bool CmdOptions::add(CmdOptionPtr& ptr)
{
  if( !ptr  ||  !CmdOption::isValidName(ptr->name()) ) {
    return false;
  }
  Options::value_type value{ptr->name(), std::move(ptr)};
  const std::pair<OptionIter,bool> result = _options.insert(std::move(value));
  return result.second;
}

bool CmdOptions::isValid(std::ostream& strm) const
{
  for(ConstOptionIter it = _options.cbegin(); it != _options.cend(); ++it) {
    const CmdOption *opt = it->second.get();
    if( !opt->isValid() ) {
      strm << "ERROR: Invalid value for option " << std::quoted(opt->name()) << "!" << std::endl;
      return false;
    }
  }
  return true;
}

bool CmdOptions::parse(std::ostream& strm, int argc, char **argv)
{
  const int numArgs = argc - 1;
  if( numArgs < 1 ) {
    printUsage(strm, argc, argv);
    return false;
  }

  for(int i = 0; i < numArgs; i++) {
    const char *arg = argv[i + 1];

    bool have_arg = false;
    for(OptionIter it = _options.begin(); it != _options.end(); ++it) {
      CmdOption *opt = it->second.get();

      if( !opt->isOption(arg) ) {
        continue;
      }
      have_arg = true;

      if( !opt->parse(arg) ) {
        strm << "ERROR: Invalid value for option " << std::quoted(opt->name()) << "!" << std::endl;
        return false;
      }

      break;
    } // For each option

    if( !have_arg ) {
      strm << "ERROR: Unknown argument " << std::quoted(arg) << "!" << std::endl;
      return false;
    }
  } // For each argument

  return isValid(strm);
}

void CmdOptions::printUsage(std::ostream& strm, int /*argc*/, char **argv) const
{
  strm << "Usage: " << argv[0] << std::endl;
  strm << std::endl;
  for(ConstOptionIter it = _options.cbegin(); it != _options.cend(); ++it) {
    it->second->printUsage(strm);
  }
  strm << std::endl;
}

void CmdOptions::setLongFormat(const bool on)
{
  for(OptionIter it = _options.begin(); it != _options.end(); ++it) {
    it->second->setLongFormat(on);
  }
}

const CmdOption *CmdOptions::get(const std::string& name) const
{
  const ConstOptionIter it = _options.find(name);
  return it != _options.cend()
      ? it->second.get()
      : nullptr;
}

CmdOptionsPtr CmdOptions::make()
{
  return std::make_unique<CmdOptions>(ctor_tag());
}
