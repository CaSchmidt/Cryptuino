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

#include <cstdio>
#include <cstdlib>

#include <charconv>
#include <format>
#include <string>

#include <csUtil/csDualLogger.h>
#include <csUtil/csLogger.h>
#include <csUtil/csStringUtil.h>

template<typename T>
using if_int_bool = std::enable_if_t<std::is_integral_v<T>,bool>;

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

struct Options {
  bool isValid() const
  {
    return count > 0  &&  serialDevice.size() > 0  &&  serialRate > 0;
  }

  void print(FILE *file = stdout) const
  {
    fprintf(file, "count      = %d\n", count);
    fprintf(file, "ser-device = %s\n", serialDevice.data());
    fprintf(file, "ser-rate   = %d\n", serialRate);
  }

  int count{0};
  std::string serialDevice;
  int serialRate{0};
};

inline bool isOption(const char *arg, const char *opt)
{
  return cs::startsWith(arg, cs::length(arg), opt, cs::length(opt));
}

inline const char *getOptionString(const char *arg, const char *opt)
{
  return isOption(arg, opt)
      ? arg + cs::length(opt)
      : nullptr;
}

template<typename T>
inline if_int_bool<T> parseIntOption(const char *opt, T& value, const T defValue = T{0})
{
  value = defValue;
  const std::from_chars_result result =
      std::from_chars(opt, opt + cs::length(opt), value, 10);
  return result.ec == std::errc{};
}

bool parseOptions(Options& options, int argc, char **argv)
{
  options = Options();

  const int numArgs = argc - 1;
  if( numArgs < 1 ) {
    fprintf(stdout, "%s <OPTIONS>\n\n", argv[0]);
    fprintf(stdout, "OPTIONS:\n\n");
    fprintf(stdout, "--count=INT\n");
    fprintf(stdout, "--ser-device=STRING\n");
    fprintf(stdout, "--ser-rate=INT\n");
    return false;
  }

  for(int i = 0; i < numArgs; i++) {
    const char *arg = argv[i + 1];

    const char *opt = nullptr;
    if(        (opt = getOptionString(arg, "--count=")) != nullptr ) {
      parseIntOption(opt, options.count);
    } else if( (opt = getOptionString(arg, "--ser-device=")) != nullptr ) {
      options.serialDevice = opt;
    } else if( (opt = getOptionString(arg, "--ser-rate=")) != nullptr ) {
      parseIntOption(opt, options.serialRate);
    } else {
      fprintf(stderr, "ERROR: Unknown option \"%s\"!\n", arg);
      return false;
    }
  }

  if( !options.isValid() ) {
    fprintf(stderr, "ERROR: Invalid or missing option!\n");
    return false;
  }

  return options.isValid();
}

int main(int argc, char **argv)
{
  // (1) Options /////////////////////////////////////////////////////////////

  Options options;
  if( !parseOptions(options, argc, argv) ) {
    return EXIT_FAILURE;
  }
  options.print();

  // (2) Logging /////////////////////////////////////////////////////////////

  constexpr const char *log_filename = "sequence.log.txt";
  FILE *file = fopen(log_filename, "wb");
  if( file == NULL ) {
    fprintf(stderr, "ERROR: Unable to open file \"%s\"!\n", log_filename);
    return EXIT_FAILURE;
  }

  const csLogger     log_con;
  const csLogger     log_file(file);
  const csDualLogger log_dual(&log_file, &log_con);

  const csILogger *logger = &log_dual;

  // (3) Sequence ////////////////////////////////////////////////////////////

  const int maxIter       = options.count - 1;
  const int numIterDigits = countDigits(maxIter);
  for(int i = 0; i <= maxIter; i++) {
    const std::string progress = std::format("{:0{}}/{}", i, numIterDigits, maxIter);
    logger->logText(cs::UTF8(progress.data()));
  }

  // Done! ///////////////////////////////////////////////////////////////////

  fclose(file);

  return EXIT_SUCCESS;
}
