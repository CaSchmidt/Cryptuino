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

#include <cstdio>
#include <cstdlib>
#include <cstdint>

#include <iostream>
#include <limits>

#include <cs/Logging/Logger.h>

#include "MatInput.h"
#include "Matrix.h"
#include "TriggerSelect.h"

////// (1) Test MAT I/O //////////////////////////////////////////////////////

namespace test_matio {

  bool run(const char *filename, const char *varname,
           const cs::ILogger *logger)
  {
    const SampleBuffer buffer = readMatVector(filename, varname, logger);
    if( buffer.empty() ) {
      return false;
    }

    for(const double d : buffer) {
      printf("%.3f\n", d);
    }

    return true;
  }

} // namespace test_matio

////// (2) Test Matrix Order /////////////////////////////////////////////////

namespace test_matrix {

  const char *printBool(const bool b)
  {
    return b ? "true" : "false";
  }

  template<typename T, typename TraitsT>
  void printInfo(const Matrix<T,TraitsT>& M, const char *name = nullptr)
  {
    printf("   name = %s\n", name);
    printf("columns = %d\n", int(M.columns()));
    printf("   rows = %d\n", int(M.rows()));
    printf("col maj = %s\n", printBool(M.isColumnMajor()));
    printf("row maj = %s\n", printBool(M.isRowMajor()));
    printf(" @col#0 = 0x%p\n", M.columnData(0));
    printf(" @col#1 = 0x%p\n", M.columnData(1));
    printf(" @row#0 = 0x%p\n", M.rowData(0));
    printf(" @row#1 = 0x%p\n", M.rowData(1));
    printf("\n"); fflush(stdout);
  }

  void run()
  {
    ColMajMatrix<double> M1(4, 4);
    RowMajMatrix<double> M2(4, 4);

    printInfo(M1, "M1");
    printInfo(M2, "M2");
  }

} // namespace test_matrix

////// (3) Test Trigger //////////////////////////////////////////////////////

namespace test_trigger {

  class TriggerNEQ : public ITrigger {
  public:
    TriggerNEQ(const double level) noexcept
      : _level{level}
    {
    }

    ~TriggerNEQ() noexcept
    {
    }

    bool eval(const double x) const
    {
      return x != _level;
    }

  private:
    double _level{0};
  };

  void print(const SampleBuffer& buffer, const char *name = nullptr)
  {
    FILE *file = stdout;

    if( name != nullptr ) {
      fprintf(file, "%s =", name);
    }
    for(const SampleBuffer::value_type val : buffer) {
      fprintf(file, " %4.1f", val);
    }
    fprintf(file, "\n"); fflush(file);
  }

  void run()
  {
    TriggerPtr event= std::make_unique<TriggerNEQ>(0);

    const SampleBuffer  signal{0, 1, 2,  3, 4,  5, 6, 7, 8, 9};
    const SampleBuffer trigger{0, 1, 1, -1, 1, -1, 1, 1, 1, 0};

    print(signal,  " signal");
    print(trigger, "trigger");

    print(selectTrigger(signal, SampleBuffer{}, event), "  empty");
    print(selectTrigger(signal, trigger, event), "  event");
    print(selectTrigger(signal, trigger, event, 25), "event25");
    print(selectTrigger(signal, trigger, event, 38), "event38");
  }

} // namespace test_trigger

////// (4) Test Floating Point Count /////////////////////////////////////////

namespace test_count {

  template<typename T, typename Y = std::size_t>
  constexpr Y max_count()
  {
    constexpr Y ONE = 1;
    return sizeof(T) > sizeof(Y)
        ? std::numeric_limits<Y>::max()
        : (ONE << std::numeric_limits<T>::digits) - 1;
  }

  void run()
  {
    using Y = std::size_t;
    std::cout << "float : " << "0x" << std::hex << max_count<float,Y>() << std::endl;
    std::cout << "double: " << "0x" << std::hex << max_count<double,Y>() << std::endl;
  }

} // namespace test_count

////// (X) Main //////////////////////////////////////////////////////////////

int main(int /*argc*/, char **argv)
{
  const cs::Logger con_logger;
  const cs::ILogger *logger = &con_logger;

#if 0
  if( !test_matio::run(argv[1], argv[2], logger) ) {
    return EXIT_FAILURE;
  }
#endif

#if 0
  test_matrix::run();
#endif

#if 0
  test_trigger::run();
#endif

#if 0
  test_count::run();
#endif

  return EXIT_SUCCESS;
}
