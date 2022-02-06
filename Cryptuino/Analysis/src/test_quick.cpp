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

#include <iostream>

#include <csUtil/csLogger.h>

#include "MatInput.h"
#include "Matrix.h"

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

int main(int /*argc*/, char **argv)
{
#if 0
  const csLogger con_logger;
  const csILogger *logger = &con_logger;

  const SampleBuffer buffer = readMatVector(argv[1], argv[2], logger);
  if( buffer.empty() ) {
    return EXIT_FAILURE;
  }

  for(const double d : buffer) {
    printf("%.3f\n", d);
  }
#else
  ColMajMatrix<double> M1(4, 4);
  RowMajMatrix<double> M2(4, 4);

  printInfo(M1, "M1");
  printInfo(M2, "M2");
#endif

  return EXIT_SUCCESS;
}
