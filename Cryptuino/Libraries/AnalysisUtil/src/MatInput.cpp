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

#include <cstring>

#include <matio.h>

#define HAVE_STD_FORMAT
#include <csUtil/csILogger.h>

#include "MatInput.h"

#include "ScopeGuard.h"

bool haveMatVariable(const std::filesystem::path& path, const std::string& varname,
                     const csILogger *logger)
{
  mat_t   *file = NULL;
  matvar_t *var = NULL;
  ScopeGuard guard([&](void) -> void {
    if( var != NULL ) {
      Mat_VarFree(var);
      var = NULL;
    }
    if( file != NULL ) {
      Mat_Close(file);
      file = NULL;
    }
  });

  file = Mat_Open(cs::CSTR(path.generic_u8string().data()), MAT_ACC_RDONLY);
  if( file == NULL ) {
    logger->logErrorf(u8"Unable to open file \"{}\"!", cs::CSTR(path.generic_u8string().data()));
    return false;
  }

  var = Mat_VarReadInfo(file, varname.data());
  const bool have_var = var != NULL;

  return have_var;
}

SampleBuffer readMatVector(const std::filesystem::path& path, const std::string& varname,
                           const csILogger *logger)
{
  mat_t   *file = NULL;
  matvar_t *var = NULL;
  ScopeGuard guard([&](void) -> void {
    if( var != NULL ) {
      Mat_VarFree(var);
      var = NULL;
    }
    if( file != NULL ) {
      Mat_Close(file);
      file = NULL;
    }
  });

  // (1) Open MAT file ///////////////////////////////////////////////////////

  file = Mat_Open(cs::CSTR(path.generic_u8string().data()), MAT_ACC_RDONLY);
  if( file == NULL ) {
    logger->logErrorf(u8"Unable to open file \"{}\"!", cs::CSTR(path.generic_u8string().data()));
    return SampleBuffer();
  }

  // (2) Read variable ///////////////////////////////////////////////////////

  var = Mat_VarRead(file, varname.data());
  if( var == NULL ) {
    logger->logErrorf(u8"Unable to read variable \"{}\"!", varname);
    return SampleBuffer();
  }

  // (3) Sanitize variable ///////////////////////////////////////////////////

  if( var->isComplex != 0 ) {
    logger->logErrorf(u8"Variable \"{}\" is complex!", varname);
    return SampleBuffer();
  }
  if( var->rank != 2  ||  (var->dims[0] > 1  &&  var->dims[1] > 1) ) {
    logger->logErrorf(u8"Variable \"{}\" is no vector!", varname);
    return SampleBuffer();
  }
  if( var->data_type != MAT_T_DOUBLE  ||  var->class_type != MAT_C_DOUBLE ) {
    logger->logErrorf(u8"Variable \"{}\" is not of type DOUBLE!", varname);
    return SampleBuffer();
  }

  // (4) Create buffer ///////////////////////////////////////////////////////

  SampleBuffer buffer;
  try {
    const std::size_t numElements = var->dims[0]*var->dims[1];
    buffer.resize(numElements, double{0});
    std::memcpy(buffer.data(), var->data, numElements*sizeof(double));
  } catch(...) {
    logger->logErrorf(u8"Unable to allocate memory for variable \"{}\"!", varname);
    return SampleBuffer();
  }

  // Done! ///////////////////////////////////////////////////////////////////

  return buffer;
}
