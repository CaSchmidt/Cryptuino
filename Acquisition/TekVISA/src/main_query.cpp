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

#include <visa.h>

#define HANDLE_ERROR(obj,reas)            \
  if( handleError(obj, status, reas) ) {  \
    return EXIT_FAILURE;                  \
  }

bool handleError(const ViObject obj, const ViStatus status, const char *reason = nullptr)
{
  const bool is_error = status < VI_SUCCESS;
  if( is_error ) {
    ViChar buffer[1024];
    viStatusDesc(obj, status, buffer);
    if( reason != nullptr ) {
      fprintf(stderr, "%s: ", reason);
    }
    fprintf(stderr, "%s\n", buffer);
  }
  return is_error;
}

int main(int /*argc*/, char ** /*argv*/)
{
  ViChar desc[1024];
  ViStatus status;
  ViSession rm;
  ViFindList list;
  ViUInt32 numList;

  status = viOpenDefaultRM(&rm);
  HANDLE_ERROR(rm, "viOpenDefaultRM()");

  status = viFindRsrc(rm, (ViChar*)"?*INSTR", &list, &numList, desc);
  HANDLE_ERROR(rm, "viFindRsrc()");

  if( numList < 1 ) {
    fprintf(stderr, "No devices!\n");
  }

  for(ViUInt32 i = 0; i < numList; i++) {
    ViSession vi;
    float rate;

    fprintf(stdout, "Device: %s\n", desc);

    status = viOpen(rm, desc, VI_NULL, VI_NULL, &vi);
    HANDLE_ERROR(rm, "viOpen()");

    /*
    status = viSetAttribute(vi, VI_ATTR_WR_BUF_OPER_MODE, VI_FLUSH_ON_ACCESS);
    status = viSetAttribute(vi, VI_ATTR_RD_BUF_OPER_MODE, VI_FLUSH_ON_ACCESS);
    */

    status = viPrintf(vi, (ViChar*)"header off\n");
    HANDLE_ERROR(vi, "viPrintf()");

    status = viQueryf(vi, (ViChar*)"horizontal:samplerate?\n", (ViChar*)"%f", &rate);
    HANDLE_ERROR(vi, "viQueryf()");

    fprintf(stdout, "Rate = %f\n", rate);

    viClose(vi);

    if( i < (numList - 1) ) {
      status = viFindNext(list, desc);
      HANDLE_ERROR(rm, "viFindNext()");
    }
  }

  viClose(list);
  viClose(rm);

  return EXIT_SUCCESS;
}
