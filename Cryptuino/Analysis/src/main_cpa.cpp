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

#include <array>
#include <bit>
#include <limits>

#define HAVE_STD_FORMAT
#include <cs/Logging/Logger.h>
#include <cs/Logging/OutputContext.h>
#include <cs/Logging/Progress.h>

#include "CampaignReader.h"
#include "Cryptuino.h"
#include "CPA.h"
#include "PowerAES.h"
#include "TriggerGT.h"

////// Main //////////////////////////////////////////////////////////////////

int main(int /*argc*/, char **argv)
{
  const std::filesystem::path filename(argv[1]);

  const cs::Logger        logger{stderr};
  const cs::Progress    progress{stderr};
  const cs::OutputContext output{&logger, true, &progress, true};

  CPAcontext ctx;

  if( !readCampaign(&ctx.campaign, filename, output.logger()) ) {
    return EXIT_FAILURE;
  }

  ctx.event = TriggerGT::make(2.5);
  ctx.model = PowerAES128EncRnd1::make();

  ctx.numTraces = 500;
  ctx.pctRange  = 15;
  ctx.sizBlock  = AES_BLOCK_SIZE;
  ctx.sizKey    = AES128_KEY_SIZE;

  runCPA(ctx, &output);

  return EXIT_SUCCESS;
}
