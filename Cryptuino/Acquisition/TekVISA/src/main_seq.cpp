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

#include <iostream>

#define HAVE_STD_FORMAT
#include <csUtil/csDualLogger.h>
#include <csUtil/csLogger.h>
#include <csUtil/csSerial.h>

#include "CmdOptions.h"
#include "instrutil.h"
#include "Randomizer.h"
#include "ScopeGuard.h"
#include "AcqUtil.h"

CmdOptionsPtr options()
{
  CmdOptionsPtr opts = CmdOptions::make();

  CmdOptionPtr ptr;

  ptr = make_option<CmdStringOption>("ser-device",
                                     [](const std::string& s) -> bool { return s.size() > 0; });
  opts->add(ptr);

  ptr = make_option<CmdStringOption>("channels",
                                     [](const std::string& s) -> bool {
    return s.size() == 2  &&  cs::isDigit(s[0])  &&  cs::isDigit(s[1]);
  }, "12");
  ptr->setRequired(false);
  opts->add(ptr);

  ptr = make_option<CmdIntOption>("ser-rate",
                                  [](const int i) -> bool { return i > 0; }, 9600);
  ptr->setRequired(false);
  opts->add(ptr);

  ptr = make_option<CmdIntOption>("count",
                                  [](const int i) -> bool { return i > 0; });
  opts->add(ptr);

  ptr = make_option<CmdBooleanOption>("no-instrument");
  ptr->setRequired(false);
  opts->add(ptr);

  ptr = make_option<CmdUIntOption>("tout-instrument",
                                   [](const unsigned int i) -> bool { return i > 0; }, 2);
  ptr->setRequired(false);
  opts->add(ptr);

  ptr = make_option<CmdUIntOption>("tout-serial",
                                   [](const unsigned int i) -> bool {return i > 0; }, 2);
  ptr->setRequired(false);
  opts->add(ptr);

  opts->setLongFormat(true);

  return opts;
}

int main(int argc, char **argv)
{
  // (1) Options /////////////////////////////////////////////////////////////

  CmdOptionsPtr opts = options();
  if( !opts->parse(argc, argv, std::cerr) ) {
    return EXIT_FAILURE;
  }

  const std::string         channels =  opts->value<std::string>("channels");
  const int                    count =  opts->value<int>("count");
  const std::string       ser_device =  opts->value<std::string>("ser-device");
  const int               ser_rate   =  opts->value<int>("ser-rate");
  const unsigned int tout_instrument =  opts->value<unsigned int>("tout-instrument");
  const unsigned int tout_serial     =  opts->value<unsigned int>("tout-serial");
  const bool          use_instrument = !opts->value<bool>("no-instrument");

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

  // (3) Setup ///////////////////////////////////////////////////////////////

  ViSession rm = VI_NULL; // cf. VI_WARN_NULL_OBJECT
  ViSession vi = VI_NULL;
  if( use_instrument  &&  !initializeInstrument(logger, &rm, &vi) ) {
    return EXIT_FAILURE;
  }
  ScopeGuard guard_rm([&](void) -> void {
    if( rm != VI_NULL ) {
      viClose(rm);
    }
  });

  Randomizer randomizer;

  csSerial serial;
  if( !serial.open(cs::UTF8(ser_device.data()), ser_rate) ) {
    logger->logErrorf(u8"Unable to open serial device \"{}\"!", ser_device);
    return EXIT_FAILURE;
  }

  txAesCmd('@', serial, randomizer);
  rxAesCmd(logger, serial, tout_serial);

  // (4) Sequence ////////////////////////////////////////////////////////////

  const int maxIter       = count - 1;
  const int numIterDigits = countDigits(maxIter);
  for(int i = 0; i <= maxIter; i++) {
    logger->logTextf(u8"{:0{}}/{}", i, numIterDigits, maxIter);

    // (4.1) Arm instrument //////////////////////////////////////////////////

    if( use_instrument ) {
      armInstrument(logger, vi, tout_instrument);
    }

    // (4.2) Request encryption //////////////////////////////////////////////

    txAesCmd('#', serial, randomizer);

    // (4.3) Wait for response ///////////////////////////////////////////////

    rxAesCmd(logger, serial, tout_serial);

    // (4.4) Store samples ///////////////////////////////////////////////////

    if( use_instrument ) {
      const std::string filename = std::format("{:0{}}.mat", i, numIterDigits);
      writeMatOutput(logger, vi, filename, channels);
    }
  }

  // Done! ///////////////////////////////////////////////////////////////////

  return EXIT_SUCCESS;
}
