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
#include <cs/Math/Numeric.h>
#define HAVE_SIMD128_PREFETCH
#include <cs/Math/Statistics.h>

#include <cs/System/Time.h>

#include "Campaign.h"
#include "CampaignReader.h"
#include "MatInput.h"
#include "Matrix.h"
#include "Trigger.h"

////// Types /////////////////////////////////////////////////////////////////

namespace impl {

  struct      attack_tag {};
  struct correlation_tag {};
  struct       trace_tag {};

} // namespace impl

using      AttackMatrix = ColMajMatrix<double,impl::attack_tag>;
using CorrelationMatrix = ColMajMatrix<double,impl::correlation_tag>;
using       TraceMatrix = ColMajMatrix<double,impl::trace_tag>;

using NumVec = std::vector<double>;

using math = cs::math<double>;

////// Helper ////////////////////////////////////////////////////////////////

void progress(const cs::ILogger *logger, const std::size_t pos, const std::size_t max,
              const std::size_t step = 10)
{
  constexpr std::size_t HUNDRED = 100;
  constexpr std::size_t    ZERO = 0;

  if( max < 1  ||  pos < 0  ||  pos > max  ||  step < 1 ) {
    return;
  }

  if( pos%step == ZERO  ||  pos == max ) {
    const std::size_t   pct = (pos*HUNDRED)/max;
    const std::size_t width = cs::countDigits(max);

    logger->logTextf(u8"Progress: {:3}% ({:{}}/{})", pct, pos, width, max);
  }
}

////// Implementation ////////////////////////////////////////////////////////

double powerModel(const uint8_t data, const uint8_t key)
{
  // cf. https://en.wikipedia.org/wiki/Rijndael_S-box
  const std::array<uint8_t,256> SBOX{
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16
  };

  // cf. https://en.wikipedia.org/wiki/Hamming_weight
  return static_cast<double>(std::popcount<uint8_t>(SBOX[data ^ key]));
}

AttackMatrix buildAttackMatrix(const Campaign& campaign, const std::size_t numD,
                               const std::size_t run)
{
  constexpr std::size_t KEY_VALUES = 256;

  // (1) Sanity Check ////////////////////////////////////////////////////////

  if( run < 0  ||  run >= AES128_KEY_SIZE ) {
    return AttackMatrix();
  }

  // (2) Create Attack Matrix ////////////////////////////////////////////////

  AttackMatrix A;
  if( !A.resize(numD, KEY_VALUES) ) {
    return AttackMatrix();
  }

  // (3) Fill Attack Matrix //////////////////////////////////////////////////

  CampaignEntries::const_iterator iter = campaign.entries.cbegin();
  for(std::size_t i = 0; i < numD; i++, ++iter) {
    const uint8_t plain = iter->plain[run];
    for(std::size_t j = 0; j < KEY_VALUES; j++) {
      A(i, j) = powerModel(plain, uint8_t(j));
    }
  }

  // Done! ///////////////////////////////////////////////////////////////////

  return A;
}

SampleBuffer readTrace(const std::filesystem::path& path,
                       const TriggerEvent& event, const std::size_t range,
                       const cs::ILogger *logger)
{
  // (1) Read Signal AKA Full Trace //////////////////////////////////////////

  const SampleBuffer signal = readMatVector(path, "trace", logger);
  if( signal.empty() ) {
    return SampleBuffer();
  }

  // (2) Read (Optional) Trigger /////////////////////////////////////////////

  const bool    have_trigger = haveMatVariable(path, "trigger", logger);
  const SampleBuffer trigger = have_trigger
      ? readMatVector(path, "trigger", logger)
      : SampleBuffer();
  if( have_trigger  &&  trigger.empty() ) {
    return SampleBuffer();
  }

  // (3) Apply Trigger Condition and/or Range ////////////////////////////////

  const SampleBuffer trace = !trigger.empty()
      ? selectTrigger(signal, trigger, event, range)
      : copyRange(signal, range);
  if( trace.empty() ) {
    logger->logErrorf(u8"Empty trace for file \"{}\"!",
                      cs::CSTR(path.generic_u8string().data()));
    return SampleBuffer();
  }

  return trace;
}

TraceMatrix buildTraceMatrix(const std::filesystem::path& base, const Campaign& campaign,
                             const std::size_t numD,
                             const TriggerEvent& event, const std::size_t range,
                             const cs::ILogger *logger)
{
  using Traces = std::list<SampleBuffer>;

  // (1) Read all Traces /////////////////////////////////////////////////////

  Traces    traces;
  std::size_t numT = std::numeric_limits<std::size_t>::max();

  CampaignEntries::const_iterator entry = campaign.entries.cbegin();
  for(std::size_t i = 0; i < numD; i++, ++entry) {
    SampleBuffer trace = readTrace(entry->path(base), event, range, logger);
    if( trace.empty() ) {
      return TraceMatrix();
    }

    traces.push_back(std::move(trace));

    if( traces.back().size() < numT ) {
      numT = traces.back().size();
    }

    progress(logger, i + 1, numD);
  }

  // (2) Create Trace Matrix /////////////////////////////////////////////////

  TraceMatrix T;
  if( !T.resize(numD, numT) ) {
    return TraceMatrix();
  }

  // (3) Fill Trace Matrix ///////////////////////////////////////////////////

  Traces::const_iterator trace = traces.cbegin();
  for(std::size_t i = 0; i < numD; i++, ++trace) {
    for(std::size_t j = 0; j < numT; j++) {
      T(i, j) = (*trace)[j];
    }
  }

  return T;
}

template<typename TraitsT>
NumVec columnMean(const Matrix<double,TraitsT>& M)
{
  NumVec mean;
  try {
    mean.resize(M.columns());
  } catch(...) {
    return NumVec();
  }

  for(std::size_t j = 0; j < M.columns(); j++) {
    mean[j] = cs::mean(M.columnData(j), M.rows());
  }

  return mean;
}

template<typename TraitsT>
NumVec columnStdDev(const Matrix<double,TraitsT>& M, const NumVec& mean)
{
  if( M.columns() != mean.size() ) {
    return NumVec();
  }

  NumVec stddev;
  try {
    stddev.resize(M.columns());
  } catch(...) {
    return NumVec();
  }

  for(std::size_t j = 0; j < M.columns(); j++) {
    stddev[j] = cs::stddev(M.columnData(j), M.rows(), mean[j]);
  }

  return stddev;
}

CorrelationMatrix computeCorrelation(const AttackMatrix& A, const TraceMatrix& T,
                                     const NumVec& meanT, const NumVec& stddevT)
{
  // (1) Setup ///////////////////////////////////////////////////////////////

  const std::size_t numD = A.rows();    // ... or T.rows()
  const std::size_t numK = A.columns();
  const std::size_t numT = T.columns();

  // (2) Create Correlation Matrix ///////////////////////////////////////////

  CorrelationMatrix R;
  if( !R.resize(numK, numT) ) {
    return CorrelationMatrix();
  }

  // (3) Compute Correlation /////////////////////////////////////////////////

  const uint64_t beg = cs::tickCountMs();

  const NumVec meanA = columnMean(A);
  if( meanA.empty() ) {
    return CorrelationMatrix();
  }

  const NumVec stddevA = columnStdDev(A, meanA);
  if( stddevA.empty() ) {
    return CorrelationMatrix();
  }

  for(std::size_t i = 0; i < numK; i++) {
    for(std::size_t j = 0; j < numT; j++) {
      R(i, j)  = cs::cov(A.columnData(i), T.columnData(j), numD, meanA[i], meanT[j]);
      R(i, j) /= stddevA[i]*stddevT[j];
    }
  }

  const uint64_t end = cs::tickCountMs();
  printf("duration = %llums\n", end - beg);

  return R;
}

////// Main //////////////////////////////////////////////////////////////////

int main(int /*argc*/, char **argv)
{
  // (0) Configuration ///////////////////////////////////////////////////////

  const std::filesystem::path campaignPath(argv[1]);

  const std::size_t numTraces = 500;

  const TriggerEvent event = [](const double d) -> bool {
    return d > 2.5;
  };
  const std::size_t range = 15;

  const cs::Logger con_logger(stderr);
  const cs::ILogger *logger = &con_logger;

  // (1) Load Campaign ///////////////////////////////////////////////////////

  Campaign campaign;
  if( !readCampaign(&campaign, campaignPath, logger) ) {
    return EXIT_FAILURE;
  }

  if( !campaign.isValid() ) {
    logger->logErrorf(u8"Invalid campaign \"{}\"!",
                      cs::CSTR(campaignPath.generic_u8string().data()));
    return EXIT_FAILURE;
  }

  // (2) Sanitize Number of Traces ///////////////////////////////////////////

  const std::size_t numD = campaign.numEntries(campaignPath, numTraces);
  if( numD < 1 ) {
    logger->logErrorf(u8"No traces for campaign \"{}\"!",
                      cs::CSTR(campaignPath.generic_u8string().data()));
    return EXIT_FAILURE;
  }

  // (3) Create Trace Matrix /////////////////////////////////////////////////

  logger->logText(u8"Step 1: Build trace matrix.");
  const TraceMatrix T = buildTraceMatrix(campaignPath, campaign, numD, event, range, logger);
  if( T.isEmpty() ) {
    logger->logError(u8"Unable to build trace matrix!");
    return EXIT_FAILURE;
  }

  const NumVec   meanT = columnMean(T);
  const NumVec stddevT = columnStdDev(T, meanT);
  if( meanT.empty()  ||  stddevT.empty() ) {
    logger->logError(u8"Unable to compute trace auxiliaries!");
    return EXIT_FAILURE;
  }

  // (4) Attack with Correlation /////////////////////////////////////////////

  std::array<std::size_t,AES128_KEY_SIZE> keyi;
  std::array<double,AES128_KEY_SIZE> keyv;

  for(std::size_t k = 0; k < AES128_KEY_SIZE; k++) {
    const std::string pstr =
        std::format("{:{}}/{}",
                    k + 1, cs::countDigits(AES128_KEY_SIZE), AES128_KEY_SIZE);

    // (4.1) Create Attack Matrix ////////////////////////////////////////////

    logger->logTextf(u8"Step 2 [{}]: Build attack matrix.", pstr);
    const AttackMatrix A = buildAttackMatrix(campaign, numD, k);
    if( A.isEmpty() ) {
      logger->logError(u8"Unable to build attack matrix!");
      return EXIT_FAILURE;
    }

    // (4.2) Compute Correlation Matrix //////////////////////////////////////

    logger->logTextf(u8"Step 3 [{}]: Compute correlation matrix.", pstr);
    const CorrelationMatrix R = computeCorrelation(A, T, meanT, stddevT);
    if( R.isEmpty() ) {
      logger->logError(u8"Unable to compute correlation matrix!");
      return EXIT_FAILURE;
    }

    // (4.3) Guess Key ///////////////////////////////////////////////////////

    keyi[k] = std::numeric_limits<std::size_t>::max();
    keyv[k] = 0;
    for(std::size_t i = 0; i < R.rows(); i++) {
      for(std::size_t j = 0; j < R.columns(); j++) {
        const double v = math::abs(R(i, j));
        if( v > keyv[k] ) {
          keyi[k] = i;
          keyv[k] = v;
        }
      }
    }
  } // For Each Byte of Key

  // (5) Output Result ///////////////////////////////////////////////////////

  std::string keystr;
  for(std::size_t k = 0; k < AES128_KEY_SIZE; k++) {
    keystr += std::format(" {:2X}", keyi[k]);
  }
  logger->logTextf(u8"key ={}", keystr);

  std::string corstr;
  for(std::size_t k = 0; k < AES128_KEY_SIZE; k++) {
    corstr += std::format(" {:.3}", keyv[k]);
  }
  logger->logTextf(u8"cor ={}", corstr);

  return EXIT_SUCCESS;
}
