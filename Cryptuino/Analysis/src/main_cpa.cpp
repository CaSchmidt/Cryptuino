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
#include <cs/Math/Numeric.h>
#define HAVE_SIMD128_PREFETCH
#include <cs/Math/Statistics.h>

#include <cs/System/Time.h>

#include "Campaign.h"
#include "CampaignReader.h"
#include "MatInput.h"
#include "Matrix.h"
#include "PowerAES.h"
#include "TriggerSelect.h"
#include "TriggerGT.h"

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

////// Implementation ////////////////////////////////////////////////////////

AttackMatrix buildAttackMatrix(const Campaign& campaign, const std::size_t numD,
                               const std::size_t run, const PowerModelPtr& model)
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
      A(i, j) = model->eval(plain, uint8_t(j));
    }
  }

  // Done! ///////////////////////////////////////////////////////////////////

  return A;
}

SampleBuffer readTrace(const std::filesystem::path& path,
                       const TriggerPtr& event, const std::size_t range,
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
                             const TriggerPtr& event, const std::size_t range,
                             const cs::OutputContext *ctx)
{
  using Traces = std::list<SampleBuffer>;

  // (0) Setup Progress //////////////////////////////////////////////////////

  ctx->setProgressRange(0, int(numD));

  // (1) Read all Traces /////////////////////////////////////////////////////

  Traces    traces;
  std::size_t numT = std::numeric_limits<std::size_t>::max();

  CampaignEntries::const_iterator entry = campaign.entries.cbegin();
  for(std::size_t i = 0; i < numD; i++, ++entry) {
    SampleBuffer trace = readTrace(entry->path(base), event, range, ctx->logger());
    if( trace.empty() ) {
      return TraceMatrix();
    }

    traces.push_back(std::move(trace));

    if( traces.back().size() < numT ) {
      numT = traces.back().size();
    }

    ctx->setProgressValue(int(i + 1));
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
                                     const NumVec& meanT, const NumVec& stddevT,
                                     const cs::ILogger *logger)
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
  logger->logTextf(u8"duration = {}ms", end - beg);

  return R;
}

////// Main //////////////////////////////////////////////////////////////////

int main(int /*argc*/, char **argv)
{
  // (0) Configuration ///////////////////////////////////////////////////////

  const std::filesystem::path campaignPath(argv[1]);

  const std::size_t numTraces = 500;

  const TriggerPtr  event = TriggerGT::make(2.5);
  const std::size_t range = 15;

  const PowerModelPtr model = PowerAES128EncRnd1::make();

  const cs::Logger     logger(stderr);
  const cs::Progress progress(stderr);
  const cs::OutputContext ctx(&logger, true, &progress, true);

  // (1) Load Campaign ///////////////////////////////////////////////////////

  Campaign campaign;
  if( !readCampaign(&campaign, campaignPath, ctx.logger()) ) {
    return EXIT_FAILURE;
  }

  if( !campaign.isValid() ) {
    ctx.logger()->logErrorf(u8"Invalid campaign \"{}\"!",
                            cs::CSTR(campaignPath.generic_u8string().data()));
    return EXIT_FAILURE;
  }

  // (2) Sanitize Number of Traces ///////////////////////////////////////////

  const std::size_t numD = campaign.numEntries(campaignPath, numTraces);
  if( numD < 1 ) {
    ctx.logger()->logErrorf(u8"No traces for campaign \"{}\"!",
                            cs::CSTR(campaignPath.generic_u8string().data()));
    return EXIT_FAILURE;
  }

  // (3) Create Trace Matrix /////////////////////////////////////////////////

  ctx.logger()->logText(u8"Step 1: Build trace matrix.");
  const TraceMatrix T = buildTraceMatrix(campaignPath, campaign, numD, event, range, &ctx);
  if( T.isEmpty() ) {
    ctx.logger()->logError(u8"Unable to build trace matrix!");
    return EXIT_FAILURE;
  }

  const NumVec   meanT = columnMean(T);
  const NumVec stddevT = columnStdDev(T, meanT);
  if( meanT.empty()  ||  stddevT.empty() ) {
    ctx.logger()->logError(u8"Unable to compute trace auxiliaries!");
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

    ctx.logger()->logTextf(u8"Step 2 [{}]: Build attack matrix.", pstr);
    const AttackMatrix A = buildAttackMatrix(campaign, numD, k, model);
    if( A.isEmpty() ) {
      ctx.logger()->logError(u8"Unable to build attack matrix!");
      return EXIT_FAILURE;
    }

    // (4.2) Compute Correlation Matrix //////////////////////////////////////

    ctx.logger()->logTextf(u8"Step 3 [{}]: Compute correlation matrix.", pstr);
    const CorrelationMatrix R = computeCorrelation(A, T, meanT, stddevT, ctx.logger());
    if( R.isEmpty() ) {
      ctx.logger()->logError(u8"Unable to compute correlation matrix!");
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
  ctx.logger()->logTextf(u8"key ={}", keystr);

  std::string corstr;
  for(std::size_t k = 0; k < AES128_KEY_SIZE; k++) {
    corstr += std::format(" {:.3}", keyv[k]);
  }
  ctx.logger()->logTextf(u8"cor ={}", corstr);

  return EXIT_SUCCESS;
}
