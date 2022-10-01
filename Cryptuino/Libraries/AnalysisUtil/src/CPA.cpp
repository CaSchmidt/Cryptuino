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

#include <cs/Logging/Progress.h>
#include <cs/Math/Math.h>
#include <cs/Math/Numeric.h>
#define HAVE_SIMD128_PREFETCH
#include <cs/Math/Statistics.h>
#include <cs/System/Time.h>

#include "CPA.h"

#include "MatInput.h"
#include "Matrix.h"
#include "TriggerSelect.h"

namespace impl_cpa {

  struct      attack_tag {};
  struct correlation_tag {};
  struct       trace_tag {};

  using      AttackMatrix = ColMajMatrix<double,attack_tag>;
  using CorrelationMatrix = ColMajMatrix<double,correlation_tag>;
  using       TraceMatrix = ColMajMatrix<double,trace_tag>;

  using NumVec = std::vector<double>;

  using math = cs::math<double>;

  ////// Helpers /////////////////////////////////////////////////////////////

  template<typename TraitsT>
  NumVec columnMean(const Matrix<TraitsT>& M)
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
  NumVec columnStdDev(const Matrix<TraitsT>& M, const NumVec& mean)
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

  ////// Attack Matrix ///////////////////////////////////////////////////////

  AttackMatrix buildAttackMatrix(const CPAcontext& ctx,
                                 const std::size_t numD, const std::size_t run)
  {
    constexpr std::size_t KEY_VALUES = 256;

    // (1) Sanity Check //////////////////////////////////////////////////////

    if( run < 0  ||  run >= ctx.sizKey ) {
      return AttackMatrix();
    }

    // (2) Create Attack Matrix //////////////////////////////////////////////

    AttackMatrix A;
    if( !A.resize(numD, KEY_VALUES) ) {
      return AttackMatrix();
    }

    // (3) Fill Attack Matrix ////////////////////////////////////////////////

    CampaignEntries::const_iterator iter = ctx.campaign.entries.cbegin();
    for(std::size_t i = 0; i < numD; i++, ++iter) {
      const uint8_t plain = iter->plain[run];
      for(std::size_t j = 0; j < KEY_VALUES; j++) {
        A(i, j) = ctx.model->eval(plain, uint8_t(j));
      }
    }

    // Done! /////////////////////////////////////////////////////////////////

    return A;
  }

  ////// Correlation Matrix //////////////////////////////////////////////////

  CorrelationMatrix computeCorrelation(const AttackMatrix& A, const TraceMatrix& T,
                                       const NumVec& meanT, const NumVec& stddevT,
                                       const cs::OutputContext *output)
  {
    // (1) Setup /////////////////////////////////////////////////////////////

    const std::size_t numD = A.rows();    // ... or T.rows()
    const std::size_t numK = A.columns();
    const std::size_t numT = T.columns();

    // (2) Create Correlation Matrix /////////////////////////////////////////

    CorrelationMatrix R;
    if( !R.resize(numK, numT) ) {
      return CorrelationMatrix();
    }

    // (3) Compute Correlation ///////////////////////////////////////////////

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
    output->logTextf(u8"duration = {}ms", end - beg);

    return R;
  }

  ////// Trace Matrix ////////////////////////////////////////////////////////

  SampleBuffer readTrace(const CPAcontext& ctx, const cs::OutputContext *output,
                         const std::filesystem::path& filename)
  {
    // (1) Read Signal AKA Full Trace ////////////////////////////////////////

    const SampleBuffer signal = readMatVector(filename, "trace", output->logger());
    if( signal.empty() ) {
      return SampleBuffer();
    }

    // (2) Read (Optional) Trigger ///////////////////////////////////////////

    const bool    have_trigger = ctx.event  &&  haveMatVariable(filename, "trigger", output->logger());
    const SampleBuffer trigger = have_trigger
        ? readMatVector(filename, "trigger", output->logger())
        : SampleBuffer();
    if( have_trigger  &&  trigger.empty() ) {
      return SampleBuffer();
    }

    // (3) Apply Trigger Condition and/or Range //////////////////////////////

    const SampleBuffer trace = !trigger.empty()
        ? selectTrigger(signal, trigger, ctx.event, ctx.pctRange)
        : copyRange(signal, ctx.pctRange);
    if( trace.empty() ) {
      output->logErrorf(u8"Empty trace for file \"{}\"!",
                        cs::CSTR(filename.generic_u8string().data()));
      return SampleBuffer();
    }

    return trace;
  }

  TraceMatrix buildTraceMatrix(const CPAcontext& ctx, const cs::OutputContext *output,
                               const std::size_t numD)
  {
    using Traces = std::list<SampleBuffer>;

    // (0) Setup Progress ////////////////////////////////////////////////////

    output->setProgressRange(0, int(numD));

    // (1) Read all Traces ///////////////////////////////////////////////////

    Traces    traces;
    std::size_t numT = std::numeric_limits<std::size_t>::max();

    CampaignEntries::const_iterator entry = ctx.campaign.entries.cbegin();
    for(std::size_t i = 0; i < numD; i++, ++entry) {
      const std::filesystem::path filename = entry->filename(ctx.campaign.filename);
      SampleBuffer trace = readTrace(ctx, output, filename);
      if( trace.empty() ) {
        return TraceMatrix();
      }

      traces.push_back(std::move(trace));

      if( traces.back().size() < numT ) {
        numT = traces.back().size();
      }

      output->setProgressValue(int(i + 1));
    }

    // (2) Create Trace Matrix ///////////////////////////////////////////////

    TraceMatrix T;
    if( !T.resize(numD, numT) ) {
      return TraceMatrix();
    }

    // (3) Fill Trace Matrix /////////////////////////////////////////////////

    Traces::const_iterator trace = traces.cbegin();
    for(std::size_t i = 0; i < numD; i++, ++trace) {
      for(std::size_t j = 0; j < numT; j++) {
        T(i, j) = (*trace)[j];
      }
    }

    return T;
  }

} // namespace impl_cpa

////// CPAcontext - public ///////////////////////////////////////////////////

bool CPAcontext::isValid() const
{
  if( numTraces < 1 ) {
    return false;
  }

  if( pctRange < 1  ||  pctRange > 100 ) {
    return false;
  }

  if( sizBlock < 1 ) {
    return false;
  }

  if( sizKey < 1 ) {
    return false;
  }

  if( !campaign.isValid(sizKey, sizBlock) ) {
    return false;
  }

  if( !model ) {
    return false;
  }

  return true;
}

////// Public ////////////////////////////////////////////////////////////////

void runCPA(const CPAcontext& ctx, const cs::OutputContext *output)
{
  using namespace impl_cpa;

  // (1) Sanitize Input //////////////////////////////////////////////////////

  if( !ctx.campaign.isValid(ctx.sizKey, ctx.sizBlock) ) {
    output->logErrorf(u8"Invalid campaign \"{}\"!",
                      cs::CSTR(ctx.campaign.filename.generic_u8string().data()));
    return;
  }

  if( !ctx.isValid() ) {
    output->logError(u8"Invalid context!");
    return;
  }

  // (2) Sanitize Number of Traces ///////////////////////////////////////////

  const std::size_t numD = ctx.campaign.numEntries(ctx.numTraces);
  if( numD < 1 ) {
    output->logErrorf(u8"No traces for campaign \"{}\"!",
                      cs::CSTR(ctx.campaign.filename.generic_u8string().data()));
    return;
  }

  // (3) Create Trace Matrix /////////////////////////////////////////////////

  output->logText(u8"Step 1: Build trace matrix.");
  const TraceMatrix T = buildTraceMatrix(ctx, output, numD);
  if( T.isEmpty() ) {
    output->logError(u8"Unable to build trace matrix!");
    return;
  }

  const NumVec   meanT = columnMean(T);
  const NumVec stddevT = columnStdDev(T, meanT);
  if( meanT.empty()  ||  stddevT.empty() ) {
    output->logError(u8"Unable to compute trace auxiliaries!");
    return;
  }

  // (4) Attack with Correlation /////////////////////////////////////////////

  std::vector<std::size_t> keyi;
  std::vector<double>      keyv;

  try {
    keyi.resize(ctx.sizKey);
    keyv.resize(ctx.sizKey);
  } catch(...) {
    output->logError(u8"Unable to allocate result buffer!");
    return;
  }

  output->setProgressRange(0, int(ctx.sizKey));
  for(std::size_t k = 0; k < ctx.sizKey; k++) {
    const std::string pstr =
        std::format("{:{}}/{}",
                    k + 1, cs::countDigits(ctx.sizKey), ctx.sizKey);

    // (4.1) Create Attack Matrix ////////////////////////////////////////////

    output->logTextf(u8"Step 2 [{}]: Build attack matrix.", pstr);
    const AttackMatrix A = buildAttackMatrix(ctx, numD, k);
    if( A.isEmpty() ) {
      output->logError(u8"Unable to build attack matrix!");
      return;
    }

    // (4.2) Compute Correlation Matrix //////////////////////////////////////

    output->logTextf(u8"Step 3 [{}]: Compute correlation matrix.", pstr);
    const CorrelationMatrix R = computeCorrelation(A, T, meanT, stddevT, output);
    if( R.isEmpty() ) {
      output->logError(u8"Unable to compute correlation matrix!");
      return;
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

    output->setProgressValue(int(k + 1));
  } // For Each Byte of Key

  // (5) Output Result ///////////////////////////////////////////////////////

  std::string keystr;
  for(std::size_t k = 0; k < ctx.sizKey; k++) {
    keystr += std::format(" {:2X}", keyi[k]);
  }
  output->logTextf(u8"key ={}", keystr);

  std::string corstr;
  for(std::size_t k = 0; k < ctx.sizKey; k++) {
    corstr += std::format(" {:.3}", keyv[k]);
  }
  output->logTextf(u8"cor ={}", corstr);
}
