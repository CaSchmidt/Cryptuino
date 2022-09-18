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

#include <cs/Math/Math.h>

#include "CPA.h"

#include "Matrix.h"

namespace impl_cpa {

  struct      attack_tag {};
  struct correlation_tag {};
  struct       trace_tag {};

  using      AttackMatrix = ColMajMatrix<double,attack_tag>;
  using CorrelationMatrix = ColMajMatrix<double,correlation_tag>;
  using       TraceMatrix = ColMajMatrix<double,trace_tag>;

  using NumVec = std::vector<double>;

  using math = cs::math<double>;

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

} // namespace impl_cpa
