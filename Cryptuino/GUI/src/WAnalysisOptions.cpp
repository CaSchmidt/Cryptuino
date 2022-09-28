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

#include "WAnalysisOptions.h"
#include "ui_WAnalysisOptions.h"

#include <cs/Core/QStringUtil.h>

#include "Campaign.h"
#include "PowerAES.h"
#include "TriggerImpl.h"

////// Private ///////////////////////////////////////////////////////////////

namespace impl {

  TriggerPtr makeEvent(const std::u8string& name, const double level)
  {
    if(        TriggerGreater::name() == name ) {
      return TriggerGreater::make(level);
    } else if( TriggerLess::name() == name ) {
      return TriggerLess::make(level);
    }
    return TriggerPtr{};
  }

  inline TriggerPtr makeEvent(const QComboBox *combo, const QDoubleSpinBox *spin)
  {
    return makeEvent(cs::toUtf8String(combo->currentText()), spin->value());
  }

  PowerModelPtr makeModel(const std::u8string& name)
  {
    if( PowerAES::EncryptionRound1SubBytes::name() == name ) {
      return PowerAES::EncryptionRound1SubBytes::make();
    }
    return PowerModelPtr{};
  }

  inline PowerModelPtr makeModel(const QComboBox *combo)
  {
    return makeModel(cs::toUtf8String(combo->currentText()));
  }

} // namespace impl

////// public ////////////////////////////////////////////////////////////////

WAnalysisOptions::WAnalysisOptions(QWidget *parent, Qt::WindowFlags f)
  : QDialog(parent, f)
  , ui(new Ui::WAnalysisOptions)
{
  ui->setupUi(this);

  // Initialization //////////////////////////////////////////////////////////

  initializeAlgorithm();
  initializeEvent();
  initializeModel();
  initializeTrace(1);
}

WAnalysisOptions::~WAnalysisOptions()
{
  delete ui;
}

void WAnalysisOptions::set(const Campaign& c)
{
  initializeTrace(int(c.entries.size()));
}

CPAcontext WAnalysisOptions::context() const
{
  CPAcontext ctx;

  ctx.event = impl::makeEvent(ui->eventCombo, ui->eventSpin);
  ctx.model = impl::makeModel(ui->modelCombo);

  ctx.numTraces = std::size_t(ui->numTracesSpin->value());
  ctx.pctRange  = std::size_t(ui->pctRangeSpin->value());
  ctx.sizBlock  = std::size_t(ui->sizBlockSpin->value());
  ctx.sizKey    = std::size_t(ui->sizKeySpin->value());

  return ctx;
}

////// private ///////////////////////////////////////////////////////////////

void WAnalysisOptions::initializeAlgorithm()
{
  ui->sizBlockSpin->setRange(0, 128);
  ui->sizBlockSpin->setSuffix(QStringLiteral(" [byte]"));
  ui->sizBlockSpin->setValue(0);

  ui->sizKeySpin->setRange(0, 128);
  ui->sizKeySpin->setSuffix(QStringLiteral(" [byte]"));
  ui->sizKeySpin->setValue(0);
}

void WAnalysisOptions::initializeEvent()
{
  ui->eventCombo->clear();
  ui->eventCombo->addItem(QStringLiteral("<none>"));
  ui->eventCombo->addItem(cs::toQString(TriggerGreater::name()));
  ui->eventCombo->addItem(cs::toQString(TriggerLess::name()));

  ui->eventSpin->setDecimals(2);
  ui->eventSpin->setMinimum(-10);
  ui->eventSpin->setMaximum(10);
  ui->eventSpin->setValue(0);

}

void WAnalysisOptions::initializeModel()
{
  ui->modelCombo->clear();
  ui->modelCombo->addItem(cs::toQString(PowerAES::EncryptionRound1SubBytes::name()));
}

void WAnalysisOptions::initializeTrace(const int numTraces)
{
  ui->numTracesSpin->setRange(0, numTraces);
  ui->numTracesSpin->setValue(0);

  ui->pctRangeSpin->setRange(0, 100);
  ui->pctRangeSpin->setSuffix(QStringLiteral("%"));
  ui->pctRangeSpin->setValue(0);
}
