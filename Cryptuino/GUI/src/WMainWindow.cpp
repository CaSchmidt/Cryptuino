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

#include <QtWidgets/QFileDialog>

#include <cs/Core/QStringUtil.h>
#include <cs/Logging/DualLogger.h>
#include <cs/Logging/IProgress.h>
#include <cs/Logging/OutputContext.h>
#include <cs/Logging/WProgressLogger.h>
#include <Plot/PlotWidget.h>

#include "WMainWindow.h"
#include "ui_WMainWindow.h"

#include "CampaignModel.h"
#include "CampaignReader.h"
#include "WAnalysisOptions.h"

////// public ////////////////////////////////////////////////////////////////

WMainWindow::WMainWindow(QWidget *parent, Qt::WindowFlags flags)
  : QMainWindow(parent, flags)
  , ui(new Ui::WMainWindow)
{
  ui->setupUi(this);

  setAttribute(Qt::WA_DeleteOnClose);

  // Item Model //////////////////////////////////////////////////////////////

  CampaignModel *model = new CampaignModel(ui->tracesView);
  ui->tracesView->setModel(model);

  // Signals & Slots /////////////////////////////////////////////////////////

  connect(ui->tracesView, &QTableView::doubleClicked,
          this, &WMainWindow::plotTrace);

  // Signals & Slots - Menu //////////////////////////////////////////////////

  connect(ui->openAction, &QAction::triggered,
          this, &WMainWindow::openCampaign);
  connect(ui->openButton, &QPushButton::clicked,
          this, &WMainWindow::openCampaign);

  connect(ui->runAnalysisAction, &QAction::triggered,
          this, &WMainWindow::runAnalysis);

  connect(ui->quitAction, &QAction::triggered,
          this, &WMainWindow::close);
}

WMainWindow::~WMainWindow()
{
  delete ui;
}

////// private slots /////////////////////////////////////////////////////////

void WMainWindow::clearCampaign()
{
  _plot.reset();

  CampaignModel *model = CAMPAIGN_MODEL(ui->tracesView->model());

  model->clear();
  ui->filenameEdit->clear();
  ui->keyEdit->clear();
}

void WMainWindow::openCampaign()
{
  const QString filename =
      QFileDialog::getOpenFileName(this, tr("Open Campaign"), QString(), tr("Campaigns (*.txt)"));
  if( filename.isEmpty() ) {
    return;
  }

  clearCampaign();

  Campaign campaign;

  const std::filesystem::path path = cs::toPath(filename);
  if( !readCampaign(&campaign, path, ui->logBrowser) ) {
    return;
  }

  setCampaign(campaign);
}

void WMainWindow::plotTrace(const QModelIndex& index)
{
  if( !index.isValid() ) {
    return;
  }

  const std::filesystem::path filename =
      CAMPAIGN_MODEL(ui->tracesView->model())->traceFilename(std::size_t(index.row()));
  if( filename.empty() ) {
    return;
  }

  _plot = makeTracePlot(filename, ui->logBrowser);
  if( _plot ) {
    _plot->show();
  }
}

void WMainWindow::runAnalysis()
{
  _plot.reset();

  // (1) Campaign ////////////////////////////////////////////////////////////

  const Campaign campaign = CAMPAIGN_MODEL(ui->tracesView->model())->campaign();

  // (2) CPA Context /////////////////////////////////////////////////////////

  WAnalysisOptions options(this);
  options.set(campaign);
  options.exec();

  CPAcontext context = options.context();
  context.campaign = campaign;

  // (3) Output //////////////////////////////////////////////////////////////

  cs::WProgressLogger progressLogger(this);
  progressLogger.setWindowTitle(tr("Progress"));
  progressLogger.show();

  cs::DualLogger dualLogger{ui->logBrowser, progressLogger.logger()};
  cs::OutputContext  output{&dualLogger, true, progressLogger.progress(), true};

  // (4) Run CPA /////////////////////////////////////////////////////////////

  runCPA(context, &output);

  // (X) Done! ///////////////////////////////////////////////////////////////

  progressLogger.exec();
}

////// Private ///////////////////////////////////////////////////////////////

void WMainWindow::setCampaign(const Campaign& campaign)
{
  CampaignModel *model = CAMPAIGN_MODEL(ui->tracesView->model());

  model->set(campaign);
  ui->filenameEdit->setText(model->filename());
  ui->keyEdit->setText(model->key());
}
