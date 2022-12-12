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

#include <cs/Core/QStringUtil.h>

#include <Plot/Data/ConstantIntervalData.h>
#include <Plot/PlotWidget.h>

#include "Plot.h"

#include "MatInput.h"

PlotWidgetPtr makeTracePlot(const std::filesystem::path& filename,
                            const cs::ILogger *logger)
{
  using TracePlotData = plot::ConstantIntervalData<SampleBuffer::value_type>;

  // (0) Sanitize Input //////////////////////////////////////////////////////

  std::error_code ec;
  if( !std::filesystem::exists(filename, ec) ) {
    return PlotWidgetPtr{};
  }

  if( !haveMatVariable(filename, "trace", logger) ) {
    return PlotWidgetPtr{};
  }

  // (1) Create Plot Widget //////////////////////////////////////////////////

  plot::PlotSeriesDataPtr plot_data;
  SampleBuffer::value_type xStart = 0, xInter = 1;

  PlotWidgetPtr widget = std::make_unique<plot::PlotWidget>();
  widget->setWindowTitle(QStringLiteral("Plot - [%1]")
                         .arg(cs::toQString(filename.filename())));

  // (2) Read & Plot Trace Data //////////////////////////////////////////////

  const SampleBuffer trace = readMatVector(filename, "trace", logger);
  if( trace.empty() ) {
    return PlotWidgetPtr{};
  }

  if( haveMatVariable(filename, "t_trace", logger) ) {
    const SampleBuffer time_buffer = readMatVector(filename, "t_trace", logger);
    if( !time_buffer.empty()  &&  time_buffer.size() == trace.size() ) {
      xStart = time_buffer[0];
      xInter = time_buffer[1] - time_buffer[0];
    }
  }

  plot_data = TracePlotData::make(QStringLiteral("trace"), QStringLiteral("V1"),
                                  xStart, xInter, trace);

  widget->insert(plot_data.release(), Qt::blue);

  // (3) Read & Plot Trigger Data ////////////////////////////////////////////

  if( haveMatVariable(filename, "trigger", logger) ) {
    const SampleBuffer trigger = readMatVector(filename, "trigger", logger);
    if( !trigger.empty()  &&  trigger.size() == trace.size() ) {
      plot_data = TracePlotData::make(QStringLiteral("trigger"), QStringLiteral("V2"),
                                      xStart, xInter, trigger);
      widget->insert(plot_data.release(), Qt::red);
    }
  }

  // (4) Configure Axis Labels ///////////////////////////////////////////////

  plot::PlotTheme theme = widget->theme();
  theme.setAxisLabelFormat(plot::PlotTheme::XAxis, {'g', 3});
  theme.setAxisLabelFormat(plot::PlotTheme::YAxis, {'g', 3});
  widget->setTheme(theme);

  // Done! ///////////////////////////////////////////////////////////////////

  return widget;
}
