/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, University of Oxford
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of Oxford nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Authors: Marlin Strub

#include "esp_tikz/initial_solution_duration_pdf_plotter.h"

#include <stdlib.h>
#include <algorithm>
#include <fstream>

#include <ompl/util/Console.h>

#include "esp_tikz/pgf_axis.h"
#include "esp_tikz/pgf_fillbetween.h"
#include "esp_tikz/pgf_plot.h"
#include "esp_tikz/pgf_table.h"

namespace esp {

namespace ompltools {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

InitialSolutionDurationPdfPlotter::InitialSolutionDurationPdfPlotter(
    const std::shared_ptr<const Configuration>& config, const Statistics& stats) :
    LatexPlotter(config),
    stats_(stats) {
}

std::shared_ptr<PgfAxis> InitialSolutionDurationPdfPlotter::createInitialSolutionDurationPdfAxis()
    const {
  axis_ = std::make_shared<PgfAxis>();
  setInitialSolutionDurationPdfAxisOptions(axis_);

  for (const auto& name : config_->get<std::vector<std::string>>("Experiment/planners")) {
    auto plot = createInitialSolutionDurationPdfPlot(name);
    plot->options.fillOpacity = config_->get<double>("InitialSolutionPlots/combinedFillOpacity");
    plot->options.lineWidth = config_->get<double>("InitialSolutionPlots/lineWidth");
    axis_->addPlot(plot);
  }

  return axis_;
}

std::shared_ptr<PgfAxis> InitialSolutionDurationPdfPlotter::createInitialSolutionDurationPdfAxis(
    const std::string& plannerName) const {
  axis_ = std::make_shared<PgfAxis>();
  setInitialSolutionDurationPdfAxisOptions(axis_);
  axis_->options.name = plannerName + "InitialSolutionDurationPdfAxis"s;
  axis_->addPlot(createInitialSolutionDurationPdfPlot(plannerName));
  return axis_;
}

fs::path InitialSolutionDurationPdfPlotter::createInitialSolutionDurationPdfPicture() const {
  // Create the picture and add the axis.
  TikzPicture picture(config_);
  axis_ = createInitialSolutionDurationPdfAxis();
  picture.addAxis(axis_);

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("Experiment/results")).parent_path() /
                     fs::path("tikz/all_planners_initial_solution_pdf_plot.tikz");
  picture.write(picturePath);
  return picturePath;
}

fs::path InitialSolutionDurationPdfPlotter::createInitialSolutionDurationPdfPicture(
    const std::string& plannerName) const {
  // Create the picture and add the axis.
  TikzPicture picture(config_);
  axis_ = createInitialSolutionDurationPdfAxis(plannerName);
  picture.addAxis(axis_);

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("Experiment/results")).parent_path() /
                     fs::path("tikz/"s + plannerName + "_initial_solution_pdf_plot.tikz"s);
  picture.write(picturePath);
  return picturePath;
}

std::shared_ptr<PgfPlot> InitialSolutionDurationPdfPlotter::createInitialSolutionDurationPdfPlot(
    const std::string& plannerName) const {
  // Load the data into a pgf table.
  auto table = std::make_shared<PgfTable>(stats_.extractInitialSolutionDurationPdf(plannerName),
                                          "bin begin durations", "bin counts");

  // This table should not clean its data (or should it?).
  table->setCleanData(false);

  // We need extra data points to make this look like a proper histogram.
  auto firstRow = table->getRow(0u);
  auto lastRow = table->getRow(table->getNumRows() - 1u);
  table->prependRow({firstRow.at(0u), 0.0});
  table->appendRow({lastRow.at(0u), 0.0});

  // This is a bit of a hack, but I don't see an elegant way of doing this given the class architecture.
  if (!std::isfinite(axis_->options.ymax) || axis_->options.ymax < table->getMaxValueInCol(1u)) {
    axis_->options.ymax = table->getMaxValueInCol(1u);
  }

  // Create the plot.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth = 0.0;
  plot->options.namePath = plannerName + "InitialSolutionDurationPdf"s;
  plot->options.color = config_->get<std::string>("PlannerPlotColors/" + plannerName);
  plot->options.fill = config_->get<std::string>("PlannerPlotColors/" + plannerName);

  return plot;
}

void InitialSolutionDurationPdfPlotter::setInitialSolutionDurationPdfAxisOptions(
    std::shared_ptr<PgfAxis> axis) const {
  axis->options.height = config_->get<std::string>("InitialSolutionPlots/axisHeight");
  axis->options.width = config_->get<std::string>("InitialSolutionPlots/axisWidth");
  axis->options.name = "InitialSolutionDurationPdfAxis"s;
  axis->options.xlog = config_->get<bool>("InitialSolutionPlots/xlog");
  axis->options.ymin = 0.0;
  axis->options.enlargeYLimits = "upper";
  axis->options.xminorgrids = config_->get<bool>("InitialSolutionPlots/xminorgrids");
  axis->options.xmajorgrids = config_->get<bool>("InitialSolutionPlots/xmajorgrids");
  axis->options.yminorgrids = config_->get<bool>("InitialSolutionPlots/yminorgrids");
  axis->options.ymajorgrids = config_->get<bool>("InitialSolutionPlots/ymajorgrids");
  axis->options.xlabel = "Computation time [s]";
  axis->options.ylabel = "Counts";
  axis->options.ylabelAbsolute = true;
  axis->options.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
}

}  // namespace ompltools

}  // namespace esp
