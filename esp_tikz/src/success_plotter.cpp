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

#include "esp_tikz/success_plotter.h"

#include "esp_tikz/pgf_axis.h"
#include "esp_tikz/pgf_plot.h"
#include "esp_tikz/pgf_table.h"
#include "esp_tikz/tikz_picture.h"

namespace esp {

namespace ompltools {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

SuccessPlotter::SuccessPlotter(const std::shared_ptr<const Configuration>& config,
                               const Statistics& stats) :
    LatexPlotter(config),
    stats_(stats) {
  // Determine the min and max durations to be plotted.
  maxDurationToBePlotted_ = config_->get<double>("Contexts/"s + config_->get<std::string>("Experiment/context") + "/maxTime");
  minDurationToBePlotted_ = stats_.getMinInitialSolutionDuration();
}

std::shared_ptr<PgfAxis> SuccessPlotter::createSuccessAxis() const {
  auto axis = std::make_shared<PgfAxis>();
  setSuccessAxisOptions(axis);

  // Fill the axis with the success plots of all planners.
  for (const auto& name : config_->get<std::vector<std::string>>("Experiment/planners")) {
    axis->addPlot(createSuccessPlot(name));
  }

  return axis;
}

std::shared_ptr<PgfAxis> SuccessPlotter::createSuccessAxis(const std::string& plannerName) const {
  auto axis = std::make_shared<PgfAxis>();
  setSuccessAxisOptions(axis);
  axis->addPlot(createSuccessPlot(plannerName));
  return axis;
}

fs::path SuccessPlotter::createSuccessPicture() const {
  // Create the picture and add the axis.
  TikzPicture picture(config_);
  picture.addAxis(createSuccessAxis());

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("Experiment/results")).parent_path() /
                     fs::path("tikz/all_planners_success_plot.tikz");
  picture.write(picturePath);
  return picturePath;
}

fs::path SuccessPlotter::createSuccessPicture(const std::string& plannerName) const {
  // Create the picture and add the axis.
  TikzPicture picture(config_);
  picture.addAxis(createSuccessAxis(plannerName));

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("Experiment/results")).parent_path() /
                     fs::path("tikz/"s + plannerName + "_success_plot.tikz"s);
  picture.write(picturePath);
  return picturePath;
}

void SuccessPlotter::setSuccessAxisOptions(std::shared_ptr<PgfAxis> axis) const {
  axis->options.name = "SuccessAxis";
  axis->options.width = config_->get<std::string>("SuccessPlots/axisWidth");
  axis->options.height = config_->get<std::string>("SuccessPlots/axisHeight");
  axis->options.xmin = minDurationToBePlotted_;
  axis->options.xmax = maxDurationToBePlotted_;
  axis->options.ymin = 0;
  axis->options.ymax = 100;
  axis->options.xlog = config_->get<bool>("SuccessPlots/xlog");
  axis->options.xminorgrids = config_->get<bool>("SuccessPlots/xminorgrids");
  axis->options.xmajorgrids = config_->get<bool>("SuccessPlots/xmajorgrids");
  axis->options.yminorgrids = config_->get<bool>("SuccessPlots/yminorgrids");
  axis->options.ymajorgrids = config_->get<bool>("SuccessPlots/ymajorgrids");
  axis->options.xlabel = "Computation time [s]"s;
  axis->options.ytick = config_->get<std::string>("SuccessPlots/ytick");
  axis->options.ylabel = "Success [\\%]"s;
  axis->options.ylabelAbsolute = true;
  axis->options.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
}

std::shared_ptr<PgfPlot> SuccessPlotter::createSuccessPlot(const std::string& plannerName) const {
  // Store the initial solution cdf in a pgf table.
  auto table = std::make_shared<PgfTable>(stats_.extractInitialSolutionDurationCdf(plannerName),
                                          "durations", "cdf");

  // Multiply the cdf values by 100 to get the percentage.
  table->replaceInCodomain([](double number) { return 100.0 * number; });

  // A zero in the domain will result in a jumped coordinate, because its a logarithmic plot.
  table->replaceInDomain(0.0, 1e-9);

  // Remove all rows for which domain is infinite.
  table->removeRowIfDomainEquals(std::numeric_limits<double>::infinity());

  // Add a row to draw the last element.
  table->appendRow({stats_.getMaxDuration(), table->getRow(table->getNumRows() - 1u).at(1u)});

  // Create the plot from this table.
  auto plot = std::make_shared<PgfPlot>(table);

  // Add the appropriate options.
  plot->options.markSize = config_->get<double>("SuccessPlots/markSize");
  plot->options.lineWidth = config_->get<double>("SuccessPlots/lineWidth");
  plot->options.color = config_->get<std::string>("PlannerPlotColors/" + plannerName);
  plot->options.namePath = plannerName + "Success"s;

  return plot;
}

}  // namespace ompltools

}  // namespace esp
