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

#include "esp_plotters/query_percentile_cost_vs_time_line_plotter.h"

#include <iomanip>

#include "esp_tikz/pgf_axis.h"
#include "esp_tikz/pgf_fillbetween.h"
#include "esp_tikz/pgf_plot.h"
#include "esp_tikz/pgf_table.h"
#include "esp_tikz/tikz_picture.h"

namespace esp {

namespace ompltools {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

CostPercentileEvolutionPlotter::CostPercentileEvolutionPlotter(
    const std::shared_ptr<const Configuration>& config, const Statistics& stats) :
    LatexPlotter(config),
    stats_(stats) {
  // Compute the duration bin size.
  auto contextName = config_->get<std::string>("experiment/context");
  std::size_t numBins = static_cast<std::size_t>(
      std::ceil(config_->get<double>("context/" + contextName + "/maxTime")) *
      config_->get<double>("experiment/logFrequency"));
  double binSize = 1.0 / config_->get<double>("experiment/logFrequency");
  binnedDurations_.reserve(numBins);
  for (std::size_t i = 0u; i < numBins; ++i) {
    binnedDurations_.emplace_back(static_cast<double>(i + 1u) * binSize);
  }

  // Determine the min and max durations to be plotted.
  maxDurationToBePlotted_ = binnedDurations_.back();
}

std::shared_ptr<PgfAxis> CostPercentileEvolutionPlotter::createCostPercentileEvolutionAxis(
    const std::string& plannerName) const {
  auto axis = std::make_shared<PgfAxis>();
  setCostPercentileEvolutionAxisOptions(axis);

  // Add all the cost percentil evolution plots.
  for (const auto percentile : percentiles_) {
    axis->addPlot(createCostPercentileEvolutionPlot(plannerName, percentile));
  }
  axis->options.name = plannerName + "CostPercentileEvolutionAxis";
  axis->options.xmin = stats_.getMinInitialSolutionDuration(plannerName);
  return axis;
}

fs::path CostPercentileEvolutionPlotter::createCostPercentileEvolutionPicture(
    const std::string& plannerName) const {
  // Create the picture and add the axis.
  TikzPicture picture(config_);
  picture.addAxis(createCostPercentileEvolutionAxis(plannerName));

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/"s + plannerName + "_cost_percentile_evolution_plot.tikz"s);
  picture.write(picturePath);
  return picturePath;
}

void CostPercentileEvolutionPlotter::setCostPercentileEvolutionAxisOptions(
    std::shared_ptr<PgfAxis> axis) const {
  axis->options.width = config_->get<std::string>("costPercentileEvolutionPlots/axisWidth");
  axis->options.height = config_->get<std::string>("costPercentileEvolutionPlots/axisHeight");
  axis->options.xmax = maxDurationToBePlotted_;
  axis->options.ymax = stats_.getMaxNonInfCost();
  axis->options.xlog = config_->get<bool>("costPercentileEvolutionPlots/xlog");
  axis->options.xminorgrids = config_->get<bool>("costPercentileEvolutionPlots/xminorgrids");
  axis->options.xmajorgrids = config_->get<bool>("costPercentileEvolutionPlots/xmajorgrids");
  axis->options.yminorgrids = config_->get<bool>("costPercentileEvolutionPlots/yminorgrids");
  axis->options.ymajorgrids = config_->get<bool>("costPercentileEvolutionPlots/ymajorgrids");
  axis->options.xlabel = "Computation time [s]"s;
  axis->options.ylabel = "Cost"s;
  axis->options.ylabelAbsolute = true;
  axis->options.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
  axis->options.legendStyle = "font=\\footnotesize, legend cell align=left, legend columns=-1";
}

std::shared_ptr<PgfPlot> CostPercentileEvolutionPlotter::createCostPercentileEvolutionPlot(
    const std::string& plannerName, double percentile) const {
  // This cannot be applied to planners that aren't anytime.
  if (!stats_.getConfig()->get<bool>("planner/"s + plannerName + "/isAnytime"s)) {
    auto msg = "Cannot create cost percentile evolution plot of nonanytime planner '"s +
               plannerName + "'."s;
    throw std::invalid_argument(msg);
  }

  // Get the table from the appropriate file.
  std::stringstream percentileName;
  percentileName << std::setprecision(3) << "percentile" << percentile;
  auto table = std::make_shared<PgfTable>(stats_.extractCostPercentiles(plannerName, percentiles_),
                                          "durations", percentileName.str());

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  // Common plot options.
  plot->options.mark = "\"none\""s;
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + percentileName.str() + "CostEvolution"s;

  // Plot options that depend on the percentile.
  if (percentile == 0.01 || percentile == 0.99) {
    plot->options.lineWidth = 0.5;
    plot->options.denselyDotted = true;
  } else if (percentile == 0.05 || percentile == 0.95) {
    plot->options.lineWidth = 0.5;
    plot->options.denselyDashed = true;
  } else if (percentile == 0.25 || percentile == 0.75) {
    plot->options.lineWidth = 0.5;
  } else if (percentile == 0.50) {
    plot->options.lineWidth = 1.0;
  }

  plot->setLegend(percentileName.str().substr(10, 12));

  return plot;
}

}  // namespace ompltools

}  // namespace esp
