/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014--2022
 *  Estimation, Search, and Planning (ESP) Research Group
 *  All rights reserved
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
 *   * Neither the names of the organizations nor the names of its
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

#include "pdt/plotters/query_percentile_cost_vs_time_line_plotter.h"

#include <iomanip>

#include "pdt/pgftikz/pgf_axis.h"
#include "pdt/pgftikz/pgf_fillbetween.h"
#include "pdt/pgftikz/pgf_plot.h"
#include "pdt/pgftikz/pgf_table.h"
#include "pdt/pgftikz/tikz_picture.h"

namespace pdt {

namespace plotters {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

QueryPercentileCostVsTimeLinePlotter::QueryPercentileCostVsTimeLinePlotter(
    const std::shared_ptr<const config::Configuration>& config,
    const statistics::PlanningStatistics& stats) :
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
    binnedDurations_.push_back(static_cast<double>(i + 1u) * binSize);
  }

  // Determine the min and max durations to be plotted.
  maxDurationToBePlotted_ = binnedDurations_.back();
}

std::shared_ptr<pgftikz::PgfAxis>
QueryPercentileCostVsTimeLinePlotter::createCostPercentileEvolutionAxis(
    const std::string& plannerName) const {
  auto axis = std::make_shared<pgftikz::PgfAxis>();
  setCostPercentileEvolutionAxisOptions(axis);

  // Add all the cost percentil evolution plots.
  for (const auto percentile : percentiles_) {
    axis->addPlot(createCostPercentileEvolutionPlot(plannerName, percentile));
  }
  axis->options.name = plannerName + "CostPercentileEvolutionAxis";
  axis->options.xmin = stats_.getMinInitialSolutionDuration(plannerName);
  return axis;
}

fs::path QueryPercentileCostVsTimeLinePlotter::createCostPercentileEvolutionPicture(
    const std::string& plannerName) const {
  // Create the picture and add the axis.
  pgftikz::TikzPicture picture(config_);
  picture.addAxis(createCostPercentileEvolutionAxis(plannerName));

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/"s + plannerName + "_cost_percentile_evolution_plot.tikz"s);
  picture.write(picturePath);
  return picturePath;
}

void QueryPercentileCostVsTimeLinePlotter::setCostPercentileEvolutionAxisOptions(
    std::shared_ptr<pgftikz::PgfAxis> axis) const {
  axis->options.width = config_->get<std::string>("report/costPercentileEvolutionPlots/axisWidth");
  axis->options.height =
      config_->get<std::string>("report/costPercentileEvolutionPlots/axisHeight");
  axis->options.xmax = maxDurationToBePlotted_;
  axis->options.ymax = stats_.getMaxNonInfCost();
  axis->options.xlog = config_->get<bool>("report/costPercentileEvolutionPlots/xlog");
  axis->options.xminorgrids = config_->get<bool>("report/costPercentileEvolutionPlots/xminorgrids");
  axis->options.xmajorgrids = config_->get<bool>("report/costPercentileEvolutionPlots/xmajorgrids");
  axis->options.yminorgrids = config_->get<bool>("report/costPercentileEvolutionPlots/yminorgrids");
  axis->options.ymajorgrids = config_->get<bool>("report/costPercentileEvolutionPlots/ymajorgrids");
  axis->options.xlabel = "Computation time [s]"s;
  axis->options.ylabel = "Cost"s;
  axis->options.ylabelAbsolute = true;
  axis->options.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
  axis->options.legendStyle = "font=\\footnotesize, legend cell align=left, legend columns=-1";
}

std::shared_ptr<pgftikz::PgfPlot>
QueryPercentileCostVsTimeLinePlotter::createCostPercentileEvolutionPlot(
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
  auto table = std::make_shared<pgftikz::PgfTable>(
      stats_.extractCostPercentiles(plannerName, percentiles_), "durations", percentileName.str());

  // Create the plot and set the options.
  auto plot = std::make_shared<pgftikz::PgfPlot>(table);
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

}  // namespace plotters

}  // namespace pdt
