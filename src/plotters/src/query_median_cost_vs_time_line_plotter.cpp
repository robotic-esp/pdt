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

#include "pdt/plotters/query_median_cost_vs_time_line_plotter.h"

#include "pdt/pgftikz/pgf_axis.h"
#include "pdt/pgftikz/pgf_fillbetween.h"
#include "pdt/pgftikz/pgf_plot.h"
#include "pdt/pgftikz/pgf_table.h"
#include "pdt/pgftikz/tikz_picture.h"

namespace pdt {

namespace plotters {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

QueryMedianCostVsTimeLinePlotter::QueryMedianCostVsTimeLinePlotter(
    const std::shared_ptr<const config::Configuration>& config,
    const statistics::PlanningStatistics& stats) :
    LatexPlotter(config),
    stats_(stats) {
  // Compute the duration bin size.
  auto contextName = config_->get<std::string>("experiment/context");
  std::size_t numBins = static_cast<std::size_t>(
      std::ceil(config_->get<double>("context/" + contextName + "/maxTime") *
                config_->get<double>("experiment/logFrequency")));
  double binSize = 1.0 / config_->get<double>("experiment/logFrequency");
  binnedDurations_.reserve(numBins);
  for (std::size_t i = 0u; i < numBins; ++i) {
    binnedDurations_.push_back(static_cast<double>(i + 1u) * binSize);
  }

  // Determine the min and max durations to be plotted.
  maxDurationToBePlotted_ = binnedDurations_.back();
  minDurationToBePlotted_ = stats_.getMinInitialSolutionDuration();
}

std::shared_ptr<pgftikz::PgfAxis> QueryMedianCostVsTimeLinePlotter::createMedianCostEvolutionAxis()
    const {
  auto axis = std::make_shared<pgftikz::PgfAxis>();
  setMedianCostAxisOptions(axis);

  // Fill the axis with the median cost plots of all planners.
  for (const auto& name : config_->get<std::vector<std::string>>("experiment/planners")) {
    if (config_->get<bool>("planner/"s + name + "/isAnytime"s)) {
      // First the lower and upper confidence bounds, if desired.
      if (config_->get<bool>("report/medianCostPlots/plotConfidenceIntervalInAllPlots")) {
        std::shared_ptr<pgftikz::PgfPlot> upperCi = createMedianCostEvolutionUpperCiPlot(name);
        std::shared_ptr<pgftikz::PgfPlot> lowerCi = createMedianCostEvolutionLowerCiPlot(name);
        std::shared_ptr<pgftikz::PgfPlot> fillCi = createMedianCostEvolutionFillCiPlot(name);
        if (!upperCi->empty() && !lowerCi->empty() && !fillCi->empty()) {
          axis->addPlot(upperCi);
          axis->addPlot(lowerCi);
          axis->addPlot(fillCi);
        }
      }

      // Then the median cost evolution.
      axis->addPlot(createMedianCostEvolutionPlot(name));
    }
  }
  axis->options.name = "AllPlannersMedianCostAxis";

  return axis;
}

std::shared_ptr<pgftikz::PgfAxis> QueryMedianCostVsTimeLinePlotter::createMedianCostEvolutionAxis(
    const std::string& plannerName) const {
  auto axis = std::make_shared<pgftikz::PgfAxis>();
  setMedianCostAxisOptions(axis);

  // Add all the the median cost evolution plots.
  std::shared_ptr<pgftikz::PgfPlot> upperCi = createMedianCostEvolutionUpperCiPlot(plannerName);
  std::shared_ptr<pgftikz::PgfPlot> lowerCi = createMedianCostEvolutionLowerCiPlot(plannerName);
  std::shared_ptr<pgftikz::PgfPlot> fillCi = createMedianCostEvolutionFillCiPlot(plannerName);
  if (!upperCi->empty() && !lowerCi->empty() && !fillCi->empty()) {
    axis->addPlot(upperCi);
    axis->addPlot(lowerCi);
    axis->addPlot(fillCi);
  }
  axis->addPlot(createMedianCostEvolutionPlot(plannerName));
  axis->options.name = plannerName + "MedianCostAxis";

  return axis;
}

fs::path QueryMedianCostVsTimeLinePlotter::createMedianCostEvolutionPicture() const {
  // Create the picture and add the axis.
  pgftikz::TikzPicture picture(config_);
  auto axis = createMedianCostEvolutionAxis();
  picture.addAxis(axis);

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/all_planners_median_cost_plot.tikz");
  picture.write(picturePath);
  return picturePath;
}

fs::path QueryMedianCostVsTimeLinePlotter::createMedianCostEvolutionPicture(
    const std::string& plannerName) const {
  // Create the picture and add the axis.
  pgftikz::TikzPicture picture(config_);
  auto axis = createMedianCostEvolutionAxis(plannerName);
  picture.addAxis(axis);

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/"s + plannerName + "_median_cost_plot.tikz"s);
  picture.write(picturePath);
  return picturePath;
}

void QueryMedianCostVsTimeLinePlotter::setMedianCostAxisOptions(
    std::shared_ptr<pgftikz::PgfAxis> axis) const {
  axis->options.width = config_->get<std::string>("report/medianCostPlots/axisWidth");
  axis->options.height = config_->get<std::string>("report/medianCostPlots/axisHeight");
  axis->options.xmax = maxDurationToBePlotted_;
  axis->options.ymax = stats_.getMaxNonInfCost();
  axis->options.xlog = config_->get<bool>("report/medianCostPlots/xlog");
  axis->options.xminorgrids = config_->get<bool>("report/medianCostPlots/xminorgrids");
  axis->options.xmajorgrids = config_->get<bool>("report/medianCostPlots/xmajorgrids");
  axis->options.yminorgrids = config_->get<bool>("report/medianCostPlots/yminorgrids");
  axis->options.ymajorgrids = config_->get<bool>("report/medianCostPlots/ymajorgrids");
  axis->options.xlabel = "Computation time [s]"s;
  axis->options.ylabel = "Cost"s;
  axis->options.ylabelAbsolute = true;
  axis->options.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
}

std::shared_ptr<pgftikz::PgfPlot> QueryMedianCostVsTimeLinePlotter::createMedianCostEvolutionPlot(
    const std::string& plannerName) const {
  // This cannot be applied to planners that aren't anytime.
  if (!config_->get<bool>("planner/"s + plannerName + "/isAnytime"s)) {
    auto msg =
        "Cannot create median cost evolution plot of nonanytime planner '"s + plannerName + "'."s;
    throw std::invalid_argument(msg);
  }

  // Get the table from the appropriate file.
  auto table = std::make_shared<pgftikz::PgfTable>(
      stats_.extractMedians(plannerName, config_->get<double>("report/medianCostPlots/confidence")),
      "durations", "median costs");

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  // Create the plot and set the options.
  auto plot = std::make_shared<pgftikz::PgfPlot>(table);
  plot->options.mark = "\"none\""s;
  plot->options.lineWidth = config_->get<double>("report/medianCostPlots/lineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianCostEvolution"s;

  return plot;
}

std::shared_ptr<pgftikz::PgfPlot>
QueryMedianCostVsTimeLinePlotter::createMedianCostEvolutionUpperCiPlot(
    const std::string& plannerName) const {
  // This cannot be applied to planners that aren't anytime.
  if (!config_->get<bool>("planner/"s + plannerName + "/isAnytime"s)) {
    auto msg = "Cannot create median cost upper CI evolution plot of nonanytime planner '"s +
               plannerName + "'."s;
    throw std::invalid_argument(msg);
  }

  // Get the table from the appropriate file.
  auto table = std::make_shared<pgftikz::PgfTable>(
      stats_.extractMedians(plannerName, config_->get<double>("report/medianCostPlots/confidence")),
      "durations", "upper confidence bound");

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  if (table->empty()) {
    return std::make_shared<pgftikz::PgfPlot>();
  }

  // Replace the infinite values with very high values, otherwise they're not plotted.
  table->replaceInCodomain(std::numeric_limits<double>::infinity(), 3 * stats_.getMaxNonInfCost());

  // Create the plot and set the options.
  auto plot = std::make_shared<pgftikz::PgfPlot>(table);
  plot->options.mark = "\"none\""s;
  plot->options.lineWidth =
      config_->get<double>("report/medianCostPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianCostEvolutionUpperConfidence"s;
  plot->options.drawOpacity =
      config_->get<float>("report/medianCostPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity =
      config_->get<float>("report/medianCostPlots/confidenceIntervalFillOpacity");

  return plot;
}

std::shared_ptr<pgftikz::PgfPlot>
QueryMedianCostVsTimeLinePlotter::createMedianCostEvolutionLowerCiPlot(
    const std::string& plannerName) const {
  // This cannot be applied to planners that aren't anytime.
  if (!config_->get<bool>("planner/"s + plannerName + "/isAnytime"s)) {
    auto msg = "Cannot create median cost lower CI evolution plot of nonanytime planner '"s +
               plannerName + "'."s;
    throw std::invalid_argument(msg);
  }

  // Get the table from the appropriate file.
  auto table = std::make_shared<pgftikz::PgfTable>(
      stats_.extractMedians(plannerName, config_->get<double>("report/medianCostPlots/confidence")),
      "durations", "lower confidence bound");

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  if (table->empty()) {
    return std::make_shared<pgftikz::PgfPlot>();
  }

  // Create the plot and set the options.
  auto plot = std::make_shared<pgftikz::PgfPlot>(table);
  plot->options.mark = "\"none\""s;
  plot->options.lineWidth =
      config_->get<double>("report/medianCostPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianCostEvolutionLowerConfidence"s;
  plot->options.drawOpacity =
      config_->get<float>("report/medianCostPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity =
      config_->get<float>("report/medianCostPlots/confidenceIntervalFillOpacity");

  return plot;
}

std::shared_ptr<pgftikz::PgfPlot>
QueryMedianCostVsTimeLinePlotter::createMedianCostEvolutionFillCiPlot(
    const std::string& plannerName) const {
  // Fill the areas between the upper and lower bound.
  auto fillBetween =
      std::make_shared<pgftikz::PgfFillBetween>(plannerName + "MedianCostEvolutionUpperConfidence",
                                                plannerName + "MedianCostEvolutionLowerConfidence");

  // Create the plot.
  auto plot = std::make_shared<pgftikz::PgfPlot>(fillBetween);
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.fillOpacity =
      config_->get<float>("report/medianCostPlots/confidenceIntervalFillOpacity");
  plot->options.drawOpacity = 0.0;

  return plot;
}

}  // namespace plotters

}  // namespace pdt
