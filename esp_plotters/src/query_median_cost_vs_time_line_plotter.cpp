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

#include "esp_plotters/query_median_cost_vs_time_line_plotter.h"

#include "esp_tikz/pgf_axis.h"
#include "esp_tikz/pgf_fillbetween.h"
#include "esp_tikz/pgf_plot.h"
#include "esp_tikz/pgf_table.h"
#include "esp_tikz/tikz_picture.h"

namespace esp {

namespace ompltools {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

QueryMedianCostVsTimeLinePlotter::QueryMedianCostVsTimeLinePlotter(
    const std::shared_ptr<const Configuration>& config, const Statistics& stats) :
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
    binnedDurations_.emplace_back(static_cast<double>(i + 1u) * binSize);
  }

  // Determine the min and max durations to be plotted.
  maxDurationToBePlotted_ = binnedDurations_.back();
  minDurationToBePlotted_ = stats_.getMinInitialSolutionDuration();
}

std::shared_ptr<PgfAxis> QueryMedianCostVsTimeLinePlotter::createMedianCostEvolutionAxis() const {
  auto axis = std::make_shared<PgfAxis>();
  setMedianCostAxisOptions(axis);

  // Fill the axis with the median cost plots of all planners.
  for (const auto& name : config_->get<std::vector<std::string>>("experiment/planners")) {
    if (config_->get<bool>("planner/"s + name + "/isAnytime"s)) {
      // First the lower and upper confidence bounds, if desired.
      if (config_->get<bool>("medianCostPlots/plotConfidenceIntervalInAllPlots")) {
        std::shared_ptr<PgfPlot> upperCi, lowerCi, fillCi;
        bool successCi = true;
        try {
          upperCi = createMedianCostEvolutionUpperCiPlot(name);
          lowerCi = createMedianCostEvolutionLowerCiPlot(name);
          fillCi = createMedianCostEvolutionFillCiPlot(name);
        } catch (const std::runtime_error& e) {
          // If the above methods throw, the corresponding plots should not be added.
          successCi = false;
        }
        if (successCi) {
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

std::shared_ptr<PgfAxis> QueryMedianCostVsTimeLinePlotter::createMedianCostEvolutionAxis(
    const std::string& plannerName) const {
  auto axis = std::make_shared<PgfAxis>();
  setMedianCostAxisOptions(axis);

  // Add all the the median cost evolution plots.
  std::shared_ptr<PgfPlot> upperCi, lowerCi, fillCi;
  bool successCi = true;
  try {
    upperCi = createMedianCostEvolutionUpperCiPlot(plannerName);
    lowerCi = createMedianCostEvolutionLowerCiPlot(plannerName);
    fillCi = createMedianCostEvolutionFillCiPlot(plannerName);
  } catch (const std::runtime_error& e) {
    // If the above methods throw, the corresponding plots should not be added.
    successCi = false;
  }
  if (successCi) {
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
  TikzPicture picture(config_);
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
  TikzPicture picture(config_);
  auto axis = createMedianCostEvolutionAxis(plannerName);
  picture.addAxis(axis);

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/"s + plannerName + "_median_cost_plot.tikz"s);
  picture.write(picturePath);
  return picturePath;
}

void QueryMedianCostVsTimeLinePlotter::setMedianCostAxisOptions(std::shared_ptr<PgfAxis> axis) const {
  axis->options.width = config_->get<std::string>("medianCostPlots/axisWidth");
  axis->options.height = config_->get<std::string>("medianCostPlots/axisHeight");
  axis->options.xmax = maxDurationToBePlotted_;
  axis->options.ymax = stats_.getMaxNonInfCost();
  axis->options.xlog = config_->get<bool>("medianCostPlots/xlog");
  axis->options.xminorgrids = config_->get<bool>("medianCostPlots/xminorgrids");
  axis->options.xmajorgrids = config_->get<bool>("medianCostPlots/xmajorgrids");
  axis->options.yminorgrids = config_->get<bool>("medianCostPlots/yminorgrids");
  axis->options.ymajorgrids = config_->get<bool>("medianCostPlots/ymajorgrids");
  axis->options.xlabel = "Computation time [s]"s;
  axis->options.ylabel = "Cost"s;
  axis->options.ylabelAbsolute = true;
  axis->options.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
}

std::shared_ptr<PgfPlot> QueryMedianCostVsTimeLinePlotter::createMedianCostEvolutionPlot(
    const std::string& plannerName) const {
  // This cannot be applied to planners that aren't anytime.
  if (!config_->get<bool>("planner/"s + plannerName + "/isAnytime"s)) {
    auto msg =
        "Cannot create median cost evolution plot of nonanytime planner '"s + plannerName + "'."s;
    throw std::invalid_argument(msg);
  }

  // Get the table from the appropriate file.
  auto table = std::make_shared<PgfTable>(
      stats_.extractMedians(plannerName, config_->get<double>("medianCostPlots/confidence")),
      "durations", "median costs");

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.mark = "\"none\""s;
  plot->options.lineWidth = config_->get<double>("medianCostPlots/lineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianCostEvolution"s;

  return plot;
}

std::shared_ptr<PgfPlot> QueryMedianCostVsTimeLinePlotter::createMedianCostEvolutionUpperCiPlot(
    const std::string& plannerName) const {
  // This cannot be applied to planners that aren't anytime.
  if (!config_->get<bool>("planner/"s + plannerName + "/isAnytime"s)) {
    auto msg = "Cannot create median cost upper CI evolution plot of nonanytime planner '"s +
               plannerName + "'."s;
    throw std::invalid_argument(msg);
  }

  // Get the table from the appropriate file.
  auto table = std::make_shared<PgfTable>(
      stats_.extractMedians(plannerName, config_->get<double>("medianCostPlots/confidence")),
      "durations", "upper confidence bound");

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  if (table->empty()) {
    throw std::runtime_error("Cannot create upper CI median cost plot of '"s + plannerName + "'.");
  }

  // Replace the infinite values with very high values, otherwise they're not plotted.
  table->replaceInCodomain(std::numeric_limits<double>::infinity(), 3 * stats_.getMaxNonInfCost());

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.mark = "\"none\""s;
  plot->options.lineWidth = config_->get<double>("medianCostPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianCostEvolutionUpperConfidence"s;
  plot->options.drawOpacity = config_->get<float>("medianCostPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity = config_->get<float>("medianCostPlots/confidenceIntervalFillOpacity");

  return plot;
}

std::shared_ptr<PgfPlot> QueryMedianCostVsTimeLinePlotter::createMedianCostEvolutionLowerCiPlot(
    const std::string& plannerName) const {
  // This cannot be applied to planners that aren't anytime.
  if (!config_->get<bool>("planner/"s + plannerName + "/isAnytime"s)) {
    auto msg = "Cannot create median cost lower CI evolution plot of nonanytime planner '"s +
               plannerName + "'."s;
    throw std::invalid_argument(msg);
  }

  // Get the table from the appropriate file.
  auto table = std::make_shared<PgfTable>(
      stats_.extractMedians(plannerName, config_->get<double>("medianCostPlots/confidence")),
      "durations", "lower confidence bound");

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  if (table->empty()) {
    throw std::runtime_error("Cannot create lower CI median cost plot of '"s + plannerName + "'.");
  }

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.mark = "\"none\""s;
  plot->options.lineWidth = config_->get<double>("medianCostPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianCostEvolutionLowerConfidence"s;
  plot->options.drawOpacity = config_->get<float>("medianCostPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity = config_->get<float>("medianCostPlots/confidenceIntervalFillOpacity");

  return plot;
}

std::shared_ptr<PgfPlot> QueryMedianCostVsTimeLinePlotter::createMedianCostEvolutionFillCiPlot(
    const std::string& plannerName) const {
  // Fill the areas between the upper and lower bound.
  auto fillBetween =
      std::make_shared<PgfFillBetween>(plannerName + "MedianCostEvolutionUpperConfidence",
                                       plannerName + "MedianCostEvolutionLowerConfidence");

  // Create the plot.
  auto plot = std::make_shared<PgfPlot>(fillBetween);
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.fillOpacity = config_->get<float>("medianCostPlots/confidenceIntervalFillOpacity");
  plot->options.drawOpacity = 0.0;

  return plot;
}

}  // namespace ompltools

}  // namespace esp
