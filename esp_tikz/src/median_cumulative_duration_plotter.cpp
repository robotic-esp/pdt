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

#include "esp_tikz/median_cumulative_duration_plotter.h"

#include "esp_tikz/pgf_axis.h"
#include "esp_tikz/pgf_fillbetween.h"
#include "esp_tikz/pgf_plot.h"
#include "esp_tikz/pgf_table.h"
#include "esp_tikz/tikz_picture.h"

namespace esp {

namespace ompltools {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

MedianCumulativeDurationPlotter::MedianCumulativeDurationPlotter(
    const std::shared_ptr<const Configuration>& config, const MultiqueryStatistics& stats) :
    LatexPlotter(config),
    stats_(stats) {
  // Compute the duration bin size.
  auto contextName = config_->get<std::string>("experiment/context");
}

std::shared_ptr<PgfAxis> MedianCumulativeDurationPlotter::createMedianCumulativeDurationAxis()
    const {
  auto axis = std::make_shared<PgfAxis>();
  setMedianCumulativeDurationAxisOptions(axis);

  // Fill the axis with the median cost plots of all planners.
  for (const auto& name : config_->get<std::vector<std::string>>("experiment/planners")) {
    // First the lower and upper confidence bounds, if desired.
    if (config_->get<bool>(
            "medianCumulativeInitialDurationPlots/plotConfidenceIntervalInAllPlots")) {
      std::shared_ptr<PgfPlot> upperCi, lowerCi, fillCi;
      bool successCi = true;
      try {
        upperCi = createMedianCumulativeDurationUpperCiPlot(name);
        lowerCi = createMedianCumulativeDurationLowerCiPlot(name);
        fillCi = createMedianCumulativeDurationFillCiPlot(name);
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
    axis->addPlot(createMedianCumulativeDurationPlot(name));
  }
  axis->options.name = "AllPlannersMedianCumulativeDurationAxis";

  return axis;
}

std::shared_ptr<PgfAxis> MedianCumulativeDurationPlotter::createMedianCumulativeDurationAxis(
    const std::string& plannerName) const {
  auto axis = std::make_shared<PgfAxis>();
  setMedianCumulativeDurationAxisOptions(axis);

  // Add all the the median cost evolution plots.
  std::shared_ptr<PgfPlot> upperCi, lowerCi, fillCi;
  bool successCi = true;
  try {
    upperCi = createMedianCumulativeDurationUpperCiPlot(plannerName);
    lowerCi = createMedianCumulativeDurationLowerCiPlot(plannerName);
    fillCi = createMedianCumulativeDurationFillCiPlot(plannerName);
  } catch (const std::runtime_error& e) {
    // If the above methods throw, the corresponding plots should not be added.
    successCi = false;
  }
  if (successCi) {
    axis->addPlot(upperCi);
    axis->addPlot(lowerCi);
    axis->addPlot(fillCi);
  }
  axis->addPlot(createMedianCumulativeDurationPlot(plannerName));
  axis->options.name = plannerName + "MedianCumulativeDurationAxis";

  return axis;
}

fs::path MedianCumulativeDurationPlotter::createMedianCumulativeDurationPicture() const {
  // Create the picture and add the axis.
  TikzPicture picture(config_);
  auto axis = createMedianCumulativeDurationAxis();
  picture.addAxis(axis);

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/all_planners_median_cumulative_duration_plot.tikz");
  picture.write(picturePath);
  return picturePath;
}

fs::path MedianCumulativeDurationPlotter::createMedianCumulativeDurationPicture(
    const std::string& plannerName) const {
  // Create the picture and add the axis.
  TikzPicture picture(config_);
  auto axis = createMedianCumulativeDurationAxis(plannerName);
  picture.addAxis(axis);

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/"s + plannerName + "_median_cumulative_duration_plot.tikz"s);
  picture.write(picturePath);
  return picturePath;
}

void MedianCumulativeDurationPlotter::setMedianCumulativeDurationAxisOptions(
    std::shared_ptr<PgfAxis> axis) const {
  axis->options.width = config_->get<std::string>("medianCumulativeInitialDurationPlots/axisWidth");
  axis->options.height =
      config_->get<std::string>("medianCumulativeInitialDurationPlots/axisHeight");
  // axis->options.xmax = maxDurationToBePlotted_;
  axis->options.ymax = stats_.getMaxNonInfCumulativeDuration();
  // axis->options.xlog = config_->get<bool>("medianCumulativeInitialDurationPlots/xlog");
  axis->options.ylog = true;
  axis->options.xminorgrids =
      config_->get<bool>("medianCumulativeInitialDurationPlots/xminorgrids");
  axis->options.xmajorgrids =
      config_->get<bool>("medianCumulativeInitialDurationPlots/xmajorgrids");
  axis->options.yminorgrids =
      config_->get<bool>("medianCumulativeInitialDurationPlots/yminorgrids");
  axis->options.ymajorgrids =
      config_->get<bool>("medianCumulativeInitialDurationPlots/ymajorgrids");
  axis->options.xlabel = "Query Number"s;
  axis->options.ylabel = "\\footnotesize Cumulative Duration [s]"s;
  axis->options.ylabelAbsolute = true;
  axis->options.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
}

std::shared_ptr<PgfPlot> MedianCumulativeDurationPlotter::createMedianCumulativeDurationPlot(
    const std::string& plannerName) const {
  // Get the table from the appropriate file.
  auto table = std::make_shared<PgfTable>(
      stats_.extractMedianCumulativeInitialSolutionPerQuery(
          plannerName, config_->get<double>("medianCumulativeInitialDurationPlots/confidence")),
      "query number", "cumulative median initial solution duration");

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth = config_->get<double>("medianCumulativeInitialDurationPlots/lineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianCumulativeInitialDuration"s;

  return plot;
}

std::shared_ptr<PgfPlot> MedianCumulativeDurationPlotter::createMedianCumulativeDurationUpperCiPlot(
    const std::string& plannerName) const {
  // Get the table from the appropriate file.
  auto table = std::make_shared<PgfTable>(
      stats_.extractMedianCumulativeInitialSolutionPerQuery(
          plannerName, config_->get<double>("medianCumulativeInitialDurationPlots/confidence")),
      "query number", "upper cumulative initial solution duration confidence bound");

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  if (table->empty()) {
    return std::make_shared<PgfPlot>();
  }

  // Replace the infinite values with very high values, otherwise they're not plotted.
  table->replaceInCodomain(std::numeric_limits<double>::infinity(),
                           3 * stats_.getMaxNonInfCumulativeDuration());

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth =
      config_->get<double>("medianCumulativeInitialDurationPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianCumulativeInitialDurationUpperConfidence"s;
  plot->options.drawOpacity =
      config_->get<float>("medianCumulativeInitialDurationPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity =
      config_->get<float>("medianCumulativeInitialDurationPlots/confidenceIntervalFillOpacity");

  return plot;
}

std::shared_ptr<PgfPlot> MedianCumulativeDurationPlotter::createMedianCumulativeDurationLowerCiPlot(
    const std::string& plannerName) const {
  // Get the table from the appropriate file.
  auto table = std::make_shared<PgfTable>(
      stats_.extractMedianCumulativeInitialSolutionPerQuery(
          plannerName, config_->get<double>("medianCumulativeInitialDurationPlots/confidence")),
      "query number", "lower cumulative initial solution duration confidence bound");

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  if (table->empty()) {
    return std::make_shared<PgfPlot>();
  }

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth =
      config_->get<double>("medianCumulativeInitialDurationPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianCumulativeInitialDurationLowerConfidence"s;
  plot->options.drawOpacity =
      config_->get<float>("medianCumulativeInitialDurationPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity =
      config_->get<float>("medianCumulativeInitialDurationPlots/confidenceIntervalFillOpacity");

  return plot;
}

std::shared_ptr<PgfPlot> MedianCumulativeDurationPlotter::createMedianCumulativeDurationFillCiPlot(
    const std::string& plannerName) const {
  // Fill the areas between the upper and lower bound.
  auto fillBetween = std::make_shared<PgfFillBetween>(
      plannerName + "MedianCumulativeInitialDurationUpperConfidence",
      plannerName + "MedianCumulativeInitialDurationLowerConfidence");

  // Create the plot.
  auto plot = std::make_shared<PgfPlot>(fillBetween);
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.fillOpacity =
      config_->get<float>("medianCumulativeInitialDurationPlots/confidenceIntervalFillOpacity");
  plot->options.drawOpacity = 0.0;

  return plot;
}

}  // namespace ompltools

}  // namespace esp
