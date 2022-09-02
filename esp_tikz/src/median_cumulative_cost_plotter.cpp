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

#include "esp_tikz/median_cumulative_cost_plotter.h"

#include "esp_tikz/pgf_axis.h"
#include "esp_tikz/pgf_fillbetween.h"
#include "esp_tikz/pgf_plot.h"
#include "esp_tikz/pgf_table.h"
#include "esp_tikz/tikz_picture.h"

namespace esp {

namespace ompltools {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

MedianCumulativeCostPlotter::MedianCumulativeCostPlotter(
    const std::shared_ptr<const Configuration>& config, const MultiqueryStatistics& stats) :
    LatexPlotter(config),
    stats_(stats) {
  auto contextName = config_->get<std::string>("experiment/context");
}

std::shared_ptr<PgfAxis> MedianCumulativeCostPlotter::createMedianCumulativeCostAxis(
    const bool initial) const {
  auto axis = std::make_shared<PgfAxis>();
  setMedianCumulativeCostAxisOptions(axis);

  // Fill the axis with the median cost plots of all planners.
  for (const auto& name : config_->get<std::vector<std::string>>("experiment/planners")) {
    // First the lower and upper confidence bounds, if desired.
    if (config_->get<bool>("medianCumulativeCostPlots/plotConfidenceIntervalInAllPlots")) {
      std::shared_ptr<PgfPlot> upperCI, lowerCI, fillCI;
      bool successCI = true;
      try {
        upperCI = createMedianCumulativeCostUpperCIPlot(name, initial);
        lowerCI = createMedianCumulativeCostLowerCIPlot(name, initial);
        fillCI = createMedianCumulativeCostFillCIPlot(name);
      } catch (const std::runtime_error& e) {
        // If the above methods throw, the corresponding plots should not be added.
        successCI = false;
      }
      if (successCI) {
        axis->addPlot(upperCI);
        axis->addPlot(lowerCI);
        axis->addPlot(fillCI);
      }
    }

    // Then the median cost evolution.
    axis->addPlot(createMedianCumulativeCostPlot(name, initial));
  }
  axis->options.name = "AllPlannersMedianCumulativeCostAxis";

  return axis;
}

std::shared_ptr<PgfAxis> MedianCumulativeCostPlotter::createMedianCumulativeCostAxis(
    const std::string& plannerName, const bool initial) const {
  auto axis = std::make_shared<PgfAxis>();
  setMedianCumulativeCostAxisOptions(axis);

  // Add all the the median cost evolution plots.
  std::shared_ptr<PgfPlot> upperCI, lowerCI, fillCI;
  bool successCI = true;
  try {
    upperCI = createMedianCumulativeCostUpperCIPlot(plannerName, initial);
    lowerCI = createMedianCumulativeCostLowerCIPlot(plannerName, initial);
    fillCI = createMedianCumulativeCostFillCIPlot(plannerName);
  } catch (const std::runtime_error& e) {
    // If the above methods throw, the corresponding plots should not be added.
    successCI = false;
  }
  if (successCI) {
    axis->addPlot(upperCI);
    axis->addPlot(lowerCI);
    axis->addPlot(fillCI);
  }
  axis->addPlot(createMedianCumulativeCostPlot(plannerName, initial));
  axis->options.name = plannerName + "MedianCumulativeCostAxis";

  return axis;
}

fs::path MedianCumulativeCostPlotter::createMedianCumulativeCostPicture(const bool initial) const {
  // Create the picture and add the axis.
  TikzPicture picture(config_);
  auto axis = createMedianCumulativeCostAxis(initial);
  picture.addAxis(axis);

  // Generate the tikz file.
  fs::path picturePath;
  if (initial) {
    picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                  fs::path("tikz/all_planners_median_cumulative_cost_plot.tikz");
  } else {
    picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                  fs::path("tikz/all_planners_median_cumulative_final_cost_plot.tikz");
  }
  picture.write(picturePath);
  return picturePath;
}

fs::path MedianCumulativeCostPlotter::createMedianCumulativeCostPicture(
    const std::string& plannerName, const bool initial) const {
  // Create the picture and add the axis.
  TikzPicture picture(config_);
  auto axis = createMedianCumulativeCostAxis(plannerName, initial);
  picture.addAxis(axis);

  // Generate the tikz file.
  fs::path picturePath;
  if (initial) {
    picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                  fs::path("tikz/"s + plannerName + "_median_cumulative_cost_plot.tikz"s);
  } else {
    picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                  fs::path("tikz/"s + plannerName + "_median_cumulative_final_cost_plot.tikz"s);
  }
  picture.write(picturePath);
  return picturePath;
}

void MedianCumulativeCostPlotter::setMedianCumulativeCostAxisOptions(
    std::shared_ptr<PgfAxis> axis) const {
  axis->options.width = config_->get<std::string>("medianCumulativeCostPlots/axisWidth");
  axis->options.height = config_->get<std::string>("medianCumulativeCostPlots/axisHeight");
  // axis->options.xmax = maxCostToBePlotted_;
  axis->options.ymax = stats_.getMaxNonInfCumulativeCost();
  // axis->options.xlog = config_->get<bool>("medianCumulativeCostPlots/xlog");
  axis->options.ylog = true;
  axis->options.xminorgrids = config_->get<bool>("medianCumulativeCostPlots/xminorgrids");
  axis->options.xmajorgrids = config_->get<bool>("medianCumulativeCostPlots/xmajorgrids");
  axis->options.yminorgrids = config_->get<bool>("medianCumulativeCostPlots/yminorgrids");
  axis->options.ymajorgrids = config_->get<bool>("medianCumulativeCostPlots/ymajorgrids");
  axis->options.xlabel = "Query Number"s;
  axis->options.ylabel = "Cumulative Cost"s;
  axis->options.ylabelAbsolute = true;
  axis->options.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
}

std::shared_ptr<PgfPlot> MedianCumulativeCostPlotter::createMedianCumulativeCostPlot(
    const std::string& plannerName, const bool initial) const {
  // Get the table from the appropriate file.
  std::shared_ptr<PgfTable> table;
  if (initial) {
    table = std::make_shared<PgfTable>(
        stats_.extractMedianCumulativeInitialSolutionPerQuery(
            plannerName, config_->get<double>("medianCumulativeCostPlots/confidence")),
        "query number", "cumulative median initial solution cost");
  } else {
    table = std::make_shared<PgfTable>(
        stats_.extractMedianCumulativeFinalCostPerQuery(
            plannerName, config_->get<double>("medianCumulativeCostPlots/confidence")),
        "query number", "cumulative median final solution cost");
  }

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth = config_->get<double>("medianCumulativeCostPlots/lineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianCostEvolution"s;

  return plot;
}

std::shared_ptr<PgfPlot> MedianCumulativeCostPlotter::createMedianCumulativeCostUpperCIPlot(
    const std::string& plannerName, const bool initial) const {
  // Get the table from the appropriate file.
  std::shared_ptr<PgfTable> table;
  if (initial) {
    table = std::make_shared<PgfTable>(
        stats_.extractMedianCumulativeInitialSolutionPerQuery(
            plannerName, config_->get<double>("medianCumulativeCostPlots/confidence")),
        "query number", "upper cumulative initial solution cost confidence bound");
  } else {
    table = std::make_shared<PgfTable>(
        stats_.extractMedianCumulativeFinalCostPerQuery(
            plannerName, config_->get<double>("medianCumulativeCostPlots/confidence")),
        "query number", "upper cumulative final solution cost confidence bound");
  }

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  if (table->empty()) {
    throw std::runtime_error("Cannot create UpperCI plot of '"s + plannerName + "'.");
  }

  // Replace the infinite values with very high values, otherwise they're not plotted.
  table->replaceInCodomain(std::numeric_limits<double>::infinity(),
                           3 * stats_.getMaxNonInfCumulativeCost());

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth =
      config_->get<double>("medianCumulativeCostPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianCostEvolutionUpperConfidence"s;
  plot->options.drawOpacity =
      config_->get<float>("medianCumulativeCostPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity =
      config_->get<float>("medianCumulativeCostPlots/confidenceIntervalFillOpacity");

  return plot;
}

std::shared_ptr<PgfPlot> MedianCumulativeCostPlotter::createMedianCumulativeCostLowerCIPlot(
    const std::string& plannerName, const bool initial) const {
  // Get the table from the appropriate file.
  std::shared_ptr<PgfTable> table;
  if (initial) {
    table = std::make_shared<PgfTable>(
        stats_.extractMedianCumulativeInitialSolutionPerQuery(
            plannerName, config_->get<double>("medianCumulativeCostPlots/confidence")),
        "query number", "lower cumulative initial solution cost confidence bound");
  } else {
    table = std::make_shared<PgfTable>(
        stats_.extractMedianCumulativeFinalCostPerQuery(
            plannerName, config_->get<double>("medianCumulativeCostPlots/confidence")),
        "query number", "lower cumulative final solution cost confidence bound");
  }

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  if (table->empty()) {
    throw std::runtime_error("Cannot create LowerCI plot of '"s + plannerName + "'.");
  }

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth =
      config_->get<double>("medianCumulativeCostPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianCostEvolutionLowerConfidence"s;
  plot->options.drawOpacity =
      config_->get<float>("medianCumulativeCostPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity =
      config_->get<float>("medianCumulativeCostPlots/confidenceIntervalFillOpacity");

  return plot;
}

std::shared_ptr<PgfPlot> MedianCumulativeCostPlotter::createMedianCumulativeCostFillCIPlot(
    const std::string& plannerName) const {
  // Fill the areas between the upper and lower bound.
  auto fillBetween =
      std::make_shared<PgfFillBetween>(plannerName + "MedianCostEvolutionUpperConfidence",
                                       plannerName + "MedianCostEvolutionLowerConfidence");

  // Create the plot.
  auto plot = std::make_shared<PgfPlot>(fillBetween);
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.fillOpacity =
      config_->get<float>("medianCumulativeCostPlots/confidenceIntervalFillOpacity");
  plot->options.drawOpacity = 0.0;

  return plot;
}

}  // namespace ompltools

}  // namespace esp
