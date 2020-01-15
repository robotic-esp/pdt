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

#include "esp_tikz/median_cost_evolution_plotter.h"

#include "esp_tikz/pgf_axis.h"
#include "esp_tikz/pgf_fillbetween.h"
#include "esp_tikz/pgf_plot.h"
#include "esp_tikz/pgf_table.h"
#include "esp_tikz/tikz_picture.h"

namespace esp {

namespace ompltools {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

MedianCostEvolutionPlotter::MedianCostEvolutionPlotter(
    const std::shared_ptr<const Configuration>& config, const Statistics& stats) :
    LatexPlotter(config),
    stats_(stats) {
  // Compute the duration bin size.
  auto contextName = config_->get<std::string>("experiment/context");
  std::size_t numBins = std::ceil(config_->get<double>("context/" + contextName + "/maxTime") *
                                  config_->get<double>("experiment/logFrequency"));
  double binSize = 1.0 / config_->get<double>("experiment/logFrequency");
  binnedDurations_.reserve(numBins);
  for (std::size_t i = 0u; i < numBins; ++i) {
    binnedDurations_.emplace_back(static_cast<double>(i + 1u) * binSize);
  }

  // Determine the min and max durations to be plotted.
  maxDurationToBePlotted_ = binnedDurations_.back();
  minDurationToBePlotted_ = stats_.getMinInitialSolutionDuration();
}

// The lookup table for confidence intervals.
struct Interval {
  std::size_t lower{0u}, upper{0u};
  float probability{0.0f};
};
static const std::map<std::size_t, std::map<std::size_t, Interval>> medianConfidenceIntervals = {
    {10u, {{95u, {1u, 8u, 0.9511}}, {99u, {0u, 9u, 0.9910}}}},
    {50u, {{95u, {17u, 31u, 0.9511}}, {99u, {14u, 33u, 0.9910}}}},
    {100u, {{95u, {39u, 59u, 0.9540}}, {99u, {36u, 62u, 0.9907}}}},
    {200u, {{95u, {85u, 113u, 0.9520}}, {99u, {80u, 117u, 0.9906}}}},
    {250u, {{95u, {109u, 140u, 0.9503}}, {99u, {102u, 143u, 0.9900}}}},
    {300u, {{95u, {132u, 166u, 0.9502}}, {99u, {126u, 171u, 0.9903}}}},
    {400u, {{95u, {178u, 218u, 0.9522}}, {99u, {173u, 225u, 0.9907}}}},
    {500u, {{95u, {227u, 271u, 0.9508}}, {99u, {220u, 278u, 0.9905}}}},
    {600u, {{95u, {273u, 322u, 0.9508}}, {99u, {266u, 330u, 0.9907}}}},
    {700u, {{95u, {323u, 375u, 0.9517}}, {99u, {313u, 382u, 0.9901}}}},
    {800u, {{95u, {370u, 426u, 0.9511}}, {99u, {362u, 435u, 0.9900}}}},
    {900u, {{95u, {419u, 478u, 0.9503}}, {99u, {409u, 487u, 0.9904}}}},
    {1000u, {{95u, {468u, 530u, 0.9500}}, {99u, {457u, 530u, 0.9905}}}},
    {2000u, {{95u, {954u, 1042u, 0.9504}}, {99u, {939u, 1055u, 0.9901}}}},
    {5000u, {{95u, {2428u, 2567u, 0.9503}}, {99u, {2405u, 2588u, 0.9901}}}},
    {10000u, {{95u, {4896u, 5093u, 0.9500}}, {99u, {4868u, 5126u, 0.9900}}}},
    {100000u, {{95u, {49686u, 50306u, 0.9500}}, {99u, {49587u, 50402u, 0.9900}}}},
    {1000000u, {{95u, {499017u, 500977u, 0.9500}}, {99u, {498706u, 501282u, 0.9900}}}}};

std::shared_ptr<PgfAxis> MedianCostEvolutionPlotter::createMedianCostEvolutionAxis() const {
  auto axis = std::make_shared<PgfAxis>();
  setMedianCostAxisOptions(axis);

  // Fill the axis with the median cost plots of all planners.
  for (const auto& name : config_->get<std::vector<std::string>>("experiment/planners")) {
    if (config_->get<bool>("planner/"s + name + "/isAnytime"s)) {
      // First the lower and upper confidence bounds, if desired.
      if (config_->get<bool>("medianCostPlots/plotConfidenceIntervalInAllPlots")) {
        std::shared_ptr<PgfPlot> upperCI, lowerCI, fillCI;
        bool successCI = true;
        try {
          upperCI = createMedianCostEvolutionUpperCIPlot(name);
          lowerCI = createMedianCostEvolutionLowerCIPlot(name);
          fillCI = createMedianCostEvolutionFillCIPlot(name);
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
      axis->addPlot(createMedianCostEvolutionPlot(name));
    }
  }
  axis->options.name = "AllPlannersMedianCostAxis";

  return axis;
}

std::shared_ptr<PgfAxis> MedianCostEvolutionPlotter::createMedianCostEvolutionAxis(
    const std::string& plannerName) const {
  auto axis = std::make_shared<PgfAxis>();
  setMedianCostAxisOptions(axis);

  // Add all the the median cost evolution plots.
  std::shared_ptr<PgfPlot> upperCI, lowerCI, fillCI;
  bool successCI = true;
  try {
    upperCI = createMedianCostEvolutionUpperCIPlot(plannerName);
    lowerCI = createMedianCostEvolutionLowerCIPlot(plannerName);
    fillCI = createMedianCostEvolutionFillCIPlot(plannerName);
  } catch (const std::runtime_error& e) {
    // If the above methods throw, the corresponding plots should not be added.
    successCI = false;
  }
  if (successCI) {
    axis->addPlot(upperCI);
    axis->addPlot(lowerCI);
    axis->addPlot(fillCI);
  }
  axis->addPlot(createMedianCostEvolutionPlot(plannerName));
  axis->options.name = plannerName + "MedianCostAxis";

  return axis;
}

fs::path MedianCostEvolutionPlotter::createMedianCostEvolutionPicture() const {
  // Create the picture and add the axis.
  TikzPicture picture(config_);
  auto axis = createMedianCostEvolutionAxis();
  picture.addAxis(axis);

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/results")).parent_path() /
                     fs::path("tikz/all_planners_median_cost_plot.tikz");
  picture.write(picturePath);
  return picturePath;
}

fs::path MedianCostEvolutionPlotter::createMedianCostEvolutionPicture(
    const std::string& plannerName) const {
  // Create the picture and add the axis.
  TikzPicture picture(config_);
  auto axis = createMedianCostEvolutionAxis(plannerName);
  picture.addAxis(axis);

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/results")).parent_path() /
                     fs::path("tikz/"s + plannerName + "_median_cost_plot.tikz"s);
  picture.write(picturePath);
  return picturePath;
}

void MedianCostEvolutionPlotter::setMedianCostAxisOptions(std::shared_ptr<PgfAxis> axis) const {
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

std::shared_ptr<PgfPlot> MedianCostEvolutionPlotter::createMedianCostEvolutionPlot(
    const std::string& plannerName) const {
  // This cannot be applied to planners that aren't anytime.
  if (!config_->get<bool>("planner/"s + plannerName + "/isAnytime"s)) {
    auto msg =
        "Cannot create median cost evolution plot of nonanytime planner '"s + plannerName + "'."s;
    throw std::invalid_argument(msg);
  }

  // Get the table from the appropriate file.
  auto table =
      std::make_shared<PgfTable>(stats_.extractMedians(plannerName), "durations", "median costs");

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth = config_->get<double>("medianCostPlots/lineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianCostEvolution"s;

  return plot;
}

std::shared_ptr<PgfPlot> MedianCostEvolutionPlotter::createMedianCostEvolutionUpperCIPlot(
    const std::string& plannerName) const {
  // This cannot be applied to planners that aren't anytime.
  if (!config_->get<bool>("planner/"s + plannerName + "/isAnytime"s)) {
    auto msg = "Cannot create median cost upper CI evolution plot of nonanytime planner '"s +
               plannerName + "'."s;
    throw std::invalid_argument(msg);
  }

  // Get the table from the appropriate file.
  auto table = std::make_shared<PgfTable>(stats_.extractMedians(plannerName), "durations",
                                          "upper confidence bound");

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  if (table->empty()) {
    throw std::runtime_error("Cannot create UpperCI plot of '"s + plannerName + "'.");
  }

  // Replace the infinite values with very high values, otherwise they're not plotted.
  table->replaceInCodomain(std::numeric_limits<double>::infinity(), 3 * stats_.getMaxNonInfCost());

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth = config_->get<double>("medianCostPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianCostEvolutionUpperConfidence"s;
  plot->options.drawOpacity = config_->get<double>("medianCostPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity = config_->get<double>("medianCostPlots/confidenceIntervalFillOpacity");

  return plot;
}

std::shared_ptr<PgfPlot> MedianCostEvolutionPlotter::createMedianCostEvolutionLowerCIPlot(
    const std::string& plannerName) const {
  // This cannot be applied to planners that aren't anytime.
  if (!config_->get<bool>("planner/"s + plannerName + "/isAnytime"s)) {
    auto msg = "Cannot create median cost lower CI evolution plot of nonanytime planner '"s +
               plannerName + "'."s;
    throw std::invalid_argument(msg);
  }

  // Get the table from the appropriate file.
  auto table = std::make_shared<PgfTable>(stats_.extractMedians(plannerName), "durations",
                                          "lower confidence bound");

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  if (table->empty()) {
    throw std::runtime_error("Cannot create LowerCI plot of '"s + plannerName + "'.");
  }

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth = config_->get<double>("medianCostPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianCostEvolutionLowerConfidence"s;
  plot->options.drawOpacity = config_->get<double>("medianCostPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity = config_->get<double>("medianCostPlots/confidenceIntervalFillOpacity");

  return plot;
}

std::shared_ptr<PgfPlot> MedianCostEvolutionPlotter::createMedianCostEvolutionFillCIPlot(
    const std::string& plannerName) const {
  // Fill the areas between the upper and lower bound.
  auto fillBetween =
      std::make_shared<PgfFillBetween>(plannerName + "MedianCostEvolutionUpperConfidence",
                                       plannerName + "MedianCostEvolutionLowerConfidence");

  // Create the plot.
  auto plot = std::make_shared<PgfPlot>(fillBetween);
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.fillOpacity = config_->get<double>("medianCostPlots/confidenceIntervalFillOpacity");
  plot->options.drawOpacity = 0.0;

  return plot;
}

}  // namespace ompltools

}  // namespace esp
