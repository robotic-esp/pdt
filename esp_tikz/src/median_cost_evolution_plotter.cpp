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
  auto contextName = config_->get<std::string>("Experiment/context");
  std::size_t numBins = std::ceil(config_->get<double>("Contexts/" + contextName + "/maxTime") *
                                  config_->get<double>("Experiment/logFrequency"));
  double binSize = 1.0 / config_->get<double>("Experiment/logFrequency");
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
    {50u, {{95u, {18u, 32u, 0.9511}}, {99u, {15u, 34u, 0.9910}}}},
    {100u, {{95u, {40u, 60u, 0.9540}}, {99u, {37u, 63u, 0.9907}}}},
    {200u, {{95u, {86u, 114u, 0.9520}}, {99u, {81u, 118u, 0.9906}}}},
    {250u, {{95u, {110u, 141u, 0.9503}}, {99u, {104u, 145u, 0.9900}}}},
    {300u, {{95u, {133u, 167u, 0.9502}}, {99u, {127u, 172u, 0.9903}}}},
    {400u, {{95u, {179u, 219u, 0.9522}}, {99u, {174u, 226u, 0.9907}}}},
    {500u, {{95u, {228u, 272u, 0.9508}}, {99u, {221u, 279u, 0.9905}}}},
    {600u, {{95u, {274u, 323u, 0.9508}}, {99u, {267u, 331u, 0.9907}}}},
    {700u, {{95u, {324u, 376u, 0.9517}}, {99u, {314u, 383u, 0.9901}}}},
    {800u, {{95u, {371u, 427u, 0.9511}}, {99u, {363u, 436u, 0.9900}}}},
    {900u, {{95u, {420u, 479u, 0.9503}}, {99u, {410u, 488u, 0.9904}}}},
    {1000u, {{95u, {469u, 531u, 0.9500}}, {99u, {458u, 531u, 0.9905}}}},
    {2000u, {{95u, {955u, 1043u, 0.9504}}, {99u, {940u, 1056u, 0.9901}}}},
    {5000u, {{95u, {2429u, 2568u, 0.9503}}, {99u, {2406u, 2589u, 0.9901}}}},
    {10000u, {{95u, {4897u, 5094u, 0.9500}}, {99u, {4869u, 5127u, 0.9900}}}},
    {100000u, {{95u, {49687u, 50307u, 0.9500}}, {99u, {49588u, 50403u, 0.9900}}}},
    {1000000u, {{95u, {499018u, 500978u, 0.9500}}, {99u, {498707u, 501283u, 0.9900}}}}};

std::shared_ptr<PgfAxis> MedianCostEvolutionPlotter::createMedianCostEvolutionAxis() const {
  auto axis = std::make_shared<PgfAxis>();
  setMedianCostAxisOptions(axis);

  // Fill the axis with the median cost plots of all planners.
  for (const auto& name : config_->get<std::vector<std::string>>("Experiment/planners")) {
    if (name != "RRTConnect"s) {
      // First the lower and upper confidence bounds, if desired.
      if (config_->get<bool>("MedianCostPlots/plotConfidenceIntervalInAllPlots")) {
        axis->addPlot(createMedianCostEvolutionUpperCIPlot(name));
        axis->addPlot(createMedianCostEvolutionLowerCIPlot(name));
        axis->addPlot(createMedianCostEvolutionFillCIPlot(name));
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
  axis->addPlot(createMedianCostEvolutionUpperCIPlot(plannerName));
  axis->addPlot(createMedianCostEvolutionLowerCIPlot(plannerName));
  axis->addPlot(createMedianCostEvolutionFillCIPlot(plannerName));
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
  auto picturePath = fs::path(config_->get<std::string>("Experiment/results")).parent_path() /
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
  auto picturePath = fs::path(config_->get<std::string>("Experiment/results")).parent_path() /
                     fs::path("tikz/"s + plannerName + "_median_cost_plot.tikz"s);
  picture.write(picturePath);
  return picturePath;
}

void MedianCostEvolutionPlotter::setMedianCostAxisOptions(std::shared_ptr<PgfAxis> axis) const {
  axis->options.width = config_->get<std::string>("MedianCostPlots/axisWidth");
  axis->options.height = config_->get<std::string>("MedianCostPlots/axisHeight");
  axis->options.xmax = maxDurationToBePlotted_;
  axis->options.ymax = stats_.getMaxNonInfCost();
  axis->options.xlog = config_->get<bool>("MedianCostPlots/xlog");
  axis->options.xminorgrids = config_->get<bool>("MedianCostPlots/xminorgrids");
  axis->options.xmajorgrids = config_->get<bool>("MedianCostPlots/xmajorgrids");
  axis->options.yminorgrids = config_->get<bool>("MedianCostPlots/yminorgrids");
  axis->options.ymajorgrids = config_->get<bool>("MedianCostPlots/ymajorgrids");
  axis->options.xlabel = "Computation time [s]"s;
  axis->options.ylabel = "Cost"s;
  axis->options.ylabelAbsolute = true;
  axis->options.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
}

std::shared_ptr<PgfPlot> MedianCostEvolutionPlotter::createMedianCostEvolutionPlot(
    const std::string& plannerName) const {
  // This cannot be applied to planners that aren't anytime.
  if (plannerName == "RRTConnect"s) {
    auto msg =
        "Cannot create median cost evolution plot of nonanytime planner '"s + plannerName + "'."s;
    throw std::invalid_argument(msg);
  }

  // Get the table from the appropriate file.
  auto table =
      std::make_shared<PgfTable>(stats_.extractMedians(plannerName), "durations", "median costs");

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth = config_->get<double>("MedianCostPlots/lineWidth");
  plot->options.color = config_->get<std::string>("PlannerPlotColors/" + plannerName);
  plot->options.namePath = plannerName + "MedianCostEvolution"s;

  return plot;
}

std::shared_ptr<PgfPlot> MedianCostEvolutionPlotter::createMedianCostEvolutionUpperCIPlot(
    const std::string& plannerName) const {
  // This cannot be applied to planners that aren't anytime.
  if (plannerName == "RRTConnect"s) {
    auto msg = "Cannot create median cost upper CI evolution plot of nonanytime planner '"s +
               plannerName + "'."s;
    throw std::invalid_argument(msg);
  }

  // Get the table from the appropriate file.
  auto table = std::make_shared<PgfTable>(stats_.extractMedians(plannerName), "durations",
                                          "upper confidence bound");

  // Replace the infinite values with very high values, otherwise they're not plotted.
  table->replaceInCodomain(std::numeric_limits<double>::infinity(), 3 * stats_.getMaxNonInfCost());

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth = config_->get<double>("MedianCostPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("PlannerPlotColors/" + plannerName);
  plot->options.namePath = plannerName + "MedianCostEvolutionUpperConfidence"s;
  plot->options.drawOpacity = config_->get<double>("MedianCostPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity = config_->get<double>("MedianCostPlots/confidenceIntervalFillOpacity");

  return plot;
}

std::shared_ptr<PgfPlot> MedianCostEvolutionPlotter::createMedianCostEvolutionLowerCIPlot(
    const std::string& plannerName) const {
  // This cannot be applied to planners that aren't anytime.
  if (plannerName == "RRTConnect"s) {
    auto msg = "Cannot create median cost lower CI evolution plot of nonanytime planner '"s +
               plannerName + "'."s;
    throw std::invalid_argument(msg);
  }

  // Get the table from the appropriate file.
  auto table = std::make_shared<PgfTable>(stats_.extractMedians(plannerName), "durations",
                                          "lower confidence bound");

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth = config_->get<double>("MedianCostPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("PlannerPlotColors/" + plannerName);
  plot->options.namePath = plannerName + "MedianCostEvolutionLowerConfidence"s;
  plot->options.drawOpacity = config_->get<double>("MedianCostPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity = config_->get<double>("MedianCostPlots/confidenceIntervalFillOpacity");

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
  plot->options.color = config_->get<std::string>("PlannerPlotColors/" + plannerName);
  plot->options.fillOpacity = config_->get<double>("MedianCostPlots/confidenceIntervalFillOpacity");
  plot->options.drawOpacity = 0.0;

  return plot;
}

}  // namespace ompltools

}  // namespace esp
