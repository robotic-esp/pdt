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

#include "esp_tikz/median_initial_solution_plotter.h"

#include "esp_tikz/pgf_axis.h"
#include "esp_tikz/pgf_fillbetween.h"
#include "esp_tikz/pgf_plot.h"
#include "esp_tikz/pgf_table.h"
#include "esp_tikz/tikz_picture.h"

namespace esp {

namespace ompltools {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

MedianInitialSolutionPlotter::MedianInitialSolutionPlotter(
    const std::shared_ptr<const Configuration>& config, const Statistics& stats) :
    LatexPlotter(config),
    stats_(stats) {
  // Determine the min and max durations to be plotted.
  maxDurationToBePlotted_ = stats_.getMaxNonInfInitialSolutionDuration();
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

std::shared_ptr<PgfAxis> MedianInitialSolutionPlotter::createMedianInitialSolutionAxis() const {
  auto axis = std::make_shared<PgfAxis>();
  setMedianInitialSolutionAxisOptions(axis);

  // Add the initial solution plots.
  for (const auto& name : config_->get<std::vector<std::string>>("Experiment/planners")) {
    axis->addPlot(createMedianInitialSolutionPlot(name));
    axis->addPlot(createMedianInitialSolutionDurationCIPlot(name));
    axis->addPlot(createMedianInitialSolutionCostCIPlot(name));
  }

  return axis;
}

std::shared_ptr<PgfAxis> MedianInitialSolutionPlotter::createMedianInitialSolutionAxis(
    const std::string& plannerName) const {
  auto axis = std::make_shared<PgfAxis>();
  setMedianInitialSolutionAxisOptions(axis);

  // Add the initial solution plots.
  axis->addPlot(createMedianInitialSolutionPlot(plannerName));
  axis->addPlot(createMedianInitialSolutionDurationCIPlot(plannerName));
  axis->addPlot(createMedianInitialSolutionCostCIPlot(plannerName));

  return axis;
}

fs::path MedianInitialSolutionPlotter::createMedianInitialSolutionPicture() const {
  // Create the picture and add the axis.
  TikzPicture picture(config_);
  picture.addAxis(createMedianInitialSolutionAxis());

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("Experiment/results")).parent_path() /
                     fs::path("tikz/all_planners_median_initial_solution_plot.tikz");
  picture.write(picturePath);
  return picturePath;
}

fs::path MedianInitialSolutionPlotter::createMedianInitialSolutionPicture(
    const std::string& plannerName) const {
  // Create the picture and add the axis.
  TikzPicture picture(config_);
  picture.addAxis(createMedianInitialSolutionAxis(plannerName));

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("Experiment/results")).parent_path() /
                     fs::path("tikz/"s + plannerName + "_median_initial_solution_plot.tikz"s);
  picture.write(picturePath);
  return picturePath;
}

void MedianInitialSolutionPlotter::setMedianInitialSolutionAxisOptions(
    std::shared_ptr<PgfAxis> axis) const {
  axis->options.name = "MedianCostAxis";
  axis->options.width = config_->get<std::string>("MedianInitialSolutionPlots/axisWidth");
  axis->options.height = config_->get<std::string>("MedianInitialSolutionPlots/axisHeight");
  axis->options.xmin = minDurationToBePlotted_;
  axis->options.xmax = maxDurationToBePlotted_;
  axis->options.ymax = stats_.getMaxNonInfCost();
  axis->options.xlog = config_->get<bool>("MedianInitialSolutionPlots/xlog");
  axis->options.xminorgrids = config_->get<bool>("MedianInitialSolutionPlots/xminorgrids");
  axis->options.xmajorgrids = config_->get<bool>("MedianInitialSolutionPlots/xmajorgrids");
  axis->options.yminorgrids = config_->get<bool>("MedianInitialSolutionPlots/yminorgrids");
  axis->options.ymajorgrids = config_->get<bool>("MedianInitialSolutionPlots/ymajorgrids");
  axis->options.xlabel = "Computation time [s]"s;
  axis->options.ylabel = "Median cost"s;
  axis->options.ylabelAbsolute = true;
  axis->options.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
}

std::shared_ptr<PgfPlot> MedianInitialSolutionPlotter::createMedianInitialSolutionPlot(
    const std::string& plannerName) const {
  // Load the median initial duration and cost into a table.
  auto table = std::make_shared<PgfTable>(
      stats_.extractMedianInitialSolution(
          plannerName, config_->get<std::size_t>("MedianInitialSolutionPlots/confidence")),
      "median initial solution duration", "median initial solution cost");

  // Create the plot.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = config_->get<double>("MedianInitialSolutionPlots/markSize");
  plot->options.onlyMarks = true;
  plot->options.namePath = plannerName + "MedianInitialSolution"s;
  plot->options.color = config_->get<std::string>("PlannerPlotColors/" + plannerName);

  return plot;
}

std::shared_ptr<PgfPlot> MedianInitialSolutionPlotter::createMedianInitialSolutionDurationCIPlot(
    const std::string& plannerName) const {
  // Totally misusing the table class for reading in values from csvs...
  // Load the median initial solution.
  PgfTable medianInitialSolution(
      stats_.extractMedianInitialSolution(
          plannerName, config_->get<std::size_t>("MedianInitialSolutionPlots/confidence")),
      "median initial solution duration", "median initial solution cost");

  // Load the duration confidence interval.
  PgfTable interval(
      stats_.extractMedianInitialSolution(
          plannerName, config_->get<std::size_t>("MedianInitialSolutionPlots/confidence")),
      "lower initial solution duration confidence bound",
      "upper initial solution duration confidence bound");

  double medianCost = medianInitialSolution.getRow(0u).at(1u);
  double lowerDurationBound = interval.getRow(0u).at(0u);
  double upperDurationBound = interval.getRow(0u).at(1u);

  // Create a table with the correct coordinates.
  auto table = std::make_shared<PgfTable>();
  table->appendRow({lowerDurationBound, medianCost});
  table->appendRow({upperDurationBound, medianCost});

  // Create the plot.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = config_->get<double>("MedianInitialSolutionPlots/markSize");
  plot->options.mark = "|";
  plot->options.lineWidth = config_->get<double>("MedianInitialSolutionPlots/lineWidth");
  plot->options.namePath = plannerName + "MedianInitialSolutionDurationConfidenceInterval"s;
  plot->options.color = config_->get<std::string>("PlannerPlotColors/" + plannerName);

  return plot;
}

std::shared_ptr<PgfPlot> MedianInitialSolutionPlotter::createMedianInitialSolutionCostCIPlot(
    const std::string& plannerName) const {
  // Totally misusing the table class for reading in values from csvs...
  // Load the median initial solution.
  PgfTable medianInitialSolution(
      stats_.extractMedianInitialSolution(
          plannerName, config_->get<std::size_t>("MedianInitialSolutionPlots/confidence")),
      "median initial solution duration", "median initial solution cost");

  // Load the duration confidence interval.
  PgfTable interval(
      stats_.extractMedianInitialSolution(
          plannerName, config_->get<std::size_t>("MedianInitialSolutionPlots/confidence")),
      "lower initial solution cost confidence bound",
      "upper initial solution cost confidence bound");

  double medianDuration = medianInitialSolution.getRow(0u).at(0u);
  double lowerCostBound = interval.getRow(0u).at(0u);
  double upperCostBound = interval.getRow(0u).at(1u);

  // Create a table with the correct coordinates.
  auto table = std::make_shared<PgfTable>();
  table->appendRow({medianDuration, lowerCostBound});
  table->appendRow({medianDuration, upperCostBound});

  // Create the plot.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = config_->get<double>("MedianInitialSolutionPlots/markSize");
  plot->options.mark = "-";
  plot->options.lineWidth = config_->get<double>("MedianInitialSolutionPlots/lineWidth");
  plot->options.namePath = plannerName + "MedianInitialSolutionDurationConfidenceInterval"s;
  plot->options.color = config_->get<std::string>("PlannerPlotColors/" + plannerName);

  return plot;
}

}  // namespace ompltools

}  // namespace esp
