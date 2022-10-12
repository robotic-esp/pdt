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

#include "pdt/plotters/query_median_cost_at_first_vs_median_time_at_first_point_plotter.h"

#include "pdt/pgftikz/pgf_axis.h"
#include "pdt/pgftikz/pgf_fillbetween.h"
#include "pdt/pgftikz/pgf_plot.h"
#include "pdt/pgftikz/pgf_table.h"
#include "pdt/pgftikz/tikz_picture.h"

namespace pdt {

namespace plotters {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

QueryMedianCostAtFirstVsMedianTimeAtFirstPointPlotter::
    QueryMedianCostAtFirstVsMedianTimeAtFirstPointPlotter(
        const std::shared_ptr<const config::Configuration>& config,
        const statistics::PlanningStatistics& stats) :
    LatexPlotter(config),
    stats_(stats) {
  // Determine the min and max durations to be plotted.
  maxDurationToBePlotted_ = stats_.getMaxNonInfInitialSolutionDuration();
  minDurationToBePlotted_ = stats_.getMinInitialSolutionDuration();
}

std::shared_ptr<pgftikz::PgfAxis>
QueryMedianCostAtFirstVsMedianTimeAtFirstPointPlotter::createMedianInitialSolutionAxis() const {
  auto axis = std::make_shared<pgftikz::PgfAxis>();
  setMedianInitialSolutionAxisOptions(axis);

  // Add the initial solution plots.
  for (const auto& name : config_->get<std::vector<std::string>>("experiment/planners")) {
    axis->addPlot(createMedianInitialSolutionPlot(name));
    axis->addPlot(createMedianInitialSolutionDurationCiPlot(name));
    axis->addPlot(createMedianInitialSolutionCostCiPlot(name));
  }

  return axis;
}

std::shared_ptr<pgftikz::PgfAxis>
QueryMedianCostAtFirstVsMedianTimeAtFirstPointPlotter::createMedianInitialSolutionAxis(
    const std::string& plannerName) const {
  auto axis = std::make_shared<pgftikz::PgfAxis>();
  setMedianInitialSolutionAxisOptions(axis);

  // Add the initial solution plots.
  axis->addPlot(createMedianInitialSolutionPlot(plannerName));
  axis->addPlot(createMedianInitialSolutionDurationCiPlot(plannerName));
  axis->addPlot(createMedianInitialSolutionCostCiPlot(plannerName));

  return axis;
}

fs::path QueryMedianCostAtFirstVsMedianTimeAtFirstPointPlotter::createMedianInitialSolutionPicture()
    const {
  // Create the picture and add the axis.
  pgftikz::TikzPicture picture(config_);
  picture.addAxis(createMedianInitialSolutionAxis());

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/all_planners_median_initial_solution_plot.tikz");
  picture.write(picturePath);
  return picturePath;
}

fs::path QueryMedianCostAtFirstVsMedianTimeAtFirstPointPlotter::createMedianInitialSolutionPicture(
    const std::string& plannerName) const {
  // Create the picture and add the axis.
  pgftikz::TikzPicture picture(config_);
  picture.addAxis(createMedianInitialSolutionAxis(plannerName));

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/"s + plannerName + "_median_initial_solution_plot.tikz"s);
  picture.write(picturePath);
  return picturePath;
}

void QueryMedianCostAtFirstVsMedianTimeAtFirstPointPlotter::setMedianInitialSolutionAxisOptions(
    std::shared_ptr<pgftikz::PgfAxis> axis) const {
  axis->options.name = "MedianCostAxis";
  axis->options.width = config_->get<std::string>("report/medianInitialSolutionPlots/axisWidth");
  axis->options.height = config_->get<std::string>("report/medianInitialSolutionPlots/axisHeight");
  axis->options.xmin = minDurationToBePlotted_;
  axis->options.xmax = maxDurationToBePlotted_;
  axis->options.ymax = stats_.getMaxNonInfCost();
  axis->options.xlog = config_->get<bool>("report/medianInitialSolutionPlots/xlog");
  axis->options.xminorgrids = config_->get<bool>("report/medianInitialSolutionPlots/xminorgrids");
  axis->options.xmajorgrids = config_->get<bool>("report/medianInitialSolutionPlots/xmajorgrids");
  axis->options.yminorgrids = config_->get<bool>("report/medianInitialSolutionPlots/yminorgrids");
  axis->options.ymajorgrids = config_->get<bool>("report/medianInitialSolutionPlots/ymajorgrids");
  axis->options.xlabel = "Computation time [s]"s;
  axis->options.ylabel = "Median cost"s;
  axis->options.ylabelAbsolute = true;
  axis->options.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
}

std::shared_ptr<pgftikz::PgfPlot>
QueryMedianCostAtFirstVsMedianTimeAtFirstPointPlotter::createMedianInitialSolutionPlot(
    const std::string& plannerName) const {
  // Load the median initial duration and cost into a table.
  auto table = std::make_shared<pgftikz::PgfTable>(
      stats_.extractMedianInitialSolution(
          plannerName, config_->get<double>("report/medianInitialSolutionPlots/confidence")),
      "median initial solution duration", "median initial solution cost");

  // Create the plot.
  auto plot = std::make_shared<pgftikz::PgfPlot>(table);
  plot->options.markSize = config_->get<double>("report/medianInitialSolutionPlots/markSize");
  plot->options.onlyMarks = true;
  plot->options.namePath = plannerName + "MedianInitialSolution"s;
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);

  return plot;
}

std::shared_ptr<pgftikz::PgfPlot>
QueryMedianCostAtFirstVsMedianTimeAtFirstPointPlotter::createMedianInitialSolutionDurationCiPlot(
    const std::string& plannerName) const {
  // Totally misusing the table class for reading in values from csvs...
  // Load the median initial solution.
  pgftikz::PgfTable medianInitialSolution(
      stats_.extractMedianInitialSolution(
          plannerName, config_->get<double>("report/medianInitialSolutionPlots/confidence")),
      "median initial solution duration", "median initial solution cost");

  // Load the duration confidence interval.
  pgftikz::PgfTable interval(
      stats_.extractMedianInitialSolution(
          plannerName, config_->get<double>("report/medianInitialSolutionPlots/confidence")),
      "lower initial solution duration confidence bound",
      "upper initial solution duration confidence bound");

  double medianCost = medianInitialSolution.getRow(0u).at(1u);
  double lowerDurationBound = interval.getRow(0u).at(0u);
  double upperDurationBound = interval.getRow(0u).at(1u);

  // Create a table with the correct coordinates.
  auto table = std::make_shared<pgftikz::PgfTable>();
  table->appendRow({lowerDurationBound, medianCost});
  table->appendRow({upperDurationBound, medianCost});

  // Create the plot.
  auto plot = std::make_shared<pgftikz::PgfPlot>(table);
  plot->options.markSize = config_->get<double>("report/medianInitialSolutionPlots/markSize");
  plot->options.mark = "|";
  plot->options.lineWidth = config_->get<double>("report/medianInitialSolutionPlots/lineWidth");
  plot->options.namePath = plannerName + "MedianInitialSolutionDurationConfidenceInterval"s;
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);

  return plot;
}

std::shared_ptr<pgftikz::PgfPlot>
QueryMedianCostAtFirstVsMedianTimeAtFirstPointPlotter::createMedianInitialSolutionCostCiPlot(
    const std::string& plannerName) const {
  // Totally misusing the table class for reading in values from csvs...
  // Load the median initial solution.
  pgftikz::PgfTable medianInitialSolution(
      stats_.extractMedianInitialSolution(
          plannerName, config_->get<double>("report/medianInitialSolutionPlots/confidence")),
      "median initial solution duration", "median initial solution cost");

  // Load the duration confidence interval.
  pgftikz::PgfTable interval(
      stats_.extractMedianInitialSolution(
          plannerName, config_->get<double>("report/medianInitialSolutionPlots/confidence")),
      "lower initial solution cost confidence bound",
      "upper initial solution cost confidence bound");

  double medianDuration = medianInitialSolution.getRow(0u).at(0u);
  double lowerCostBound = interval.getRow(0u).at(0u);
  double upperCostBound = interval.getRow(0u).at(1u);

  // Create a table with the correct coordinates.
  auto table = std::make_shared<pgftikz::PgfTable>();
  table->appendRow({medianDuration, lowerCostBound});
  table->appendRow({medianDuration, upperCostBound});

  // Create the plot.
  auto plot = std::make_shared<pgftikz::PgfPlot>(table);
  plot->options.markSize = config_->get<double>("report/medianInitialSolutionPlots/markSize");
  plot->options.mark = "-";
  plot->options.lineWidth = config_->get<double>("report/medianInitialSolutionPlots/lineWidth");
  plot->options.namePath = plannerName + "MedianInitialSolutionDurationConfidenceInterval"s;
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);

  return plot;
}

}  // namespace plotters

}  // namespace pdt
