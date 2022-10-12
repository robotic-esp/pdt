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

#include "pdt/plotters/median_cost_at_first_vs_query_line_plotter.h"

#include "pdt/pgftikz/pgf_axis.h"
#include "pdt/pgftikz/pgf_fillbetween.h"
#include "pdt/pgftikz/pgf_plot.h"
#include "pdt/pgftikz/pgf_table.h"
#include "pdt/pgftikz/tikz_picture.h"

namespace pdt {

namespace plotters {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

MedianCostAtFirstVsQueryLinePlotter::MedianCostAtFirstVsQueryLinePlotter(
    const std::shared_ptr<const config::Configuration>& config,
    const statistics::MultiqueryStatistics& stats) :
    LatexPlotter(config),
    stats_(stats) {
}

std::shared_ptr<pgftikz::PgfAxis> MedianCostAtFirstVsQueryLinePlotter::createMedianInitialCostAxis()
    const {
  auto axis = std::make_shared<pgftikz::PgfAxis>();
  setMedianInitialCostAxisOptions(axis);

  // Fill the axis with the median initial cost plots of all planners.
  for (const auto& name : config_->get<std::vector<std::string>>("experiment/planners")) {
    // First the lower and upper confidence bounds, if desired.
    if (config_->get<bool>(
            "report/medianInitialCostPerQueryPlots/plotConfidenceIntervalInAllPlots")) {
      std::shared_ptr<pgftikz::PgfPlot> upperCi = createMedianInitialCostUpperCiPlot(name);
      std::shared_ptr<pgftikz::PgfPlot> lowerCi = createMedianInitialCostLowerCiPlot(name);
      std::shared_ptr<pgftikz::PgfPlot> fillCi = createMedianInitialCostFillCiPlot(name);
      if (!upperCi->empty() && !lowerCi->empty() && !fillCi->empty()) {
        axis->addPlot(upperCi);
        axis->addPlot(lowerCi);
        axis->addPlot(fillCi);
      }
    }

    // Then the median initial cost.
    axis->addPlot(createMedianInitialCostPlot(name));
  }
  axis->options.name = "AllPlannersMedianInitialCostAxis";

  return axis;
}

std::shared_ptr<pgftikz::PgfAxis> MedianCostAtFirstVsQueryLinePlotter::createMedianInitialCostAxis(
    const std::string& plannerName) const {
  auto axis = std::make_shared<pgftikz::PgfAxis>();
  setMedianInitialCostAxisOptions(axis);

  // Add all the the median initial cost plots.
  std::shared_ptr<pgftikz::PgfPlot> upperCi = createMedianInitialCostUpperCiPlot(plannerName);
  std::shared_ptr<pgftikz::PgfPlot> lowerCi = createMedianInitialCostLowerCiPlot(plannerName);
  std::shared_ptr<pgftikz::PgfPlot> fillCi = createMedianInitialCostFillCiPlot(plannerName);
  if (!upperCi->empty() && !lowerCi->empty() && !fillCi->empty()) {
    axis->addPlot(upperCi);
    axis->addPlot(lowerCi);
    axis->addPlot(fillCi);
  }
  axis->addPlot(createMedianInitialCostPlot(plannerName));
  axis->options.name = plannerName + "MedianInitialCostAxis";

  return axis;
}

fs::path MedianCostAtFirstVsQueryLinePlotter::createMedianInitialCostPicture() const {
  // Create the picture and add the axis.
  pgftikz::TikzPicture picture(config_);
  auto axis = createMedianInitialCostAxis();
  picture.addAxis(axis);

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/all_planners_median_initial_cost_query_plot.tikz");
  picture.write(picturePath);
  return picturePath;
}

fs::path MedianCostAtFirstVsQueryLinePlotter::createMedianInitialCostPicture(
    const std::string& plannerName) const {
  // Create the picture and add the axis.
  pgftikz::TikzPicture picture(config_);
  auto axis = createMedianInitialCostAxis(plannerName);
  picture.addAxis(axis);

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/"s + plannerName + "_median_initial_cost_query_plot.tikz"s);
  picture.write(picturePath);
  return picturePath;
}

void MedianCostAtFirstVsQueryLinePlotter::setMedianInitialCostAxisOptions(
    std::shared_ptr<pgftikz::PgfAxis> axis) const {
  axis->options.width =
      config_->get<std::string>("report/medianInitialCostPerQueryPlots/axisWidth");
  axis->options.height =
      config_->get<std::string>("report/medianInitialCostPerQueryPlots/axisHeight");
  // axis->options.xmax = maxCostToBePlotted_;
  axis->options.ymax = stats_.getMaxNonInfCost();
  axis->options.ylog = true;
  axis->options.xminorgrids =
      config_->get<bool>("report/medianInitialCostPerQueryPlots/xminorgrids");
  axis->options.xmajorgrids =
      config_->get<bool>("report/medianInitialCostPerQueryPlots/xmajorgrids");
  axis->options.yminorgrids =
      config_->get<bool>("report/medianInitialCostPerQueryPlots/yminorgrids");
  axis->options.ymajorgrids =
      config_->get<bool>("report/medianInitialCostPerQueryPlots/ymajorgrids");
  axis->options.xlabel = "Query Number"s;
  axis->options.ylabel = "Initial Solution Cost [s]"s;
  axis->options.ylabelAbsolute = true;
  axis->options.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
}

std::shared_ptr<pgftikz::PgfPlot> MedianCostAtFirstVsQueryLinePlotter::createMedianInitialCostPlot(
    const std::string& plannerName) const {
  // Get the table from the appropriate file.
  auto table = std::make_shared<pgftikz::PgfTable>(
      stats_.extractMedianInitialSolutionPerQuery(
          plannerName, config_->get<double>("report/medianInitialCostPerQueryPlots/confidence")),
      "query number", "median initial solution cost");

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  // Create the plot and set the options.
  auto plot = std::make_shared<pgftikz::PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth = config_->get<double>("report/medianInitialCostPerQueryPlots/lineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianInitialCostPerQuery"s;
  plot->options.constPlot = false;

  return plot;
}

std::shared_ptr<pgftikz::PgfPlot>
MedianCostAtFirstVsQueryLinePlotter::createMedianInitialCostUpperCiPlot(
    const std::string& plannerName) const {
  // Get the table from the appropriate file.
  auto table = std::make_shared<pgftikz::PgfTable>(
      stats_.extractMedianInitialSolutionPerQuery(
          plannerName, config_->get<double>("report/medianInitialCostPerQueryPlots/confidence")),
      "query number", "upper initial solution cost confidence bound");

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
  plot->options.markSize = 0.0;
  plot->options.lineWidth =
      config_->get<double>("report/medianInitialCostPerQueryPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianInitialCostPerQueryUpperConfidence"s;
  plot->options.drawOpacity =
      config_->get<float>("report/medianInitialCostPerQueryPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity =
      config_->get<float>("report/medianInitialCostPerQueryPlots/confidenceIntervalFillOpacity");

  plot->options.constPlot = false;

  return plot;
}

std::shared_ptr<pgftikz::PgfPlot>
MedianCostAtFirstVsQueryLinePlotter::createMedianInitialCostLowerCiPlot(
    const std::string& plannerName) const {
  // Get the table from the appropriate file.
  auto table = std::make_shared<pgftikz::PgfTable>(
      stats_.extractMedianInitialSolutionPerQuery(
          plannerName, config_->get<double>("report/medianInitialCostPerQueryPlots/confidence")),
      "query number", "lower initial solution cost confidence bound");

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  if (table->empty()) {
    return std::make_shared<pgftikz::PgfPlot>();
  }

  // Create the plot and set the options.
  auto plot = std::make_shared<pgftikz::PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth =
      config_->get<double>("report/medianInitialCostPerQueryPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianInitialCostPerQueryLowerConfidence"s;
  plot->options.drawOpacity =
      config_->get<float>("report/medianInitialCostPerQueryPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity =
      config_->get<float>("report/medianInitialCostPerQueryPlots/confidenceIntervalFillOpacity");
  plot->options.constPlot = false;

  return plot;
}

std::shared_ptr<pgftikz::PgfPlot>
MedianCostAtFirstVsQueryLinePlotter::createMedianInitialCostFillCiPlot(
    const std::string& plannerName) const {
  // Fill the areas between the upper and lower bound.
  auto fillBetween = std::make_shared<pgftikz::PgfFillBetween>(
      plannerName + "MedianInitialCostPerQueryUpperConfidence",
      plannerName + "MedianInitialCostPerQueryLowerConfidence");

  // Create the plot.
  auto plot = std::make_shared<pgftikz::PgfPlot>(fillBetween);
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.fillOpacity =
      config_->get<float>("report/medianInitialCostPerQueryPlots/confidenceIntervalFillOpacity");
  plot->options.drawOpacity = 0.0;

  return plot;
}

}  // namespace plotters

}  // namespace pdt
