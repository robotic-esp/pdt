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

#include "pdt/plotters/median_cost_at_last_vs_query_line_plotter.h"

#include "pdt/pgftikz/pgf_axis.h"
#include "pdt/pgftikz/pgf_fillbetween.h"
#include "pdt/pgftikz/pgf_plot.h"
#include "pdt/pgftikz/pgf_table.h"
#include "pdt/pgftikz/tikz_picture.h"

namespace pdt {

namespace plotters {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

MedianCostAtLastVsQueryLinePlotter::MedianCostAtLastVsQueryLinePlotter(
    const std::shared_ptr<const config::Configuration>& config,
    const statistics::MultiqueryStatistics& stats) :
    LatexPlotter(config),
    stats_(stats) {
}

std::shared_ptr<pgftikz::PgfAxis> MedianCostAtLastVsQueryLinePlotter::createMedianFinalCostAxis()
    const {
  auto axis = std::make_shared<pgftikz::PgfAxis>();
  setMedianFinalCostAxisOptions(axis);

  // Fill the axis with the median final cost plots of all planners.
  for (const auto& name : config_->get<std::vector<std::string>>("experiment/planners")) {
    // First the lower and upper confidence bounds, if desired.
    if (config_->get<bool>(
            "report/medianFinalCostPerQueryPlots/plotConfidenceIntervalInAllPlots")) {
      std::shared_ptr<pgftikz::PgfPlot> upperCi = createMedianFinalCostUpperCiPlot(name);
      std::shared_ptr<pgftikz::PgfPlot> lowerCi = createMedianFinalCostLowerCiPlot(name);
      std::shared_ptr<pgftikz::PgfPlot> fillCi = createMedianFinalCostFillCiPlot(name);
      if (!upperCi->empty() && !lowerCi->empty() && !fillCi->empty()) {
        axis->addPlot(upperCi);
        axis->addPlot(lowerCi);
        axis->addPlot(fillCi);
      }
    }

    // Then the median final cost per query.
    axis->addPlot(createMedianFinalCostPlot(name));
  }
  axis->options.name = "AllPlannersMedianFinalCostAxis";

  return axis;
}

std::shared_ptr<pgftikz::PgfAxis> MedianCostAtLastVsQueryLinePlotter::createMedianFinalCostAxis(
    const std::string& plannerName) const {
  auto axis = std::make_shared<pgftikz::PgfAxis>();
  setMedianFinalCostAxisOptions(axis);

  // Add all the the median final cost plots.
  std::shared_ptr<pgftikz::PgfPlot> upperCi = createMedianFinalCostUpperCiPlot(plannerName);
  std::shared_ptr<pgftikz::PgfPlot> lowerCi = createMedianFinalCostLowerCiPlot(plannerName);
  std::shared_ptr<pgftikz::PgfPlot> fillCi = createMedianFinalCostFillCiPlot(plannerName);
  if (!upperCi->empty() && !lowerCi->empty() && !fillCi->empty()) {
    axis->addPlot(upperCi);
    axis->addPlot(lowerCi);
    axis->addPlot(fillCi);
  }
  axis->addPlot(createMedianFinalCostPlot(plannerName));
  axis->options.name = plannerName + "MedianFinalCostAxis";

  return axis;
}

fs::path MedianCostAtLastVsQueryLinePlotter::createMedianFinalCostPicture() const {
  // Create the picture and add the axis.
  pgftikz::TikzPicture picture(config_);
  auto axis = createMedianFinalCostAxis();
  picture.addAxis(axis);

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/all_planners_median_final_cost_query_plot.tikz");
  picture.write(picturePath);
  return picturePath;
}

fs::path MedianCostAtLastVsQueryLinePlotter::createMedianFinalCostPicture(
    const std::string& plannerName) const {
  // Create the picture and add the axis.
  pgftikz::TikzPicture picture(config_);
  auto axis = createMedianFinalCostAxis(plannerName);
  picture.addAxis(axis);

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/"s + plannerName + "_median_final_cost_query_plot.tikz"s);
  picture.write(picturePath);
  return picturePath;
}

void MedianCostAtLastVsQueryLinePlotter::setMedianFinalCostAxisOptions(
    std::shared_ptr<pgftikz::PgfAxis> axis) const {
  axis->options.width = config_->get<std::string>("report/medianFinalCostPerQueryPlots/axisWidth");
  axis->options.height =
      config_->get<std::string>("report/medianFinalCostPerQueryPlots/axisHeight");
  // axis->options.xmax = maxCostToBePlotted_;
  axis->options.ymax = stats_.getMaxNonInfCost();
  // axis->options.xlog = config_->get<bool>("report/medianFinalCostPerQueryPlots/xlog");
  axis->options.ylog = true;
  axis->options.xminorgrids = config_->get<bool>("report/medianFinalCostPerQueryPlots/xminorgrids");
  axis->options.xmajorgrids = config_->get<bool>("report/medianFinalCostPerQueryPlots/xmajorgrids");
  axis->options.yminorgrids = config_->get<bool>("report/medianFinalCostPerQueryPlots/yminorgrids");
  axis->options.ymajorgrids = config_->get<bool>("report/medianFinalCostPerQueryPlots/ymajorgrids");
  axis->options.xlabel = "Query Number"s;
  axis->options.ylabel = "Final Cost [s]"s;
  axis->options.ylabelAbsolute = true;
  axis->options.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
}

std::shared_ptr<pgftikz::PgfPlot> MedianCostAtLastVsQueryLinePlotter::createMedianFinalCostPlot(
    const std::string& plannerName) const {
  // Get the table from the appropriate file.
  auto table = std::make_shared<pgftikz::PgfTable>(
      stats_.extractMedianFinalSolutionPerQuery(
          plannerName, config_->get<double>("report/medianFinalCostPerQueryPlots/confidence")),
      "query number", "median last solution cost");

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  // Create the plot and set the options.
  auto plot = std::make_shared<pgftikz::PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth = config_->get<double>("report/medianFinalCostPerQueryPlots/lineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianFinalCostPerQuery"s;
  plot->options.constPlot = false;

  return plot;
}

std::shared_ptr<pgftikz::PgfPlot>
MedianCostAtLastVsQueryLinePlotter::createMedianFinalCostUpperCiPlot(
    const std::string& plannerName) const {
  // Get the table from the appropriate file.
  auto table = std::make_shared<pgftikz::PgfTable>(
      stats_.extractMedianFinalSolutionPerQuery(
          plannerName, config_->get<double>("report/medianFinalCostPerQueryPlots/confidence")),
      "query number", "upper last solution cost confidence bound");

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
      config_->get<double>("report/medianFinalCostPerQueryPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianFinalCostPerQueryUpperConfidence"s;
  plot->options.drawOpacity =
      config_->get<float>("report/medianFinalCostPerQueryPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity =
      config_->get<float>("report/medianFinalCostPerQueryPlots/confidenceIntervalFillOpacity");
  plot->options.constPlot = false;

  return plot;
}

std::shared_ptr<pgftikz::PgfPlot>
MedianCostAtLastVsQueryLinePlotter::createMedianFinalCostLowerCiPlot(
    const std::string& plannerName) const {
  // Get the table from the appropriate file.
  auto table = std::make_shared<pgftikz::PgfTable>(
      stats_.extractMedianFinalSolutionPerQuery(
          plannerName, config_->get<double>("report/medianFinalCostPerQueryPlots/confidence")),
      "query number", "lower last solution cost confidence bound");

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
      config_->get<double>("report/medianFinalCostPerQueryPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianFinalCostPerQueryLowerConfidence"s;
  plot->options.drawOpacity =
      config_->get<float>("report/medianFinalCostPerQueryPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity =
      config_->get<float>("report/medianFinalCostPerQueryPlots/confidenceIntervalFillOpacity");
  plot->options.constPlot = false;

  return plot;
}

std::shared_ptr<pgftikz::PgfPlot>
MedianCostAtLastVsQueryLinePlotter::createMedianFinalCostFillCiPlot(
    const std::string& plannerName) const {
  // Fill the areas between the upper and lower bound.
  auto fillBetween = std::make_shared<pgftikz::PgfFillBetween>(
      plannerName + "MedianFinalCostPerQueryUpperConfidence",
      plannerName + "MedianFinalCostPerQueryLowerConfidence");

  // Create the plot.
  auto plot = std::make_shared<pgftikz::PgfPlot>(fillBetween);
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.fillOpacity =
      config_->get<float>("report/medianFinalCostPerQueryPlots/confidenceIntervalFillOpacity");
  plot->options.drawOpacity = 0.0;
  plot->options.constPlot = false;

  return plot;
}

}  // namespace plotters

}  // namespace pdt
