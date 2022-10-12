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

#include "pdt/plotters/median_summed_cost_at_time_vs_query_line_plotter.h"

#include "pdt/pgftikz/pgf_axis.h"
#include "pdt/pgftikz/pgf_fillbetween.h"
#include "pdt/pgftikz/pgf_plot.h"
#include "pdt/pgftikz/pgf_table.h"
#include "pdt/pgftikz/tikz_picture.h"

namespace pdt {

namespace plotters {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

MedianSummedCostAtTimeVsQueryLinePlotter::MedianSummedCostAtTimeVsQueryLinePlotter(
    const std::shared_ptr<const config::Configuration>& config,
    const statistics::MultiqueryStatistics& stats) :
    LatexPlotter(config),
    stats_(stats) {
  auto contextName = config_->get<std::string>("experiment/context");
}

std::shared_ptr<pgftikz::PgfAxis>
MedianSummedCostAtTimeVsQueryLinePlotter::createMedianCumulativeCostAxis(const bool initial) const {
  auto axis = std::make_shared<pgftikz::PgfAxis>();
  setMedianCumulativeCostAxisOptions(axis);

  // Fill the axis with the median cumulative cost plots of all planners.
  for (const auto& name : config_->get<std::vector<std::string>>("experiment/planners")) {
    // First the lower and upper confidence bounds, if desired.
    if (config_->get<bool>("report/medianCumulativeCostPlots/plotConfidenceIntervalInAllPlots")) {
      std::shared_ptr<pgftikz::PgfPlot> upperCi =
          createMedianCumulativeCostUpperCiPlot(name, initial);
      std::shared_ptr<pgftikz::PgfPlot> lowerCi =
          createMedianCumulativeCostLowerCiPlot(name, initial);
      std::shared_ptr<pgftikz::PgfPlot> fillCi = createMedianCumulativeCostFillCiPlot(name);
      if (!upperCi->empty() && !lowerCi->empty() && !fillCi->empty()) {
        axis->addPlot(upperCi);
        axis->addPlot(lowerCi);
        axis->addPlot(fillCi);
      }
    }

    // Then the median cumulative cost.
    axis->addPlot(createMedianCumulativeCostPlot(name, initial));
  }
  axis->options.name = "AllPlannersMedianCumulativeCostAxis";

  return axis;
}

std::shared_ptr<pgftikz::PgfAxis>
MedianSummedCostAtTimeVsQueryLinePlotter::createMedianCumulativeCostAxis(
    const std::string& plannerName, const bool initial) const {
  auto axis = std::make_shared<pgftikz::PgfAxis>();
  setMedianCumulativeCostAxisOptions(axis);

  // Add all the the median cumulative cost plots.
  std::shared_ptr<pgftikz::PgfPlot> upperCi =
      createMedianCumulativeCostUpperCiPlot(plannerName, initial);
  std::shared_ptr<pgftikz::PgfPlot> lowerCi =
      createMedianCumulativeCostLowerCiPlot(plannerName, initial);
  std::shared_ptr<pgftikz::PgfPlot> fillCi = createMedianCumulativeCostFillCiPlot(plannerName);
  if (!upperCi->empty() && !lowerCi->empty() && !fillCi->empty()) {
    axis->addPlot(upperCi);
    axis->addPlot(lowerCi);
    axis->addPlot(fillCi);
  }
  axis->addPlot(createMedianCumulativeCostPlot(plannerName, initial));
  axis->options.name = plannerName + "MedianCumulativeCostAxis";

  return axis;
}

fs::path MedianSummedCostAtTimeVsQueryLinePlotter::createMedianCumulativeCostPicture(
    const bool initial) const {
  // Create the picture and add the axis.
  pgftikz::TikzPicture picture(config_);
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

fs::path MedianSummedCostAtTimeVsQueryLinePlotter::createMedianCumulativeCostPicture(
    const std::string& plannerName, const bool initial) const {
  // Create the picture and add the axis.
  pgftikz::TikzPicture picture(config_);
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

void MedianSummedCostAtTimeVsQueryLinePlotter::setMedianCumulativeCostAxisOptions(
    std::shared_ptr<pgftikz::PgfAxis> axis) const {
  axis->options.width = config_->get<std::string>("report/medianCumulativeCostPlots/axisWidth");
  axis->options.height = config_->get<std::string>("report/medianCumulativeCostPlots/axisHeight");
  // axis->options.xmax = maxCostToBePlotted_;
  axis->options.ymax = stats_.getMaxNonInfCumulativeCost();
  // axis->options.xlog = config_->get<bool>("report/medianCumulativeCostPlots/xlog");
  axis->options.ylog = true;
  axis->options.xminorgrids = config_->get<bool>("report/medianCumulativeCostPlots/xminorgrids");
  axis->options.xmajorgrids = config_->get<bool>("report/medianCumulativeCostPlots/xmajorgrids");
  axis->options.yminorgrids = config_->get<bool>("report/medianCumulativeCostPlots/yminorgrids");
  axis->options.ymajorgrids = config_->get<bool>("report/medianCumulativeCostPlots/ymajorgrids");
  axis->options.xlabel = "Query Number"s;
  axis->options.ylabel = "Cumulative Cost"s;
  axis->options.ylabelAbsolute = true;
  axis->options.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
}

std::shared_ptr<pgftikz::PgfPlot>
MedianSummedCostAtTimeVsQueryLinePlotter::createMedianCumulativeCostPlot(
    const std::string& plannerName, const bool initial) const {
  // Get the table from the appropriate file.
  std::shared_ptr<pgftikz::PgfTable> table;
  if (initial) {
    table = std::make_shared<pgftikz::PgfTable>(
        stats_.extractMedianCumulativeInitialSolutionPerQuery(
            plannerName, config_->get<double>("report/medianCumulativeCostPlots/confidence")),
        "query number", "cumulative median initial solution cost");
  } else {
    table = std::make_shared<pgftikz::PgfTable>(
        stats_.extractMedianCumulativeFinalCostPerQuery(
            plannerName, config_->get<double>("report/medianCumulativeCostPlots/confidence")),
        "query number", "cumulative median final solution cost");
  }

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  // Create the plot and set the options.
  auto plot = std::make_shared<pgftikz::PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth = config_->get<double>("report/medianCumulativeCostPlots/lineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianCostEvolution"s;

  return plot;
}

std::shared_ptr<pgftikz::PgfPlot>
MedianSummedCostAtTimeVsQueryLinePlotter::createMedianCumulativeCostUpperCiPlot(
    const std::string& plannerName, const bool initial) const {
  // Get the table from the appropriate file.
  std::shared_ptr<pgftikz::PgfTable> table;
  if (initial) {
    table = std::make_shared<pgftikz::PgfTable>(
        stats_.extractMedianCumulativeInitialSolutionPerQuery(
            plannerName, config_->get<double>("report/medianCumulativeCostPlots/confidence")),
        "query number", "upper cumulative initial solution cost confidence bound");
  } else {
    table = std::make_shared<pgftikz::PgfTable>(
        stats_.extractMedianCumulativeFinalCostPerQuery(
            plannerName, config_->get<double>("report/medianCumulativeCostPlots/confidence")),
        "query number", "upper cumulative final solution cost confidence bound");
  }

  // Remove all nans from the table.
  table->removeRowIfDomainIsNan();
  table->removeRowIfCodomainIsNan();

  if (table->empty()) {
    return std::make_shared<pgftikz::PgfPlot>();
  }

  // Replace the infinite values with very high values, otherwise they're not plotted.
  table->replaceInCodomain(std::numeric_limits<double>::infinity(),
                           3 * stats_.getMaxNonInfCumulativeCost());

  // Create the plot and set the options.
  auto plot = std::make_shared<pgftikz::PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth =
      config_->get<double>("report/medianCumulativeCostPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianCumulativeCostUpperConfidence"s;
  plot->options.drawOpacity =
      config_->get<float>("report/medianCumulativeCostPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity =
      config_->get<float>("report/medianCumulativeCostPlots/confidenceIntervalFillOpacity");

  return plot;
}

std::shared_ptr<pgftikz::PgfPlot>
MedianSummedCostAtTimeVsQueryLinePlotter::createMedianCumulativeCostLowerCiPlot(
    const std::string& plannerName, const bool initial) const {
  // Get the table from the appropriate file.
  std::shared_ptr<pgftikz::PgfTable> table;
  if (initial) {
    table = std::make_shared<pgftikz::PgfTable>(
        stats_.extractMedianCumulativeInitialSolutionPerQuery(
            plannerName, config_->get<double>("report/medianCumulativeCostPlots/confidence")),
        "query number", "lower cumulative initial solution cost confidence bound");
  } else {
    table = std::make_shared<pgftikz::PgfTable>(
        stats_.extractMedianCumulativeFinalCostPerQuery(
            plannerName, config_->get<double>("report/medianCumulativeCostPlots/confidence")),
        "query number", "lower cumulative final solution cost confidence bound");
  }

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
      config_->get<double>("report/medianCumulativeCostPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "MedianCumulativeCostLowerConfidence"s;
  plot->options.drawOpacity =
      config_->get<float>("report/medianCumulativeCostPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity =
      config_->get<float>("report/medianCumulativeCostPlots/confidenceIntervalFillOpacity");

  return plot;
}

std::shared_ptr<pgftikz::PgfPlot>
MedianSummedCostAtTimeVsQueryLinePlotter::createMedianCumulativeCostFillCiPlot(
    const std::string& plannerName) const {
  // Fill the areas between the upper and lower bound.
  auto fillBetween = std::make_shared<pgftikz::PgfFillBetween>(
      plannerName + "MedianCumulativeCostUpperConfidence",
      plannerName + "MedianCumulativeCostLowerConfidence");

  // Create the plot.
  auto plot = std::make_shared<pgftikz::PgfPlot>(fillBetween);
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.fillOpacity =
      config_->get<float>("report/medianCumulativeCostPlots/confidenceIntervalFillOpacity");
  plot->options.drawOpacity = 0.0;

  return plot;
}

}  // namespace plotters

}  // namespace pdt
