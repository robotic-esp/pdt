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

#include "pdt/plotters/query_success_vs_time_line_plotter.h"

#include "pdt/pgftikz/pgf_axis.h"
#include "pdt/pgftikz/pgf_fillbetween.h"
#include "pdt/pgftikz/pgf_plot.h"
#include "pdt/pgftikz/pgf_table.h"
#include "pdt/pgftikz/tikz_picture.h"

namespace pdt {

namespace plotters {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

QuerySuccessVsTimeLinePlotter::QuerySuccessVsTimeLinePlotter(
    const std::shared_ptr<const config::Configuration>& config,
    const statistics::PlanningStatistics& stats) :
    LatexPlotter(config),
    stats_(stats) {
  // Determine the min and max durations to be plotted.
  maxDurationToBePlotted_ = config_->get<double>(
      "context/"s + config_->get<std::string>("experiment/context") + "/maxTime");
  minDurationToBePlotted_ = stats_.getMinInitialSolutionDuration();
}

std::shared_ptr<pgftikz::PgfAxis> QuerySuccessVsTimeLinePlotter::createSuccessAxis() const {
  auto axis = std::make_shared<pgftikz::PgfAxis>();
  setSuccessAxisOptions(axis);

  // Fill the axis with the success plots of all planners.
  for (const auto& name : config_->get<std::vector<std::string>>("experiment/planners")) {
    // First the lower and upper confidence bounds, if desired.
    if (config_->get<bool>("report/successPlots/plotConfidenceIntervalInAllPlots")) {
      std::shared_ptr<pgftikz::PgfPlot> upperCi = createSuccessUpperCiPlot(name);
      std::shared_ptr<pgftikz::PgfPlot> lowerCi = createSuccessLowerCiPlot(name);
      std::shared_ptr<pgftikz::PgfPlot> fillCi = createSuccessFillCiPlot(name);
      if (!upperCi->empty() && !lowerCi->empty() && !fillCi->empty()) {
        axis->addPlot(upperCi);
        axis->addPlot(lowerCi);
        axis->addPlot(fillCi);
      }
    }

    // Then the estimate.
    axis->addPlot(createSuccessPlot(name));
  }

  return axis;
}

std::shared_ptr<pgftikz::PgfAxis> QuerySuccessVsTimeLinePlotter::createSuccessAxis(
    const std::string& plannerName) const {
  auto axis = std::make_shared<pgftikz::PgfAxis>();
  setSuccessAxisOptions(axis);

  // First the lower and upper confidence bounds, if desired.
  if (config_->get<bool>("report/successPlots/plotConfidenceIntervalInAllPlots")) {
    std::shared_ptr<pgftikz::PgfPlot> upperCi = createSuccessUpperCiPlot(plannerName);
    std::shared_ptr<pgftikz::PgfPlot> lowerCi = createSuccessLowerCiPlot(plannerName);
    std::shared_ptr<pgftikz::PgfPlot> fillCi = createSuccessFillCiPlot(plannerName);
    if (!upperCi->empty() && !lowerCi->empty() && !fillCi->empty()) {
      axis->addPlot(upperCi);
      axis->addPlot(lowerCi);
      axis->addPlot(fillCi);
    }
  }

  // Then the estimate.
  axis->addPlot(createSuccessPlot(plannerName));
  return axis;
}

fs::path QuerySuccessVsTimeLinePlotter::createSuccessPicture() const {
  // Create the picture and add the axis.
  pgftikz::TikzPicture picture(config_);
  picture.addAxis(createSuccessAxis());

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/all_planners_success_plot.tikz");
  picture.write(picturePath);
  return picturePath;
}

fs::path QuerySuccessVsTimeLinePlotter::createSuccessPicture(const std::string& plannerName) const {
  // Create the picture and add the axis.
  pgftikz::TikzPicture picture(config_);
  picture.addAxis(createSuccessAxis(plannerName));

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/"s + plannerName + "_success_plot.tikz"s);
  picture.write(picturePath);
  return picturePath;
}

void QuerySuccessVsTimeLinePlotter::setSuccessAxisOptions(
    std::shared_ptr<pgftikz::PgfAxis> axis) const {
  axis->options.name = "SuccessAxis";
  axis->options.width = config_->get<std::string>("report/successPlots/axisWidth");
  axis->options.height = config_->get<std::string>("report/successPlots/axisHeight");
  axis->options.xmin = minDurationToBePlotted_;
  axis->options.xmax = maxDurationToBePlotted_;
  axis->options.ymin = 0;
  axis->options.ymax = 100;
  axis->options.xlog = config_->get<bool>("report/successPlots/xlog");
  axis->options.xminorgrids = config_->get<bool>("report/successPlots/xminorgrids");
  axis->options.xmajorgrids = config_->get<bool>("report/successPlots/xmajorgrids");
  axis->options.yminorgrids = config_->get<bool>("report/successPlots/yminorgrids");
  axis->options.ymajorgrids = config_->get<bool>("report/successPlots/ymajorgrids");
  axis->options.xlabel = "Computation time [s]"s;
  axis->options.ytick = config_->get<std::string>("report/successPlots/ytick");
  axis->options.ylabel = "Success [\\%]"s;
  axis->options.ylabelAbsolute = true;
  axis->options.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
}

std::shared_ptr<pgftikz::PgfPlot> QuerySuccessVsTimeLinePlotter::createSuccessPlot(
    const std::string& plannerName) const {
  // Store the initial solution edf in a pgf table.
  auto table = std::make_shared<pgftikz::PgfTable>(
      stats_.extractInitialSolutionDurationEdf(
          plannerName, config_->get<double>("report/successPlots/confidence")),
      "durations", "edf");

  // Remove all rows for which domain is infinite.
  table->removeRowIfDomainEquals(std::numeric_limits<double>::infinity());

  // Multiply the edf values by 100 to get the percentage.
  table->replaceInCodomain([](double number) { return 100.0 * number; });

  // A zero in the domain will result in a jumped coordinate, because its a logarithmic plot.
  table->replaceInDomain(0.0, 1e-9);

  // Add a row to draw the last element.
  table->appendRow({stats_.getMaxDuration(), table->getRow(table->getNumRows() - 1u).at(1u)});

  // Create the plot from this table.
  auto plot = std::make_shared<pgftikz::PgfPlot>(table);

  // Add the appropriate options.
  plot->options.mark = "\"none\""s;
  plot->options.lineWidth = config_->get<double>("report/successPlots/lineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "Success"s;

  return plot;
}

std::shared_ptr<pgftikz::PgfPlot> QuerySuccessVsTimeLinePlotter::createSuccessUpperCiPlot(
    const std::string& plannerName) const {
  // Get the table from the appropriate file.
  auto table = std::make_shared<pgftikz::PgfTable>(
      stats_.extractInitialSolutionDurationEdf(
          plannerName, config_->get<double>("report/successPlots/confidence")),
      "durations", "upper confidence bound");

  // Remove all rows for which domain is infinite.
  table->removeRowIfDomainEquals(std::numeric_limits<double>::infinity());

  if (table->empty()) {
    return std::make_shared<pgftikz::PgfPlot>();
  }

  // Multiply the cdf values by 100 to get the percentage.
  table->replaceInCodomain([](double number) { return 100.0 * number; });

  // A zero in the domain will result in a jumped coordinate, because its a logarithmic plot.
  table->replaceInDomain(0.0, 1e-9);

  // Add a row to draw the last element.
  table->appendRow({stats_.getMaxDuration(), table->getRow(table->getNumRows() - 1u).at(1u)});

  // Create the plot and set the options.
  auto plot = std::make_shared<pgftikz::PgfPlot>(table);
  plot->options.mark = "\"none\""s;
  plot->options.lineWidth = config_->get<double>("report/successPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "SuccessUpperConfidence"s;
  plot->options.drawOpacity =
      config_->get<float>("report/successPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity =
      config_->get<float>("report/successPlots/confidenceIntervalFillOpacity");

  return plot;
}

std::shared_ptr<pgftikz::PgfPlot> QuerySuccessVsTimeLinePlotter::createSuccessLowerCiPlot(
    const std::string& plannerName) const {
  // Get the table from the appropriate file.
  auto table = std::make_shared<pgftikz::PgfTable>(
      stats_.extractInitialSolutionDurationEdf(
          plannerName, config_->get<double>("report/successPlots/confidence")),
      "durations", "lower confidence bound");

  // Remove all rows for which domain is infinite.
  table->removeRowIfDomainEquals(std::numeric_limits<double>::infinity());

  if (table->empty()) {
    return std::make_shared<pgftikz::PgfPlot>();
  }

  // Multiply the cdf values by 100 to get the percentage.
  table->replaceInCodomain([](double number) { return 100.0 * number; });

  // A zero in the domain will result in a jumped coordinate, because its a logarithmic plot.
  table->replaceInDomain(0.0, 1e-9);

  // Add a row to draw the last element.
  table->appendRow({stats_.getMaxDuration(), table->getRow(table->getNumRows() - 1u).at(1u)});

  // Create the plot and set the options.
  auto plot = std::make_shared<pgftikz::PgfPlot>(table);
  plot->options.mark = "\"none\""s;
  plot->options.lineWidth = config_->get<double>("report/successPlots/confidenceIntervalLineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "SuccessLowerConfidence"s;
  plot->options.drawOpacity =
      config_->get<float>("report/successPlots/confidenceIntervalDrawOpacity");
  plot->options.fillOpacity =
      config_->get<float>("report/successPlots/confidenceIntervalFillOpacity");

  return plot;
}

std::shared_ptr<pgftikz::PgfPlot> QuerySuccessVsTimeLinePlotter::createSuccessFillCiPlot(
    const std::string& plannerName) const {
  // Fill the areas between the upper and lower bound.
  auto fillBetween = std::make_shared<pgftikz::PgfFillBetween>(
      plannerName + "SuccessUpperConfidence", plannerName + "SuccessLowerConfidence");

  // Create the plot.
  auto plot = std::make_shared<pgftikz::PgfPlot>(fillBetween);
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.fillOpacity =
      config_->get<float>("report/successPlots/confidenceIntervalFillOpacity");
  plot->options.drawOpacity = 0.0;

  return plot;
}

}  // namespace plotters

}  // namespace pdt
