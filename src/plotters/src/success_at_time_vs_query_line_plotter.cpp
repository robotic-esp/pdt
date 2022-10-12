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

#include "pdt/plotters/success_at_time_vs_query_line_plotter.h"

#include "pdt/pgftikz/pgf_axis.h"
#include "pdt/pgftikz/pgf_fillbetween.h"
#include "pdt/pgftikz/pgf_plot.h"
#include "pdt/pgftikz/pgf_table.h"
#include "pdt/pgftikz/tikz_picture.h"

namespace pdt {

namespace plotters {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

SuccessAtTimeVsQueryLinePlotter::SuccessAtTimeVsQueryLinePlotter(
    const std::shared_ptr<const config::Configuration>& config,
    const statistics::MultiqueryStatistics& stats) :
    LatexPlotter(config),
    stats_(stats) {
  auto contextName = config_->get<std::string>("experiment/context");
}

std::shared_ptr<pgftikz::PgfAxis> SuccessAtTimeVsQueryLinePlotter::createSuccessRateQueryAxis(
    const unsigned int percentage) const {
  if (percentage != 100u && percentage != 75u && percentage != 50u && percentage != 25u) {
    throw std::runtime_error("Invalid percentage in success per query plotter.");
  }

  auto axis = std::make_shared<pgftikz::PgfAxis>();
  setSuccessRateQueryAxisOptions(axis);

  // Fill the axis with the success rate plots of all planners.
  for (const auto& name : config_->get<std::vector<std::string>>("experiment/planners")) {
    axis->addPlot(createSuccessRateQueryPercentPlot(name, percentage));
  }
  axis->options.name = "AllPlannersSuccessRateQueryAxis" + std::to_string(percentage);

  return axis;
}

std::shared_ptr<pgftikz::PgfAxis> SuccessAtTimeVsQueryLinePlotter::createSuccessRateQueryAxis(
    const std::string& plannerName, const unsigned int percentage) const {
  if (percentage != 100u && percentage != 75u && percentage != 50u && percentage != 25u) {
    throw std::runtime_error("Invalid percentage in success per query plotter.");
  }

  auto axis = std::make_shared<pgftikz::PgfAxis>();
  setSuccessRateQueryAxisOptions(axis);

  axis->addPlot(createSuccessRateQueryPercentPlot(plannerName, percentage));

  axis->options.name = plannerName + "SuccessRateQueryAxis" + std::to_string(percentage);

  return axis;
}

fs::path SuccessAtTimeVsQueryLinePlotter::createSuccessRateQueryPicture(
    const unsigned int percentage) const {
  // Create the picture and add the axis.
  pgftikz::TikzPicture picture(config_);
  auto axis = createSuccessRateQueryAxis(percentage);
  picture.addAxis(axis);

  // Generate the tikz file.
  auto picturePath =
      fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
      fs::path("tikz/all_success_rate_query_plot_" + std::to_string(percentage) + "_percent.tikz");
  picture.write(picturePath);
  return picturePath;
}

fs::path SuccessAtTimeVsQueryLinePlotter::createSuccessRateQueryPicture(
    const std::string& plannerName, const unsigned int percentage) const {
  // Create the picture and add the axis.
  pgftikz::TikzPicture picture(config_);
  auto axis = createSuccessRateQueryAxis(plannerName, percentage);
  picture.addAxis(axis);

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/"s + plannerName + "_success_rate_query_plot_" +
                              std::to_string(percentage) + "_percent.tikz"s);
  picture.write(picturePath);
  return picturePath;
}

void SuccessAtTimeVsQueryLinePlotter::setSuccessRateQueryAxisOptions(
    std::shared_ptr<pgftikz::PgfAxis> axis) const {
  axis->options.width = config_->get<std::string>("report/successRatePlots/axisWidth");
  axis->options.height = config_->get<std::string>("report/successRatePlots/axisHeight");
  axis->options.ymin = 0.;
  axis->options.ymax = 1.05;
  axis->options.ylog = false;
  axis->options.xminorgrids = config_->get<bool>("report/successRatePlots/xminorgrids");
  axis->options.xmajorgrids = config_->get<bool>("report/successRatePlots/xmajorgrids");
  axis->options.yminorgrids = config_->get<bool>("report/successRatePlots/yminorgrids");
  axis->options.ymajorgrids = config_->get<bool>("report/successRatePlots/ymajorgrids");
  axis->options.xlabel = "Query Number"s;
  axis->options.ylabel = "Success Rate [\\%]"s;
  axis->options.ylabelAbsolute = true;
  axis->options.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
}

std::shared_ptr<pgftikz::PgfPlot>
SuccessAtTimeVsQueryLinePlotter::createSuccessRateQueryPercentPlot(
    const std::string& plannerName, const unsigned int percentage) const {
  const auto percentString = std::to_string(percentage);

  // Get the table from the appropriate file.
  auto table = std::make_shared<pgftikz::PgfTable>(stats_.extractSuccessPerQuery(plannerName),
                                                   "query number",
                                                   "success rate at " + percentString + " percent");

  // Create the plot and set the options.
  auto plot = std::make_shared<pgftikz::PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth = config_->get<double>("report/successRatePlots/lineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "SuccessRatePerQuery"s + percentString;

  return plot;
}

}  // namespace plotters

}  // namespace pdt
