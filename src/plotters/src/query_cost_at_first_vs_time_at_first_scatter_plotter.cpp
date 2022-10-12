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

#include "pdt/plotters/query_cost_at_first_vs_time_at_first_scatter_plotter.h"

#include <stdlib.h>
#include <algorithm>
#include <fstream>

#include <ompl/util/Console.h>

#include "pdt/pgftikz/pgf_axis.h"
#include "pdt/pgftikz/pgf_fillbetween.h"
#include "pdt/pgftikz/pgf_plot.h"
#include "pdt/pgftikz/pgf_table.h"

namespace pdt {

namespace plotters {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

QueryCostAtFirstVsTimeAtFirstScatterPlotter::QueryCostAtFirstVsTimeAtFirstScatterPlotter(
    const std::shared_ptr<const config::Configuration>& config,
    const statistics::PlanningStatistics& stats) :
    LatexPlotter(config),
    stats_(stats) {
}

std::shared_ptr<pgftikz::PgfAxis>
QueryCostAtFirstVsTimeAtFirstScatterPlotter::createInitialSolutionScatterAxis() const {
  auto axis = std::make_shared<pgftikz::PgfAxis>();
  setInitialSolutionScatterAxisOptions(axis);

  for (const auto& name : config_->get<std::vector<std::string>>("experiment/planners")) {
    auto plot = createInitialSolutionScatterPlot(name);
    plot->options.fillOpacity =
        config_->get<float>("report/initialSolutionScatterPlots/combinedFillOpacity");
    plot->options.lineWidth = config_->get<double>("report/initialSolutionScatterPlots/lineWidth");
    axis->addPlot(plot);
  }

  return axis;
}

std::shared_ptr<pgftikz::PgfAxis>
QueryCostAtFirstVsTimeAtFirstScatterPlotter::createInitialSolutionScatterAxis(
    const std::string& plannerName) const {
  auto axis = std::make_shared<pgftikz::PgfAxis>();
  setInitialSolutionScatterAxisOptions(axis);
  axis->options.name = plannerName + "InitialSolutionScatterAxis"s;
  axis->addPlot(createInitialSolutionScatterPlot(plannerName));
  return axis;
}

fs::path QueryCostAtFirstVsTimeAtFirstScatterPlotter::createInitialSolutionScatterPicture() const {
  // Create the picture and add the axis.
  pgftikz::TikzPicture picture(config_);
  picture.addAxis(createInitialSolutionScatterAxis());

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/all_planners_initial_solution_histogram_plot.tikz");
  picture.write(picturePath);
  return picturePath;
}

fs::path QueryCostAtFirstVsTimeAtFirstScatterPlotter::createInitialSolutionScatterPicture(
    const std::string& plannerName) const {
  // Create the picture and add the axis.
  pgftikz::TikzPicture picture(config_);
  picture.addAxis(createInitialSolutionScatterAxis(plannerName));

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/"s + plannerName + "_initial_solution_histogram_plot.tikz"s);
  picture.write(picturePath);
  return picturePath;
}

std::shared_ptr<pgftikz::PgfPlot>
QueryCostAtFirstVsTimeAtFirstScatterPlotter::createInitialSolutionScatterPlot(
    const std::string& plannerName) const {
  // Load the data into a pgf table.
  auto table = std::make_shared<pgftikz::PgfTable>(stats_.extractInitialSolutions(plannerName),
                                                   "durations", "costs");

  // This table should not clean its data (or should it?).
  table->setCleanData(false);

  // Create the plot.
  auto plot = std::make_shared<pgftikz::PgfPlot>(table);
  plot->options.markSize = config_->get<double>("report/initialSolutionScatterPlots/markSize");
  plot->options.mark = "x";
  plot->options.lineWidth = 0.1;
  plot->options.constPlot = false;
  plot->options.onlyMarks = true;
  plot->options.namePath = plannerName + "InitialSolutionScatterPlotlineWidth"s;
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);

  return plot;
}

void QueryCostAtFirstVsTimeAtFirstScatterPlotter::setInitialSolutionScatterAxisOptions(
    std::shared_ptr<pgftikz::PgfAxis> axis) const {
  axis->options.height = config_->get<std::string>("report/initialSolutionScatterPlots/axisHeight");
  axis->options.width = config_->get<std::string>("report/initialSolutionScatterPlots/axisWidth");
  axis->options.name = "InitialSolutionScatterAxis"s;
  axis->options.xlog = config_->get<bool>("report/initialSolutionScatterPlots/xlog");
  axis->options.xminorgrids = config_->get<bool>("report/initialSolutionScatterPlots/xminorgrids");
  axis->options.xmajorgrids = config_->get<bool>("report/initialSolutionScatterPlots/xmajorgrids");
  axis->options.yminorgrids = config_->get<bool>("report/initialSolutionScatterPlots/yminorgrids");
  axis->options.ymajorgrids = config_->get<bool>("report/initialSolutionScatterPlots/ymajorgrids");
  axis->options.xlabel = "Computation time [s]";
  axis->options.ylabel = "Cost";
  axis->options.ylabelAbsolute = true;
  axis->options.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
}

}  // namespace plotters

}  // namespace pdt
