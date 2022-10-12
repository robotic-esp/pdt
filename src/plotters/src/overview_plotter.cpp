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

#include "pdt/plotters/overview_plotter.h"

#include "pdt/pgftikz/pgf_axis.h"
#include "pdt/pgftikz/pgf_fillbetween.h"
#include "pdt/pgftikz/pgf_plot.h"
#include "pdt/pgftikz/pgf_table.h"
#include "pdt/pgftikz/tikz_picture.h"
#include "pdt/plotters/query_median_cost_vs_time_line_plotter.h"
#include "pdt/plotters/query_success_vs_time_line_plotter.h"

namespace pdt {

namespace plotters {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

OverviewPlotter::OverviewPlotter(const std::shared_ptr<const config::Configuration>& config,
                                 const statistics::PlanningStatistics& stats) :
    LatexPlotter(config),
    stats_(stats) {
}

fs::path OverviewPlotter::createCombinedPicture() const {
  // Create the success axis and override some options.
  QuerySuccessVsTimeLinePlotter successPlotter(config_, stats_);
  auto successAxis = successPlotter.createSuccessAxis();
  successAxis->options.name = "AllPlannersCombinedSuccessAxis"s;
  successAxis->options.xlabel = "{\\empty}"s;
  successAxis->options.xticklabel = "{\\empty}"s;

  // Create the median cost axis and override some options.
  QueryMedianCostVsTimeLinePlotter medianCostPlotter(config_, stats_);
  auto medianCostAxis = medianCostPlotter.createMedianCostEvolutionAxis();
  medianCostAxis->options.at = "($(AllPlannersCombinedSuccessAxis.south) - (0.0em, 0.3em)$)";
  medianCostAxis->options.anchor = "north";
  medianCostAxis->options.name = "AllPlannersCombinedMedianCostAxis"s;

  // Make sure these axis cover the same domain.
  pgftikz::PgfAxis::alignAbszissen(medianCostAxis.get(), successAxis.get());

  // Create the picture and add the axes.
  pgftikz::TikzPicture picture(config_);
  picture.addAxis(successAxis);
  picture.addAxis(medianCostAxis);

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/all_planners_combined_success_median_cost_plot.tikz");
  picture.write(picturePath);
  return picturePath;
}

fs::path OverviewPlotter::createCombinedPicture(const std::string& plannerName) const {
  // Create the success axis and override some options.
  QuerySuccessVsTimeLinePlotter successPlotter(config_, stats_);
  auto successAxis = successPlotter.createSuccessAxis(plannerName);
  successAxis->options.name = plannerName + "CombinedSuccessAxis"s;
  successAxis->options.xlabel = "{\\empty}"s;
  successAxis->options.xticklabel = "{\\empty}"s;

  // Create the median cost axis and override some options.
  QueryMedianCostVsTimeLinePlotter medianCostPlotter(config_, stats_);
  auto medianCostAxis = medianCostPlotter.createMedianCostEvolutionAxis(plannerName);
  medianCostAxis->options.at =
      "($("s + plannerName + "CombinedSuccessAxis.south) - (0.0em, 0.3em)$)"s;
  medianCostAxis->options.anchor = "north";
  medianCostAxis->options.name = plannerName + "CombinedMedianCostAxis"s;

  // Create the picture and add the axes.
  pgftikz::TikzPicture picture(config_);
  picture.addAxis(successAxis);
  picture.addAxis(medianCostAxis);

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/"s + plannerName + "_combined_success_median_cost_plot.tikz"s);
  picture.write(picturePath);
  return picturePath;
}

}  // namespace plotters

}  // namespace pdt
