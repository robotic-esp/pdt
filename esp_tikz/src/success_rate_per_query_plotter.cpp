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

#include "esp_tikz/success_rate_per_query_plotter.h"

#include "esp_tikz/pgf_axis.h"
#include "esp_tikz/pgf_fillbetween.h"
#include "esp_tikz/pgf_plot.h"
#include "esp_tikz/pgf_table.h"
#include "esp_tikz/tikz_picture.h"

namespace esp {

namespace ompltools {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

SuccessRateQueryPlotter::SuccessRateQueryPlotter(
    const std::shared_ptr<const Configuration>& config, const MultiQueryStatistics& stats) :
    LatexPlotter(config),
    stats_(stats) {
  // Compute the duration bin size.
  auto contextName = config_->get<std::string>("experiment/context");
}

std::shared_ptr<PgfAxis> SuccessRateQueryPlotter::createSuccessRateQueryAxis(
    const unsigned int percentage) const {
  if (percentage != 100 && percentage != 75 && percentage != 50 && percentage != 25){
    throw std::runtime_error("Invalid percentage in success per query plotter.");
  }

  auto axis = std::make_shared<PgfAxis>();
  setSuccessRateQueryAxisOptions(axis);

  // Fill the axis with the median cost plots of all planners.
  for (const auto& name : config_->get<std::vector<std::string>>("experiment/planners")) {
    std::shared_ptr<PgfPlot> plt;

    switch(percentage){
      case 25: plt = createSuccessRateQuery25PercentPlot(name); break;
      case 50: plt = createSuccessRateQuery50PercentPlot(name); break;
      case 75: plt = createSuccessRateQuery75PercentPlot(name); break;
      case 100: plt = createSuccessRateQuery100PercentPlot(name); break;
    }

    axis->addPlot(plt);
  }
  axis->options.name = "AllPlannersSuccessRateQueryAxis" + std::to_string(percentage);

  return axis;
}

std::shared_ptr<PgfAxis> SuccessRateQueryPlotter::createSuccessRateQueryAxis(
    const std::string& plannerName, const unsigned int percentage) const {
  if (percentage != 100 && percentage != 75 && percentage != 50 && percentage != 25){
    throw std::runtime_error("Invalid percentage in success per query plotter.");
  }

  auto axis = std::make_shared<PgfAxis>();
  setSuccessRateQueryAxisOptions(axis);

  std::shared_ptr<PgfPlot> plt;

  switch(percentage){
    case 25: plt = createSuccessRateQuery25PercentPlot(plannerName); break;
    case 50: plt = createSuccessRateQuery50PercentPlot(plannerName); break;
    case 75: plt = createSuccessRateQuery75PercentPlot(plannerName); break;
    case 100: plt = createSuccessRateQuery100PercentPlot(plannerName); break;
  }

  axis->addPlot(plt);

  axis->options.name = plannerName + "SuccessRateQueryAxis" + std::to_string(percentage);

  return axis;
}

fs::path SuccessRateQueryPlotter::createSuccessRateQueryPicture() const {
  // Create the picture and add the axis.
  TikzPicture picture(config_);
  auto axis = createSuccessRateQueryAxis();
  picture.addAxis(axis);

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/all_success_rate_query_plot.tikz");
  picture.write(picturePath);
  return picturePath;
}

fs::path SuccessRateQueryPlotter::createSuccessRateQueryPicture(
    const std::string& plannerName) const {
  // Create the picture and add the axis.
  TikzPicture picture(config_);
  auto axis = createSuccessRateQueryAxis(plannerName);
  picture.addAxis(axis);

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                     fs::path("tikz/"s + plannerName + "_success_rate_query_plot.tikz"s);
  picture.write(picturePath);
  return picturePath;
}

void SuccessRateQueryPlotter::setSuccessRateQueryAxisOptions(std::shared_ptr<PgfAxis> axis) const {
  axis->options.width = config_->get<std::string>("successRatePlots/axisWidth");
  axis->options.height = config_->get<std::string>("successRatePlots/axisHeight");
  axis->options.ymin = 0.;
  axis->options.ymax = 1.05;
  axis->options.ylog = false;
  axis->options.xminorgrids = config_->get<bool>("successRatePlots/xminorgrids");
  axis->options.xmajorgrids = config_->get<bool>("successRatePlots/xmajorgrids");
  axis->options.yminorgrids = config_->get<bool>("successRatePlots/yminorgrids");
  axis->options.ymajorgrids = config_->get<bool>("successRatePlots/ymajorgrids");
  axis->options.xlabel = "Query Number"s;
  axis->options.ylabel = "Success Rate [\\%]"s;
  axis->options.ylabelAbsolute = true;
  axis->options.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
}

std::shared_ptr<PgfPlot> SuccessRateQueryPlotter::createSuccessRateQuery25PercentPlot(
    const std::string& plannerName) const {

  // Get the table from the appropriate file.
  auto table =
      std::make_shared<PgfTable>(stats_.extractSuccessPerQuery(plannerName), "query number", "success rate at 25 percent");

  // Remove all nans from the table.
  //table->removeRowIfDomainIsNan();
  //table->removeRowIfCodomainIsNan();

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth = config_->get<double>("successRatePlots/lineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "SuccessRatePerQuery25"s;

  return plot;
}

std::shared_ptr<PgfPlot> SuccessRateQueryPlotter::createSuccessRateQuery50PercentPlot(
    const std::string& plannerName) const {

  // Get the table from the appropriate file.
  auto table =
      std::make_shared<PgfTable>(stats_.extractSuccessPerQuery(plannerName), "query number", "success rate at 50 percent");

  // Remove all nans from the table.
  //table->removeRowIfDomainIsNan();
  //table->removeRowIfCodomainIsNan();

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth = config_->get<double>("successRatePlots/lineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "SuccessRatePerQuery50"s;

  return plot;
}

std::shared_ptr<PgfPlot> SuccessRateQueryPlotter::createSuccessRateQuery75PercentPlot(
    const std::string& plannerName) const {

  // Get the table from the appropriate file.
  auto table =
      std::make_shared<PgfTable>(stats_.extractSuccessPerQuery(plannerName), "query number", "success rate at 75 percent");

  // Remove all nans from the table.
  //table->removeRowIfDomainIsNan();
  //table->removeRowIfCodomainIsNan();

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth = config_->get<double>("successRatePlots/lineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "SuccessRatePerQuery75"s;

  return plot;
}

std::shared_ptr<PgfPlot> SuccessRateQueryPlotter::createSuccessRateQuery100PercentPlot(
    const std::string& plannerName) const {

  // Get the table from the appropriate file.
  auto table =
      std::make_shared<PgfTable>(stats_.extractSuccessPerQuery(plannerName), "query number", "success rate at 100 percent");

  // Remove all nans from the table.
  //table->removeRowIfDomainIsNan();
  //table->removeRowIfCodomainIsNan();

  // Create the plot and set the options.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth = config_->get<double>("successRatePlots/lineWidth");
  plot->options.color = config_->get<std::string>("planner/"s + plannerName + "/report/color"s);
  plot->options.namePath = plannerName + "SuccessRatePerQuery100"s;

  return plot;
}

}  // namespace ompltools

}  // namespace esp
