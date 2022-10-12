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

#pragma once

#include <experimental/filesystem>
#include <memory>
#include <string>

#include "pdt/config/configuration.h"
#include "pdt/pgftikz/pgf_axis.h"
#include "pdt/plotters/latex_plotter.h"
#include "pdt/statistics/planning_statistics.h"

namespace pdt {

namespace plotters {

class QuerySuccessVsTimeLinePlotter : public LatexPlotter {
 public:
  QuerySuccessVsTimeLinePlotter(const std::shared_ptr<const config::Configuration>& config,
                                const statistics::PlanningStatistics& stats);
  ~QuerySuccessVsTimeLinePlotter() = default;

  // Creates a pgf axis that holds the success percentage over time for all planners.
  std::shared_ptr<pgftikz::PgfAxis> createSuccessAxis() const;

  // Creates a pgf axis that holds the success percentage over time for the specified planner.
  std::shared_ptr<pgftikz::PgfAxis> createSuccessAxis(const std::string& plannerName) const;

  // Creates a tikz picture that contains the success axis of all planners.
  std::experimental::filesystem::path createSuccessPicture() const;

  // Creates a tikz picture that contains the success axis of the specified planner.
  std::experimental::filesystem::path createSuccessPicture(const std::string& plannerName) const;

 private:
  std::shared_ptr<pgftikz::PgfPlot> createSuccessPlot(const std::string& plannerName) const;
  std::shared_ptr<pgftikz::PgfPlot> createSuccessUpperCiPlot(const std::string& plannerName) const;
  std::shared_ptr<pgftikz::PgfPlot> createSuccessLowerCiPlot(const std::string& plannerName) const;
  std::shared_ptr<pgftikz::PgfPlot> createSuccessFillCiPlot(const std::string& plannerName) const;

  void setSuccessAxisOptions(std::shared_ptr<pgftikz::PgfAxis> axis) const;

  double maxDurationToBePlotted_{std::numeric_limits<double>::infinity()};
  double minDurationToBePlotted_{std::numeric_limits<double>::infinity()};

  const statistics::PlanningStatistics& stats_;
};

}  // namespace plotters

}  // namespace pdt
