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
#include <limits>
#include <memory>
#include <string>

#include "esp_config/configuration.h"
#include "esp_plotters/latex_plotter.h"
#include "esp_statistics/statistics.h"

namespace pdt {

namespace plotters {

class QueryTimeAtFirstHistogramPlotter : public LatexPlotter {
 public:
  QueryTimeAtFirstHistogramPlotter(const std::shared_ptr<const config::Configuration>& config,
                                          const statistics::Statistics& stats);
  ~QueryTimeAtFirstHistogramPlotter() = default;

  // Creates a pgf axis that hold the initial solution duration histogram of all planners.
  std::shared_ptr<pgftikz::PgfAxis> createInitialSolutionDurationHistogramAxis() const;

  // Creates a pgf axis that hold the initial solution duration histogram of the specified planner.
  std::shared_ptr<pgftikz::PgfAxis> createInitialSolutionDurationHistogramAxis(
      const std::string& plannerName) const;

  // Creates a tikz picture that contains the initial solution duration histogram axis of all
  // planners.
  std::experimental::filesystem::path createInitialSolutionDurationHistogramPicture() const;

  // Creates a tikz picture that contains the initial solution duration histogram axis of all
  // planners.
  std::experimental::filesystem::path createInitialSolutionDurationHistogramPicture(
      const std::string& plannerName) const;

 private:
  std::shared_ptr<pgftikz::PgfPlot> createInitialSolutionDurationHistogramPlot(
      const std::string& plannerName) const;

  void setInitialSolutionDurationHistogramAxisOptions(std::shared_ptr<pgftikz::PgfAxis> axis) const;

  mutable std::shared_ptr<pgftikz::PgfAxis> axis_;
  const statistics::Statistics& stats_;
};

}  // namespace plotters

}  // namespace pdt
