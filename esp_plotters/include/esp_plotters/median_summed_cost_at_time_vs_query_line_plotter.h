/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Toronto
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
 *   * Neither the name of the University of Toronto nor the names of its
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

#include "esp_configuration/configuration.h"
#include "esp_plotters/latex_plotter.h"
#include "esp_statistics/multiquery_statistics.h"
#include "esp_tikz/pgf_axis.h"

namespace esp {

namespace ompltools {

class MedianSummedCostAtTimeVsQueryLinePlotter : public LatexPlotter {
 public:
  MedianSummedCostAtTimeVsQueryLinePlotter(const std::shared_ptr<const Configuration>& config, const MultiqueryStatistics& stats);
  ~MedianSummedCostAtTimeVsQueryLinePlotter() = default;

  // Creates a pgf axis that holds the median cumulative cost at either the first 
  // or the last solution (indicated by initial) at each query for all planners.
  std::shared_ptr<PgfAxis> createMedianCumulativeCostAxis(const bool initial) const;

  // Creates a pgf axis that holds the median cumulative cost at either the first
  // or the last solution (indicated by the parameter initial) for the specified planner.
  std::shared_ptr<PgfAxis> createMedianCumulativeCostAxis(const std::string& plannerName, const bool initial) const;

  // Creates a tikz picture that contains the median cumulative cost axis of all planners.
  std::experimental::filesystem::path createMedianCumulativeCostPicture(const bool initial) const;

  // Creates a tikz picture that contains the median cumulative cost axis of the specified planner.
  std::experimental::filesystem::path createMedianCumulativeCostPicture(const std::string& plannerName, const bool initial) const;

 private:
  std::shared_ptr<PgfPlot> createMedianCumulativeCostPlot(const std::string& plannerName, const bool initial) const;
  std::shared_ptr<PgfPlot> createMedianCumulativeCostUpperCiPlot(
      const std::string& plannerName, const bool initial) const;
  std::shared_ptr<PgfPlot> createMedianCumulativeCostLowerCiPlot(
      const std::string& plannerName, const bool initial) const;
  std::shared_ptr<PgfPlot> createMedianCumulativeCostFillCiPlot(
      const std::string& plannerName) const;

  void setMedianCumulativeCostAxisOptions(std::shared_ptr<PgfAxis> axis) const;

  const MultiqueryStatistics& stats_;
};

}  // namespace ompltools

}  // namespace esp