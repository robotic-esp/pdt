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
#include <set>
#include <sstream>
#include <string>

#include "pdt/config/configuration.h"
#include "pdt/pgftikz/tikz_picture.h"
#include "pdt/plotters/latex_plotter.h"
#include "pdt/plotters/overview_plotter.h"
#include "pdt/plotters/query_cost_at_first_vs_time_at_first_scatter_plotter.h"
#include "pdt/plotters/query_median_cost_at_first_vs_median_time_at_first_point_plotter.h"
#include "pdt/plotters/query_median_cost_vs_time_line_plotter.h"
#include "pdt/plotters/query_percentile_cost_vs_time_line_plotter.h"
#include "pdt/plotters/query_success_vs_time_line_plotter.h"
#include "pdt/plotters/query_time_at_first_histogram_plotter.h"
#include "pdt/reports/base_report.h"
#include "pdt/statistics/planning_statistics.h"

namespace pdt {

namespace reports {

class SingleQueryReport : public BaseReport {
 public:
  SingleQueryReport(const std::shared_ptr<config::Configuration>& config,
                    const statistics::PlanningStatistics& stats);
  ~SingleQueryReport() = default;

  std::experimental::filesystem::path generateReport() override;

 private:
  std::stringstream overview() const;
  std::stringstream individualResults() const;

  // Plotters.
  plotters::LatexPlotter latexPlotter_;
  plotters::OverviewPlotter overviewPlotter_;
  plotters::QueryCostAtFirstVsTimeAtFirstScatterPlotter
      queryCostAtFirstVsTimeAtFirstScatterPlotter_;
  plotters::QueryMedianCostAtFirstVsMedianTimeAtFirstPointPlotter
      queryMedianCostAtFirstVsMedianTimeAtFirstPointPlotter_;
  plotters::QueryMedianCostVsTimeLinePlotter queryMedianCostVsTimeLinePlotter_;
  plotters::QueryPercentileCostVsTimeLinePlotter queryPercentileCostVsTimeLinePlotter_;
  plotters::QuerySuccessVsTimeLinePlotter querySuccessVsTimeLinePlotter_;
  plotters::QueryTimeAtFirstHistogramPlotter queryTimeAtFirstHistogramPlotter_;

  const statistics::PlanningStatistics& stats_;
};

}  // namespace reports

}  // namespace pdt
