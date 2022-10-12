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

// Authors: Valentin Hartmann

#pragma once

#include <experimental/filesystem>
#include <memory>
#include <set>
#include <sstream>
#include <string>

#include "pdt/config/configuration.h"
#include "pdt/pgftikz/tikz_picture.h"
#include "pdt/plotters/latex_plotter.h"
#include "pdt/plotters/median_cost_at_first_vs_query_line_plotter.h"
#include "pdt/plotters/median_cost_at_last_vs_query_line_plotter.h"
#include "pdt/plotters/median_summed_cost_at_time_vs_query_line_plotter.h"
#include "pdt/plotters/median_summed_time_at_first_vs_query_line_plotter.h"
#include "pdt/plotters/median_time_at_first_vs_query_line_plotter.h"
#include "pdt/plotters/success_at_time_vs_query_line_plotter.h"
#include "pdt/reports/base_report.h"
#include "pdt/statistics/multiquery_statistics.h"

namespace pdt {

namespace reports {

class MultiqueryReport : public BaseReport {
 public:
  MultiqueryReport(const std::shared_ptr<config::Configuration>& config,
                   const statistics::MultiqueryStatistics& stats);
  ~MultiqueryReport() = default;

  std::experimental::filesystem::path generateReport() override;

 private:
  std::stringstream overview() const;
  std::stringstream individualResults() const;

  // Plotters.
  plotters::MedianCostAtFirstVsQueryLinePlotter medianCostAtFirstVsQueryLinePlotter_;
  plotters::MedianCostAtLastVsQueryLinePlotter medianCostAtLastVsQueryLinePlotter_;
  plotters::MedianSummedCostAtTimeVsQueryLinePlotter medianSummedCostAtTimeVsQueryLinePlotter_;
  plotters::MedianSummedTimeAtFirstVsQueryLinePlotter medianSummedTimeAtFirstVsQueryLinePlotter_;
  plotters::MedianTimeAtFirstVsQueryLinePlotter medianTimeAtFirstVsQueryLinePlotter_;
  plotters::SuccessAtTimeVsQueryLinePlotter successAtTimeVsQueryLinePlotter_;

  const statistics::MultiqueryStatistics& stats_;
};

}  // namespace reports

}  // namespace pdt
