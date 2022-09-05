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

// Authors: Valentin Hartmann

#pragma once

#include <experimental/filesystem>
#include <memory>
#include <set>
#include <sstream>
#include <string>

#include "esp_configuration/configuration.h"
#include "esp_statistics/multiquery_statistics.h"
#include "esp_tikz/base_report.h"
#include "esp_tikz/median_cumulative_cost_plotter.h"
#include "esp_tikz/median_cumulative_duration_plotter.h"
#include "esp_tikz/median_initial_solution_query_plotter.h"
#include "esp_tikz/median_initial_solution_cost_query_plotter.h"
#include "esp_tikz/median_final_solution_cost_query_plotter.h"
#include "esp_tikz/success_rate_per_query_plotter.h"
#include "esp_tikz/latex_plotter.h"
#include "esp_tikz/tikz_picture.h"

namespace esp {

namespace ompltools {

class MultiqueryReport : public BaseReport{
 public:
  MultiqueryReport(const std::shared_ptr<Configuration>& config, const MultiqueryStatistics& stats);
  ~MultiqueryReport() = default;

  std::experimental::filesystem::path generateReport() override;

 private:
  std::stringstream overview() const;
  std::stringstream individualResults() const;

  // Plotters.
  MedianCumulativeCostPlotter medianCumulativeCostPlotter_;
  MedianCumulativeDurationPlotter medianCumulativeDurationPlotter_;
  MedianInitialSolutionQueryPlotter medianInitialDurationQueryPlotter_;
  MedianInitialSolutionCostQueryPlotter medianInitialCostQueryPlotter_;
  MedianFinalSolutionCostQueryPlotter medianFinalCostQueryPlotter_;
  SuccessRateQueryPlotter successRateQueryPlotter_;

  const MultiqueryStatistics& stats_;
};

}  // namespace ompltools

}  // namespace esp
