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
#include <set>
#include <sstream>
#include <string>

#include "esp_configuration/configuration.h"
#include "esp_statistics/statistics.h"
#include "esp_tikz/cost_percentile_evolution_plotter.h"
#include "esp_tikz/initial_solution_duration_pdf_plotter.h"
#include "esp_tikz/initial_solution_scatter_plotter.h"
#include "esp_tikz/latex_plotter.h"
#include "esp_tikz/median_cost_evolution_plotter.h"
#include "esp_tikz/median_initial_solution_plotter.h"
#include "esp_tikz/overview_plotter.h"
#include "esp_tikz/success_plotter.h"
#include "esp_tikz/tikz_picture.h"

namespace esp {

namespace ompltools {

class ExperimentReport {
 public:
  ExperimentReport(const std::shared_ptr<Configuration>& config, const Statistics& stats);
  ~ExperimentReport() = default;

  std::experimental::filesystem::path generateReport();
  std::experimental::filesystem::path compileReport() const;

 private:
  std::stringstream preamble() const;
  std::stringstream overview() const;
  std::stringstream individualResults() const;
  std::stringstream appendix() const;

  const std::set<std::string> requirePackages_{"luatex85"};
  const std::set<std::string> usePackages_{"appendix", "booktabs", "caption",  "listings",
                                           "tabularx", "tikz",     "pgfplots", "xcolor"};
  const std::set<std::string> lstSet_{};
  const std::set<std::string> tikzLibraries_{"calc", "plotmarks"};
  const std::set<std::string> pgfLibraries_{"fillbetween"};
  const std::set<std::string> pgfPlotsset_{"compat=1.15"};

  std::string experimentName_{};
  std::map<std::string, std::string> plotPlannerNames_{};

  // Plotters.
  LatexPlotter latexPlotter_;
  CostPercentileEvolutionPlotter costPercentileEvolutionPlotter_;
  InitialSolutionDurationPdfPlotter initialSolutionDurationPdfPlotter_;
  InitialSolutionScatterPlotter initialSolutionScatterPlotter_;
  MedianCostEvolutionPlotter medianCostEvolutionPlotter_;
  MedianInitialSolutionPlotter medianInitialSolutionPlotter_;
  SuccessPlotter successPlotter_;
  OverviewPlotter overviewPlotter_;

  const std::shared_ptr<const Configuration> config_;
  const Statistics& stats_;

  // Helper to replace _ with \_, see [1].
  void findAndReplaceAll(std::string* string, const std::string& key,
                         const std::string& replacement) const;
};

}  // namespace ompltools

}  // namespace esp

// [1] https://thispointer.com/find-and-replace-all-occurrences-of-a-sub-string-in-c/
