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

/*
Algorithm | t_init,min | t_init,med | t_init,max | c_init,min | c_init,med | c_init,max | c_fin,min
| c_fin,med | c_fin,max |
 */

#include "esp_tikz/kpi_table.h"

#include <fstream>
#include <iomanip>

namespace esp {

namespace ompltools {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

KpiTable::KpiTable(const std::shared_ptr<const Configuration>& config, const Statistics& stats) :
    config_(config),
    stats_(stats) {
  options.rowSep = "\\\\[0.5em]"s;
}

void KpiTable::addKpi(const std::string& plannerName, const std::string& plannerPlotName) {
  plannerNames_.push_back(plannerPlotName);
  auto initMinDuration = stats_.getMinInitialSolutionDuration(plannerName);
  auto initMedDuration = stats_.getMedianInitialSolutionDuration(plannerName);
  auto initMaxDuration = stats_.getMaxInitialSolutionDuration(plannerName);
  auto initMinCost = stats_.getMinInitialSolutionCost(plannerName);
  auto initMedCost = stats_.getMedianInitialSolutionCost(plannerName);
  auto initMaxCost = stats_.getMaxInitialSolutionCost(plannerName);
  if (config_->get<bool>("planner/"s + plannerName + "/isAnytime"s)) {
    auto finalMinCost = stats_.getMinFinalCost(plannerName);
    auto finalMedCost = stats_.getMedianFinalCost(plannerName);
    auto finalMaxCost = stats_.getMaxFinalCost(plannerName);
    appendRow({initMinDuration, initMedDuration, initMaxDuration, initMinCost, initMedCost,
               initMaxCost, finalMinCost, finalMedCost, finalMaxCost});
  } else {
    auto finalMinCost = initMinCost;
    auto finalMedCost = initMedCost;
    auto finalMaxCost = initMaxCost;
    appendRow({initMinDuration, initMedDuration, initMaxDuration, initMinCost, initMedCost,
               initMaxCost, finalMinCost, finalMedCost, finalMaxCost});
  }
}

std::string KpiTable::string() const {
  if (data_.empty() || data_.at(0u).empty()) {
    return {};
  }

  // Figure out which values are the best.
  std::map<std::size_t, std::pair<std::size_t, double>> best{};  // col, {row, val}.
  for (std::size_t col = 0u; col < data_.size(); ++col) {
    best[col] = {0u, std::numeric_limits<double>::infinity()};
    for (std::size_t row = 0u; row < data_.at(0u).size(); ++row) {
      if (data_.at(col).at(row) < best.at(col).second) {
        best.at(col) = {row, data_.at(col).at(row)};
      }
    }
  }

  std::stringstream stream;
  stream << std::fixed;
  stream << "{\\tiny\n";
  stream << "\\begin{tabularx}{\\textwidth}[c]{Xccccccccc}\\toprule\n";
  stream << "Planner " << options.colSep << " \\(t_\\mathrm{initial}^\\mathrm{min}\\) "
         << options.colSep << " \\(t_\\mathrm{initial}^\\mathrm{med}\\) " << options.colSep
         << " \\(t_\\mathrm{initial}^\\mathrm{max}\\) " << options.colSep
         << " \\(c_\\mathrm{initial}^\\mathrm{min}\\) " << options.colSep
         << " \\(c_\\mathrm{initial}^\\mathrm{med}\\) " << options.colSep
         << " \\(c_\\mathrm{initial}^\\mathrm{max}\\) " << options.colSep
         << " \\(c_\\mathrm{final}^\\mathrm{min}\\) " << options.colSep
         << " \\(c_\\mathrm{final}^\\mathrm{med}\\) " << options.colSep
         << " \\(c_\\mathrm{final}^\\mathrm{max}\\) " << options.rowSep << "\\midrule\n";
  for (std::size_t row = 0u; row < data_.at(0u).size(); ++row) {
    stream << plannerNames_.at(row) << options.colSep << ' ';
    for (std::size_t col = 0u; col < data_.size(); ++col) {
      if (col < 3) {
        stream << std::setprecision(5);
      } else {
        stream << std::setprecision(3);
      }
      if (data_.at(col).at(row) == std::numeric_limits<double>::infinity()) {
        stream << "\\infty ";
      } else {
        if (row == best.at(col).first) {
          stream << "\\bfseries ";
        }
        stream << data_.at(col).at(row) << ' ';
      }
      if (col != data_.size() - 1u) {
        stream << options.colSep << ' ';
      } else {
        stream << options.rowSep << '\n';
      }
    }
  }
  stream << "\\bottomrule\n\\end{tabularx}\n}";

  return stream.str();
}

}  // namespace ompltools

}  // namespace esp
