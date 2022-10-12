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

/*
Algorithm | t_cum, init,min | t_cum,init,med | t_cum,init,max | c_cum,init,min | c_cum,init,med |
c_cum,init,max | c_cum,fin,min | c_cum,fin,med | c_cum,fin,max | success|
 */

#include "pdt/pgftikz/mq_kpi_table.h"
#include "pdt/statistics/multiquery_statistics.h"

#include <fstream>
#include <iomanip>

namespace pdt {

namespace pgftikz {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

MqKpiTable::MqKpiTable(const std::shared_ptr<const config::Configuration>& config,
                       const statistics::MultiqueryStatistics& stats) :
    config_(config),
    stats_(stats) {
  options.rowSep = "\\\\[0.5em]"s;
}

void MqKpiTable::addKpi(const std::string& plannerName, const std::string& plannerPlotName) {
  plannerNames_.push_back(plannerPlotName);
  auto cumInitMinCost = stats_.getMinCumulativeInitialSolutionCost(plannerName);
  auto cumInitMedCost = stats_.getMedianCumulativeInitialSolutionCost(plannerName);
  auto cumInitMaxCost = stats_.getMaxCumulativeInitialSolutionCost(plannerName);
  auto cumInitMinDuration = stats_.getMinCumulativeInitialSolutionDuration(plannerName);
  auto cumInitMedDuration = stats_.getMedianCumulativeInitialSolutionDuration(plannerName);
  auto cumInitMaxDuration = std::numeric_limits<double>::infinity();
  if (std::isfinite(cumInitMaxCost)) {
    cumInitMaxDuration = stats_.getMaxCumulativeInitialSolutionDuration(plannerName);
  }
  auto success = stats_.getSuccessRate(plannerName);

  if (config_->get<bool>("planner/"s + plannerName + "/isAnytime"s)) {
    auto cumFinalMinCost = stats_.getMinCumulativeFinalCost(plannerName);
    auto cumFinalMedCost = stats_.getMedianCumulativeFinalCost(plannerName);
    auto cumFinalMaxCost = stats_.getMaxCumulativeFinalCost(plannerName);
    appendRow({cumInitMinDuration, cumInitMedDuration, cumInitMaxDuration, cumInitMinCost,
               cumInitMedCost, cumInitMaxCost, cumFinalMinCost, cumFinalMedCost, cumFinalMaxCost,
               success});
  } else {
    auto cumFinalMinCost = cumInitMinCost;
    auto cumFinalMedCost = cumInitMedCost;
    auto cumFinalMaxCost = cumInitMaxCost;
    appendRow({cumInitMinDuration, cumInitMedDuration, cumInitMaxDuration, cumInitMinCost,
               cumInitMedCost, cumInitMaxCost, cumFinalMinCost, cumFinalMedCost, cumFinalMaxCost,
               success});
  }
}

std::string MqKpiTable::string() const {
  if (data_.empty() || data_.at(0u).empty()) {
    return {};
  }

  // Figure out which values are the best to maek them bold in the report
  std::map<std::size_t, std::pair<std::size_t, double>> best{};  // col, {row, val}.
  for (std::size_t col = 0u; col < data_.size(); ++col) {
    if (col != data_.size() - 1u) {
      best[col] = {0u, std::numeric_limits<double>::infinity()};
      for (std::size_t row = 0u; row < data_.at(0u).size(); ++row) {
        if (data_.at(col).at(row) < best.at(col).second) {
          best.at(col) = {row, data_.at(col).at(row)};
        }
      }
    } else {
      best[col] = {0u, 0.0};
      for (std::size_t row = 0u; row < data_.at(0u).size(); ++row) {
        if (data_.at(col).at(row) > best.at(col).second) {
          best.at(col) = {row, data_.at(col).at(row)};
        }
      }
    }
  }

  std::stringstream stream;
  stream << std::fixed;
  stream << "\\begin{table}[!h]\n";
  stream << "\\caption{Summary of cumulative solution durations and costs, and average planner "
            "success.}";
  stream << "{\\tiny\n";
  stream << "\\setlength{\\tabcolsep}{0.8em}\n";
  stream << "\\begin{tabularx}{\\textwidth}[c]{Xcccccccccc}\\toprule\n";
  stream << "Planner " << options.colSep << " \\(t_{\\Sigma\\mathrm{init}}^\\mathrm{min}\\) "
         << options.colSep << " \\(t_{\\Sigma\\mathrm{init}}^\\mathrm{med}\\) " << options.colSep
         << " \\(t_{\\Sigma\\mathrm{init}}^\\mathrm{max}\\) " << options.colSep
         << " \\(c_{\\Sigma\\mathrm{init}}^\\mathrm{min}\\) " << options.colSep
         << " \\(c_{\\Sigma\\mathrm{init}}^\\mathrm{med}\\) " << options.colSep
         << " \\(c_{\\Sigma\\mathrm{init}}^\\mathrm{max}\\) " << options.colSep
         << " \\(c_{\\Sigma\\mathrm{final}}^\\mathrm{min}\\) " << options.colSep
         << " \\(c_{\\Sigma\\mathrm{final}}^\\mathrm{med}\\) " << options.colSep
         << " \\(c_{\\Sigma\\mathrm{final}}^\\mathrm{max}\\) " << options.colSep
         << "Avg.\\newline Succ." << options.rowSep << "\\midrule\n";
  for (std::size_t row = 0u; row < data_.at(0u).size(); ++row) {
    stream << plannerNames_.at(row) << options.colSep << ' ';
    for (std::size_t col = 0u; col < data_.size(); ++col) {
      if (col == data_.size() - 1u) {
        stream << std::setprecision(2);
      } else {
        stream << std::setprecision(3);
      }

      if (data_.at(col).at(row) == std::numeric_limits<double>::infinity()) {
        stream << "\\infty ";
      } else {
        if (data_.at(col).at(row) == best.at(col).second) {
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
  stream << "\\end{table}";

  return stream.str();
}

}  // namespace pgftikz

}  // namespace pdt
