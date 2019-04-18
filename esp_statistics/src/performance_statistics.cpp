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

#include "esp_statistics/performance_statistics.h"

#include <algorithm>
#include <fstream>
#include <iostream>

#include <ompl/util/Console.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Weffc++"
#include "csv/parser.hpp"
#pragma GCC diagnostic pop

#include "esp_statistics/linear_interpolator.h"

namespace esp {

namespace ompltools {

using namespace std::string_literals;

void PlannerData::addMeasuredRun(const PlannerData::RunData& run) {
  measuredRuns_.emplace_back(run);
}

const PlannerData::RunData& PlannerData::getMeasuredRun(std::size_t i) const {
  return measuredRuns_.at(i);
}

void PlannerData::clearMeasuredRuns() {
  measuredRuns_.clear();
}

std::size_t PlannerData::numMeasuredRuns() const {
  return measuredRuns_.size();
}

void PlannerData::addInterpolatedRun(const PlannerData::RunData& run) {
  interpolatedRuns_.emplace_back(run);
}

const PlannerData::RunData& PlannerData::getInterpolatedRun(std::size_t i) const {
  return interpolatedRuns_.at(i);
}

void PlannerData::clearInterpolatedRuns() {
  interpolatedRuns_.clear();
}

std::size_t PlannerData::numInterpolatedRuns() const {
  return interpolatedRuns_.size();
}

PerformanceStatistics::PerformanceStatistics(const std::experimental::filesystem::path& filename) :
    filename_(filename) {
  // Open the file.
  std::ifstream csvFile(filename_);
  if (csvFile.fail()) {
    throw std::runtime_error("Unable to open csv file for performance statistics.");
  }
  // Set up the parser.
  aria::csv::CsvParser parser(csvFile);

  // Parse the file in bulk for now. We could already extract some statistics here, e.g. fastest
  // initial solution time and associated cost for every planner.
  bool timeRow = true;
  std::string name{""};
  PlannerData::RunData run;
  for (auto& row : parser) {
    if (row.size() == 0) {
      throw std::runtime_error("Empty row.");
    }
    if (timeRow) {
      name = row.at(0);
      run.clear();
      if (std::find(plannerNames_.cbegin(), plannerNames_.cend(), name) == plannerNames_.cend()) {
        plannerNames_.emplace_back(name);
      }
    } else if (row.at(0) != name) {
      throw std::runtime_error("Csv file has unexpected structure.");
    }
    for (std::size_t i = 1u; i < row.size(); ++i) {
      if (timeRow) {
        run.emplace_back(std::stod(row.at(i)), std::numeric_limits<double>::signaling_NaN());
      } else {
        run.at(i - 1u).second = std::stod(row.at(i));
      }
    }
    if (!timeRow) {
      data_[name].addMeasuredRun(run);
    }
    timeRow = !timeRow;
  }
}

std::vector<std::string> PerformanceStatistics::getPlannerNames() const {
  return plannerNames_;
}

std::vector<double> PerformanceStatistics::getQuantileEvolution(
    const std::string& name, double quantile, const std::vector<double>& durations) const {
  if (name == "RRTConnect"s) {
    throw std::runtime_error("Non-anytime planners don't have a quantile evolution.");
  }
  if (durations.empty()) {
    throw std::runtime_error("Expected at least one duration.");
  }
  if (quantile != 0.5) {
    throw std::runtime_error("Not implemented for other quantile than median.");
  }
  std::vector<std::vector<double>> interpolatedRuns(durations.size());

  // Generate values (i.e., interpolate or extrapolate) for each run.
  const auto& plannerData = data_.at(name);
  for (std::size_t runIdx = 0u; runIdx < plannerData.numMeasuredRuns(); ++runIdx) {
    const auto& measuredRun = plannerData.getMeasuredRun(runIdx);
    // Create an interpolator for this run.
    LinearInterpolator<double, double> interpolant(measuredRun);

    // Create the interpolated data for this run (std::pair has lexicographical compare, so this
    // works).
    auto [minDuration, maxDuration] = (std::minmax_element(measuredRun.begin(), measuredRun.end()));

    for (std::size_t i = 0u; i < durations.size(); ++i) {
      auto duration = durations[i];
      auto& run = interpolatedRuns[i];
      if (duration < minDuration->first) {
        run.emplace_back(std::numeric_limits<double>::infinity());
      } else if (duration > maxDuration->first) {  // What to do here?
        OMPL_ERROR("Requested to extrapolate. Max duration: %d, queried duration: %d", *maxDuration,
                   duration);
        throw std::runtime_error("Fairness error.");
      } else {
        run.emplace_back(interpolant(duration));
      }
    }
  }

  // Get the quantiles.
  std::vector<double> quantiles;
  quantiles.reserve(durations.size());
  for (std::size_t i = 0u; i < durations.size(); ++i) {
    quantiles.emplace_back(getQuantile(&interpolatedRuns.at(i), quantile));
  }

  return quantiles;
}

std::pair<double, double> PerformanceStatistics::getInitialSolution(const std::string& name,
                                                                    double quantile) const {
  if (data_.find(name) == data_.end()) {
    auto msg = "Cannot find statistics for planner '"s + name + "'."s;
    throw std::runtime_error(msg);
  }
  std::vector<double> initialDurations{};
  initialDurations.reserve(data_.at(name).numMeasuredRuns());
  std::vector<double> initialCosts{};
  initialCosts.reserve(data_.at(name).numMeasuredRuns());

  const auto& plannerData = data_.at(name);
  for (std::size_t run = 0u; run < plannerData.numMeasuredRuns(); ++run) {
    // Get the durations and costs of this run.
    const auto& measuredRun = plannerData.getMeasuredRun(run);

    // Find the first cost that's less than infinity.
    double initialDuration = std::numeric_limits<double>::infinity();
    double initialCost = std::numeric_limits<double>::infinity();
    for (const auto& measurement : measuredRun) {
      if (measurement.second < std::numeric_limits<double>::infinity()) {
        initialDuration = measurement.first;
        initialCost = measurement.second;
        break;
      }
    }
    initialDurations.emplace_back(initialDuration);
    initialCosts.emplace_back(initialCost);
  }

  return {getQuantile(&initialDurations, quantile), getQuantile(&initialCosts, quantile)};
}

double PerformanceStatistics::getQuantile(std::vector<double>* values, double quantile) const {
  if (quantile != 0.5) {
    auto msg = "Quantile not implemented except for 0.5"s;
    throw std::runtime_error(msg);
  }
  auto numValues = values->size();
  if (numValues % 2 == 1) {
    auto median = values->begin() + ((numValues - 1u) / 2);
    std::nth_element(values->begin(), median, values->end());
    return *median;
  } else {
    auto lowerMedian = values->begin() + ((numValues - 1u) / 2);
    std::nth_element(values->begin(), lowerMedian, values->end());
    auto low = *lowerMedian;
    auto upperMedian = values->begin() + ((numValues + 1u) / 2);
    std::nth_element(values->begin(), upperMedian, values->end());
    auto up = *upperMedian;
    return (low + up) / 2.0;
  }
}

}  // namespace ompltools

}  // namespace esp
