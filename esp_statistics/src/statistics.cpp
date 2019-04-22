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

#include "esp_statistics/statistics.h"

#include <algorithm>
#include <cassert>
#include <cmath>
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

const std::vector<PlannerResults::PlannerResult>& PlannerResults::getAllRunsAt(
    const std::vector<double>& durations) const {
  // Check if we have to generate the costs at these durations or if we have computed them before.
  bool cached = true;
  for (const auto& interpolatedRun : interpolatedRuns_) {
    if (!std::equal(durations.begin(), durations.end(), interpolatedRun.begin(),
                    interpolatedRun.end(),
                    [](const auto& a, const auto& b) { return a == b.first; })) {
      cached = false;
    }
  }
  if (cached && !interpolatedRuns_.empty()) {
    return interpolatedRuns_;
  }

  // We have to generate new values. Clear the vector in case we comuted different values before.
  interpolatedRuns_.clear();

  // Generate values (i.e., interpolate or make infinity) for each run.
  for (const auto& measuredRun : measuredRuns_) {
    // Each measured run gets an associated interpolated run.
    interpolatedRuns_.emplace_back();
    interpolatedRuns_.back().reserve(durations.size());

    // Create an interpolant for this run.
    LinearInterpolator<double, double> interpolant(measuredRun);

    // Get the min and max elements of this run to detect extrapolation.
    // This gets the min and max wrt the durations as std::pair uses a lexicographical comparator.
    auto [min, max] = std::minmax_element(measuredRun.begin(), measuredRun.end());

    // Compute the costs for each requested duration.
    for (const auto duration : durations) {
      if (duration < min->first) {
        interpolatedRuns_.back().emplace_back(duration, std::numeric_limits<double>::infinity());
      } else if (duration > max->first) {
        OMPL_ERROR("Requested to extrapolate. Max duration: %d, queried duration: %d", max->first,
                   duration);
        throw std::runtime_error("Fairness error.");
      } else {
        interpolatedRuns_.back().emplace_back(duration, interpolant(duration));
      }
    }
  }

  return interpolatedRuns_;
}

void PlannerResults::addMeasuredRun(const PlannerResults::PlannerResult& run) {
  measuredRuns_.emplace_back(run);
}

const PlannerResults::PlannerResult& PlannerResults::getMeasuredRun(std::size_t i) const {
  return measuredRuns_.at(i);
}

void PlannerResults::clearMeasuredRuns() {
  measuredRuns_.clear();
}

std::size_t PlannerResults::numMeasuredRuns() const {
  return measuredRuns_.size();
}

Statistics::Statistics(const std::shared_ptr<Configuration>& config) :
  config_(config),
  resultsPath_(config_->get<std::string>("Experiment/results")) {
  // Open the file.
  std::ifstream csvFile(resultsPath_.string());
  if (csvFile.fail()) {
    auto msg = "Statistics cannot open results at '"s + resultsPath_.string() + "'."s;
    throw std::runtime_error(msg);
  }

  // Set up the parser.
  aria::csv::CsvParser parser(csvFile);

  // Parse the file in bulk for now. We could already extract some statistics here, e.g. fastest
  // initial solution time and associated cost for every planner.
  bool timeRow = true;
  std::string name{""};
  PlannerResults::PlannerResult run;
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
        if (std::stod(row.at(i)) < minDuration_) {
          minDuration_ = std::stod(row.at(i));
        }
        if (std::stod(row.at(i)) > maxDuration_) {
          maxDuration_ = std::stod(row.at(i));
        }
      } else {
        run.at(i - 1u).second = std::stod(row.at(i));
        if (std::stod(row.at(i)) < minCost_) {
          minCost_ = std::stod(row.at(i));
        }
        if (std::stod(row.at(i)) > maxCost_) {
          maxCost_ = std::stod(row.at(i));
        }
        if (std::stod(row.at(i)) != std::numeric_limits<double>::infinity() &&
            std::stod(row.at(i)) > maxNonInfCost_) {
          maxNonInfCost_ = std::stod(row.at(i));
        }
      }
    }
    if (!timeRow) {
      data_[name].addMeasuredRun(run);
    }
    timeRow = !timeRow;
  }
}

std::vector<std::string> Statistics::getPlannerNames() const {
  return plannerNames_;
}

std::size_t Statistics::getNumRunsPerPlanner() const {
  if (data_.empty()) {
    return 0u;
  }
  // Make sure all planners have the same amount of runs.
  for (auto it = ++data_.begin(); it != data_.end(); ++it) {
    if ((--it)->second.numMeasuredRuns() != (++it)->second.numMeasuredRuns()) {
      auto msg = "Not all planners have the same amount of runs."s;
      throw std::runtime_error(msg);
    }
  }
  return data_.begin()->second.numMeasuredRuns();
}

double Statistics::getMinCost() const {
  return minCost_;
}

double Statistics::getMaxCost() const {
  return maxCost_;
}

double Statistics::getMaxNonInfCost() const {
  return maxNonInfCost_;
}

double Statistics::getMinDuration() const {
  return minDuration_;
}

double Statistics::getMaxDuration() const {
  return maxDuration_;
}

std::vector<double> Statistics::getNthCosts(const std::string& name, std::size_t n,
                                                       const std::vector<double>& durations) const {
  if (name == "RRTConnect"s) {
    auto msg = "Cannot specify durations for planners with nonanytime behaviour."s;
    throw std::runtime_error(msg);
  }
  if (durations.empty()) {
    auto msg = "Expected at least one duration."s;
    throw std::runtime_error(msg);
  }
  std::vector<double> nthCosts;
  nthCosts.reserve(durations.size());
  const auto& interpolatedRuns = data_.at(name).getAllRunsAt(durations);
  for (std::size_t durationIndex = 0u; durationIndex < durations.size(); ++durationIndex) {
    std::vector<double> costs;
    costs.reserve(interpolatedRuns.size());
    for (const auto& run : interpolatedRuns) {
      assert(run.at(durationIndex).first == durations.at(durationIndex));
      costs.emplace_back(run.at(durationIndex).second);
    }
    if (n > costs.size()) {
      auto msg = "Cannot get "s + std::to_string(n) + "th cost, there are only "s +
                 std::to_string(costs.size()) + " costs at this time."s;
      throw std::runtime_error(msg);
    }
    auto nthCost = costs.begin() + n;
    std::nth_element(costs.begin(), nthCost, costs.end());
    nthCosts.emplace_back(*nthCost);
  }
  return nthCosts;
}

std::vector<double> Statistics::getInitialSolutionDurations(
    const std::string& name) const {
  // Get the durations of the initial solutions of all runs.
  std::vector<double> initialDurations{};
  initialDurations.reserve(data_.at(name).numMeasuredRuns());
  const auto& plannerData = data_.at(name);
  for (std::size_t run = 0u; run < plannerData.numMeasuredRuns(); ++run) {
    // Get the durations and costs of this run.
    const auto& measuredRun = plannerData.getMeasuredRun(run);

    // Find the first cost that's less than infinity.
    for (const auto& measurement : measuredRun) {
      if (measurement.second < std::numeric_limits<double>::infinity()) {
        initialDurations.emplace_back(measurement.first);
        break;
      }
    }
  }

  return initialDurations;
}

std::vector<double> Statistics::getInitialSolutionCosts(const std::string& name) const {
  if (data_.find(name) == data_.end()) {
    auto msg = "Cannot find statistics for planner '"s + name + "'."s;
    throw std::runtime_error(msg);
  }

  // Get the costs of the initial solutions of all runs.
  std::vector<double> initialCosts{};
  initialCosts.reserve(data_.at(name).numMeasuredRuns());
  const auto& plannerData = data_.at(name);
  for (std::size_t run = 0u; run < plannerData.numMeasuredRuns(); ++run) {
    // Get the durations and costs of this run.
    const auto& measuredRun = plannerData.getMeasuredRun(run);

    // Find the first cost that's less than infinity.
    for (const auto& measurement : measuredRun) {
      if (measurement.second < std::numeric_limits<double>::infinity()) {
        initialCosts.emplace_back(measurement.second);
        break;
      }
    }
  }

  return initialCosts;
}

double Statistics::getNthInitialSolutionDuration(const std::string& name,
                                                            std::size_t n) const {
  if (data_.find(name) == data_.end()) {
    auto msg = "Cannot find statistics for planner '"s + name + "'."s;
    throw std::runtime_error(msg);
  }

  // Get the durations of the initial solutions of all runs.
  auto initialDurations = getInitialSolutionDurations(name);

  // Get the nth element of this collection of durations.
  auto nthDuration = initialDurations.begin() + n;
  std::nth_element(initialDurations.begin(), nthDuration, initialDurations.end());

  return *nthDuration;
}

double Statistics::getNthInitialSolutionCost(const std::string& name,
                                                        std::size_t n) const {
  if (data_.find(name) == data_.end()) {
    auto msg = "Cannot find statistics for planner '"s + name + "'."s;
    throw std::runtime_error(msg);
  }

  // Get the costs of the initial solutions of all runs.
  auto initialCosts = getInitialSolutionCosts(name);

  // Get the nth element of this collection of costs.
  auto nthCost = initialCosts.begin() + n;
  std::nth_element(initialCosts.begin(), nthCost, initialCosts.end());

  return *nthCost;
}

}  // namespace ompltools

}  // namespace esp
