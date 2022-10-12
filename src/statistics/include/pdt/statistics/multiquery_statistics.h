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
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "pdt/config/configuration.h"
#include "pdt/statistics/planning_statistics.h"

namespace pdt {

namespace statistics {

class MultiqueryStatistics {
 public:
  MultiqueryStatistics(const std::shared_ptr<config::Configuration>& config,
                       const std::vector<PlanningStatistics>& stats, bool forceComputation);
  ~MultiqueryStatistics() = default;

  std::experimental::filesystem::path extractMedianInitialSolutionPerQuery(
      const std::string& plannerName, const double confidence) const;
  std::experimental::filesystem::path extractMedianFinalSolutionPerQuery(
      const std::string& plannerName, const double confidence) const;

  std::experimental::filesystem::path extractMedianCumulativeInitialSolutionPerQuery(
      const std::string& plannerName, const double confidence) const;

  std::experimental::filesystem::path extractMedianCumulativeInitialCostPerQuery(
      const std::string& plannerName, const double confidence) const;
  std::experimental::filesystem::path extractMedianCumulativeFinalCostPerQuery(
      const std::string& plannerName, const double confidence) const;

  std::experimental::filesystem::path extractFinalSolutionPerQuery(
      const std::string& plannerName) const;

  std::experimental::filesystem::path extractSuccessPerQuery(const std::string& plannerName) const;

  std::size_t getNumQueries() const { return numQueries_; };

  double getMaxDuration() const;
  double getMaxNonInfInitialDuration() const;

  double getMaxCumulativeDuration() const;
  double getMaxNonInfCumulativeDuration() const;

  double getMaxCost() const;
  double getMaxNonInfCost() const;

  double getMaxCumulativeCost() const;
  double getMaxNonInfCumulativeCost() const;

  double getMaxCumulativeInitialCost() const;

  double getMinCumulativeInitialSolutionCost(const std::string& plannerName) const;
  double getMedianCumulativeInitialSolutionCost(const std::string& plannerName) const;
  double getMaxCumulativeInitialSolutionCost(const std::string& plannerName) const;

  double getMinCumulativeInitialSolutionDuration(const std::string& plannerName) const;
  double getMedianCumulativeInitialSolutionDuration(const std::string& plannerName) const;

  double getMaxCumulativeNonInfInitialSolutionDuration(const std::string& plannerName) const;
  double getMaxCumulativeInitialSolutionDuration(const std::string& plannerName) const;

  double getSuccessRate(const std::string& plannerName) const;

  double getMinCumulativeFinalCost(const std::string& plannerName) const;
  double getMedianCumulativeFinalCost(const std::string& plannerName) const;
  double getMaxCumulativeFinalCost(const std::string& plannerName) const;

  PlanningStatistics getQueryStatistics(const unsigned int i) const;

 private:
  void computeCumulativeMetricsForPlanner(const std::string& plannerName);
  void computeCumulativeFinalCost(const std::string& plannerName);

  std::shared_ptr<config::Configuration> config_;
  const std::experimental::filesystem::path statisticsDirectory_;

  // When this is false, all extractions check if the file they would creat exists. If it does, the
  // path to the file is returned instead of recomputing.
  bool forceComputation_{false};

  // All the single-query statistics
  std::vector<PlanningStatistics> stats_;

  // Planner specific min and max values.
  std::map<std::string, double> medianCumulativeInitialSolutionCosts_{};
  std::map<std::string, double> minCumulativeInitialSolutionCosts_{};
  std::map<std::string, double> maxCumulativeInitialSolutionCosts_{};

  std::map<std::string, double> medianCumulativeFinalCosts_{};
  std::map<std::string, double> minCumulativeFinalCosts_{};
  std::map<std::string, double> maxCumulativeFinalCosts_{};
  std::map<std::string, double> maxCumulativeNonInfCosts_{};

  std::map<std::string, double> medianCumulativeInitialSolutionDurations_{};
  std::map<std::string, double> minCumulativeInitialSolutionDurations_{};
  std::map<std::string, double> maxCumulativeInitialSolutionDurations_{};
  std::map<std::string, double> maxCumulativeNonInfInitialSolutionDurations_{};

  std::map<std::string, double> successRates_{};

  double maxCost_{std::numeric_limits<double>::lowest()};
  double maxNonInfCost_{std::numeric_limits<double>::lowest()};

  double maxDuration_{std::numeric_limits<double>::lowest()};
  double maxNonInfInitialDuration_{std::numeric_limits<double>::lowest()};

  double maxCumulativeDuration_{std::numeric_limits<double>::lowest()};
  double maxNonInfCumulativeDuration_{0.0};

  double maxCumulativeCost_{std::numeric_limits<double>::lowest()};
  double maxNonInfCumulativeCost_{0.0};

  std::size_t numQueries_{0u};
};

}  // namespace statistics

}  // namespace pdt
