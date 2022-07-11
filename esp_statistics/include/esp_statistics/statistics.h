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
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "esp_configuration/configuration.h"

namespace esp {

namespace ompltools {

class PlannerResults {
 public:
  // The safest option is a vector of pairs, so that durations and costs cannot get decoupled.
  using PlannerResult = std::vector<std::pair<double, double>>;
  PlannerResults() = default;
  ~PlannerResults() = default;

  // Interpolate the runs.
  const std::vector<PlannerResult>& getAllRunsAt(const std::vector<double>& durations) const;

  // Access to measured runs.
  void addMeasuredRun(const PlannerResult& run);
  const PlannerResult& getMeasuredRun(std::size_t i) const;
  void clearMeasuredRuns();
  std::size_t numMeasuredRuns() const;

 private:
  std::vector<PlannerResult> measuredRuns_{};
  mutable std::vector<PlannerResult> interpolatedRuns_{};
};

class Statistics {
 public:
  Statistics(const std::shared_ptr<Configuration>& config, const std::experimental::filesystem::path &resultsPath, bool forceComputation = false);
  ~Statistics() = default;

  std::experimental::filesystem::path extractMedians(
      const std::string& plannerName, std::size_t confidence = 99u,
      const std::vector<double>& binDurations = {}) const;

  std::experimental::filesystem::path extractCostPercentiles(
      const std::string& plannerName, std::set<double> percentile,
      const std::vector<double>& binDurations = {}) const;

  std::experimental::filesystem::path extractMedianInitialSolution(
      const std::string& plannerName, std::size_t confidence = 99u) const;

  std::experimental::filesystem::path extractInitialSolutionDurationCdf(
      const std::string& plannerName) const;

  std::experimental::filesystem::path extractInitialSolutionDurationPdf(
      const std::string& plannerName, const std::vector<double>& binDurations = {}) const;

  std::experimental::filesystem::path extractInitialSolutions(const std::string& plannerName) const;

  std::size_t getNumRunsPerPlanner() const;

  // Getters for overall min max values.
  double getMinCost() const;
  double getMaxCost() const;
  double getMaxNonInfCost() const;
  double getMinFinalCost() const;
  double getMaxFinalCost() const;
  double getMinDuration() const;
  double getMaxDuration() const;
  double getMinInitialSolutionDuration() const;
  double getMaxNonInfInitialSolutionDuration() const;

  // Getters for planner specific min max values.
  double getMinCost(const std::string& plannerName) const;
  double getMaxCost(const std::string& plannerName) const;
  double getMaxNonInfCost(const std::string& plannerName) const;
  double getMinInitialSolutionCost(const std::string& plannerName) const;
  double getMaxInitialSolutionCost(const std::string& plannerName) const;
  double getMinFinalCost(const std::string& plannerName) const;
  double getMedianFinalCost(const std::string& plannerName) const;
  double getMaxFinalCost(const std::string& plannerName) const;
  double getMinDuration(const std::string& plannerName) const;
  double getMaxDuration(const std::string& plannerName) const;
  double getMinInitialSolutionDuration(const std::string& plannerName) const;
  double getMaxInitialSolutionDuration(const std::string& plannerName) const;
  double getMaxNonInfInitialSolutionDuration(const std::string& plannerName) const;
  double getMedianInitialSolutionDuration(const std::string& plannerName) const;
  double getMedianInitialSolutionCost(const std::string& plannerName) const;
  double getSuccessRate(const std::string& plannerName) const;

  std::vector<double> getDefaultBinDurations() const;
  std::shared_ptr<Configuration> getConfig() const;

 private:
  // The identifying header line that starts each file produced by this class.
  std::string createHeader(const std::string& statisticType, const std::string& plannerName) const;

  // Get the median confidence interval.
  struct ConfidenceInterval {
    // The lower and upper bounds are zero-based indices. If I have 10 measurements in a sorted
    // vector, e.g.,
    //   v = { 0.1 0.3 0.4 0.5 0.8 0.9 1.1 1.3 1.7 1.9 },
    // then lower and upper bound indices of 1 and 8, respectively, mean that the median is with
    // <probability> certainty between v[1] and v[8].
    std::size_t lower{0u}, upper{0u};
    float probability{0.0f};
  };
  ConfidenceInterval getMedianConfidenceInterval(std::size_t confidence) const;

  std::vector<double> getMedianCosts(const PlannerResults& results,
                                     const std::vector<double>& durations) const;
  std::vector<double> getNthCosts(const PlannerResults& results, std::size_t n,
                                  const std::vector<double>& durations) const;

  double getMedianInitialSolutionDuration(const PlannerResults& results) const;
  double getMedianInitialSolutionCost(const PlannerResults& results) const;

  double getNthInitialSolutionDuration(const PlannerResults& results, std::size_t n) const;
  double getNthInitialSolutionCost(const PlannerResults& results, std::size_t n) const;

  std::vector<double> getInitialSolutionDurations(const PlannerResults& results) const;
  std::vector<double> getInitialSolutionCosts(const PlannerResults& results) const;

  std::shared_ptr<Configuration> config_;
  const std::experimental::filesystem::path statisticsDirectory_;

  // When this is false, all extractions check if the file they would creat exists. If it does, the
  // path to the file is returned instead of recomputing.
  bool forceComputation_{false};

  // Default binning durations.
  std::vector<double> defaultMedianBinDurations_{};
  std::vector<double> defaultInitialSolutionBinDurations_{};

  // The number of runs per planner.
  std::size_t numRunsPerPlanner_{0u};

  // Can we afford loading all of this into memory? Let's see.
  std::map<std::string, PlannerResults> results_{};

  // Planner specific min and max values.
  std::map<std::string, double> minCosts_{};
  std::map<std::string, double> maxCosts_{};
  std::map<std::string, double> minInitialSolutionCosts_{};
  std::map<std::string, double> maxInitialSolutionCosts_{};
  std::map<std::string, double> minFinalCosts_{};
  std::map<std::string, double> maxFinalCosts_{};
  std::map<std::string, double> maxNonInfCosts_{};
  std::map<std::string, double> minDurations_{};
  std::map<std::string, double> maxDurations_{};
  std::map<std::string, double> minInitialSolutionDurations_{};
  std::map<std::string, double> maxInitialSolutionDurations_{};
  std::map<std::string, double> maxNonInfInitialSolutionDurations_{};
  std::map<std::string, double> successRates_{};

  // Overall min and max values.
  double minCost_{std::numeric_limits<double>::infinity()};
  double maxCost_{std::numeric_limits<double>::lowest()};
  double minFinalCost_{std::numeric_limits<double>::infinity()};
  double maxFinalCost_{std::numeric_limits<double>::lowest()};
  double maxNonInfCost_{std::numeric_limits<double>::lowest()};
  double minDuration_{std::numeric_limits<double>::infinity()};
  double maxDuration_{std::numeric_limits<double>::lowest()};
  double minInitialSolutionDuration_{std::numeric_limits<double>::infinity()};
  double maxNonInfInitialSolutionDuration_{std::numeric_limits<double>::lowest()};
};

}  // namespace ompltools

}  // namespace esp
