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
  Statistics(const std::shared_ptr<Configuration>& config);
  ~Statistics() = default;

  std::experimental::filesystem::path extractMedians(
      const std::string& plannerName, const std::vector<double>& binDurations = {}) const;

  std::experimental::filesystem::path extractMedianConfidenceIntervals(
      const std::string& plannerName, std::size_t confidence = 99u,
      const std::vector<double>& binDurations = {}) const;

  std::experimental::filesystem::path extractInitialSolutionDurationCdf(
      const std::string& plannerName) const;

  std::vector<std::string> getPlannerNames() const;
  std::size_t getNumRunsPerPlanner() const;
  double getMinCost() const;
  double getMaxCost() const;
  double getMaxNonInfCost() const;
  double getMinDuration() const;
  double getMaxDuration() const;

  std::vector<double> getNthCosts(const std::string& name, std::size_t n,
                                  const std::vector<double>& durations) const;
  std::vector<double> getInitialSolutionDurations(const std::string& name) const;
  std::vector<double> getInitialSolutionCosts(const std::string& name) const;
  double getNthInitialSolutionDuration(const std::string& name, std::size_t n) const;
  double getNthInitialSolutionCost(const std::string& name, std::size_t n) const;

 private:
  // The identifying header line that starts each file produced by this class.
  std::string createHeader(const std::string& statisticType, const std::string& plannerName) const;

  // Get the median confidence interval.
  struct ConfidenceInterval {
    std::size_t lower{0u}, upper{0u};
    float probability{0.0f};
  };
  ConfidenceInterval getMedianConfidenceInterval(std::size_t confidence) const;

  std::vector<double> getMedianCosts(const PlannerResults& results,
                                     const std::vector<double>& durations) const;
  std::vector<double> getNthCosts(const PlannerResults& results, std::size_t n,
                                  const std::vector<double>& durations) const;

  std::shared_ptr<Configuration> config_;
  const std::experimental::filesystem::path resultsPath_;
  const std::experimental::filesystem::path statisticsDirectory_;
  std::vector<std::string> plannerNames_{};

  // Default binning durations.
  std::vector<double> defaultBinDurations_{};

  // The number of runs per planner.
  std::size_t numRunsPerPlanner_{0u};

  // Can we afford loading all of this into memory? Let's see.
  std::map<std::string, PlannerResults> results_{};
  double minCost_{std::numeric_limits<double>::infinity()};
  double maxCost_{std::numeric_limits<double>::lowest()};
  double maxNonInfCost_{std::numeric_limits<double>::lowest()};
  double minDuration_{std::numeric_limits<double>::infinity()};
  double maxDuration_{std::numeric_limits<double>::lowest()};

  // Paths to the generated files in the form
  // planner -> statstype -> path, e.g. statisticsPaths_["SBITstar"]["median"] -> path.
  std::map<std::string, std::map<std::string, std::experimental::filesystem::path>>
      statisticsPaths_{};
};

}  // namespace ompltools

}  // namespace esp
