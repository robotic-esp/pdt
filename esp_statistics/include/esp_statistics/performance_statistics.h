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

class PlannerData {
 public:
  // The safest option is a vector of pairs, so that durations and costs cannot get decoupled.
  using RunData = std::vector<std::pair<double, double>>;
  PlannerData() = default;
  ~PlannerData() = default;

  // Interpolate the runs.
  const std::vector<RunData>& getAllRunsAt(const std::vector<double>& durations) const;

  // Access to measured runs.
  void addMeasuredRun(const RunData& run);
  const RunData& getMeasuredRun(std::size_t i) const;
  void clearMeasuredRuns();
  std::size_t numMeasuredRuns() const;

 private:
  std::vector<RunData> measuredRuns_{};
  mutable std::vector<RunData> interpolatedRuns_{};
};

class PerformanceStatistics {
 public:
  PerformanceStatistics(const std::shared_ptr<Configuration>& config);
  ~PerformanceStatistics() = default;

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
  std::shared_ptr<Configuration> config_;
  const std::experimental::filesystem::path filename_;
  std::vector<std::string> plannerNames_{};


  // Can we afford loading all of this into memory? Let's see.
  std::map<std::string, PlannerData> data_{};
  double minCost_{std::numeric_limits<double>::infinity()};
  double maxCost_{std::numeric_limits<double>::lowest()};
  double maxNonInfCost_{std::numeric_limits<double>::lowest()};
  double minDuration_{std::numeric_limits<double>::infinity()};
  double maxDuration_{std::numeric_limits<double>::lowest()};
};

}  // namespace ompltools

}  // namespace esp
