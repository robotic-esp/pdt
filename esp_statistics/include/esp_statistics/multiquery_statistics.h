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
#include <map>
#include <set>
#include <utility>
#include <vector>
#include <iomanip>
#include <iostream>

#include "esp_statistics/statistics.h"
#include "esp_configuration/configuration.h"

namespace esp {

namespace ompltools {

class MultiqueryStatistics {
 public:
  MultiqueryStatistics(const std::shared_ptr<Configuration>& config, const std::vector<Statistics> &stats,
      bool forceComputation = false);
  ~MultiqueryStatistics() = default;

  std::experimental::filesystem::path extractMedianInitialSolutionPerQuery(
      const std::string& plannerName, const double confidence = 0.99) const;
  std::experimental::filesystem::path extractMedianFinalSolutionPerQuery(
      const std::string& plannerName, const double confidence = 0.99) const;

  std::experimental::filesystem::path extractMedianCumulativeInitialSolutionPerQuery(
      const std::string& plannerName, const double confidence = 0.99) const;

  std::experimental::filesystem::path extractMedianCumulativeInitialCostPerQuery(
      const std::string& plannerName, const double confidence = 0.99) const;
  std::experimental::filesystem::path extractMedianCumulativeFinalCostPerQuery(
      const std::string& plannerName, const double confidence = 0.99) const;

  std::experimental::filesystem::path extractFinalSolutionPerQuery(
      const std::string& plannerName) const;

  std::experimental::filesystem::path extractSuccessPerQuery(
      const std::string& plannerName) const;

  std::size_t getNumQueries() const{
    return numQueries_;
  };

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

  Statistics getQueryStatistics(const unsigned int i) const {
    if (i >= stats_.size()){
      throw std::runtime_error("i is too large");
    }

    return stats_[i];
  }

 private:
  void computeCumulativeMetricsForPlanner(
      const std::string& plannerName, double confidence = 0.99) const;
  void computeCumulativeFinalCost(
      const std::string& plannerName, double confidence = 0.99) const;

  std::shared_ptr<Configuration> config_;
  const std::experimental::filesystem::path statisticsDirectory_;

  // When this is false, all extractions check if the file they would creat exists. If it does, the
  // path to the file is returned instead of recomputing.
  bool forceComputation_{false};

  // All the single-query statistics
  std::vector<Statistics> stats_;

  // Planner specific min and max values.
  mutable std::map<std::string, double> medianCumulativeInitialSolutionCosts_{};
  mutable std::map<std::string, double> minCumulativeInitialSolutionCosts_{};
  mutable std::map<std::string, double> maxCumulativeInitialSolutionCosts_{};

  mutable std::map<std::string, double> medianCumulativeFinalCosts_{};
  mutable std::map<std::string, double> minCumulativeFinalCosts_{};
  mutable std::map<std::string, double> maxCumulativeFinalCosts_{};
  mutable std::map<std::string, double> maxCumulativeNonInfCosts_{};

  mutable std::map<std::string, double> medianCumulativeInitialSolutionDurations_{};
  mutable std::map<std::string, double> minCumulativeInitialSolutionDurations_{};
  mutable std::map<std::string, double> maxCumulativeInitialSolutionDurations_{};
  mutable std::map<std::string, double> maxCumulativeNonInfInitialSolutionDurations_{};

  std::map<std::string, double> successRates_{};

  mutable double maxCost_{std::numeric_limits<double>::lowest()};
  mutable double maxNonInfCost_{std::numeric_limits<double>::lowest()};

  mutable double maxDuration_{std::numeric_limits<double>::lowest()};
  mutable double maxNonInfInitialDuration_{std::numeric_limits<double>::lowest()};

  mutable double maxCumulativeDuration_{std::numeric_limits<double>::lowest()};
  mutable double maxNonInfCumulativeDuration_{0.};

  mutable double maxCumulativeCost_{std::numeric_limits<double>::lowest()};
  mutable double maxNonInfCumulativeCost_{0.};

  std::size_t numQueries_{0u};

  template<class T>
  std::ofstream& writeVectorToFile(std::ofstream &filestream, 
      const std::string &name, const std::vector<T> &values) const;
};

template<class T>
std::ofstream& MultiqueryStatistics::writeVectorToFile(std::ofstream &filestream, 
    const std::string &name, const std::vector<T> &values) const{
  filestream << name;
  for (auto val: values) {
    filestream << std::setprecision(21) << "," << val;
  }

  return filestream; // this enables using this in a series of chained operations
}

}  // namespace ompltools

}  // namespace esp
