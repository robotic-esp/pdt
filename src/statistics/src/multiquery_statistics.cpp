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

#include "pdt/statistics/multiquery_statistics.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <ompl/util/Console.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Weffc++"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "csv/parser.hpp"
#pragma GCC diagnostic pop

#include "pdt/statistics/linear_interpolator.h"
#include "pdt/utilities/write_vector_to_file.h"

namespace pdt {

namespace statistics {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

MultiqueryStatistics::MultiqueryStatistics(const std::shared_ptr<config::Configuration>& config,
                                           const std::vector<PlanningStatistics>& stats,
                                           bool forceComputation) :
    config_(config),
    statisticsDirectory_(fs::path(config_->get<std::string>("experiment/experimentDirectory")) /
                         "statistics/aggregated/"),
    forceComputation_(forceComputation),
    stats_(stats),
    numQueries_(stats.size()) {
  // Create the statistics directory.
  fs::create_directories(statisticsDirectory_);

  const auto& plannerNames = config_->get<std::vector<std::string>>("experiment/planners");
  for (const auto &name : plannerNames) {
    minCumulativeInitialSolutionCosts_[name] = std::numeric_limits<double>::infinity();
    maxCumulativeInitialSolutionCosts_[name] = std::numeric_limits<double>::lowest();
    minCumulativeFinalCosts_[name] = std::numeric_limits<double>::infinity();
    maxCumulativeFinalCosts_[name] = std::numeric_limits<double>::lowest();
    maxCumulativeNonInfCosts_[name] = std::numeric_limits<double>::lowest();

    minCumulativeInitialSolutionDurations_[name] = std::numeric_limits<double>::infinity();
    maxCumulativeInitialSolutionDurations_[name] = std::numeric_limits<double>::lowest();
    maxCumulativeNonInfInitialSolutionDurations_[name] = std::numeric_limits<double>::lowest();

    successRates_[name] = 0.;
    computeCumulativeMetricsForPlanner(name);
    computeCumulativeFinalCost(name);
  }

  // ensure that all queries have the same number of runs
  // The constructor of the statistics is already making sure that all planners have the same number
  // of runs per q
  for (auto it = ++stats_.begin(); it != stats_.end(); ++it) {
    if ((--it)->getNumRunsPerPlanner() != (++it)->getNumRunsPerPlanner()) {
      auto msg = "Not all queries have the same amount of runs."s;
      throw std::runtime_error(msg);
    }
  }

  for (auto it = stats_.begin(); it != stats_.end(); ++it) {
    // obtain the maximum values
    const double maxDuration = it->getMaxDuration();
    if (maxDuration > maxDuration_) {
      maxDuration_ = maxDuration;
    }

    const double maxNonInfInitialDuration = it->getMaxNonInfInitialSolutionDuration();
    if (maxNonInfInitialDuration > maxNonInfInitialDuration_) {
      maxNonInfInitialDuration_ = maxNonInfInitialDuration;
    }

    const double maxCost = it->getMaxCost();
    if (maxCost > maxCost_) {
      maxCost_ = maxCost;
    }

    const double maxNonInfCost = it->getMaxNonInfCost();
    if (maxNonInfCost > maxNonInfCost_) {
      maxNonInfCost_ = maxNonInfCost;
    }

    // obtain the maximum values for the cumulative things
    // this is an upper bound on the maximum non inf cost to simplify computation
    maxNonInfCumulativeCost_ += maxNonInfCost;
    maxNonInfCumulativeDuration_ += maxNonInfInitialDuration;

    for (const auto &name : plannerNames) {
      successRates_[name] += it->getSuccessRate(name);
    }
  }

  for (const auto &name : plannerNames) {
    successRates_[name] = successRates_[name] / static_cast<double>(numQueries_);
  }
}

void MultiqueryStatistics::computeCumulativeMetricsForPlanner(const std::string& plannerName) {
  std::vector<double> medianDurations;
  std::vector<double> lowerDurationBounds;
  std::vector<double> upperDurationBounds;

  std::vector<double> medianCosts;
  std::vector<double> lowerCostBounds;
  std::vector<double> upperCostBounds;

  // the confidence intervals we use here come from where the maximum values are used
  const double initial_duration_confidence =
      config_->get<double>("report/medianCumulativeInitialDurationPlots/confidence");
  const double initial_cost_confidence =
      config_->get<double>("report/medianCumulativeCostPlots/confidence");

  // merge together the stuff that is computed in the separate statistics
  for (auto i = 0u; i < numQueries_; ++i) {
    // Get the median initial solution duration.
    const double medianDuration =
        stats_[i].getMedianInitialSolutionDuration(stats_[i].results_.at(plannerName));

    // Get the median initial solution cost.
    const double medianCost =
        stats_[i].getMedianInitialSolutionCost(stats_[i].results_.at(plannerName));

    // Get the interval for the upper and lower bounds.
    const auto initial_duration_interval =
        stats_[i].populationStats_.findPercentileConfidenceInterval(0.5,
                                                                    initial_duration_confidence);
    const auto initial_cost_interval =
        stats_[i].populationStats_.findPercentileConfidenceInterval(0.5, initial_cost_confidence);

    // Get the upper and lower confidence bounds on the median initial solution duration and cost.
    const double lowerDurationBound = stats_[i].getNthInitialSolutionDuration(
        stats_[i].results_.at(plannerName), initial_duration_interval.lower);
    const double upperDurationBound = stats_[i].getNthInitialSolutionDuration(
        stats_[i].results_.at(plannerName), initial_duration_interval.upper);
    const double lowerCostBound = stats_[i].getNthInitialSolutionCost(
        stats_[i].results_.at(plannerName), initial_cost_interval.lower);
    const double upperCostBound = stats_[i].getNthInitialSolutionCost(
        stats_[i].results_.at(plannerName), initial_cost_interval.upper);

    // save the results
    medianDurations.push_back(medianDuration);
    lowerDurationBounds.push_back(lowerDurationBound);
    upperDurationBounds.push_back(upperDurationBound);

    medianCosts.push_back(medianCost);
    lowerCostBounds.push_back(lowerCostBound);
    upperCostBounds.push_back(upperCostBound);
  }

  std::vector<double> medianCumulativeDuration;
  std::vector<double> uciCumulativeDuration;
  std::vector<double> lciCumulativeDuration;

  std::vector<double> medianCumulativeCost;
  std::vector<double> uciCumulativeCost;
  std::vector<double> lciCumulativeCost;

  // sum median/limits
  for (auto i = 0u; i < numQueries_; ++i) {
    // Get the median initial solution duration.

    const double medianDuration = medianDurations[i];
    const double lcDuration = lowerDurationBounds[i];
    double ucDuration = upperDurationBounds[i];

    const double medianCost = medianCosts[i];
    const double lcCost = lowerCostBounds[i];
    const double ucCost = upperCostBounds[i];

    if (i == 0) {
      medianCumulativeDuration.push_back(medianDuration);
      uciCumulativeDuration.push_back(ucDuration);
      lciCumulativeDuration.push_back(lcDuration);

      medianCumulativeCost.push_back(medianCost);
      uciCumulativeCost.push_back(ucCost);
      lciCumulativeCost.push_back(lcCost);
    } else {
      medianCumulativeDuration.push_back(medianCumulativeDuration.back() + medianDuration);
      uciCumulativeDuration.push_back(uciCumulativeDuration.back() + ucDuration);
      lciCumulativeDuration.push_back(lciCumulativeDuration.back() + lcDuration);

      medianCumulativeCost.push_back(medianCumulativeCost.back() + medianCost);
      uciCumulativeCost.push_back(uciCumulativeCost.back() + ucCost);
      lciCumulativeCost.push_back(lciCumulativeCost.back() + lcCost);
    }

    if (std::isfinite(uciCumulativeCost.back()) &&
        uciCumulativeCost.back() > maxNonInfCumulativeCost_) {
      maxNonInfCumulativeCost_ = uciCumulativeCost.back();
    }

    if (uciCumulativeCost.back() > maxCumulativeCost_) {
      maxCumulativeCost_ = uciCumulativeCost.back();
    }

    if (std::isfinite(uciCumulativeDuration.back()) &&
        uciCumulativeDuration.back() > maxNonInfCumulativeDuration_) {
      maxNonInfCumulativeDuration_ = uciCumulativeDuration.back();
    }

    if (uciCumulativeDuration.back() > maxCumulativeDuration_) {
      maxCumulativeDuration_ = uciCumulativeDuration.back();
    }
  }

  medianCumulativeInitialSolutionCosts_[plannerName] = medianCumulativeCost.back();
  minCumulativeInitialSolutionCosts_[plannerName] = lciCumulativeCost.back();
  maxCumulativeInitialSolutionCosts_[plannerName] = uciCumulativeCost.back();

  medianCumulativeInitialSolutionDurations_[plannerName] = medianCumulativeDuration.back();
  minCumulativeInitialSolutionDurations_[plannerName] = lciCumulativeDuration.back();
  maxCumulativeInitialSolutionDurations_[plannerName] = uciCumulativeDuration.back();
}

void MultiqueryStatistics::computeCumulativeFinalCost(const std::string& plannerName) {
  std::vector<double> medianCosts;
  std::vector<double> lowerCostBounds;
  std::vector<double> upperCostBounds;

  // the confidence intervals we use here come from where the maximum values are used
  const double final_cost_confidence =
      config_->get<double>("report/medianFinalCostPerQueryPlots/confidence");

  // merge together the stuff that is computed in the separate statistics
  for (auto i = 0u; i < numQueries_; ++i) {
    // Get the median initial solution cost.
    const double medianCost = stats_[i].getMedianFinalCost((plannerName));

    // Get the interval for the upper and lower bounds.
    const auto final_cost_interval =
        stats_[i].populationStats_.findPercentileConfidenceInterval(0.5, final_cost_confidence);

    // Get the upper and lower confidence bounds on the median initial solution duration and cost.
    const auto lowerCostBound =
        stats_[i]
            .getNthCosts(stats_[i].results_.at(plannerName), final_cost_interval.lower,
                         {stats_[i].defaultMedianBinDurations_.back()})
            .back();
    const auto upperCostBound =
        stats_[i]
            .getNthCosts(stats_[i].results_.at(plannerName), final_cost_interval.upper,
                         {stats_[i].defaultMedianBinDurations_.back()})
            .back();

    // save the results
    medianCosts.push_back(medianCost);
    lowerCostBounds.push_back(lowerCostBound);
    upperCostBounds.push_back(upperCostBound);

    if (std::isfinite(upperCostBound) && upperCostBound > maxNonInfCost_) {
      maxNonInfCost_ = upperCostBound;
    }

    if (upperCostBound > maxCost_) {
      maxCost_ = upperCostBound;
    }
  }

  std::vector<double> medianCumulativeCost;
  std::vector<double> uciCumulativeCost;
  std::vector<double> lciCumulativeCost;

  // sum median/limits
  for (auto i = 0u; i < numQueries_; ++i) {
    // Get the median initial solution duration.
    const double medianCost = medianCosts[i];
    const double lcCost = lowerCostBounds[i];
    const double ucCost = upperCostBounds[i];

    if (i == 0u) {
      medianCumulativeCost.push_back(medianCost);
      uciCumulativeCost.push_back(ucCost);
      lciCumulativeCost.push_back(lcCost);
    } else {
      medianCumulativeCost.push_back(medianCumulativeCost.back() + medianCost);
      uciCumulativeCost.push_back(uciCumulativeCost.back() + ucCost);
      lciCumulativeCost.push_back(lciCumulativeCost.back() + lcCost);
    }

    if (std::isfinite(uciCumulativeCost.back()) &&
        uciCumulativeCost.back() > maxNonInfCumulativeCost_) {
      maxNonInfCumulativeCost_ = uciCumulativeCost.back();
    }

    if (uciCumulativeCost.back() > maxCumulativeCost_) {
      maxCumulativeCost_ = uciCumulativeCost.back();
    }
  }

  medianCumulativeFinalCosts_[plannerName] = medianCumulativeCost.back();
  minCumulativeFinalCosts_[plannerName] = lciCumulativeCost.back();
  maxCumulativeFinalCosts_[plannerName] = uciCumulativeCost.back();
}

fs::path MultiqueryStatistics::extractMedianInitialSolutionPerQuery(const std::string& plannerName,
                                                                    const double confidence) const {
  // Check if the file already exists.
  fs::path filepath =
      statisticsDirectory_ / (plannerName + "_median_initial_solutions_per_query.csv"s);
  if (fs::exists(filepath) && !forceComputation_) {
    return filepath;
  }

  std::vector<double> medianDurations;
  std::vector<double> lowerDurationBounds;
  std::vector<double> upperDurationBounds;

  std::vector<double> medianCosts;
  std::vector<double> lowerCostBounds;
  std::vector<double> upperCostBounds;

  // merge together the stuff that is computed in the separate statistics
  for (auto i = 0u; i < numQueries_; ++i) {
    // Get the median initial solution duration.
    const double medianDuration =
        stats_[i].getMedianInitialSolutionDuration(stats_[i].results_.at(plannerName));

    // Get the median initial solution cost.
    const double medianCost =
        stats_[i].getMedianInitialSolutionCost(stats_[i].results_.at(plannerName));

    // Get the interval for the upper and lower bounds.
    const auto interval =
        stats_[i].populationStats_.findPercentileConfidenceInterval(0.5, confidence);

    // Get the upper and lower confidence bounds on the median initial solution duration and cost.
    const auto lowerDurationBound =
        stats_[i].getNthInitialSolutionDuration(stats_[i].results_.at(plannerName), interval.lower);
    const auto upperDurationBound =
        stats_[i].getNthInitialSolutionDuration(stats_[i].results_.at(plannerName), interval.upper);
    const auto lowerCostBound =
        stats_[i].getNthInitialSolutionCost(stats_[i].results_.at(plannerName), interval.lower);
    const auto upperCostBound =
        stats_[i].getNthInitialSolutionCost(stats_[i].results_.at(plannerName), interval.upper);

    // save the results
    medianDurations.push_back(medianDuration);
    lowerDurationBounds.push_back(lowerDurationBound);
    upperDurationBounds.push_back(upperDurationBound);

    medianCosts.push_back(medianCost);
    lowerCostBounds.push_back(lowerCostBound);
    upperCostBounds.push_back(upperCostBound);
  }

  // Write to file.
  std::ofstream filestream(filepath.string());
  if (filestream.fail()) {
    auto msg = "Cannot write median initial solution per query for '"s + plannerName + "' to '"s +
               filepath.string() + "'."s;
    throw std::ios_base::failure(msg);
  }

  filestream << stats_[0].createHeader(
      "Median initial solution with "s + std::to_string(confidence) + "% confidence bounds"s,
      plannerName);

  filestream << "query number";
  for (auto i = 0u; i < numQueries_; ++i) {
    filestream << std::setprecision(21) << "," << std::to_string(i + 1);
  }

  // this would be nicer by implementing a operator<< for vector<double>
  utilities::writeVectorToFile(filestream, "\nmedian initial solution duration", medianDurations);
  utilities::writeVectorToFile(filestream, "\nlower initial solution duration confidence bound",
                               lowerDurationBounds);
  utilities::writeVectorToFile(filestream, "\nupper initial solution duration confidence bound",
                               upperDurationBounds);

  utilities::writeVectorToFile(filestream, "\nmedian initial solution cost", medianCosts);
  utilities::writeVectorToFile(filestream, "\nlower initial solution cost confidence bound",
                               lowerCostBounds);
  utilities::writeVectorToFile(filestream, "\nupper initial solution cost confidence bound",
                               upperCostBounds);

  filestream << '\n';

  return filepath;
}

fs::path MultiqueryStatistics::extractMedianCumulativeInitialSolutionPerQuery(
    const std::string& plannerName, const double confidence) const {
  // Check if the file already exists.
  fs::path filepath =
      statisticsDirectory_ / (plannerName + "_median_cumulative_initial_solutions_per_query.csv"s);
  if (fs::exists(filepath) && !forceComputation_) {
    return filepath;
  }

  std::vector<double> medianDurations;
  std::vector<double> lowerDurationBounds;
  std::vector<double> upperDurationBounds;

  std::vector<double> medianCosts;
  std::vector<double> lowerCostBounds;
  std::vector<double> upperCostBounds;

  // merge together the stuff that is computed in the separate statistics
  for (auto i = 0u; i < numQueries_; ++i) {
    // Get the median initial solution duration.
    const double medianDuration =
        stats_[i].getMedianInitialSolutionDuration(stats_[i].results_.at(plannerName));

    // Get the median initial solution cost.
    const double medianCost =
        stats_[i].getMedianInitialSolutionCost(stats_[i].results_.at(plannerName));

    // Get the interval for the upper and lower bounds.
    const auto interval =
        stats_[i].populationStats_.findPercentileConfidenceInterval(0.5, confidence);

    // Get the upper and lower confidence bounds on the median initial solution duration and cost.
    const auto lowerDurationBound =
        stats_[i].getNthInitialSolutionDuration(stats_[i].results_.at(plannerName), interval.lower);
    const auto upperDurationBound =
        stats_[i].getNthInitialSolutionDuration(stats_[i].results_.at(plannerName), interval.upper);
    const auto lowerCostBound =
        stats_[i].getNthInitialSolutionCost(stats_[i].results_.at(plannerName), interval.lower);
    const auto upperCostBound =
        stats_[i].getNthInitialSolutionCost(stats_[i].results_.at(plannerName), interval.upper);

    // save the results
    medianDurations.push_back(medianDuration);
    lowerDurationBounds.push_back(lowerDurationBound);
    upperDurationBounds.push_back(upperDurationBound);

    medianCosts.push_back(medianCost);
    lowerCostBounds.push_back(lowerCostBound);
    upperCostBounds.push_back(upperCostBound);
  }

  std::vector<double> medianCumulativeDuration;
  std::vector<double> uciCumulativeDuration;
  std::vector<double> lciCumulativeDuration;

  std::vector<double> medianCumulativeCost;
  std::vector<double> uciCumulativeCost;
  std::vector<double> lciCumulativeCost;

  // sum median/limits
  for (auto i = 0u; i < numQueries_; ++i) {
    // Get the median initial solution duration.

    const double medianDuration = medianDurations[i];
    const double lcDuration = lowerDurationBounds[i];
    const double ucDuration = upperDurationBounds[i];

    const double medianCost = medianCosts[i];
    const double lcCost = lowerCostBounds[i];
    const double ucCost = upperCostBounds[i];

    if (i == 0u) {
      medianCumulativeDuration.push_back(medianDuration);
      uciCumulativeDuration.push_back(ucDuration);
      lciCumulativeDuration.push_back(lcDuration);

      medianCumulativeCost.push_back(medianCost);
      uciCumulativeCost.push_back(ucCost);
      lciCumulativeCost.push_back(lcCost);
    } else {
      medianCumulativeDuration.push_back(medianCumulativeDuration.back() + medianDuration);
      uciCumulativeDuration.push_back(uciCumulativeDuration.back() + ucDuration);
      lciCumulativeDuration.push_back(lciCumulativeDuration.back() + lcDuration);

      medianCumulativeCost.push_back(medianCumulativeCost.back() + medianCost);
      uciCumulativeCost.push_back(uciCumulativeCost.back() + ucCost);
      lciCumulativeCost.push_back(lciCumulativeCost.back() + lcCost);
    }
  }

  // Write to file.
  std::ofstream filestream(filepath.string());
  if (filestream.fail()) {
    auto msg = "Cannot write cumulative initial solution for '"s + plannerName + "' to '"s +
               filepath.string() + "'."s;
    throw std::ios_base::failure(msg);
  }

  filestream << stats_[0].createHeader(
      "Cumulative initial solution with "s + std::to_string(confidence) + "% confidence bounds"s,
      plannerName);

  filestream << "query number";
  for (auto i = 0u; i < numQueries_; ++i) {
    filestream << std::setprecision(21) << "," << std::to_string(i + 1);
  }

  // this would be nicer by implementing a operator<< for vector<double>
  utilities::writeVectorToFile(filestream, "\ncumulative median initial solution duration",
                               medianCumulativeDuration);
  utilities::writeVectorToFile(filestream,
                               "\nlower cumulative initial solution duration confidence bound",
                               lciCumulativeDuration);
  utilities::writeVectorToFile(filestream,
                               "\nupper cumulative initial solution duration confidence bound",
                               uciCumulativeDuration);

  utilities::writeVectorToFile(filestream, "\ncumulative median initial solution cost",
                               medianCumulativeCost);
  utilities::writeVectorToFile(
      filestream, "\nlower cumulative initial solution cost confidence bound", lciCumulativeCost);
  utilities::writeVectorToFile(
      filestream, "\nupper cumulative initial solution cost confidence bound", uciCumulativeCost);

  return filepath;
}

fs::path MultiqueryStatistics::extractMedianFinalSolutionPerQuery(const std::string& plannerName,
                                                                  const double confidence) const {
  // Check if the file already exists.
  fs::path filepath =
      statisticsDirectory_ / (plannerName + "_median_final_solutions_per_query.csv"s);
  if (fs::exists(filepath) && !forceComputation_) {
    return filepath;
  }

  std::vector<double> medianCosts;
  std::vector<double> lowerCostBounds;
  std::vector<double> upperCostBounds;

  // merge together the stuff that is computed in the separate statistics
  for (auto i = 0u; i < numQueries_; ++i) {
    // Get the median initial solution cost.
    const double medianCost = stats_[i].getMedianFinalCost((plannerName));

    // Get the interval for the upper and lower bounds.
    const auto interval =
        stats_[i].populationStats_.findPercentileConfidenceInterval(0.5, confidence);

    // Get the upper and lower confidence bounds on the median initial solution duration and cost.
    const auto lowerCostBound = stats_[i]
                                    .getNthCosts(stats_[i].results_.at(plannerName), interval.lower,
                                                 {stats_[i].defaultMedianBinDurations_.back()})
                                    .back();
    const auto upperCostBound = stats_[i]
                                    .getNthCosts(stats_[i].results_.at(plannerName), interval.upper,
                                                 {stats_[i].defaultMedianBinDurations_.back()})
                                    .back();

    // save the results
    medianCosts.push_back(medianCost);
    lowerCostBounds.push_back(lowerCostBound);
    upperCostBounds.push_back(upperCostBound);
  }

  // Write to file.
  std::ofstream filestream(filepath.string());
  if (filestream.fail()) {
    auto msg = "Cannot write median last solution per query for '"s + plannerName + "' to '"s +
               filepath.string() + "'."s;
    throw std::ios_base::failure(msg);
  }

  filestream << stats_[0].createHeader(
      "Median last solution with "s + std::to_string(confidence) + "% confidence bounds"s,
      plannerName);

  filestream << "query number";
  for (auto i = 0u; i < numQueries_; ++i) {
    filestream << std::setprecision(21) << "," << std::to_string(i + 1);
  }

  // this would be nicer by implementing a operator<< for vector<double>
  utilities::writeVectorToFile(filestream, "\nmedian last solution cost", medianCosts);
  utilities::writeVectorToFile(filestream, "\nlower last solution cost confidence bound",
                               lowerCostBounds);
  utilities::writeVectorToFile(filestream, "\nupper last solution cost confidence bound",
                               upperCostBounds);

  filestream << '\n';

  return filepath;
}

fs::path MultiqueryStatistics::extractMedianCumulativeFinalCostPerQuery(
    const std::string& plannerName, const double confidence) const {
  // Check if the file already exists.
  fs::path filepath =
      statisticsDirectory_ / (plannerName + "_median_cumulative_final_solutions_per_query.csv"s);
  if (fs::exists(filepath) && !forceComputation_) {
    return filepath;
  }

  std::vector<double> medianCosts;
  std::vector<double> lowerCostBounds;
  std::vector<double> upperCostBounds;

  // merge together the stuff that is computed in the separate statistics
  for (auto i = 0u; i < numQueries_; ++i) {
    // Get the median initial solution cost.
    const double medianCost = stats_[i].getMedianFinalCost(plannerName);

    // Get the interval for the upper and lower bounds.
    const auto interval =
        stats_[i].populationStats_.findPercentileConfidenceInterval(0.5, confidence);

    // Get the upper and lower confidence bounds on the median initial solution duration and cost.
    const auto lowerCostBound = stats_[i]
                                    .getNthCosts(stats_[i].results_.at(plannerName), interval.lower,
                                                 {stats_[i].defaultMedianBinDurations_.back()})
                                    .back();
    const auto upperCostBound = stats_[i]
                                    .getNthCosts(stats_[i].results_.at(plannerName), interval.upper,
                                                 {stats_[i].defaultMedianBinDurations_.back()})
                                    .back();

    // save the results
    medianCosts.push_back(medianCost);
    lowerCostBounds.push_back(lowerCostBound);
    upperCostBounds.push_back(upperCostBound);
  }

  std::vector<double> medianCumulativeCost;
  std::vector<double> uciCumulativeCost;
  std::vector<double> lciCumulativeCost;

  // sum median/limits
  for (auto i = 0u; i < numQueries_; ++i) {
    // Get the median initial solution duration.
    const double medianCost = medianCosts[i];
    const double lcCost = lowerCostBounds[i];
    const double ucCost = upperCostBounds[i];

    if (i == 0) {
      medianCumulativeCost.push_back(medianCost);
      uciCumulativeCost.push_back(ucCost);
      lciCumulativeCost.push_back(lcCost);
    } else {
      medianCumulativeCost.push_back(medianCumulativeCost.back() + medianCost);
      uciCumulativeCost.push_back(uciCumulativeCost.back() + ucCost);
      lciCumulativeCost.push_back(lciCumulativeCost.back() + lcCost);
    }
  }

  // Write to file.
  std::ofstream filestream(filepath.string());
  if (filestream.fail()) {
    auto msg = "Cannot write cumulative final solution for '"s + plannerName + "' to '"s +
               filepath.string() + "'."s;
    throw std::ios_base::failure(msg);
  }

  filestream << stats_[0].createHeader(
      "Cumulative final solution with "s + std::to_string(confidence) + "% confidence bounds"s,
      plannerName);

  filestream << "query number";
  for (auto i = 0u; i < numQueries_; ++i) {
    filestream << std::setprecision(21) << "," << std::to_string(i + 1);
  }

  // this would be nicer by implementing a operator<< for vector<double>
  utilities::writeVectorToFile(filestream, "\ncumulative median final solution cost",
                               medianCumulativeCost);
  utilities::writeVectorToFile(
      filestream, "\nlower cumulative final solution cost confidence bound", lciCumulativeCost);
  utilities::writeVectorToFile(
      filestream, "\nupper cumulative final solution cost confidence bound", uciCumulativeCost);

  return filepath;
}

fs::path MultiqueryStatistics::extractFinalSolutionPerQuery(const std::string& plannerName) const {
  // Check if the file already exists.
  fs::path filepath = statisticsDirectory_ / (plannerName + "_final_solution_per_query.csv"s);
  if (fs::exists(filepath) && !forceComputation_) {
    return filepath;
  }

  std::vector<std::size_t> queryNumber{};
  std::vector<double> queryCosts{};
  std::vector<double> queryDurations{};

  std::size_t cnt = 0u;
  for (const auto& stat : stats_) {
    auto durations = stat.getLastSolutionDurations(stat.results_.at(plannerName));
    auto costs = stat.getLastSolutionCosts(stat.results_.at(plannerName));

    for (auto i = 0u; i < durations.size(); ++i) {
      queryDurations.push_back(durations[i]);
      queryCosts.push_back(costs[i]);
      queryNumber.push_back(cnt);
    }

    ++cnt;
  }

  // Write to file.
  std::ofstream filestream(filepath.string());
  if (filestream.fail()) {
    auto msg =
        "Cannot write finale solution for '"s + plannerName + "' to '"s + filepath.string() + "'."s;
    throw std::ios_base::failure(msg);
  }

  filestream << stats_[0u].createHeader("Final solutions "s, plannerName);

  // this would be nicer by implementing a operator<< for vector<double>
  utilities::writeVectorToFile(filestream, "Query number", queryNumber);
  utilities::writeVectorToFile(filestream, "\nFinal solution duration", queryDurations);
  utilities::writeVectorToFile(filestream, "\nFinal solution cost", queryCosts);

  return filepath;
}

std::experimental::filesystem::path MultiqueryStatistics::extractSuccessPerQuery(
    const std::string& plannerName) const {
  // Check if the file already exists.
  fs::path filepath = statisticsDirectory_ / (plannerName + "_success_rate_per_query.csv"s);
  if (fs::exists(filepath) && !forceComputation_) {
    return filepath;
  }

  const auto maxDuration = stats_[0].getMaxDuration();
  std::vector<double> timeFractions = {.25, .5, .75, 1.};
  std::vector<double> times;

  for (const auto f : timeFractions) {
    times.push_back(f * maxDuration);
  }

  std::vector<std::vector<double>> successRate;

  for (const auto t : times) {
    std::vector<double> tmp;
    for (auto i = 0u; i < numQueries_; ++i) {
      // Get the median initial solution duration.
      const auto durations =
          stats_[i].getInitialSolutionDurations(stats_[i].results_.at(plannerName));

      auto successfulRuns = 0u;
      for (const auto& duration : durations) {
        if (duration < t) {
          ++successfulRuns;
        }
      }
      tmp.push_back(successfulRuns / static_cast<double>(durations.size()));
    }
    // std::cout << std::endl;
    successRate.push_back(tmp);
  }

  // Write to file.
  std::ofstream filestream(filepath.string());
  if (filestream.fail()) {
    auto msg =
        "Cannot write success rate for '"s + plannerName + "' to '"s + filepath.string() + "'."s;
    throw std::ios_base::failure(msg);
  }

  filestream << stats_[0u].createHeader("Success rate", plannerName);

  filestream << "query number";
  for (auto i = 0u; i < numQueries_; ++i) {
    filestream << std::setprecision(21) << "," << std::to_string(i + 1);
  }

  // this would be nicer by implementing a operator<< for vector<double>
  auto cnt = 0u;
  for (const auto f : timeFractions) {
    utilities::writeVectorToFile(
        filestream, "\nsuccess rate at " + std::to_string(static_cast<int>(100.0 * f)) + " percent",
        successRate[cnt]);
    ++cnt;
  }

  return filepath;
}

PlanningStatistics MultiqueryStatistics::getQueryStatistics(const unsigned int i) const {
  if (i >= stats_.size()) {
    throw std::runtime_error("i is too large");
  }

  return stats_[i];
}

double MultiqueryStatistics::getMaxDuration() const {
  return maxDuration_;
}

double MultiqueryStatistics::getMaxNonInfInitialDuration() const {
  return maxNonInfInitialDuration_;
}

double MultiqueryStatistics::getMaxCumulativeDuration() const {
  return maxCumulativeDuration_;
}

double MultiqueryStatistics::getMaxNonInfCumulativeDuration() const {
  return maxNonInfCumulativeDuration_;
}

double MultiqueryStatistics::getMaxCost() const {
  return maxCost_;
}

double MultiqueryStatistics::getMaxNonInfCost() const {
  return maxNonInfCost_;
}

double MultiqueryStatistics::getMaxCumulativeCost() const {
  return maxCumulativeCost_;
}

double MultiqueryStatistics::getMaxNonInfCumulativeCost() const {
  return maxNonInfCumulativeCost_;
}

double MultiqueryStatistics::getMinCumulativeInitialSolutionCost(
    const std::string& plannerName) const {
  return minCumulativeInitialSolutionCosts_.at(plannerName);
}
double MultiqueryStatistics::getMedianCumulativeInitialSolutionCost(
    const std::string& plannerName) const {
  return medianCumulativeInitialSolutionCosts_.at(plannerName);
}
double MultiqueryStatistics::getMaxCumulativeInitialSolutionCost(
    const std::string& plannerName) const {
  return maxCumulativeInitialSolutionCosts_.at(plannerName);
}

double MultiqueryStatistics::getMinCumulativeInitialSolutionDuration(
    const std::string& plannerName) const {
  return minCumulativeInitialSolutionDurations_.at(plannerName);
}
double MultiqueryStatistics::getMedianCumulativeInitialSolutionDuration(
    const std::string& plannerName) const {
  return medianCumulativeInitialSolutionDurations_.at(plannerName);
}

double MultiqueryStatistics::getMaxCumulativeNonInfInitialSolutionDuration(
    const std::string& plannerName) const {
  return maxCumulativeNonInfInitialSolutionDurations_.at(plannerName);
}
double MultiqueryStatistics::getMaxCumulativeInitialSolutionDuration(
    const std::string& plannerName) const {
  return maxCumulativeInitialSolutionDurations_.at(plannerName);
}

double MultiqueryStatistics::getSuccessRate(const std::string& plannerName) const {
  return successRates_.at(plannerName);
}
double MultiqueryStatistics::getMinCumulativeFinalCost(const std::string& plannerName) const {
  return minCumulativeFinalCosts_.at(plannerName);
}
double MultiqueryStatistics::getMedianCumulativeFinalCost(const std::string& plannerName) const {
  return medianCumulativeFinalCosts_.at(plannerName);
}
double MultiqueryStatistics::getMaxCumulativeFinalCost(const std::string& plannerName) const {
  return maxCumulativeFinalCosts_.at(plannerName);
}

}  // namespace statistics

}  // namespace pdt
