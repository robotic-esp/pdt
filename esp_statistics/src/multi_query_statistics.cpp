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

// Authors: Valentin Hartmann

#include "esp_statistics/multi_query_statistics.h"

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

#include "esp_statistics/linear_interpolator.h"

namespace esp {

namespace ompltools {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

MultiQueryStatistics::MultiQueryStatistics(const std::shared_ptr<Configuration>& config,
    const std::vector<Statistics> &stats,
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
  for (const auto name: plannerNames){
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
  // The constructor of the statistics is already making sure that all planners have the same number of runs per q
  for (auto it = ++stats_.begin(); it != stats_.end(); ++it) {
    if ((--it)->getNumRunsPerPlanner() != (++it)->getNumRunsPerPlanner()) {
      auto msg = "Not all queries have the same amount of runs."s;
      throw std::runtime_error(msg);
    }
  }

  for (auto it = stats_.begin(); it != stats_.end(); ++it) {
    // obtain the maximum values
    const double maxDuration = it->getMaxDuration();
    if (maxDuration > maxDuration_){
      maxDuration_ = maxDuration;
    }

    const double maxNonInfInitialDuration = it->getMaxNonInfInitialSolutionDuration();
    if (maxNonInfInitialDuration > maxNonInfInitialDuration_){
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

    for (const auto name: plannerNames){
      successRates_[name] += it->getSuccessRate(name);
    }
  }

  for (const auto name: plannerNames){
    successRates_[name] = successRates_[name] / static_cast<double>(numQueries_);
  }
}

void MultiQueryStatistics::computeCumulativeMetricsForPlanner(
    const std::string& plannerName, double confidence) const{
  std::vector<double> medianDurations;
  std::vector<double> lowerDurationBounds;
  std::vector<double> upperDurationBounds;

  std::vector<double> medianCosts;
  std::vector<double> lowerCostBounds;
  std::vector<double> upperCostBounds;

  // merge together the stuff that is computed in the separate statistics
  for (std::size_t i=0; i<numQueries_; ++i){
    // Get the median initial solution duration.
    double medianDuration = stats_[i].getMedianInitialSolutionDuration(stats_[i].results_.at(plannerName));

    // Get the median initial solution cost.
    double medianCost = stats_[i].getMedianInitialSolutionCost(stats_[i].results_.at(plannerName));

    // Get the interval for the upper and lower bounds.
    auto interval = stats_[i].populationStats_.findPercentileConfidenceInterval(0.5, confidence);

    // Get the upper and lower confidence bounds on the median initial solution duration and cost.
    auto lowerDurationBound = stats_[i].getNthInitialSolutionDuration(stats_[i].results_.at(plannerName), interval.lower);
    auto upperDurationBound = stats_[i].getNthInitialSolutionDuration(stats_[i].results_.at(plannerName), interval.upper);
    auto lowerCostBound = stats_[i].getNthInitialSolutionCost(stats_[i].results_.at(plannerName), interval.lower);
    auto upperCostBound = stats_[i].getNthInitialSolutionCost(stats_[i].results_.at(plannerName), interval.upper);

    // save the results
    medianDurations.emplace_back(medianDuration);
    lowerDurationBounds.emplace_back(lowerDurationBound);
    upperDurationBounds.emplace_back(upperDurationBound);

    medianCosts.emplace_back(medianCost);
    lowerCostBounds.emplace_back(lowerCostBound);
    upperCostBounds.emplace_back(upperCostBound);
  }

  std::vector<double> medianCumulativeDuration;
  std::vector<double> uciCumulativeDuration;
  std::vector<double> lciCumulativeDuration;

  std::vector<double> medianCumulativeCost;
  std::vector<double> uciCumulativeCost;
  std::vector<double> lciCumulativeCost;

  // sum median/limits
  for (std::size_t i=0; i<numQueries_; ++i){
    // Get the median initial solution duration.

    double medianDuration = medianDurations[i];
    double lcDuration = lowerDurationBounds[i];
    double ucDuration = upperDurationBounds[i];

    double medianCost = medianCosts[i];
    double lcCost = lowerCostBounds[i];
    double ucCost = upperCostBounds[i];

    if (i == 0){
      medianCumulativeDuration.push_back(medianDuration);
      uciCumulativeDuration.push_back(ucDuration);
      lciCumulativeDuration.push_back(lcDuration);

      medianCumulativeCost.push_back(medianCost);
      uciCumulativeCost.push_back(ucCost);
      lciCumulativeCost.push_back(lcCost);
    }
    else{
      medianCumulativeDuration.push_back(medianCumulativeDuration.back() + medianDuration);
      uciCumulativeDuration.push_back(uciCumulativeDuration.back() + ucDuration);
      lciCumulativeDuration.push_back(lciCumulativeDuration.back() + lcDuration);

      medianCumulativeCost.push_back(medianCumulativeCost.back() + medianCost);
      uciCumulativeCost.push_back(uciCumulativeCost.back() + ucCost);
      lciCumulativeCost.push_back(lciCumulativeCost.back() + lcCost);
    }

    if (std::isfinite(uciCumulativeCost.back()) && uciCumulativeCost.back() > maxNonInfCumulativeCost_){
      maxNonInfCumulativeCost_ = uciCumulativeCost.back();
    }

    if (uciCumulativeCost.back() > maxCumulativeCost_){
      maxCumulativeCost_ = uciCumulativeCost.back();
    }

    if (std::isfinite(uciCumulativeDuration.back()) && uciCumulativeDuration.back() > maxNonInfCumulativeDuration_){
      maxNonInfCumulativeDuration_ = uciCumulativeDuration.back();
    }

    if (uciCumulativeDuration.back() > maxCumulativeDuration_){
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

void MultiQueryStatistics::computeCumulativeFinalCost(
      const std::string& plannerName, double confidence) const{
  std::vector<double> medianCosts;
  std::vector<double> lowerCostBounds;
  std::vector<double> upperCostBounds;

  // merge together the stuff that is computed in the separate statistics
  for (std::size_t i=0; i<numQueries_; ++i){
    // Get the median initial solution cost.
    double medianCost = stats_[i].getMedianFinalCost((plannerName));

    // Get the interval for the upper and lower bounds.
    auto interval = stats_[i].populationStats_.findPercentileConfidenceInterval(0.5, confidence);

    // Get the upper and lower confidence bounds on the median initial solution duration and cost.
    auto lowerCostBound = stats_[i].getNthCosts(stats_[i].results_.at(plannerName), interval.lower, {stats_[i].defaultMedianBinDurations_.back()}).back();
    auto upperCostBound = stats_[i].getNthCosts(stats_[i].results_.at(plannerName), interval.upper, {stats_[i].defaultMedianBinDurations_.back()}).back();

    // save the results
    medianCosts.emplace_back(medianCost);
    lowerCostBounds.emplace_back(lowerCostBound);
    upperCostBounds.emplace_back(upperCostBound);

    if (std::isfinite(upperCostBound) &&  upperCostBound > maxNonInfCost_){
      maxNonInfCost_ = upperCostBound;
    }

    if (upperCostBound > maxCost_){
      maxCost_ = upperCostBound;
    }
  }

  std::vector<double> medianCumulativeCost;
  std::vector<double> uciCumulativeCost;
  std::vector<double> lciCumulativeCost;

  // sum median/limits
  for (std::size_t i=0; i<numQueries_; ++i){
    // Get the median initial solution duration.
    double medianCost = medianCosts[i];
    double lcCost = lowerCostBounds[i];
    double ucCost = upperCostBounds[i];

    if (i == 0){
      medianCumulativeCost.push_back(medianCost);
      uciCumulativeCost.push_back(ucCost);
      lciCumulativeCost.push_back(lcCost);
    }
    else{
      medianCumulativeCost.push_back(medianCumulativeCost.back() + medianCost);
      uciCumulativeCost.push_back(uciCumulativeCost.back() + ucCost);
      lciCumulativeCost.push_back(lciCumulativeCost.back() + lcCost);
    }

    if (std::isfinite(uciCumulativeCost.back()) && uciCumulativeCost.back() > maxNonInfCumulativeCost_){
      maxNonInfCumulativeCost_ = uciCumulativeCost.back();
    }

    if (uciCumulativeCost.back() > maxCumulativeCost_){
      maxCumulativeCost_ = uciCumulativeCost.back();
    }
  }

  medianCumulativeFinalCosts_[plannerName] = medianCumulativeCost.back();
  minCumulativeFinalCosts_[plannerName] = lciCumulativeCost.back();
  maxCumulativeFinalCosts_[plannerName] = uciCumulativeCost.back();
}

fs::path MultiQueryStatistics::extractMedianInitialSolutionPerQuery(
      const std::string& plannerName, const double confidence) const{
  // Check if the file already exists.
  fs::path filepath = statisticsDirectory_ / (plannerName + "_median_initial_solutions_per_query.csv"s);
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
  for (std::size_t i=0; i<numQueries_; ++i){
    // Get the median initial solution duration.
    double medianDuration = stats_[i].getMedianInitialSolutionDuration(stats_[i].results_.at(plannerName));

    // Get the median initial solution cost.
    double medianCost = stats_[i].getMedianInitialSolutionCost(stats_[i].results_.at(plannerName));

    // Get the interval for the upper and lower bounds.
    auto interval = stats_[i].populationStats_.findPercentileConfidenceInterval(0.5, confidence);

    // Get the upper and lower confidence bounds on the median initial solution duration and cost.
    auto lowerDurationBound = stats_[i].getNthInitialSolutionDuration(stats_[i].results_.at(plannerName), interval.lower);
    auto upperDurationBound = stats_[i].getNthInitialSolutionDuration(stats_[i].results_.at(plannerName), interval.upper);
    auto lowerCostBound = stats_[i].getNthInitialSolutionCost(stats_[i].results_.at(plannerName), interval.lower);
    auto upperCostBound = stats_[i].getNthInitialSolutionCost(stats_[i].results_.at(plannerName), interval.upper);

    // save the results
    medianDurations.emplace_back(medianDuration);
    lowerDurationBounds.emplace_back(lowerDurationBound);
    upperDurationBounds.emplace_back(upperDurationBound);

    medianCosts.emplace_back(medianCost);
    lowerCostBounds.emplace_back(lowerCostBound);
    upperCostBounds.emplace_back(upperCostBound);

    if (std::isfinite(upperCostBound) &&  upperCostBound > maxNonInfCost_){
      maxNonInfCost_ = upperCostBound;
    }

    if (upperCostBound > maxCost_){
      maxCost_ = upperCostBound;
    }
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
  for (std::size_t i=0; i<numQueries_; ++i) {filestream << std::setprecision(21) << "," << std::to_string(i+1);}

  // this would be nicer by implementing a operator<< for vector<double>
  writeVectorToFile(filestream, "\nmedian initial solution duration", medianDurations);
  writeVectorToFile(filestream, "\nlower initial solution duration confidence bound", lowerDurationBounds);
  writeVectorToFile(filestream, "\nupper initial solution duration confidence bound", upperDurationBounds);

  writeVectorToFile(filestream, "\nmedian initial solution cost", medianCosts);
  writeVectorToFile(filestream, "\nlower initial solution cost confidence bound", lowerCostBounds);
  writeVectorToFile(filestream, "\nupper initial solution cost confidence bound", upperCostBounds);
  
  filestream << '\n';

  return filepath;
}

fs::path MultiQueryStatistics::extractMedianCumulativeInitialSolutionPerQuery(
    const std::string& plannerName, const double confidence) const{
  // Check if the file already exists.
  fs::path filepath = statisticsDirectory_ / (plannerName + "_median_cumulative_initial_solutions_per_query.csv"s);
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
  for (std::size_t i=0; i<numQueries_; ++i){
    // Get the median initial solution duration.
    double medianDuration = stats_[i].getMedianInitialSolutionDuration(stats_[i].results_.at(plannerName));

    // Get the median initial solution cost.
    double medianCost = stats_[i].getMedianInitialSolutionCost(stats_[i].results_.at(plannerName));

    // Get the interval for the upper and lower bounds.
    auto interval = stats_[i].populationStats_.findPercentileConfidenceInterval(0.5, confidence);

    // Get the upper and lower confidence bounds on the median initial solution duration and cost.
    auto lowerDurationBound = stats_[i].getNthInitialSolutionDuration(stats_[i].results_.at(plannerName), interval.lower);
    auto upperDurationBound = stats_[i].getNthInitialSolutionDuration(stats_[i].results_.at(plannerName), interval.upper);
    auto lowerCostBound = stats_[i].getNthInitialSolutionCost(stats_[i].results_.at(plannerName), interval.lower);
    auto upperCostBound = stats_[i].getNthInitialSolutionCost(stats_[i].results_.at(plannerName), interval.upper);

    // save the results
    medianDurations.emplace_back(medianDuration);
    lowerDurationBounds.emplace_back(lowerDurationBound);
    upperDurationBounds.emplace_back(upperDurationBound);

    medianCosts.emplace_back(medianCost);
    lowerCostBounds.emplace_back(lowerCostBound);
    upperCostBounds.emplace_back(upperCostBound);
  }

  std::vector<double> medianCumulativeDuration;
  std::vector<double> uciCumulativeDuration;
  std::vector<double> lciCumulativeDuration;

  std::vector<double> medianCumulativeCost;
  std::vector<double> uciCumulativeCost;
  std::vector<double> lciCumulativeCost;

  // sum median/limits
  for (std::size_t i=0; i<numQueries_; ++i){
    // Get the median initial solution duration.

    double medianDuration = medianDurations[i];
    double lcDuration = lowerDurationBounds[i];
    double ucDuration = upperDurationBounds[i];

    double medianCost = medianCosts[i];
    double lcCost = lowerCostBounds[i];
    double ucCost = upperCostBounds[i];

    if (i == 0){
      medianCumulativeDuration.push_back(medianDuration);
      uciCumulativeDuration.push_back(ucDuration);
      lciCumulativeDuration.push_back(lcDuration);

      medianCumulativeCost.push_back(medianCost);
      uciCumulativeCost.push_back(ucCost);
      lciCumulativeCost.push_back(lcCost);
    }
    else{
      medianCumulativeDuration.push_back(medianCumulativeDuration.back() + medianDuration);
      uciCumulativeDuration.push_back(uciCumulativeDuration.back() + ucDuration);
      lciCumulativeDuration.push_back(lciCumulativeDuration.back() + lcDuration);

      medianCumulativeCost.push_back(medianCumulativeCost.back() + medianCost);
      uciCumulativeCost.push_back(uciCumulativeCost.back() + ucCost);
      lciCumulativeCost.push_back(lciCumulativeCost.back() + lcCost);
    }

    if (std::isfinite(uciCumulativeCost.back()) && uciCumulativeCost.back() > maxNonInfCumulativeCost_){
      maxNonInfCumulativeCost_ = uciCumulativeCost.back();
    }

    if (uciCumulativeCost.back() > maxCumulativeCost_){
      maxCumulativeCost_ = uciCumulativeCost.back();
    }

    if (std::isfinite(uciCumulativeDuration.back()) && uciCumulativeDuration.back() > maxNonInfCumulativeDuration_){
      maxNonInfCumulativeDuration_ = uciCumulativeDuration.back();
    }

    if (uciCumulativeDuration.back() > maxCumulativeDuration_){
      maxCumulativeDuration_ = uciCumulativeDuration.back();
    }
  }

  minCumulativeInitialSolutionCosts_[plannerName] = lciCumulativeCost[0];
  maxCumulativeInitialSolutionCosts_[plannerName] = uciCumulativeCost[0];
  minCumulativeFinalCosts_[plannerName] = lciCumulativeCost.back();
  maxCumulativeFinalCosts_[plannerName] = uciCumulativeCost.back();

  minCumulativeInitialSolutionDurations_[plannerName] = lciCumulativeDuration.back();
  maxCumulativeInitialSolutionDurations_[plannerName] = uciCumulativeDuration.back();
  
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
  for (std::size_t i=0; i<numQueries_; ++i) {
    filestream << std::setprecision(21) << "," << std::to_string(i+1);
  }

  // this would be nicer by implementing a operator<< for vector<double>
  writeVectorToFile(filestream, "\ncumulative median initial solution duration", medianCumulativeDuration);
  writeVectorToFile(filestream, "\nlower cumulative initial solution duration confidence bound", lciCumulativeDuration);
  writeVectorToFile(filestream, "\nupper cumulative initial solution duration confidence bound", uciCumulativeDuration);

  writeVectorToFile(filestream, "\ncumulative median initial solution cost", medianCumulativeCost);
  writeVectorToFile(filestream, "\nlower cumulative initial solution cost confidence bound", lciCumulativeCost);
  writeVectorToFile(filestream, "\nupper cumulative initial solution cost confidence bound", uciCumulativeCost);

  return filepath;
}

fs::path MultiQueryStatistics::extractMedianFinalSolutionPerQuery(
      const std::string& plannerName, const double confidence) const{
  // Check if the file already exists.
  fs::path filepath = statisticsDirectory_ / (plannerName + "_median_final_solutions_per_query.csv"s);
  if (fs::exists(filepath) && !forceComputation_) {
    return filepath;
  }

  std::vector<double> medianCosts;
  std::vector<double> lowerCostBounds;
  std::vector<double> upperCostBounds;

  // merge together the stuff that is computed in the separate statistics
  for (std::size_t i=0; i<numQueries_; ++i){
    // Get the median initial solution cost.
    double medianCost = stats_[i].getMedianFinalCost((plannerName));

    // Get the interval for the upper and lower bounds.
    auto interval = stats_[i].populationStats_.findPercentileConfidenceInterval(0.5, confidence);

    // Get the upper and lower confidence bounds on the median initial solution duration and cost.
    auto lowerCostBound = stats_[i].getNthCosts(stats_[i].results_.at(plannerName), interval.lower, {stats_[i].defaultMedianBinDurations_.back()}).back();
    auto upperCostBound = stats_[i].getNthCosts(stats_[i].results_.at(plannerName), interval.upper, {stats_[i].defaultMedianBinDurations_.back()}).back();

    // save the results
    medianCosts.emplace_back(medianCost);
    lowerCostBounds.emplace_back(lowerCostBound);
    upperCostBounds.emplace_back(upperCostBound);

    if (std::isfinite(upperCostBound) &&  upperCostBound > maxNonInfCost_){
      maxNonInfCost_ = upperCostBound;
    }

    if (upperCostBound > maxCost_){
      maxCost_ = upperCostBound;
    }
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
  for (std::size_t i=0; i<numQueries_; ++i) {filestream << std::setprecision(21) << "," << std::to_string(i+1);}

  // this would be nicer by implementing a operator<< for vector<double>
  writeVectorToFile(filestream, "\nmedian last solution cost", medianCosts);
  writeVectorToFile(filestream, "\nlower last solution cost confidence bound", lowerCostBounds);
  writeVectorToFile(filestream, "\nupper last solution cost confidence bound", upperCostBounds);
  
  filestream << '\n';

  return filepath;
}

fs::path MultiQueryStatistics::extractMedianCumulativeFinalCostPerQuery(
    const std::string& plannerName, const double confidence) const{
  // Check if the file already exists.
  fs::path filepath = statisticsDirectory_ / (plannerName + "_median_cumulative_final_solutions_per_query.csv"s);
  if (fs::exists(filepath) && !forceComputation_) {
    return filepath;
  }

  std::vector<double> medianCosts;
  std::vector<double> lowerCostBounds;
  std::vector<double> upperCostBounds;

  // merge together the stuff that is computed in the separate statistics
  for (std::size_t i=0; i<numQueries_; ++i){
    // Get the median initial solution cost.
    double medianCost = stats_[i].getMedianFinalCost(plannerName);

    // Get the interval for the upper and lower bounds.
    auto interval = stats_[i].populationStats_.findPercentileConfidenceInterval(0.5, confidence);

    // Get the upper and lower confidence bounds on the median initial solution duration and cost.
    auto lowerCostBound = stats_[i].getNthCosts(stats_[i].results_.at(plannerName), interval.lower, {stats_[i].defaultMedianBinDurations_.back()}).back();
    auto upperCostBound = stats_[i].getNthCosts(stats_[i].results_.at(plannerName), interval.upper, {stats_[i].defaultMedianBinDurations_.back()}).back();

    // save the results
    medianCosts.emplace_back(medianCost);
    lowerCostBounds.emplace_back(lowerCostBound);
    upperCostBounds.emplace_back(upperCostBound);
  }

  std::vector<double> medianCumulativeCost;
  std::vector<double> uciCumulativeCost;
  std::vector<double> lciCumulativeCost;

  // sum median/limits
  for (std::size_t i=0; i<numQueries_; ++i){
    // Get the median initial solution duration.
    double medianCost = medianCosts[i];
    double lcCost = lowerCostBounds[i];
    double ucCost = upperCostBounds[i];

    if (i == 0){
      medianCumulativeCost.push_back(medianCost);
      uciCumulativeCost.push_back(ucCost);
      lciCumulativeCost.push_back(lcCost);
    }
    else{
      medianCumulativeCost.push_back(medianCumulativeCost.back() + medianCost);
      uciCumulativeCost.push_back(uciCumulativeCost.back() + ucCost);
      lciCumulativeCost.push_back(lciCumulativeCost.back() + lcCost);
    }

    if (std::isfinite(uciCumulativeCost.back()) && uciCumulativeCost.back() > maxNonInfCumulativeCost_){
      maxNonInfCumulativeCost_ = uciCumulativeCost.back();
    }

    if (uciCumulativeCost.back() > maxCumulativeCost_){
      maxCumulativeCost_ = uciCumulativeCost.back();
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
  for (std::size_t i=0; i<numQueries_; ++i) {
    filestream << std::setprecision(21) << "," << std::to_string(i+1);
  }

  // this would be nicer by implementing a operator<< for vector<double>
  writeVectorToFile(filestream, "\ncumulative median final solution cost", medianCumulativeCost);
  writeVectorToFile(filestream, "\nlower cumulative final solution cost confidence bound", lciCumulativeCost);
  writeVectorToFile(filestream, "\nupper cumulative final solution cost confidence bound", uciCumulativeCost);

  return filepath;
}

fs::path MultiQueryStatistics::extractFinalSolutionPerQuery(
      const std::string& plannerName) const{
  // Check if the file already exists.
  fs::path filepath = statisticsDirectory_ / (plannerName + "_final_solution_per_query.csv"s);
  if (fs::exists(filepath) && !forceComputation_) {
    return filepath;
  }

  std::vector<std::size_t> queryNumber{};
  std::vector<double> queryCosts{};
  std::vector<double> queryDurations{};

  std::size_t cnt = 0;
  for (const auto &stat: stats_){
    auto durations = stat.getLastSolutionDurations(stat.results_.at(plannerName));
    auto costs = stat.getLastSolutionCosts(stat.results_.at(plannerName));

    for (std::size_t i=0; i<durations.size(); ++i){
      queryDurations.push_back(durations[i]);
      queryCosts.push_back(costs[i]);
      queryNumber.push_back(cnt);
    }

    ++cnt;
  }

  // Write to file.
  std::ofstream filestream(filepath.string());
  if (filestream.fail()) {
    auto msg = "Cannot write finale solution for '"s + plannerName + "' to '"s +
               filepath.string() + "'."s;
    throw std::ios_base::failure(msg);
  }

  filestream << stats_[0].createHeader(
      "Final solutions "s,
      plannerName);

  // this would be nicer by implementing a operator<< for vector<double>
  writeVectorToFile(filestream, "Query number", queryNumber);
  writeVectorToFile(filestream, "\nFinal solution duration", queryDurations);
  writeVectorToFile(filestream, "\nFinal solution cost", queryCosts);

  return filepath;
}

std::experimental::filesystem::path MultiQueryStatistics::extractSuccessPerQuery(
    const std::string& plannerName) const{
  // Check if the file already exists.
  fs::path filepath = statisticsDirectory_ / (plannerName + "_success_rate_per_query.csv"s);
  if (fs::exists(filepath) && !forceComputation_) {
    return filepath;
  }

  const auto maxDuration = stats_[0].getMaxDuration();
  std::vector<double> timeFractions = {.25, .5, .75, 1.};
  std::vector<double> times;

  for (const auto f: timeFractions){
    times.push_back(f*maxDuration);
  }

  std::vector<std::vector<double>> successRate;

  for (const auto t: times){
    std::vector<double> tmp;
    for (std::size_t i=0; i<numQueries_; ++i){
      // Get the median initial solution duration.
      auto durations = stats_[i].getInitialSolutionDurations(stats_[i].results_.at(plannerName));

      unsigned int successfulRuns = 0u;
      for (const auto &duration: durations){
        if (duration < t){
          ++successfulRuns;
        }
      }
      tmp.push_back(successfulRuns / static_cast<double>(durations.size()));
    }
    //std::cout << std::endl;
    successRate.push_back(tmp);
  }

  // Write to file.
  std::ofstream filestream(filepath.string());
  if (filestream.fail()) {
    auto msg = "Cannot write success rate for '"s + plannerName + "' to '"s +
               filepath.string() + "'."s;
    throw std::ios_base::failure(msg);
  }

  filestream << stats_[0].createHeader(
      "Success rate",
      plannerName);

  filestream << "query number";
  for (std::size_t i=0; i<numQueries_; ++i) {
    filestream << std::setprecision(21) << "," << std::to_string(i+1);
  }

  // this would be nicer by implementing a operator<< for vector<double>
  unsigned int cnt = 0;
  for (const auto f: timeFractions){
    writeVectorToFile(filestream, "\nsuccess rate at " + std::to_string(int(100*f)) + " percent", successRate[cnt]);
    ++cnt;
  }

  return filepath;
}

double MultiQueryStatistics::getMaxDuration() const{
  return maxDuration_;
}

double MultiQueryStatistics::getMaxNonInfInitialDuration() const{
  return maxNonInfInitialDuration_;
}

double MultiQueryStatistics::getMaxCumulativeDuration() const{
  return maxCumulativeDuration_;
}

double MultiQueryStatistics::getMaxNonInfCumulativeDuration() const{
  return maxNonInfCumulativeDuration_;
}

double MultiQueryStatistics::getMaxCost() const{
  return maxCost_;
}

double MultiQueryStatistics::getMaxNonInfCost() const{
  return maxNonInfCost_;
}

double MultiQueryStatistics::getMaxCumulativeCost() const{
  return maxCumulativeCost_;
}

double MultiQueryStatistics::getMaxNonInfCumulativeCost() const{
  return maxNonInfCumulativeCost_;
}


double MultiQueryStatistics::getMinCumulativeInitialSolutionCost(const std::string& plannerName) const{
  return minCumulativeInitialSolutionCosts_.at(plannerName);
}
double MultiQueryStatistics::getMedianCumulativeInitialSolutionCost(const std::string& plannerName) const{
  return medianCumulativeInitialSolutionCosts_.at(plannerName);
}
double MultiQueryStatistics::getMaxCumulativeInitialSolutionCost(const std::string& plannerName) const{
  return maxCumulativeInitialSolutionCosts_.at(plannerName);
}

double MultiQueryStatistics::getMinCumulativeInitialSolutionDuration(const std::string& plannerName) const{
  return minCumulativeInitialSolutionDurations_.at(plannerName);

}
double MultiQueryStatistics::getMedianCumulativeInitialSolutionDuration(const std::string& plannerName) const{
  return medianCumulativeInitialSolutionDurations_.at(plannerName);
}

double MultiQueryStatistics::getMaxCumulativeNonInfInitialSolutionDuration(const std::string& plannerName) const{
  return maxCumulativeNonInfInitialSolutionDurations_.at(plannerName);
}
double MultiQueryStatistics::getMaxCumulativeInitialSolutionDuration(const std::string& plannerName) const{
  return maxCumulativeInitialSolutionDurations_.at(plannerName);
}

double MultiQueryStatistics::getSuccessRate(const std::string& plannerName) const{
  return successRates_.at(plannerName);
}
double MultiQueryStatistics::getMinCumulativeFinalCost(const std::string& plannerName) const{
  return minCumulativeFinalCosts_.at(plannerName);
}
double MultiQueryStatistics::getMedianCumulativeFinalCost(const std::string& plannerName) const{
  return medianCumulativeFinalCosts_.at(plannerName);
}
double MultiQueryStatistics::getMaxCumulativeFinalCost(const std::string& plannerName) const{
  return maxCumulativeFinalCosts_.at(plannerName);
}
}  // namespace ompltools

}  // namespace esp
