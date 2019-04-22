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
#include <iomanip>
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
namespace fs = std::experimental::filesystem;

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
    resultsPath_(config_->get<std::string>("Experiment/results")),
    statisticsDirectory_(resultsPath_.parent_path() / "statistics/") {
  // Create the statistics directory.
  fs::create_directories(statisticsDirectory_);

  // Open the results file.
  std::ifstream filestream(resultsPath_.string());
  if (filestream.fail()) {
    auto msg = "Statistics cannot open results at '"s + resultsPath_.string() + "'."s;
    throw std::runtime_error(msg);
  }

  // Set up the parser.
  aria::csv::CsvParser parser(filestream);

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
      results_[name].addMeasuredRun(run);
    }
    timeRow = !timeRow;
  }

  // Get the number of runs per planner, check that they're equal.
  if (results_.empty()) {
    numRunsPerPlanner_ = 0u;
  } else {
    // Make sure all planners have the same amount of runs.
    for (auto it = ++results_.begin(); it != results_.end(); ++it) {
      if ((--it)->second.numMeasuredRuns() != (++it)->second.numMeasuredRuns()) {
        auto msg = "Not all planners have the same amount of runs."s;
        throw std::runtime_error(msg);
      }
    }
    numRunsPerPlanner_ = results_.begin()->second.numMeasuredRuns();
  }

  // Compute the default binning durations.
  auto contextName = config_->get<std::string>("Experiment/context");
  std::size_t numMeasurements =
      std::ceil(config_->get<double>("Contexts/" + contextName + "/maxTime") *
                config_->get<double>("Experiment/logFrequency"));
  double binSize = 1.0 / config_->get<double>("Experiment/logFrequency");
  defaultBinDurations_.reserve(numMeasurements);
  for (std::size_t i = 0u; i < numMeasurements; ++i) {
    defaultBinDurations_.emplace_back(static_cast<double>(i + 1u) * binSize);
  }
}

fs::path Statistics::extractMedians(const std::string& plannerName, std::size_t confidence,
                                    const std::vector<double>& binDurations) const {
  if (plannerName == "RRTConnect") {
    auto msg = "This method extracts median costs over time for anytime planners. '" + plannerName +
               "' is not an anytime planner."s;
    throw std::runtime_error(msg);
  }

  if (results_.find(plannerName) == results_.end()) {
    auto msg =
        "Cannot find results for '" + plannerName + "' and can therefore not extract medians."s;
    throw std::runtime_error(msg);
  }

  // Get the requested bin durations.
  const auto& durations = binDurations.empty() ? defaultBinDurations_ : binDurations;

  // Get the median costs.
  auto medianCosts = getMedianCosts(results_.at(plannerName), durations);

  // Get the interval indices.
  auto interval = getMedianConfidenceInterval(confidence);

  // Get the interval bound costs.
  auto lowerCosts = getNthCosts(results_.at(plannerName), interval.lower, durations);
  auto upperCosts = getNthCosts(results_.at(plannerName), interval.upper, durations);

  // We need to clean up these costs. If the median is infinite, the lower and upper bounds should
  // be nan.
  for (std::size_t i = 0u; i < medianCosts.size(); ++i) {
    if (medianCosts.at(i) == std::numeric_limits<double>::infinity()) {
      lowerCosts.at(i) = std::numeric_limits<double>::quiet_NaN();
      upperCosts.at(i) = std::numeric_limits<double>::quiet_NaN();
    } else {
      break;  // Once the first median cost is not infinity, none of the following will be.
    }
  }

  // Write to file.
  fs::path filepath = statisticsDirectory_ / (config_->get<std::string>("Experiment/name") + '_' +
                                              plannerName + "_medians.csv");
  std::ofstream filestream(filepath.string());
  if (filestream.fail()) {
    auto msg = "Cannot write medians for '"s + plannerName + "' to '"s + filepath.string() + "'."s;
    throw std::ios_base::failure(msg);
  }

  filestream << createHeader("Median with "s + std::to_string(confidence) + "% confidence bounds"s,
                             plannerName);
  filestream << std::setprecision(21);
  filestream << "durations";
  for (const auto duration : durations) {
    filestream << ',' << duration;
  }
  filestream << "\nmedian costs";
  for (const auto cost : medianCosts) {
    filestream << ',' << cost;
  }
  filestream << "\nlower confidence bound";
  for (const auto cost : lowerCosts) {
    filestream << ',' << cost;
  }
  filestream << "\nupper confidence bound";
  for (const auto cost : upperCosts) {
    filestream << ',' << cost;
  }
  filestream << '\n';

  return filepath;  // Note: std::ofstream closes itself upon destruction.
}

fs::path Statistics::extractMedianInitialSolution(const std::string& plannerName,
                                                  std::size_t confidence) const {
  if (results_.find(plannerName) == results_.end()) {
    auto msg = "Cannot find results for '" + plannerName +
               "' and can therefore not extract median initial solution."s;
    throw std::runtime_error(msg);
  }

  // Get the median initial solution duration.
  double medianDuration = getMedianInitialSolutionDuration(results_.at(plannerName));

  // Get the median initial solution cost.
  double medianCost = getMedianInitialSolutionCost(results_.at(plannerName));

  // Get the interval for the upper and lower bounds.
  auto interval = getMedianConfidenceInterval(confidence);

  // Get the upper and lower confidence bounds on the median initial solution duration and cost.
  auto lowerDurationBound = getNthInitialSolutionDuration(results_.at(plannerName), interval.lower);
  auto upperDurationBound = getNthInitialSolutionDuration(results_.at(plannerName), interval.upper);
  auto lowerCostBound = getNthInitialSolutionCost(results_.at(plannerName), interval.lower);
  auto upperCostBound = getNthInitialSolutionCost(results_.at(plannerName), interval.upper);

  // Write to file.
  fs::path filepath = statisticsDirectory_ / (config_->get<std::string>("Experiment/name") + '_' +
                                              plannerName + "_median_initial_solution.csv");
  std::ofstream filestream(filepath.string());
  if (filestream.fail()) {
    auto msg = "Cannot write median initial solution for '"s + plannerName + "' to '"s +
               filepath.string() + "'."s;
    throw std::ios_base::failure(msg);
  }

  filestream << createHeader(
      "Median initial solution with "s + std::to_string(confidence) + "% confidence bounds"s,
      plannerName);
  filestream << std::setprecision(21) << "median initial solution duration," << medianDuration
             << "\nlower initial solution duration confidence bound," << lowerDurationBound
             << "\nupper initial solution duration confidence bound," << upperDurationBound
             << "\nmedian initial solution cost," << medianCost
             << "\nlower initial solution cost confidence bound," << lowerCostBound
             << "\nupper initial solution cost confidence bound," << upperCostBound << '\n';

  return filepath;  // Note: std::ofstream closes itself upon destruction.
}

fs::path Statistics::extractInitialSolutionDurationCdf(const std::string& plannerName) const {
  if (results_.find(plannerName) == results_.end()) {
    auto msg = "Cannot find results for '" + plannerName +
               "' and can therefore not extract initial solution duration cdf."s;
    throw std::runtime_error(msg);
  }

  // Get the initial solution durations.
  auto initialSolutionDurations = getInitialSolutionDurations(plannerName);

  // Sort them.
  std::sort(initialSolutionDurations.begin(), initialSolutionDurations.end());

  // Prepare variable to calculate solution percentages.
  std::size_t numSolvedRuns = 0;

  // Write to file.
  fs::path filepath = statisticsDirectory_ / (config_->get<std::string>("Experiment/name") + '_' +
                                              plannerName + "_initial_solution_durations_cdf.csv");
  std::ofstream filestream(filepath.string());
  if (filestream.fail()) {
    auto msg = "Cannot write initial solution duration cdf for '"s + plannerName + "' to '"s +
               filepath.string() + "'."s;
    throw std::ios_base::failure(msg);
  }

  filestream << createHeader("Initial solution duration cdf", plannerName);
  filestream << std::setprecision(21);
  filestream << "durations, 0.0";
  for (const auto duration : initialSolutionDurations) {
    filestream << ',' << duration;
  }
  filestream << "\ncdf, 0.0";
  for (std::size_t i = 0u; i < initialSolutionDurations.size(); ++i) {
    filestream << ','
               << static_cast<double>(++numSolvedRuns) / static_cast<double>(numRunsPerPlanner_);
  }
  filestream << '\n';

  return filepath;  // Note: std::ofstream is a big boy and closes itself upon destruction.
}

std::string Statistics::createHeader(const std::string& statisticType,
                                     const std::string& plannerName) const {
  std::stringstream stream;
  stream << "# Experiment: " << config_->get<std::string>("Experiment/name") << '\n';
  stream << "# Planner: " << plannerName << '\n';
  stream << "# Statistic: " << statisticType << '\n';
  return stream.str();
}

Statistics::ConfidenceInterval Statistics::getMedianConfidenceInterval(
    std::size_t confidence) const {
  static const std::map<std::size_t, std::map<std::size_t, ConfidenceInterval>>
      medianConfidenceIntervals = {
          {10u, {{95u, {1u, 8u, 0.9511}}, {99u, {0u, 9u, 0.9910}}}},
          {50u, {{95u, {18u, 32u, 0.9511}}, {99u, {15u, 34u, 0.9910}}}},
          {100u, {{95u, {40u, 60u, 0.9540}}, {99u, {37u, 63u, 0.9907}}}},
          {200u, {{95u, {86u, 114u, 0.9520}}, {99u, {81u, 118u, 0.9906}}}},
          {250u, {{95u, {110u, 141u, 0.9503}}, {99u, {104u, 145u, 0.9900}}}},
          {300u, {{95u, {133u, 167u, 0.9502}}, {99u, {127u, 172u, 0.9903}}}},
          {400u, {{95u, {179u, 219u, 0.9522}}, {99u, {174u, 226u, 0.9907}}}},
          {500u, {{95u, {228u, 272u, 0.9508}}, {99u, {221u, 279u, 0.9905}}}},
          {600u, {{95u, {274u, 323u, 0.9508}}, {99u, {267u, 331u, 0.9907}}}},
          {700u, {{95u, {324u, 376u, 0.9517}}, {99u, {314u, 383u, 0.9901}}}},
          {800u, {{95u, {371u, 427u, 0.9511}}, {99u, {363u, 436u, 0.9900}}}},
          {900u, {{95u, {420u, 479u, 0.9503}}, {99u, {410u, 488u, 0.9904}}}},
          {1000u, {{95u, {469u, 531u, 0.9500}}, {99u, {458u, 531u, 0.9905}}}},
          {2000u, {{95u, {955u, 1043u, 0.9504}}, {99u, {940u, 1056u, 0.9901}}}},
          {5000u, {{95u, {2429u, 2568u, 0.9503}}, {99u, {2406u, 2589u, 0.9901}}}},
          {10000u, {{95u, {4897u, 5094u, 0.9500}}, {99u, {4869u, 5127u, 0.9900}}}},
          {100000u, {{95u, {49687u, 50307u, 0.9500}}, {99u, {49588u, 50403u, 0.9900}}}},
          {1000000u, {{95u, {499018u, 500978u, 0.9500}}, {99u, {498707u, 501283u, 0.9900}}}}};
  if (medianConfidenceIntervals.find(numRunsPerPlanner_) == medianConfidenceIntervals.end()) {
    auto msg = "No precomputed values for the median confidence interval with "s +
               std::to_string(numRunsPerPlanner_) + " runs. The precomputed values are:\n"s;
    for (const auto& entry : medianConfidenceIntervals) {
      msg += std::to_string(entry.first) + '\n';
    }
    throw std::runtime_error(msg);
  }
  if (confidence != 95u && confidence != 99u) {
    auto msg =
        "Invalid confidence, only know confidence intervals for 95 and 99 percent confidence."s;
    throw std::runtime_error(msg);
  }
  return medianConfidenceIntervals.at(numRunsPerPlanner_).at(confidence);
}

std::vector<std::string> Statistics::getPlannerNames() const {
  return plannerNames_;
}

std::size_t Statistics::getNumRunsPerPlanner() const {
  if (results_.empty()) {
    return 0u;
  }
  // Make sure all planners have the same amount of runs.
  for (auto it = ++results_.begin(); it != results_.end(); ++it) {
    if ((--it)->second.numMeasuredRuns() != (++it)->second.numMeasuredRuns()) {
      auto msg = "Not all planners have the same amount of runs."s;
      throw std::runtime_error(msg);
    }
  }
  return results_.begin()->second.numMeasuredRuns();
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

std::vector<double> Statistics::getMedianCosts(const PlannerResults& results,
                                               const std::vector<double>& durations) const {
  std::vector<double> medianCosts;
  medianCosts.resize(durations.size(), std::numeric_limits<double>::signaling_NaN());
  if (numRunsPerPlanner_ % 2 == 1) {
    medianCosts = getNthCosts(results, (numRunsPerPlanner_ - 1u) / 2, durations);
  } else {
    std::vector<double> lowMedianCosts = getNthCosts(results, numRunsPerPlanner_ / 2, durations);
    std::vector<double> highMedianCosts =
        getNthCosts(results, (numRunsPerPlanner_ + 2u) / 2, durations);
    for (std::size_t i = 0u; i < medianCosts.size(); ++i) {
      medianCosts.at(i) = (lowMedianCosts.at(i) + highMedianCosts.at(i)) / 2.0;
    }
  }
  return medianCosts;
}

double Statistics::getMedianInitialSolutionDuration(const PlannerResults& results) const {
  if (numRunsPerPlanner_ % 2 == 1) {
    return getNthInitialSolutionDuration(results, (numRunsPerPlanner_ - 1u) / 2);
  } else {
    double lowerMedianDuration =
        getNthInitialSolutionDuration(results, (numRunsPerPlanner_ - 2u) / 2);
    double upperMedianDuration = getNthInitialSolutionDuration(results, numRunsPerPlanner_ / 2);
    return (lowerMedianDuration + upperMedianDuration) / 2.0;
  }
}

double Statistics::getMedianInitialSolutionCost(const PlannerResults& results) const {
  if (numRunsPerPlanner_ % 2 == 1) {
    return getNthInitialSolutionCost(results, (numRunsPerPlanner_ - 1u) / 2);
  } else {
    double lowerMedianCost = getNthInitialSolutionCost(results, (numRunsPerPlanner_ - 2u) / 2);
    double upperMedianCost = getNthInitialSolutionCost(results, numRunsPerPlanner_ / 2);
    return (lowerMedianCost + upperMedianCost) / 2.0;
  }
}

std::vector<double> Statistics::getNthCosts(const PlannerResults& results, std::size_t n,
                                            const std::vector<double>& durations) const {
  if (durations.empty()) {
    auto msg = "Expected at least one duration."s;
    throw std::runtime_error(msg);
  }
  std::vector<double> nthCosts;
  nthCosts.reserve(durations.size());
  const auto& interpolatedRuns = results.getAllRunsAt(durations);
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

std::vector<double> Statistics::getNthCosts(const std::string& name, std::size_t n,
                                            const std::vector<double>& durations) const {
  if (name == "RRTConnect"s) {
    auto msg = "Cannot specify durations for planners with nonanytime behaviour."s;
    throw std::runtime_error(msg);
  }
  if (results_.find(name) == results_.end()) {
    auto msg = "Cannot find results of '"s + name + "' and can therefore not get nth costs."s;
    throw std::runtime_error(msg);
  }
  return getNthCosts(results_.at(name), n, durations);
}

std::vector<double> Statistics::getInitialSolutionDurations(const PlannerResults& results) const {
  // Get the durations of the initial solutions of all runs.
  std::vector<double> initialDurations{};
  initialDurations.reserve(results.numMeasuredRuns());
  for (std::size_t run = 0u; run < results.numMeasuredRuns(); ++run) {
    // Get the durations and costs of this run.
    const auto& measuredRun = results.getMeasuredRun(run);

    // Find the first cost that's less than infinity.
    for (const auto& measurement : measuredRun) {
      if (measurement.second < std::numeric_limits<double>::infinity()) {
        initialDurations.emplace_back(measurement.first);
        break;
      }
    }

    // If all costs are infinity, there is no initial solution.
    if (initialDurations.size() == run) {
      initialDurations.emplace_back(std::numeric_limits<double>::infinity());
    }
  }

  return initialDurations;
}

std::vector<double> Statistics::getInitialSolutionDurations(const std::string& name) const {
  if (results_.find(name) == results_.end()) {
    auto msg = "Cannot find statistics for planner '"s + name + "'."s;
    throw std::runtime_error(msg);
  }

  return getInitialSolutionDurations(results_.at(name));
}

std::vector<double> Statistics::getInitialSolutionCosts(const PlannerResults& results) const {
  // Get the costs of the initial solutions of all runs.
  std::vector<double> initialCosts{};
  initialCosts.reserve(results.numMeasuredRuns());
  for (std::size_t run = 0u; run < results.numMeasuredRuns(); ++run) {
    // Get the durations and costs of this run.
    const auto& measuredRun = results.getMeasuredRun(run);

    // Find the first cost that's less than infinity.
    for (const auto& measurement : measuredRun) {
      if (measurement.second < std::numeric_limits<double>::infinity()) {
        initialCosts.emplace_back(measurement.second);
        break;
      }
    }

    // If none was less than infinity, the initial solution is infinity...?
    if (initialCosts.size() == run) {
      initialCosts.emplace_back(std::numeric_limits<double>::infinity());
    }
  }

  return initialCosts;
}

std::vector<double> Statistics::getInitialSolutionCosts(const std::string& name) const {
  if (results_.find(name) == results_.end()) {
    auto msg = "Cannot find statistics for planner '"s + name + "'."s;
    throw std::runtime_error(msg);
  }

  return getInitialSolutionCosts(results_.at(name));
}

double Statistics::getNthInitialSolutionDuration(const PlannerResults& results,
                                                 std::size_t n) const {
  // Get the durations of the initial solutions of all runs.
  auto initialDurations = getInitialSolutionDurations(results);

  // Get the nth element of this collection of durations.
  auto nthDuration = initialDurations.begin() + n;
  std::nth_element(initialDurations.begin(), nthDuration, initialDurations.end());

  return *nthDuration;
}

double Statistics::getNthInitialSolutionDuration(const std::string& name, std::size_t n) const {
  if (results_.find(name) == results_.end()) {
    auto msg = "Cannot find statistics for planner '"s + name + "'."s;
    throw std::runtime_error(msg);
  }

  return getNthInitialSolutionDuration(results_.at(name), n);
}

double Statistics::getNthInitialSolutionCost(const PlannerResults& result, std::size_t n) const {
  // Get the costs of the initial solutions of all runs.
  auto initialCosts = getInitialSolutionCosts(result);

  // Get the nth element of this collection of costs.
  auto nthCost = initialCosts.begin() + n;
  std::nth_element(initialCosts.begin(), nthCost, initialCosts.end());

  return *nthCost;
}

double Statistics::getNthInitialSolutionCost(const std::string& name, std::size_t n) const {
  if (results_.find(name) == results_.end()) {
    auto msg = "Cannot find statistics for planner '"s + name + "'."s;
    throw std::runtime_error(msg);
  }

  return getNthInitialSolutionCost(results_.at(name), n);
}

}  // namespace ompltools

}  // namespace esp
