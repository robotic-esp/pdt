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

Statistics::Statistics(const std::shared_ptr<Configuration>& config, bool forceComputation) :
    config_(config),
    statisticsDirectory_(fs::path(config_->get<std::string>("experiment/results")).parent_path() /
                         "statistics/"),
    forceComputation_(forceComputation) {
  // Create the statistics directory.
  fs::create_directories(statisticsDirectory_);

  // Get the results path.
  fs::path resultsPath = config_->get<std::string>("experiment/results");

  // Open the results file.
  std::ifstream filestream(resultsPath.string());
  if (filestream.fail()) {
    auto msg = "Statistics cannot open results at '"s + resultsPath.string() + "'."s;
    throw std::runtime_error(msg);
  }

  // Set up the parser.
  aria::csv::CsvParser parser(filestream);

  // Parse the file in bulk for now. We could already extract some statistics here, e.g. fastest
  // initial solution time and associated cost for every planner.
  bool timeRow = true;
  double lastCost{std::numeric_limits<double>::infinity()};
  std::string name{""};
  PlannerResults::PlannerResult run;
  for (auto& row : parser) {
    if (row.size() == 0) {
      throw std::runtime_error("Empty row.");
    }
    if (timeRow) {
      name = row.at(0);

      // If this is the first time we're parsing this planner, we need to setup a min max value
      // containers.
      if (results_.find(name) == results_.end() && run.empty()) {
        minCosts_[name] = std::numeric_limits<double>::infinity();
        maxCosts_[name] = std::numeric_limits<double>::lowest();
        minInitialSolutionCosts_[name] = std::numeric_limits<double>::infinity();
        maxInitialSolutionCosts_[name] = std::numeric_limits<double>::lowest();
        minFinalCosts_[name] = std::numeric_limits<double>::infinity();
        maxFinalCosts_[name] = std::numeric_limits<double>::lowest();
        maxNonInfCosts_[name] = std::numeric_limits<double>::lowest();
        minDurations_[name] = std::numeric_limits<double>::infinity();
        maxDurations_[name] = std::numeric_limits<double>::lowest();
        minInitialSolutionDurations_[name] = std::numeric_limits<double>::infinity();
        maxInitialSolutionDurations_[name] = std::numeric_limits<double>::lowest();
        maxNonInfInitialSolutionDurations_[name] = std::numeric_limits<double>::lowest();
      }
    } else if (row.at(0) != name) {
      throw std::runtime_error("Csv file has unexpected structure.");
    }
    // Let's get the min and max values on first pass.
    for (std::size_t i = 1u; i < row.size(); ++i) {
      if (timeRow) {
        double duration = std::stod(row.at(i));
        // Fill the cost row with nan's while filling the time row.
        run.emplace_back(duration, std::numeric_limits<double>::signaling_NaN());

        // Register overall min and max durations.
        if (duration < minDuration_) {
          minDuration_ = duration;
        }
        if (duration > maxDuration_) {
          maxDuration_ = duration;
        }

        // Register planner specific min and max durations.
        if (duration < minDurations_.at(name)) {
          minDurations_.at(name) = duration;
        }
        if (duration > maxDurations_.at(name)) {
          maxDurations_.at(name) = duration;
        }
      } else {
        run.at(i - 1u).second =
            std::stod(row.at(i));  // i - 1 because the row index (i) has the name as
                                   // 0th element which is not stored in the run.

        double duration = run.at(i - 1u).first;
        double cost = run.at(i - 1u).second;

        // Register overall min and max costs.
        if (cost < minCost_) {
          minCost_ = cost;
        }
        if (cost > maxCost_) {
          maxCost_ = cost;
        }
        if (cost != std::numeric_limits<double>::infinity() && cost > maxNonInfCost_) {
          maxNonInfCost_ = cost;
        }

        // Register the overall initial solution durations.
        if (cost != std::numeric_limits<double>::infinity() &&
            duration < minInitialSolutionDuration_) {
          minInitialSolutionDuration_ = duration;
        }
        if (cost != std::numeric_limits<double>::infinity() &&
            lastCost == std::numeric_limits<double>::infinity() &&
            duration > maxNonInfInitialSolutionDuration_) {
          maxNonInfInitialSolutionDuration_ = duration;
        }
        if (i == row.size() - 1u && cost > maxFinalCost_) {
          maxFinalCost_ = cost;
        }
        if (i == row.size() - 1u && cost < minFinalCost_) {
          minFinalCost_ = cost;
        }

        // Register planner specific min and max costs.
        if (cost < minCosts_.at(name)) {
          minCosts_.at(name) = cost;
        }
        if (cost > maxCosts_.at(name)) {
          maxCosts_.at(name) = cost;
        }
        if (cost != std::numeric_limits<double>::infinity() && cost > maxNonInfCosts_.at(name)) {
          maxNonInfCosts_.at(name) = cost;
        }
        if (cost != std::numeric_limits<double>::infinity() &&
            duration < minInitialSolutionDurations_.at(name)) {
          minInitialSolutionDurations_.at(name) = duration;
        }
        if (lastCost == std::numeric_limits<double>::infinity() &&
            cost < minInitialSolutionCosts_.at(name)) {
          minInitialSolutionCosts_.at(name) = cost;
        }
        if (lastCost == std::numeric_limits<double>::infinity() &&
            (cost != std::numeric_limits<double>::infinity() || i == row.size() - 1u) &&
            cost > maxInitialSolutionCosts_.at(name)) {
          maxInitialSolutionCosts_.at(name) = cost;
        }
        if (lastCost == std::numeric_limits<double>::infinity() &&
            (cost != std::numeric_limits<double>::infinity() || i == row.size() - 1u) &&
            duration > maxInitialSolutionDurations_.at(name)) {
          maxInitialSolutionDurations_.at(name) = duration;
        }
        if (cost != std::numeric_limits<double>::infinity() &&
            lastCost == std::numeric_limits<double>::infinity() &&
            duration > maxNonInfInitialSolutionDurations_.at(name)) {
          maxNonInfInitialSolutionDurations_.at(name) = duration;
        }
        if (i == row.size() - 1u && cost > maxFinalCosts_.at(name)) {
          maxFinalCosts_.at(name) = cost;
        }
        if (i == row.size() - 1u && cost < minFinalCosts_.at(name)) {
          minFinalCosts_.at(name) = cost;
        }

        // Remember this cost (for max initial solution durations).
        lastCost = cost;
      }
    }
    if (!timeRow) {
      results_[name].addMeasuredRun(run);
      run.clear();
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

  // Compute the default binning durations for the medians.
  auto contextName = config_->get<std::string>("experiment/context");
  std::size_t numMeasurements =
      std::ceil(config_->get<double>("context/" + contextName + "/maxTime") *
                config_->get<double>("experiment/logFrequency"));
  double medianBinSize = 1.0 / config_->get<double>("experiment/logFrequency");
  defaultMedianBinDurations_.reserve(numMeasurements);
  for (std::size_t i = 0u; i < numMeasurements; ++i) {
    defaultMedianBinDurations_.emplace_back(static_cast<double>(i + 1u) * medianBinSize);
  }

  // Compute the default binning durations for the initial solution pdf.
  double initDurationNumBins =
      config_->get<std::size_t>("statistics/initialSolutions/numDurationBins");
  double minExp = std::log10(minInitialSolutionDuration_);
  double maxExp = std::log10(maxNonInfInitialSolutionDuration_);
  double binExpStep = (maxExp - minExp) / initDurationNumBins;
  for (std::size_t i = 0u; i < initDurationNumBins; ++i) {
    defaultInitialSolutionBinDurations_.emplace_back(
        std::pow(10.0, minExp + static_cast<double>(i) * binExpStep));
  }
}

fs::path Statistics::extractMedians(const std::string& plannerName, std::size_t confidence,
                                    const std::vector<double>& binDurations) const {
  if (!config_->get<bool>("planner/"s + plannerName + "/isAnytime"s)) {
    auto msg = "This method extracts median costs over time for anytime planners. '" + plannerName +
               "' is not an anytime planner."s;
    throw std::runtime_error(msg);
  }

  if (results_.find(plannerName) == results_.end()) {
    auto msg =
        "Cannot find results for '" + plannerName + "' and can therefore not extract medians."s;
    throw std::runtime_error(msg);
  }

  // Check if the file already exists.
  fs::path filepath = statisticsDirectory_ / (plannerName + "_medians.csv"s);
  if (fs::exists(filepath) && !forceComputation_) {
    return filepath;
  }

  // Get the requested bin durations.
  const auto& durations = binDurations.empty() ? defaultMedianBinDurations_ : binDurations;

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

fs::path Statistics::extractCostPercentiles(const std::string& plannerName,
                                            std::set<double> percentiles,
                                            const std::vector<double>& binDurations) const {
  if (!config_->get<bool>("planner/"s + plannerName + "/isAnytime"s)) {
    auto msg = "This method extracts cost percentiles over time for anytime planners. '" +
               plannerName + "' is not an anytime planner."s;
    throw std::runtime_error(msg);
  }

  if (results_.find(plannerName) == results_.end()) {
    auto msg = "Cannot find results for '" + plannerName +
               "' and can therefore not extract cost percentiles."s;
    throw std::runtime_error(msg);
  }

  // Check if the file already exists.
  fs::path filepath = statisticsDirectory_ / (plannerName + "_cost_percentiles.csv"s);
  if (fs::exists(filepath) && !forceComputation_) {
    return filepath;
  }

  // Get the requested bin durations.
  const auto& durations = binDurations.empty() ? defaultMedianBinDurations_ : binDurations;

  // Get the percentile costs. If percentile < 0.5 flooring is conservative, if percentile > 0.5,
  // ceiling is conservative. If percentile == 0.5, we compute the median propperly.
  std::map<double, std::vector<double>> percentileCosts{};
  for (const auto percentile : percentiles) {
    std::vector<double> costs{};
    if (percentile < 0.5) {
      costs = getNthCosts(results_.at(plannerName), std::floor(percentile * numRunsPerPlanner_),
                          durations);
    } else if (percentile > 0.5) {
      costs = getNthCosts(results_.at(plannerName), std::ceil(percentile * numRunsPerPlanner_) - 1u,
                          durations);
    } else {
      costs = getMedianCosts(results_.at(plannerName), durations);
    }
    percentileCosts[percentile] = costs;
  }

  // Write to file.
  std::ofstream filestream(filepath.string());
  if (filestream.fail()) {
    auto msg =
        "Cannot write percentiles for '"s + plannerName + "' to '"s + filepath.string() + "'."s;
    throw std::ios_base::failure(msg);
  }

  filestream << createHeader("Binned percentiles", plannerName);
  filestream << std::setprecision(21);
  filestream << "durations";
  for (const auto duration : durations) {
    filestream << ',' << duration;
  }
  for (const auto& [percentile, costs] : percentileCosts) {
    filestream << std::setprecision(3) << "\npercentile" << percentile << std::setprecision(21);
    for (const auto cost : costs) {
      filestream << ',' << cost;
    }
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

  // Check if the file already exists.
  fs::path filepath = statisticsDirectory_ / (plannerName + "_median_initial_solution.csv"s);
  if (fs::exists(filepath) && !forceComputation_) {
    return filepath;
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

  // Check if the file already exists.
  fs::path filepath = statisticsDirectory_ / (plannerName + "_initial_solution_durations_cdf.csv"s);
  if (fs::exists(filepath) && !forceComputation_) {
    return filepath;
  }

  // Get the initial solution durations.
  auto initialSolutionDurations = getInitialSolutionDurations(results_.at(plannerName));

  // Sort them.
  std::sort(initialSolutionDurations.begin(), initialSolutionDurations.end());

  // Prepare variable to calculate solution percentages.
  std::size_t numSolvedRuns = 0;

  // Write to file.
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

fs::path Statistics::extractInitialSolutionDurationPdf(
    const std::string& plannerName, const std::vector<double>& binDurations) const {
  if (results_.find(plannerName) == results_.end()) {
    auto msg = "Cannot find results for '" + plannerName +
               "' and can therefore not extract initial solution duration pdf."s;
    throw std::runtime_error(msg);
  }

  // Check if the file already exists.
  fs::path filepath = statisticsDirectory_ / (plannerName + "_initial_solution_durations_pdf.csv"s);
  if (fs::exists(filepath) && !forceComputation_) {
    return filepath;
  }

  // We'll take the bin durations to bt the start of the bins.
  const auto& bins = binDurations.empty() ? defaultInitialSolutionBinDurations_ : binDurations;

  // Get the initial solution durations.
  auto initialSolutionDurations = getInitialSolutionDurations(results_.at(plannerName));

  // Count how many durations fall in each bin.
  std::vector<std::size_t> binCounts(bins.size(), 0u);
  for (auto duration : initialSolutionDurations) {
    // std::lower_bound returns an iterator to the first element that is greater or equal to, or
    // end.
    auto lower = std::lower_bound(bins.begin(), bins.end(), duration);
    if (lower == bins.end()) {
      continue;
    }
    if (lower != bins.begin()) {
      --lower;
    }
    binCounts.at(std::distance(bins.begin(), lower))++;
  }

  // Write to file.
  std::ofstream filestream(filepath.string());
  if (filestream.fail()) {
    auto msg = "Cannot write initial solution duration cdf for '"s + plannerName + "' to '"s +
               filepath.string() + "'."s;
    throw std::ios_base::failure(msg);
  }

  filestream << createHeader("Initial solution duration pdf", plannerName);
  filestream << std::setprecision(21);
  filestream << "bin begin durations";
  for (const auto duration : bins) {
    filestream << ',' << duration;
  }
  filestream << "\nbin counts";
  for (const auto count : binCounts) {
    filestream << ',' << count;
  }
  filestream << '\n';

  return filepath;  // Note: std::ofstream is a big boy and closes itself upon destruction.
}

fs::path Statistics::extractInitialSolutions(const std::string& plannerName) const {
  // Check if the file already exists.
  fs::path filepath = statisticsDirectory_ / (plannerName + "_initial_solutions.csv"s);
  if (fs::exists(filepath) && !forceComputation_) {
    return filepath;
  }

  auto durations = getInitialSolutionDurations(results_.at(plannerName));
  auto costs = getInitialSolutionCosts(results_.at(plannerName));

  // Write to file.
  std::ofstream filestream(filepath.string());
  if (filestream.fail()) {
    auto msg = "Cannot write initial solutions for '"s + plannerName + "' to '"s +
               filepath.string() + "'."s;
    throw std::ios_base::failure(msg);
  }

  filestream << createHeader("Initial solutions", plannerName);
  filestream << std::setprecision(21);
  filestream << "durations";
  for (const auto duration : durations) {
    filestream << ',' << duration;
  }
  filestream << "\ncosts";
  for (const auto cost : costs) {
    filestream << ',' << cost;
  }
  filestream << '\n';

  return filepath;  // Note: std::ofstream is a big boy and closes itself upon destruction.
}

std::string Statistics::createHeader(const std::string& statisticType,
                                     const std::string& plannerName) const {
  std::stringstream stream;
  stream << "# Experiment: " << config_->get<std::string>("experiment/name") << '\n';
  stream << "# Planner: " << plannerName << '\n';
  stream << "# Statistic: " << statisticType << '\n';
  return stream.str();
}

Statistics::ConfidenceInterval Statistics::getMedianConfidenceInterval(
    std::size_t confidence) const {
  // The lower and upper bounds in the below are off-by-one, because they are used as zero-based
  // indices. See comment in Statistics::ConfidenceInterval class.
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

double Statistics::getMinInitialSolutionDuration() const {
  return minInitialSolutionDuration_;
}

double Statistics::getMaxNonInfInitialSolutionDuration() const {
  return maxNonInfInitialSolutionDuration_;
}

double Statistics::getMinCost(const std::string& plannerName) const {
  return minCosts_.at(plannerName);
}

double Statistics::getMaxCost(const std::string& plannerName) const {
  return maxCosts_.at(plannerName);
}

double Statistics::getMinInitialSolutionCost(const std::string& plannerName) const {
  return minInitialSolutionCosts_.at(plannerName);
}

double Statistics::getMaxInitialSolutionCost(const std::string& plannerName) const {
  return maxInitialSolutionCosts_.at(plannerName);
}

double Statistics::getMinFinalCost(const std::string& plannerName) const {
  return minFinalCosts_.at(plannerName);
}

double Statistics::getMaxFinalCost(const std::string& plannerName) const {
  return maxFinalCosts_.at(plannerName);
}

double Statistics::getMedianFinalCost(const std::string& plannerName) const {
  return getMedianCosts(results_.at(plannerName), defaultMedianBinDurations_).back();
}

double Statistics::getMaxNonInfCost(const std::string& plannerName) const {
  return maxNonInfCosts_.at(plannerName);
}

double Statistics::getMinDuration(const std::string& plannerName) const {
  return minDurations_.at(plannerName);
}

double Statistics::getMaxDuration(const std::string& plannerName) const {
  return maxDurations_.at(plannerName);
}

double Statistics::getMinInitialSolutionDuration(const std::string& plannerName) const {
  return minInitialSolutionDurations_.at(plannerName);
}

double Statistics::getMaxInitialSolutionDuration(const std::string& plannerName) const {
  return maxInitialSolutionDurations_.at(plannerName);
}

double Statistics::getMaxNonInfInitialSolutionDuration(const std::string& plannerName) const {
  return maxNonInfInitialSolutionDurations_.at(plannerName);
}

double Statistics::getMedianInitialSolutionDuration(const std::string& plannerName) const {
  return getMedianInitialSolutionDuration(results_.at(plannerName));
}

double Statistics::getMedianInitialSolutionCost(const std::string& plannerName) const {
  return getMedianInitialSolutionCost(results_.at(plannerName));
}

std::vector<double> Statistics::getDefaultBinDurations() const {
  return defaultMedianBinDurations_;
}

std::shared_ptr<Configuration> Statistics::getConfig() const {
  return config_;
}

std::vector<double> Statistics::getMedianCosts(const PlannerResults& results,
                                               const std::vector<double>& durations) const {
  std::vector<double> medianCosts;
  medianCosts.resize(durations.size(), std::numeric_limits<double>::signaling_NaN());
  if (numRunsPerPlanner_ % 2 == 1) {
    medianCosts = getNthCosts(results, (numRunsPerPlanner_ - 1u) / 2, durations);
  } else {
    std::vector<double> lowMedianCosts =
        getNthCosts(results, (numRunsPerPlanner_ - 2u) / 2, durations);
    std::vector<double> highMedianCosts = getNthCosts(results, numRunsPerPlanner_ / 2, durations);
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

double Statistics::getNthInitialSolutionDuration(const PlannerResults& results,
                                                 std::size_t n) const {
  // Get the durations of the initial solutions of all runs.
  auto initialDurations = getInitialSolutionDurations(results);

  // Get the nth element of this collection of durations.
  auto nthDuration = initialDurations.begin() + n;
  std::nth_element(initialDurations.begin(), nthDuration, initialDurations.end());

  return *nthDuration;
}

double Statistics::getNthInitialSolutionCost(const PlannerResults& result, std::size_t n) const {
  // Get the costs of the initial solutions of all runs.
  auto initialCosts = getInitialSolutionCosts(result);

  // Get the nth element of this collection of costs.
  auto nthCost = initialCosts.begin() + n;
  std::nth_element(initialCosts.begin(), nthCost, initialCosts.end());

  return *nthCost;
}

}  // namespace ompltools

}  // namespace esp
