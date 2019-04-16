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

#include "esp_performance_loggers/accumulating_loggers.h"

#include <experimental/filesystem>
#include <fstream>

#include <ompl/util/Console.h>

namespace esp {

namespace ompltools {

namespace {

// Let's try to make sure the vectors have enough memory allocated.
constexpr double ALLOC_SAFETY_FACTOR = 3.0;

}  // namespace

// Convenience namespace.
namespace fs = std::experimental::filesystem;

AccumulatingCostLog::AccumulatingCostLog(const std::vector<std::string>& plannerNames,
                                         const esp::ompltools::time::Duration& maxDuration,
                                         double logFrequency) :
    plannerNames_(plannerNames),
    allocSize_(std::ceil(ALLOC_SAFETY_FACTOR * logFrequency * maxDuration.count())),
    durationBinSize_(1.0 / logFrequency),
    numDurationBins_(std::ceil(maxDuration.count() / durationBinSize_)) {
  boost::array<float, 5> quantiles = {0.01, 0.25, 0.5, 0.75, 0.99};
  for (const auto& name : plannerNames) {
    // Create the accumulators.
    for (std::size_t i = 0; i < numDurationBins_; ++i) {
      accumulators_[name][i] = std::make_shared<AccumulatorSet>(
          boost::accumulators::tag::extended_p_square::probabilities = quantiles);
    }
    // Allocate the duration vectors.
    currentDurations_[name].reserve(allocSize_);

    // Allocate the cost vectors.
    currentCosts_[name].reserve(allocSize_);
  }
}

void AccumulatingCostLog::createLogFile(const std::string& fullFilename) {
  filename_ = fullFilename;

  // Create parent directories, if needed.
  fs::create_directories(fs::path(filename_).parent_path());

  // Create the file.
  std::ofstream csvFile;
  csvFile.open(filename_, std::ofstream::out | std::ofstream::app);

  // Check on the failbit.
  if (csvFile.fail() == true) {
    throw std::ios_base::failure("Could not open log file.");
  }

  // Close the file.
  csvFile.close();

  // Set the permissions to read only.
  fs::permissions(filename_,
                  fs::perms::owner_read | fs::perms::group_read | fs::perms::others_read);
}

void AccumulatingCostLog::addMeasurement(const std::string& plannerName,
                                         const time::Duration& duration,
                                         const ompl::base::Cost& cost) {
  // Record the duration.
  currentDurations_[plannerName].emplace_back(time::seconds(duration));

  // Record the cost.
  currentCosts_[plannerName].emplace_back(cost.value());
}

void AccumulatingCostLog::processMeasurements() {
  // Interpolate this run.
  for (const auto& name : plannerNames_) {
    std::size_t idx = 0u;
    for (std::size_t i = 0u; i < numDurationBins_; ++i) {
      double binDuration = ((i * durationBinSize_) + ((i + 1) * durationBinSize_)) / 2.0;
      if (binDuration < currentDurations_.at(name).at(idx)) {
        accumulators_.at(name).at(i)->operator()(std::numeric_limits<double>::infinity());
      } else {
        while (currentDurations_.at(name).at(idx + 1u) < binDuration) {
          ++idx;
        }
        double binCost = -1.0;
        if (currentCosts_.at(name).at(idx) == currentCosts_.at(name).at(idx + 1u)) {
          binCost = currentCosts_.at(name).at(idx);
        } else if (currentCosts_.at(name).at(idx) == std::numeric_limits<double>::infinity() ||
                   currentCosts_.at(name).at(idx + 1u) == std::numeric_limits<double>::infinity()) {
          binCost = std::numeric_limits<double>::infinity();
        } else {
          binCost =
              currentCosts_.at(name).at(idx) +
              (binDuration - currentDurations_.at(name).at(idx)) *
                  (currentCosts_.at(name).at(idx + 1u) - currentCosts_.at(name).at(idx)) /
                  (currentDurations_.at(name).at(idx + 1u) - currentDurations_.at(name).at(idx));
        }
        accumulators_.at(name).at(i)->operator()(binCost);
      }
    }
  }

  // Make sure we can write to the output file.
  fs::permissions(filename_, fs::perms::owner_read | fs::perms::owner_write |
                                 fs::perms::group_read | fs::perms::others_read);

  // Open the file.
  std::ofstream csvFile;
  csvFile.open(filename_, std::ofstream::out | std::ofstream::app);

  // Check on the failbit.
  if (csvFile.fail() == true) {
    throw std::ios_base::failure("Could not open log file.");
  }

  // Set the precision.
  csvFile << std::setprecision(21);

  // Write the data.
  for (const auto& name : plannerNames_) {
    // First dump the duration measurements.
    csvFile << name;
    for (const auto& duration : currentDurations_.at(name)) {
      csvFile << ',' << duration;
    }
    csvFile << '\n';
    // Then dump the costs.
    csvFile << name;
    for (const auto& cost : currentCosts_.at(name)) {
      csvFile << ',' << cost;
    }
    csvFile << '\n';

    // We can now clear the corresponding vectors.
    currentDurations_.at(name).clear();
    currentCosts_.at(name).clear();
  }

  // Close the file:
  csvFile.close();

  // This file should not accidentally be written to.
  fs::permissions(filename_,
                  fs::perms::owner_read | fs::perms::group_read | fs::perms::others_read);
}

fs::path AccumulatingCostLog::getFilePath() const {
  return fs::current_path() / fs::path(filename_);
}

std::vector<std::string> AccumulatingCostLog::getPlannerNames() const {
  return plannerNames_;
}

std::pair<std::vector<double>, std::vector<double>> AccumulatingCostLog::getQuantile(
    const std::string& plannerName, double quantile) const {
  std::pair<std::vector<double>, std::vector<double>> plannerQuantileHistory;
  plannerQuantileHistory.first.reserve(numDurationBins_);
  plannerQuantileHistory.second.reserve(numDurationBins_);
  for (std::size_t i = 0u; i < numDurationBins_; ++i) {
    const auto& accumulator = *(accumulators_.at(plannerName).at(i));
    double duration = ((i * durationBinSize_) + ((i + 1) * durationBinSize_)) / 2.0;
    plannerQuantileHistory.first.emplace_back(duration);
    plannerQuantileHistory.second.emplace_back(boost::accumulators::quantile(
        accumulator, boost::accumulators::quantile_probability = quantile));
  }
  return plannerQuantileHistory;
}

}  // namespace ompltools

}  // namespace esp
