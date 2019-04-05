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

// Authors: Jonathan Gammell, Marlin Strub

#pragma once

#include <chrono>
#include <fstream>
#include <tuple>
#include <vector>

#include <experimental/filesystem>

#include <ompl/base/Cost.h>

#include "esp_time/time.h"

namespace esp {

namespace ompltools {

namespace fs = std::experimental::filesystem;

//******* The different pieces of data to be recorded*******//
/** \brief A null-logging class */
template <class T, class U>
class NullLogger {
 public:
  /** \brief Data type */
  typedef std::pair<T, U> data_t;

  /** \brief Constructors */
  NullLogger(double /*runTimeSeconds*/, unsigned int /*recordPeriodMicrosecond*/) {
    empty_ = false;
  };
  NullLogger(const esp::ompltools::time::Duration& /*runTime*/,
             unsigned int /*recordPeriodMicrosecond*/) {
    empty_ = false;
  };

  /** \brief Output the data with the appropriate label*/
  std::string output(const std::string& labelPrefix) {
    // Variable
    // The return value
    std::stringstream rval;

    rval << labelPrefix << ", Null" << std::endl;

    return rval.str();
  };

  /** \brief std::vector pass-throughs */
  bool empty() const { return empty_; };
  void push_back(const data_t& /*newData*/) { empty_ = true; };

  data_t back() { return std::pair<T, U>(); };

 private:
  bool empty_;
};

/** \brief A vector of time & cost */
class TimeCostLogger {
 public:
  using logData = std::pair<const esp::ompltools::time::Duration, const ompl::base::Cost>;

  /** \brief Constructors */
  TimeCostLogger(const esp::ompltools::time::Duration& maxDuration, double logFrequency);

  /** \brief Output the data with the appropriate label*/
  std::string createLogString(const std::string& labelPrefix) const;

  /** \brief std::vector pass-throughs */
  bool empty() const { return measurements_.empty(); }
  void addMeasurement(const esp::ompltools::time::Duration& duration, const ompl::base::Cost& cost);
  logData lastMeasurement() { return measurements_.back(); };

 private:
  // The measurements
  std::vector<logData> measurements_{};

  /** \brief Preallocated size */
  std::size_t allocSize_{0u};
};

/** \brief A vector of time & iteration & cost */
class TimeIterationCostLogger {
 public:
  /** \brief Data type */
  typedef std::tuple<esp::ompltools::time::Duration, unsigned int, double> data_t;

  /** \brief Constructors */
  TimeIterationCostLogger(double runTimeSeconds, unsigned int recordPeriodMicrosecond);
  TimeIterationCostLogger(const esp::ompltools::time::Duration& runTime,
                          unsigned int recordPeriodMicrosecond);

  /** \brief Output the data with the appropriate label*/
  std::string output(const std::string& labelPrefix);

  /** \brief std::vector pass-throughs */
  bool empty() const { return data_.empty(); }
  void push_back(const data_t& newData) { data_.push_back(newData); };
  data_t back() { return data_.back(); };

 private:
  /** \brief Raw data */
  std::vector<data_t> data_{};

  /** \brief Preallocated size */
  unsigned int allocSize_{0u};
};

/** \brief A vector of iteration & cost */
class IterationCostLogger {
 public:
  /** \brief Data type */
  typedef std::pair<unsigned int, double> data_t;

  /** \brief Constructors */
  IterationCostLogger(unsigned int numIterations);
  IterationCostLogger(double runTimeSeconds, unsigned int recordPeriodMicrosecond);
  IterationCostLogger(const esp::ompltools::time::Duration& runTime,
                      unsigned int recordPeriodMicrosecond);

  /** \brief Output the data with the appropriate label*/
  std::string output(const std::string& labelPrefix);

  /** \brief std::vector pass-throughs */
  bool empty() const { return data_.empty(); }
  bool empty() { return data_.empty(); }
  void push_back(const data_t& newData) { data_.push_back(newData); };
  data_t back() { return data_.back(); };

 private:
  /** \brief Raw data */
  std::vector<data_t> data_{};

  /** \brief Preallocated size */
  unsigned int allocSize_{0u};
};

/** \brief A vector of "target" & time */
class TargetTimeResults {
 public:
  /** \brief Data type */
  typedef std::pair<double, esp::ompltools::time::Duration> data_t;

  /** \brief Constructors */
  TargetTimeResults(unsigned int numTargets);

  /** \brief Output the data with the appropriate label*/
  std::string output(const std::string& labelPrefix);

  /** \brief std::vector pass-throughs */
  bool empty() const { return data_.empty(); }
  void push_back(const data_t& newData) { data_.push_back(newData); };
  data_t back() { return data_.back(); };

 private:
  /** \brief Raw data */
  std::vector<data_t> data_{};

  /** \brief Preallocated size */
  unsigned int allocSize_{0u};
};

//******* The file that writes the data to disk*******//
/** \brief A class to write results to disk. */
template <class Logger>
class PerformanceLog {
 public:
  PerformanceLog(const std::string& fullFileName) : filename_(fullFileName) {
    // Create parent directories, if needed.
    fs::create_directories(fs::path(filename_).parent_path());

    // Create the file.
    std::ofstream file(filename_);

    // Set the permissions to read only.
    fs::permissions(filename_,
                    fs::perms::owner_read | fs::perms::group_read | fs::perms::others_read);
  };

  fs::path getFilePath() const {
    return fs::current_path() / fs::path(filename_);
  }

  void addResult(const std::string& plannerName, const Logger& logger) {
    // Make sure we can write to this file.
    fs::permissions(filename_, fs::perms::owner_read | fs::perms::owner_write |
                                   fs::perms::group_read | fs::perms::others_read);

    // Open the file.
    std::ofstream csvFile;
    csvFile.open(filename_, std::ofstream::out | std::ofstream::app);

    // Check on the failbit.
    if (csvFile.fail() == true) {
      throw std::ios_base::failure("Could not open log file.");
    }

    // Write the data.
    csvFile << logger.createLogString(plannerName);

    // Close the file:
    csvFile.close();

    // This file should not accidentally be written to.
    fs::permissions(filename_,
                    fs::perms::owner_read | fs::perms::group_read | fs::perms::others_read);
  };

 private:
  std::string filename_;
};

}  // namespace ompltools

}  // namespace esp
