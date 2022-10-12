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

// Authors: Jonathan Gammell, Marlin Strub

#pragma once

#include <chrono>
#include <fstream>
#include <tuple>
#include <vector>

#include <experimental/filesystem>

#include <ompl/base/Cost.h>

#include "pdt/time/time.h"

namespace pdt {

namespace loggers {

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
  NullLogger(const time::Duration& /*runTime*/, unsigned int /*recordPeriodMicrosecond*/) {
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
  using logData = std::pair<const time::Duration, const ompl::base::Cost>;

  /** \brief Constructors */
  TimeCostLogger(const time::Duration& maxDuration, double logFrequency);

  /** \brief Output the data with the appropriate label*/
  std::string createLogString(const std::string& labelPrefix) const;

  /** \brief std::vector pass-throughs */
  bool empty() const { return measurements_.empty(); }
  void addMeasurement(const time::Duration& duration, const ompl::base::Cost& cost);
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
  typedef std::tuple<time::Duration, unsigned int, double> data_t;

  /** \brief Constructors */
  TimeIterationCostLogger(double runTimeSeconds, unsigned int recordPeriodMicrosecond);
  TimeIterationCostLogger(const time::Duration& runTime, unsigned int recordPeriodMicrosecond);

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
  IterationCostLogger(const time::Duration& runTime, unsigned int recordPeriodMicrosecond);

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
  typedef std::pair<double, time::Duration> data_t;

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
class ResultLog {
 public:
  ResultLog(const std::experimental::filesystem::path& filepath, const bool append) :
      filepath_(filepath) {
    // Create parent directories, if needed.
    fs::create_directories(filepath_.parent_path());

    // Assert user intent
    if (!append && std::experimental::filesystem::exists(filepath_)) {
      auto msg = "Told to create a new file where one already exists.";
      throw std::ios_base::failure(msg);
    } else if (append && !std::experimental::filesystem::exists(filepath_)) {
      auto msg = "Told to append to a file that does not exist.";
      throw std::ios_base::failure(msg);
    }

    if (append) {
      // In order to append to the file, we need to set write access-permissions again.
      fs::permissions(filepath_, fs::perms::owner_read | fs::perms::owner_write |
                                     fs::perms::group_read | fs::perms::others_read);
    }

    // Open the file.
    std::ofstream filestream;
    filestream.open(filepath_.string(), std::ofstream::out | std::ofstream::app);

    // Check on the failbit.
    if (filestream.fail() == true) {
      using namespace std::string_literals;
      auto msg = "Could not open results file at "s + filepath_.string() + "."s;
      throw std::ios_base::failure(msg);
    }

    // Set the permissions to read only.
    fs::permissions(filepath_,
                    fs::perms::owner_read | fs::perms::group_read | fs::perms::others_read);
  };

  fs::path getFilePath() const { return fs::absolute(filepath_); }

  void addResult(const std::string& plannerName, const Logger& logger) {
    // Make sure we can write to this file.
    fs::permissions(filepath_, fs::perms::owner_read | fs::perms::owner_write |
                                   fs::perms::group_read | fs::perms::others_read);

    // Open the file.
    std::ofstream filestream;
    filestream.open(filepath_.string(), std::ofstream::out | std::ofstream::app);

    // Check on the failbit.
    if (filestream.fail() == true) {
      using namespace std::string_literals;
      auto msg = "Could not open results file at "s + filepath_.string() + "."s;
      throw std::ios_base::failure(msg);
    }

    // Write the data.
    filestream << logger.createLogString(plannerName);

    // Close the file:
    filestream.close();

    // This file should not accidentally be written to.
    fs::permissions(filepath_,
                    fs::perms::owner_read | fs::perms::group_read | fs::perms::others_read);
  }

 private:
  std::experimental::filesystem::path filepath_;
};

}  // namespace loggers

}  // namespace pdt
