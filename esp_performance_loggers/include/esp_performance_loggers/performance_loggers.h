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

// Authors: Jonathan Gammell

#pragma once

#include <fstream>
#include <tuple>
#include <vector>

#include "esp_utilities/general_tools.h"

namespace esp {

namespace ompltools {

/** \brief A helper function to create directories using boost filesystem */
void createDirectories(std::string fileName);

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
  NullLogger(const asrl::time::duration& /*runTime*/, unsigned int /*recordPeriodMicrosecond*/) {
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
  /** \brief Data type */
  typedef std::pair<asrl::time::duration, double> data_t;

  /** \brief Constructors */
  TimeCostLogger(double runTimeSeconds, unsigned int recordPeriodMicrosecond);
  TimeCostLogger(const asrl::time::duration& runTime, unsigned int recordPeriodMicrosecond);

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

/** \brief A vector of time & iteration & cost */
class TimeIterationCostLogger {
 public:
  /** \brief Data type */
  typedef std::tuple<asrl::time::duration, unsigned int, double> data_t;

  /** \brief Constructors */
  TimeIterationCostLogger(double runTimeSeconds, unsigned int recordPeriodMicrosecond);
  TimeIterationCostLogger(const asrl::time::duration& runTime,
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
  IterationCostLogger(const asrl::time::duration& runTime, unsigned int recordPeriodMicrosecond);

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
  typedef std::pair<double, asrl::time::duration> data_t;

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
template <class Data>
class ResultsFile {
 public:
  ResultsFile(const std::string& fullFileName) : filename_(fullFileName) {
    createDirectories(filename_);
  };

  void addResult(const std::string& plannerName, Data& data) {
    // Variable:
    // The output file stream
    std::ofstream mfile;

    // Open the file:
    mfile.open(filename_.c_str(), std::ofstream::out | std::ofstream::app);

    // Check on the failbit:
    if (mfile.fail() == true) {
      throw std::ios_base::failure("Could not open file.");
    }

    // Write the data
    mfile << data.output(plannerName);

    // Flush the file:
    mfile.flush();

    // Close the file:
    mfile.close();
  };

 private:
  std::string filename_;
};

}  // namespace ompltools

}  // namespace esp
