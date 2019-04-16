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

#pragma once

#include <experimental/filesystem>
#include <map>
#include <string>
#include <vector>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

#include <ompl/base/Cost.h>

#include "esp_time/time.h"

namespace esp {

namespace ompltools {

class AccumulatingCostLog {
  // using AccumulatorSet = boost::accumulators::accumulator_set<
  //     double,
  //     boost::accumulators::features<boost::accumulators::tag::count,
  //                                   boost::accumulators::tag::min,
  //                                   boost::accumulators::tag::max,
  //                                   boost::accumulators::tag::extended_p_square,
  //                                   boost::accumulators::tag::variance>>;

  using AccumulatorSet = boost::accumulators::accumulator_set<
      double, boost::accumulators::stats<boost::accumulators::tag::extended_p_square_quantile,
                                         boost::accumulators::tag::min>>;

 public:
  AccumulatingCostLog(const std::vector<std::string>& plannerNames,
                      const esp::ompltools::time::Duration& maxDuration, double logFrequency);
  ~AccumulatingCostLog() = default;

  void createLogFile(const std::string& fullFilename);
  void addMeasurement(const std::string& plannerName, const time::Duration& duration,
                      const ompl::base::Cost& cost);
  void processMeasurements();
  std::pair<std::vector<double>, std::vector<double>> getQuantile(const std::string& plannerName,
                                                                  double quantile) const;

  std::experimental::filesystem::path getFilePath() const;
  std::vector<std::string> getPlannerNames() const;

 private:
  std::map<std::string, std::map<std::size_t, std::shared_ptr<AccumulatorSet>>> accumulators_{};
  std::map<std::string, std::vector<double>> currentDurations_{};
  std::map<std::string, std::vector<double>> currentCosts_{};
  std::vector<std::string> plannerNames_;

  std::experimental::filesystem::path filename_{};

  const std::size_t allocSize_{0u};
  const double durationBinSize_{0.0};
  const std::size_t numDurationBins_{0u};
};

}  // namespace ompltools

}  // namespace esp
