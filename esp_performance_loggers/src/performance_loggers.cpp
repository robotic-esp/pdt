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

// Authors: Jonathan Gammell, Marlin Strub

#include "esp_performance_loggers/performance_loggers.h"

#include <cmath>
#include <iomanip>

#include <experimental/filesystem>

#include <ompl/util/Console.h>

namespace esp {

namespace ompltools {

namespace {

// Constants
constexpr double ALLOC_SAFETY_FACTOR = 10.0;

}  // namespace

// Convenience namespace.
namespace fs = std::experimental::filesystem;

TimeCostLogger::TimeCostLogger(const esp::ompltools::time::Duration& maxDuration,
                               double logFrequency) :
    allocSize_(std::ceil(ALLOC_SAFETY_FACTOR *
                         std::chrono::duration<double, std::ratio<1>>(maxDuration).count() *
                         logFrequency)) {
  measurements_.reserve(allocSize_);
}

void TimeCostLogger::addMeasurement(const esp::ompltools::time::Duration& duration,
                                    const ompl::base::Cost& cost) {
  measurements_.emplace_back(duration, cost);
}

std::string TimeCostLogger::createLogString(const std::string& prefix) const {
  if (measurements_.capacity() > allocSize_) {
    OMPL_WARN(
        "The result file was under-allocated (reserved: %d, used: %d). This will affect the "
        "accuracy of timings.",
        allocSize_, measurements_.size());
  }

  // Start with the prefix.
  std::stringstream rval;
  rval << prefix << ", ";

  // Dump the time measurements.
  for (std::size_t i = 0u; i < measurements_.size(); ++i) {
    rval << std::setprecision(21)
         << std::chrono::duration<double, std::ratio<1>>(measurements_.at(i).first).count();
    if (i != measurements_.size() - 1u) {
      rval << ", ";
    }
  }
  rval << '\n';

  // Dump the cost measurements.
  rval << prefix << ", ";
  for (unsigned int i = 0u; i < measurements_.size(); ++i) {
    rval << std::setprecision(21) << measurements_.at(i).second.value();
    if (i != measurements_.size() - 1u) {
      rval << ", ";
    }
  }
  rval << '\n';

  // Return as string.
  return rval.str();
}

TimeIterationCostLogger::TimeIterationCostLogger(double runTimeSeconds,
                                                 unsigned int recordPeriodMicrosecond) {
  allocSize_ =
      ALLOC_SAFETY_FACTOR * runTimeSeconds / (static_cast<double>(recordPeriodMicrosecond) / 1e6);
  data_.reserve(allocSize_);
}
TimeIterationCostLogger::TimeIterationCostLogger(const esp::ompltools::time::Duration& runTime,
                                                 unsigned int recordPeriodMicrosecond) {
  allocSize_ = ALLOC_SAFETY_FACTOR * esp::ompltools::time::seconds(runTime) /
               (static_cast<double>(recordPeriodMicrosecond) / 1e6);
  data_.reserve(allocSize_);
}
std::string TimeIterationCostLogger::output(const std::string& labelPrefix) {
  // Variable
  // The return value
  std::stringstream rval;

  if (data_.capacity() > allocSize_) {
    std::cout << std::endl;
    std::cout << "WARNING. The result file was under allocated (reserved: " << allocSize_
              << ", used: " << data_.size() << ", capacity: " << data_.capacity()
              << "). This will affect the accuracy of timings." << std::endl;
    std::cout << std::endl;
  }

  // Write the time first:
  rval << labelPrefix << ", ";
  for (unsigned int i = 0u; i < data_.size(); ++i) {
    rval << std::setprecision(21) << esp::ompltools::time::seconds(std::get<0u>(data_.at(i)));
    if (i != data_.size() - 1u) {
      rval << ", ";
    }
  }
  rval << std::endl;

  // then the iterations:
  rval << labelPrefix << ", ";
  for (unsigned int i = 0u; i < data_.size(); ++i) {
    rval << std::setprecision(21) << std::get<1u>(data_.at(i));
    if (i != data_.size() - 1u) {
      rval << ", ";
    }
  }
  rval << std::endl;

  // then the costs:
  rval << labelPrefix << ", ";
  for (unsigned int i = 0u; i < data_.size(); ++i) {
    rval << std::setprecision(21) << std::get<2u>(data_.at(i));
    if (i != data_.size() - 1u) {
      rval << ", ";
    }
  }
  rval << std::endl;

  // Return
  return rval.str();
}

IterationCostLogger::IterationCostLogger(unsigned int numIterations) {
  allocSize_ = numIterations + 5u;
  data_.reserve(allocSize_);
}
IterationCostLogger::IterationCostLogger(double runTimeSeconds,
                                         unsigned int recordPeriodMicrosecond) {
  allocSize_ =
      ALLOC_SAFETY_FACTOR * runTimeSeconds / (static_cast<double>(recordPeriodMicrosecond) / 1e6);
  data_.reserve(allocSize_);
}
IterationCostLogger::IterationCostLogger(const esp::ompltools::time::Duration& runTime,
                                         unsigned int recordPeriodMicrosecond) {
  allocSize_ = ALLOC_SAFETY_FACTOR * esp::ompltools::time::seconds(runTime) /
               (static_cast<double>(recordPeriodMicrosecond) / 1e6);
  data_.reserve(allocSize_);
}
std::string IterationCostLogger::output(const std::string& labelPrefix) {
  // Variable
  // The return value
  std::stringstream rval;

  if (data_.capacity() > allocSize_) {
    std::cout << std::endl;
    std::cout << "WARNING. The result file was under allocated (reserved: " << allocSize_
              << ", used: " << data_.size() << ", capacity: " << data_.capacity()
              << "). This will affect the accuracy of timings." << std::endl;
    std::cout << std::endl;
  }

  // Write the iteration first:
  rval << labelPrefix << ", ";
  for (unsigned int i = 0u; i < data_.size(); ++i) {
    rval << data_.at(i).first;
    if (i != data_.size() - 1u) {
      rval << ", ";
    }
  }
  rval << std::endl;

  // then the cost
  rval << labelPrefix << ", ";
  for (unsigned int i = 0u; i < data_.size(); ++i) {
    rval << std::setprecision(21) << data_.at(i).second;
    if (i != data_.size() - 1u) {
      rval << ", ";
    }
  }
  rval << std::endl;

  // Return
  return rval.str();
}

TargetTimeResults::TargetTimeResults(unsigned int numTargets) {
  allocSize_ = numTargets + 2u;
  data_.reserve(allocSize_);
}
std::string TargetTimeResults::output(const std::string& labelPrefix) {
  // Variable
  // The return value
  std::stringstream rval;

  if (data_.capacity() > allocSize_) {
    std::cout << std::endl;
    std::cout << "WARNING. The result file was under allocated (reserved: " << allocSize_
              << ", used: " << data_.size() << ", capacity: " << data_.capacity()
              << "). This will affect the accuracy of timings." << std::endl;
    std::cout << std::endl;
  }

  // Write the targets first:
  rval << labelPrefix << ", ";
  for (unsigned int i = 0u; i < data_.size(); ++i) {
    rval << std::setprecision(21) << data_.at(i).first;
    if (i != data_.size() - 1u) {
      rval << ", ";
    }
  }
  rval << std::endl;

  // the time
  rval << labelPrefix << ", ";
  for (unsigned int i = 0u; i < data_.size(); ++i) {
    rval << std::setprecision(21) << esp::ompltools::time::seconds(data_.at(i).second);
    if (i != data_.size() - 1u) {
      rval << ", ";
    }
  }
  rval << std::endl;

  // Return
  return rval.str();
}

}  // namespace ompltools

}  // namespace esp