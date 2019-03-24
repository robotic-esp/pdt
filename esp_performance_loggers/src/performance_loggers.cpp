#include "esp_performance_loggers/performance_loggers.h"

#include <boost/filesystem.hpp>
#include <iomanip>

namespace esp {

namespace ompltools {

constexpr float ALLOC_TIME_SAFETY_FACTOR = 10.0;

void createDirectories(std::string fileName) {
  /// Variables
  // The boost::path representation of the string
  boost::filesystem::path fullPath;

  // Create a boost::path from the provided string
  fullPath = fileName.c_str();

  // Decompose the path into the parent directories and check if they exist
  if (fullPath.parent_path().empty() == false) {
    if (boost::filesystem::exists(fullPath.parent_path()) == false) {
      // If they don't exist, make them
      boost::filesystem::create_directories(fullPath.parent_path());

      //    std::cout << "Created: " << boost::filesystem::absolute(fullPath.parent_path()) << "\n";
    }
  }
  // Else, do nothing
}

//******* The different pieces of data to be recorded*******//
TimeCostLogger::TimeCostLogger(double runTimeSeconds, unsigned int recordPeriodMicrosecond) {
  allocSize_ = ALLOC_TIME_SAFETY_FACTOR * runTimeSeconds /
               (static_cast<double>(recordPeriodMicrosecond) / 1e6);
  data_.reserve(allocSize_);
}
TimeCostLogger::TimeCostLogger(const asrl::time::duration& runTime,
                                 unsigned int recordPeriodMicrosecond) {
  allocSize_ = ALLOC_TIME_SAFETY_FACTOR * asrl::time::seconds(runTime) /
               (static_cast<double>(recordPeriodMicrosecond) / 1e6);
  data_.reserve(allocSize_);
}
std::string TimeCostLogger::output(const std::string& labelPrefix) {
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
    rval << std::setprecision(21) << asrl::time::seconds(data_.at(i).first);
    if (i != data_.size() - 1u) {
      rval << ", ";
    }
  }
  rval << std::endl;

  // then costs:
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

TimeIterationCostLogger::TimeIterationCostLogger(double runTimeSeconds,
                                                   unsigned int recordPeriodMicrosecond) {
  allocSize_ = ALLOC_TIME_SAFETY_FACTOR * runTimeSeconds /
               (static_cast<double>(recordPeriodMicrosecond) / 1e6);
  data_.reserve(allocSize_);
}
TimeIterationCostLogger::TimeIterationCostLogger(const asrl::time::duration& runTime,
                                                   unsigned int recordPeriodMicrosecond) {
  allocSize_ = ALLOC_TIME_SAFETY_FACTOR * asrl::time::seconds(runTime) /
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
    rval << std::setprecision(21) << asrl::time::seconds(std::get<0u>(data_.at(i)));
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
  allocSize_ = ALLOC_TIME_SAFETY_FACTOR * runTimeSeconds /
               (static_cast<double>(recordPeriodMicrosecond) / 1e6);
  data_.reserve(allocSize_);
}
IterationCostLogger::IterationCostLogger(const asrl::time::duration& runTime,
                                           unsigned int recordPeriodMicrosecond) {
  allocSize_ = ALLOC_TIME_SAFETY_FACTOR * asrl::time::seconds(runTime) /
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
    rval << std::setprecision(21) << asrl::time::seconds(data_.at(i).second);
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
