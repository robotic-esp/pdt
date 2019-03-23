/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Oxford
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

// Author: Marlin Strub

#include "esp_configuration/configuration.h"

namespace esp {

namespace ompltools {

// Convenience namespace.
namespace fs = std::experimental::filesystem;

Configuration::Configuration(fs::path defaultConfig) {
  if (fs::exists(defaultConfig)) {
    // If its a symlink, read the actual file.
    if (fs::is_symlink(defaultConfig)) {
      defaultConfig = fs::read_symlink(defaultConfig);
    }
    std::ifstream file(defaultConfig.string());
    file >> allParameters_;
  } else {
    // The provided config file does not exist, the factory is useless, throw.
    throw fs::filesystem_error("File does not exist.", defaultConfig, std::error_code());
  }
}

const json::json &Configuration::getPlannerConfig(const std::string &planner) const {
  if (allParameters_["Planners"].count(planner) == 0) {
    throw std::runtime_error("Requested configuration for unknown planner.");
  }
  if (accessedParameters_["Planners"].count(planner) != 0) {
    if (accessedParameters_["Planners"][planner] != allParameters_["Planners"][planner]) {
      throw std::runtime_error("Accessing changed parameters, results might not be reproducable.");
    }
  } else {
    accessedParameters_["Planners"][planner] = allParameters_["Planners"][planner];
  }
  return allParameters_["Planners"][planner];
}

const json::json &Configuration::getContextConfig(const std::string &context) const {
  if (allParameters_["Contexts"].count(context) == 0) {
    throw std::runtime_error("Requested configuration for unknown context.");
  }
  if (accessedParameters_["Contexts"].count(context) != 0) {
    if (accessedParameters_["Contexts"][context] != allParameters_["Contexts"][context]) {
      throw std::runtime_error("Accessing changed parameters, results might not be reproducable.");
    }
  } else {
    accessedParameters_["Contexts"][context] = allParameters_["Contexts"][context];
  }
  return allParameters_["Contexts"][context];
}

bool Configuration::contains(const std::string &key) const {
  return allParameters_.find(key) != allParameters_.end();
}

void Configuration::dumpAll(std::ostream &out) const {
  out << allParameters_.dump(2) << '\n';
}

void Configuration::dumpAccessed(std::ostream &out) const {
  out << accessedParameters_.dump(2) << '\n';
}

}  // namespace ompltools

}  // namespace esp
