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

// Authors: Marlin Strub

#pragma once

#include <fstream>
#include <iostream>
#include <string>

#include <ompl/util/Console.h>

#include "nlohmann/json.hpp"

namespace esp {

namespace ompltools {

namespace json = nlohmann;

// A class to manage configuration for repeatable experiments.
class Configuration {
 public:
  Configuration(int argc, char** argv);
  ~Configuration() = default;

  // Get the experiment config.
  const json::json& getExperimentConfig() const;

  // Get the planner config.
  const json::json& getPlannerConfig(const std::string& planner) const;

  // Get the context config.
  const json::json& getContextConfig(const std::string& context) const;

  // Add a key-value-pair to the experiment config.
  template <typename T>
  void addToTimeField(const std::string& key, const T& value);


  // Query the config whether it contains a key.
  bool contains(const std::string& key) const;

  // Dump the parameters.
  void dumpAll(std::ostream& out = std::cout) const;
  void dumpAll(const std::string& filename) const;
  void dumpAccessed(std::ostream& out = std::cout) const;
  void dumpAccessed(const std::string& filename) const;

 private:
  json::json allParameters_{};
  mutable json::json accessedParameters_{};
};

template <typename T>
void Configuration::addToTimeField(const std::string& key, const T& value) {
  allParameters_["Time"][key] = value;
  accessedParameters_["Time"][key] = value;
}

}  // namespace ompltools

}  // namespace esp
