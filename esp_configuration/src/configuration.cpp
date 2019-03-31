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

#include <exception>
#include <iostream>
#include <string>

#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>
#include <boost/program_options.hpp>

#include "esp_configuration/directory.h"
#include "esp_configuration/version.h"
#include "esp_time/time.h"

namespace esp {

namespace ompltools {

// Convenience namespaces.
namespace fs = std::experimental::filesystem;
namespace po = boost::program_options;

Configuration::Configuration(int argc, char **argv) :
    executable_(fs::path(std::string(argv[0]).substr(2)).filename().string()) {
  load(argc, argv);
}

void Configuration::load(int argc, char **argv) {
  // Declare the available options.
  po::options_description availableOptions("Configuration options");
  availableOptions.add_options()("help,h", "Display available options.")(
      "default-config,c", po::value<std::string>(), "Location of the default configuration file.")(
      "patch-config,p", po::value<std::string>(), "Location of the patch configuration file.");

  // Parse the command line arguments to see which options were invoked.
  po::variables_map invokedOptions;
  po::store(po::parse_command_line(argc, argv, availableOptions), invokedOptions);
  po::notify(invokedOptions);

  // Help displays the available options and terminates the program.
  if (invokedOptions.count("help")) {
    std::cout << availableOptions << '\n';
    std::terminate();
  }

  // There are the following supported scenarios:
  //   - No options are invoked -> Look for a default config at default location.
  //   - A default config is provided without a patch config -> Load the provided default.
  //   - Only a patch config is provided -> Check the config whether to load a default.
  //   - A default and a patch config is provided -> Check the patch config whether to load the
  //     default.

  if (!invokedOptions.count("default-config") && !invokedOptions.count("patch-config")) {
    loadDefaultConfigFromDefaultPath();
  } else if (invokedOptions.count("default-config") && !invokedOptions.count("patch-config")) {
    fs::path defaultConfig(invokedOptions["default-config"].as<std::string>());
    loadConfigFromSpecifiedPath(defaultConfig);
  } else if (!invokedOptions.count("default-config") && invokedOptions.count("patch-config")) {
    fs::path patchConfig(invokedOptions["patch-config"].as<std::string>());
    if (fs::exists(patchConfig)) {
      // If it the file is a symlink, read the actual file.
      if (fs::is_symlink(patchConfig)) {
        patchConfig = fs::read_symlink(patchConfig);
      }
      // Load the patch config.
      std::ifstream file(patchConfig.string());
      json::json patch;
      file >> patch;
      if (patch.contains("Experiment")) {
        if (patch["Experiment"].contains("useOnlyThisConfig")) {
          // Check if the patch claims to be complete.
          if (!patch["Experiment"]["useOnlyThisConfig"]) {
            // Load the default config.
            loadDefaultConfigFromDefaultPath();
          }
        }
      }
      // Load the patch, possibly overriding default values.
      parameters_.merge_patch(patch);
      OMPL_INFORM("Loaded patch configuration from %s", patchConfig.c_str());
    } else {
      // The provided patch config file does not exist.
      OMPL_ERROR("Cannot find provided configuration file at %s", patchConfig.c_str());
      throw std::ios_base::failure("Cannot find patch config file.");
    }
  } else if (invokedOptions.count("default-config") && invokedOptions.count("patch-config")) {
    fs::path patchConfig(invokedOptions["patch-config"].as<std::string>());
    if (fs::exists(patchConfig)) {
      // If it the file is a symlink, read the actual file.
      if (fs::is_symlink(patchConfig)) {
        patchConfig = fs::read_symlink(patchConfig);
      }
      // Load the patch config.
      std::ifstream file(patchConfig.string());
      json::json patch;
      file >> patch;
      if (patch.contains("Experiment")) {
        if (patch["Experiment"].contains("useOnlyThisConfig") &&
            patch["Experiment"]["useOnlyThisConfig"].get<bool>()) {
          OMPL_WARN(
              "Provided a default config but patch says 'useOnlyThisConfig'. Not loading the "
              "provided default config.");
        } else {
          fs::path defaultConfig(invokedOptions["default-config"].as<std::string>());
          loadConfigFromSpecifiedPath(defaultConfig);
        }
      }

      // Merge the patch, possibly overriding default values.
      parameters_.merge_patch(patch);
      OMPL_INFORM("Loaded patch configuration from %s", patchConfig.c_str());
    } else {
      // The provided patch config file does not exist.
      OMPL_ERROR("Cannot find provided configuration file at %s", patchConfig.c_str());
      throw std::ios_base::failure("Cannot find patch config file.");
    }
  } else {
    // How did we get here?
    assert(false);
  }

  // Set the appropriate log level.
  if (parameters_.count("Log") != 0) {
    auto level = parameters_["Log"]["level"];
    if (level == std::string("dev2")) {
      ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_DEV2);
    } else if (level == std::string("dev1")) {
      ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_DEV1);
    } else if (level == std::string("debug")) {
      ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_DEBUG);
    } else if (level == std::string("info")) {
      ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_INFO);
    } else if (level == std::string("warn")) {
      ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_WARN);
    } else if (level == std::string("error")) {
      ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_ERROR);
    } else if (level == std::string("none")) {
      ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_NONE);
    } else {
      OMPL_WARN("Config specifies invalid log level. Setting the log level to LOG_WARN");
      ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_WARN);
    }
  } else {
    ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_WARN);
  }
}

bool Configuration::contains(const std::string &key) const {
  return contains(key, parameters_);
}

bool Configuration::contains(const std::string &key, const json::json &parameters) const {
  if (parameters.contains(key)) {
    return true;
  } else {
    if (isNestedKey(key)) {
      auto [ns, rest] = split(key);
      if (parameters.contains(ns)) {
        auto nestedParameters = parameters[ns];
        return contains(rest, nestedParameters);
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
}

void Configuration::dumpAll(std::ostream &out) const {
  out << parameters_.dump(2) << '\n';
}

void Configuration::dumpAll(const std::string &filename) const {
  // Open the file.
  std::ofstream configFile;
  configFile.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);

  // Check if it was successfully opened.
  if (configFile.fail()) {
    throw std::ios_base::failure("Could not open config file.");
  }

  // Write the config to this file.
  dumpAll(configFile);

  // Close the file.
  configFile.close();
}

void Configuration::dumpAccessed(std::ostream &out) const {
  out << accessedParameters_.dump(2) << '\n';
}

void Configuration::dumpAccessed(const std::string &filename) const {
  // Make sure the path exists.
  fs::create_directories(fs::path(filename).parent_path());

  // Open the file.
  std::ofstream configFile;
  configFile.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);

  // Check if it was successfully opened.
  if (configFile.fail()) {
    throw std::ios_base::failure("Could not open config file.");
  }

  // Write the config to this file.
  dumpAccessed(configFile);

  // Close the file.
  configFile.close();

  // This file should not accidentally be written to.
  fs::permissions(filename, fs::perms::owner_read | fs::perms::group_read | fs::perms::others_read);
}

void Configuration::loadDefaultConfigFromDefaultPath() {
  fs::path defaultConfig(Directory::SOURCE / "parameters/esp_ompltools_default_config.json");
  if (fs::exists(defaultConfig)) {
    // Load the default config.
    std::ifstream file(defaultConfig.string());
    file >> parameters_;
    OMPL_INFORM("Loaded default configuration from %s", defaultConfig.c_str());
  } else {
    // Cannot find default config at default location.
    OMPL_ERROR("No configuration file is provided and cannot find default at %s",
               defaultConfig.c_str());
    throw std::ios_base::failure("Cannot find default config file.");
  }
}

void Configuration::loadConfigFromSpecifiedPath(fs::path path) {
  if (fs::exists(path)) {
    // If it the file is a symlink, read the actual file.
    if (fs::is_symlink(path)) {
      path = fs::read_symlink(path);
    }
    // Load the provided default config.
    std::ifstream file(path.string());
    file >> parameters_;
    OMPL_INFORM("Loaded configuration from %s", path.c_str());
  } else {
    // The provided default config file does not exist.
    OMPL_ERROR("Cannot find provided configuration file at %s", path.c_str());
    throw std::ios_base::failure("Cannot find default config file.");
  }
}

bool Configuration::isNestedKey(const std::string &name) const {
  std::size_t pos = name.find('/');
  if (pos == std::string::npos) {
    return false;
  } else {
    return true;
  }
}

std::pair<const std::string, const std::string> Configuration::split(
    const std::string &name) const {
  std::size_t pos = name.find('/');
  if (pos == std::string::npos) {
    OMPL_ERROR("String '%s' does not contain '/' character.", name.c_str());
    throw std::runtime_error("Cannot split string without '/' character.");
  }
  return std::make_pair(name.substr(0, pos), name.substr(pos + 1));
}

void Configuration::registerAsExperiment() const {
  // Check the status of the working directory.
  if (Version::GIT_STATUS == std::string("DIRTY")) {
    OMPL_WARN("Working directory is dirty.");
  }

  // Make sure we're executing the right executable.
  if (parameters_.contains("Experiment")) {
    // Check we're on the same commit.
    if (parameters_["Experiment"].contains("version")) {
      if (parameters_["Experiment"]["version"].contains("commit")) {
        auto commitHash = parameters_["Experiment"]["version"]["commit"].get<std::string>();
        if (commitHash != std::string("any")) {
          if (commitHash != Version::GIT_SHA1) {
            OMPL_ERROR("Config specifies commit %s. You are currently on %s.", commitHash.c_str(),
                       Version::GIT_SHA1.c_str());
          }
        }
      }
    }

    // Check this is the correct executable.
    if (parameters_["Experiment"].contains("executable")) {
      auto executable = parameters_["Experiment"]["executable"].get<std::string>();
      if (executable != std::string("any")) {
        if (executable != executable_) {
          OMPL_ERROR("Config specifies executable '%s'. You are executing '%s'.",
                     executable.c_str(), executable_.c_str());
        }
      }
    }
  }

  // Store the executable name.
  accessedParameters_["Experiment"]["executable"] = executable_;

  // Store the current version.
  accessedParameters_["Experiment"]["version"]["commit"] = Version::GIT_SHA1;
  accessedParameters_["Experiment"]["version"]["branch"] = Version::GIT_REFSPEC;
  accessedParameters_["Experiment"]["version"]["status"] = Version::GIT_STATUS;

  // This ensures we don't load any additional config when rerunning this experiment.
  accessedParameters_["Experiment"]["useOnlyThisConfig"] = true;

  // If in debug, ensure we've set the seed.
  assert(allParameters_["Experiment"]["seed"].get<unsigned int>() == ompl::RNG::getSeed());

  // Handle seed specifications.
  handleSeedSpecification();
}

void Configuration::handleSeedSpecification() const {
  if (parameters_["Experiment"].contains("seed")) {
    auto seed = parameters_["Experiment"]["seed"].get<unsigned long>();
    ompl::RNG::setSeed(seed);
    OMPL_WARN("Configuration set seed to be %lu", seed);
    accessedParameters_["Experiment"]["seed"] = seed;
  } else {
    auto seed = ompl::RNG::getSeed();
    OMPL_INFORM("Seed is %lu", seed);
    accessedParameters_["Experiment"]["seed"] = seed;
  }
}

}  // namespace ompltools

}  // namespace esp
