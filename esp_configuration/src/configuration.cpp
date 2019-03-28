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

#include <experimental/filesystem>

#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>
#include <boost/program_options.hpp>

#include "esp_configuration/version.h"
#include "esp_time/time.h"

namespace esp {

namespace ompltools {

// Convenience namespaces.
namespace fs = std::experimental::filesystem;
namespace po = boost::program_options;

Configuration::Configuration(int argc, char **argv) {
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

  // This configuration class requires a default configuration by design.
  if (!invokedOptions.count("default-config")) {
    throw std::runtime_error(
        "Please provide the location of the default configuration file with the command line "
        "option \"--default-config /path/to/default/config\".");
  }

  // Check that the default config file exists.
  fs::path defaultConfig(invokedOptions["default-config"].as<std::string>());
  if (fs::exists(defaultConfig)) {
    // If it the file is a symlink, read the actual file.
    if (fs::is_symlink(defaultConfig)) {
      defaultConfig = fs::read_symlink(defaultConfig);
    }
    std::ifstream file(defaultConfig.string());
    file >> allParameters_;
  } else {  // The provided default config file does not exist.
    throw fs::filesystem_error(
        "The file supposedly containing the default configuration does not exist on the provided "
        "path.",
        defaultConfig, std::error_code());
  }

  // Load the config patch if one is provided.
  if (invokedOptions.count("patch-config")) {
    fs::path patchConfig(invokedOptions["patch-config"].as<std::string>());
    if (fs::exists(patchConfig)) {
      // If it the file is a symlink, read the actual file.
      if (fs::is_symlink(patchConfig)) {
        patchConfig = fs::read_symlink(patchConfig);
      }

      // Convert the config into a json datastructure.
      json::json patch;
      std::ifstream file(patchConfig.string());
      file >> patch;

      // If the patch specifies a commit, check that it is currently checked out and clean.
      if (patch.count("Version") != 0) {
        std::string experimentCommit = patch["Version"]["commit"];
        if (experimentCommit != std::string("any")) {
          if (experimentCommit != Version::GIT_SHA1) {
            OMPL_ERROR("Config specifies commit that is different from the one that's checked out");
          } else if (Version::GIT_STATUS == std::string("DIRTY")) {
            OMPL_WARN(
                "Config specifies commit that matches the one that's checked out, but there are "
                "uncommited changes.");
          }
        }
      }

      // If the patch specifies an experiment, there are some special precautions to be taken.
      if (patch.count("Experiment") != 0) {
        // Check it specifies which executable is associated with this experiment and make sure it's
        // the one that is being executed.
        if (patch["Experiment"].count("executable") == 0) {
          throw std::runtime_error("Please specify the executable name for the experiment.");
        } else if (patch["Experiment"]["executable"] != std::string(argv[0]).substr(2)) {
          throw std::runtime_error(
              "Experiment config specifies executable name that does not match the name of the "
              "current executable.");
        }

        // Handle any seed specification.
        if (patch["Experiment"].count("seed") != 0) {
          if (patch["Experiment"]["seed"] == std::string("time")) {
            patch["Experiment"]["seed"] = ompl::RNG::getSeed();
          } else {
            ompl::RNG::setSeed(patch["Experiment"]["seed"].get<unsigned long>());
            OMPL_WARN("Seed set to %lu", patch["Experiment"]["seed"].get<unsigned long>());
          }
        } else {
          patch["Experiment"]["seed"] = ompl::RNG::getSeed();
        }
      }

      // The patch seems valid, merge it into the default config.
      allParameters_.merge_patch(patch);
    } else {  // The provided patch config file does not exist.
      throw fs::filesystem_error(
          "The file supposedly containing the patch configuration does not exist on the provided "
          "path.",
          patchConfig, std::error_code());
    }
  }
  // Capture the current version.
  accessedParameters_["Version"]["commit"] = Version::GIT_SHA1;
  accessedParameters_["Version"]["branch"] = Version::GIT_REFSPEC;
  accessedParameters_["Version"]["status"] = Version::GIT_STATUS;
  if (Version::GIT_STATUS == std::string("DIRTY")) {
    OMPL_WARN("The working directory is dirty, results might not be reproducible.");
  }

  // Set the appropriate log level.
  if (allParameters_.count("Log") != 0) {
    auto level = allParameters_["Log"]["level"];
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

const json::json &Configuration::getExperimentConfig() const {
  if (allParameters_.count("Experiment") == 0) {
    throw std::runtime_error("Requested configuration for experiment, but none exists.");
  }
  if (accessedParameters_.count("Experiment") != 0) {
    if (accessedParameters_["Experiment"] != allParameters_["Experiment"]) {
      throw std::runtime_error("Accessing changed parameters, results might not be reproducable.");
    }
  } else {
    accessedParameters_["Experiment"] = allParameters_["Experiment"];
  }
  return allParameters_["Experiment"];
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
  fs::permissions(filename,
                  fs::perms::owner_read | fs::perms::group_read | fs::perms::others_read);
}

}  // namespace ompltools

}  // namespace esp
