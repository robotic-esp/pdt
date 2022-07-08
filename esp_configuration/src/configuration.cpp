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

namespace esp {

namespace ompltools {

// Convenience namespaces.
using namespace std::string_literals;
namespace fs = std::experimental::filesystem;
namespace po = boost::program_options;

Configuration::Configuration(const int argc, const char **argv) :
    executable_(fs::path(std::string(argv[0]).substr(2)).filename().string()) {
  load(argc, argv);
}

void Configuration::clear() {
  executable_ = "";
  parameters_.clear();
  accessedParameters_.clear();
}

void Configuration::load(const fs::path& config) {
  if (!fs::exists(config)) {
    OMPL_ERROR("Cannot find provided configuration file at %s", config.c_str());
    throw std::ios_base::failure("Cannot find config file.");
  }

  // Open the config file.
  std::ifstream file;
  if (fs::is_symlink(config)) {
    file.open(fs::read_symlink(config).string());
  } else {
    file.open(config.string());
  }
  if (!file.is_open()) {
    OMPL_ERROR("Found file at %s but unable to open it.", config.c_str());
    throw std::ios_base::failure("Cannot open config file.");
  }

  // Load the file as a patch.
  json::json patch;
  file >> patch;
  bool loadDefaultPlannerConfigs{true};
  bool loadDefaultContextConfigs{true};
  bool loadDefaultObjectiveConfigs{true};
  if (patch.contains("experiment")) {
    if (patch["experiment"].contains("loadDefaultPlannerConfig")) {
      loadDefaultPlannerConfigs = patch["experiment"]["loadDefaultPlannerConfig"].get<bool>();
    }
    if (patch["experiment"].contains("loadDefaultContextConfig")) {
      loadDefaultContextConfigs = patch["experiment"]["loadDefaultContextConfig"].get<bool>();
    }
    if (patch["experiment"].contains("loadDefaultObjectiveConfig")) {
      loadDefaultObjectiveConfigs =
        patch["experiment"]["loadDefaultObjectiveConfig"].get<bool>();
    }
  }
    
  // Set the appropriate log level.
  if (patch.count("log") != 0) {
    auto level = patch["log"]["level"];
    if (level == "dev2"s) {
      ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_DEV2);
    } else if (level == "dev1"s) {
      ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_DEV1);
    } else if (level == "debug"s) {
      ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_DEBUG);
    } else if (level == "info"s) {
      ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_INFO);
    } else if (level == "warn"s) {
      ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_WARN);
    } else if (level == "error"s) {
      ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_ERROR);
    } else if (level == "none"s) {
      ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_NONE);
    } else {
      OMPL_WARN("Config specifies invalid OMPL log level. Setting the log level to LOG_WARN");
      ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_WARN);
    }
  } else {
    ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_WARN);
  }

  // Load the default config.
  loadDefaultConfigs(loadDefaultContextConfigs, loadDefaultPlannerConfigs,
                     loadDefaultObjectiveConfigs);
  // Merge the patch, possibly overriding default values.
  parameters_.merge_patch(patch);
  OMPL_INFORM("Loaded configuration patch from %s", config.c_str());
}

void Configuration::load(const int argc, const char **argv) {
  // Declare the available options.
  po::options_description availableOptions("Configuration options");
  availableOptions.add_options()("help,h", "Display available options.")(
      "config-patch,c", po::value<std::string>(), "Path to the configuration patch file.")(
      "path,p", po::value<std::string>(), "Path where the experiments should be stored.");

  // Parse the command line arguments to see which options were invoked.
  po::variables_map invokedOptions;
  po::store(po::parse_command_line(argc, argv, availableOptions), invokedOptions);
  po::notify(invokedOptions);

  // Help displays the available options and terminates the program.
  if (invokedOptions.count("help")) {
    std::cout << availableOptions << '\n';
    std::terminate();
  }

  // If the user does not provide a config file, we should always load the default configs.
  if (!invokedOptions.count("config-patch")) {
    loadDefaultConfigs();
  } else {
    load(invokedOptions["config-patch"].as<std::string>());
  }

  // if no path is specified, we store the experiments where the executable is called from
  if (!invokedOptions.count("path")) {
    add<std::string>("experiment/baseDirectory", fs::absolute(executable_ + "s/").string());
  } else {
    add<std::string>("experiment/baseDirectory", fs::absolute(invokedOptions["path"].as<std::string>()).string());
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

std::string Configuration::dump(const std::string &key) const {
  if (!contains(key)) {
    auto msg = "Requested to dump nonexisting parameter '"s + key + "'.";
    throw std::invalid_argument(msg);
  }
  return dump(key, parameters_);
}

std::string Configuration::dump(const std::string &key, const json::json &parameters) const {
  if (isNestedKey(key)) {
    auto [ns, rest] = split(key);
    if (!parameters.contains(ns)) {
      auto msg = "Internally requesting nonexisting parameter '"s + ns + "'. This is a bug."s;
      throw std::invalid_argument(msg);
    }
    auto nestedParameters = parameters[ns];
    return dump(rest, nestedParameters);
  } else {
    if (!parameters.contains(key)) {
      auto msg = "Internally requesting nonexisting parameter '"s + key + "'. This is a bug."s;
      throw std::invalid_argument(msg);
    }
    return parameters[key].dump(2);
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

void Configuration::loadDefaultConfigs(bool loadDefaultContextConfigs,
                                       bool loadDefaultPlannerConfigs,
                                       bool loadDefaultObjectiveConfigs) {
  // Load the default planner configs.
  if (loadDefaultPlannerConfigs) {
    fs::path defaultPlannerConfigDirectory(Directory::SOURCE / "parameters/defaults/planners/");
    if (fs::exists(defaultPlannerConfigDirectory)) {
      // Load all files in this directory as patches.
      for (auto &directoryEntry : fs::directory_iterator(defaultPlannerConfigDirectory)) {
        // Make sure the file exists.
        if (!fs::exists(directoryEntry.path())) {
          OMPL_WARN("'%s' contains irregular file '%s' which is not loaded into the configuration.",
                    defaultPlannerConfigDirectory.c_str(), directoryEntry.path().c_str());
          continue;
        }

        // Make sure the file can be opened.
        std::ifstream configFile(directoryEntry.path().string());
        if (configFile.fail()) {
          OMPL_ERROR("File '%s' exists but cannot be opened.",
                     directoryEntry.path().string().c_str());
          throw std::ios_base::failure("Configuration error.");
        }

        OMPL_DEBUG("Loading configuration from '%s'", directoryEntry.path().c_str());

        // Load the file into a temporary config.
        json::json config;
        configFile >> config;

        // Make sure the config is a sane planner configuration.
        if (!config.contains("planner")) {
          throw std::invalid_argument("Default planner configuration at '"s +
                                      directoryEntry.path().string() +
                                      "' is not a valid planner configuration. Valid planner "
                                      "configurations must be encapsulated in a 'planner' key."s);
        }

        if (config.size() > 1) {
          throw std::invalid_argument("Configuration at '"s + directoryEntry.path().string() +
                                      "' must only contain one planner configuration."s);
        }
        if (!config.front().front().contains("type")) {
          throw std::invalid_argument(directoryEntry.path().string() +
                                      ": Planner configurations must specify the type of a planner "
                                      "in a field named \"type\"."s);
        }
        if (!config.front().front().contains("isAnytime")) {
          throw std::invalid_argument(directoryEntry.path().string() +
                                      ": Planner configurations must specify whether a planner is "
                                      "anytime in a field named \"isAnytime\"."s);
        }
        if (!config.front().front().contains("report")) {
          throw std::invalid_argument(
              directoryEntry.path().string() +
              ": Planner configurations must specify the color and name to be used in the report in a field named \"report\"."s);
        }
        if (!config.front().front()["report"].contains("color")) {
          throw std::invalid_argument(
              directoryEntry.path().string() +
              ": Planner configurations must specify the color to be used in the report in a field "
              "named \"color\" under the \"report\" field."s);
        }
        if (!config.front().front()["report"].contains("name")) {
          throw std::invalid_argument(
              directoryEntry.path().string() +
              ": Planner configurations must specify the name to be used in the report in a field named \"name\" under the \"report\" field."s);
        }

        // Check that the default configs don't specify the same parameter twice.
        for (const auto &entry : config.front().items()) {
          if (parameters_["planner"].contains(entry.key())) {
            throw std::invalid_argument(directoryEntry.path().string() +
                                        ": Overwrites existing config for planner '"s +
                                        entry.key().c_str() + "'."s);
          }
        }
        parameters_.merge_patch(config);
        OMPL_INFORM("Loaded configuration from '%s'", directoryEntry.path().c_str());
        // No need to close config, std::ifstreams are closed on destruction.
      }
    } else {
      // Cannot find default planner config at default location.
      auto msg = "Default planner config directory does not exist at '"s +
                 defaultPlannerConfigDirectory.string() + "'."s;
      throw std::ios_base::failure(msg.c_str());
    }
    OMPL_INFORM("Loaded default planner configs.");
  } else {
    OMPL_INFORM("Not loading default planner configs.");
  }

  // Load the default context configs.
  if (loadDefaultContextConfigs) {
    fs::path defaultContextConfigDirectory(Directory::SOURCE / "parameters/defaults/contexts/");
    if (fs::exists(defaultContextConfigDirectory)) {
      // Load all files in this directory as patches.
      for (auto &directoryEntry : fs::directory_iterator(defaultContextConfigDirectory)) {
        // Make sure the file exists.
        if (!fs::exists(directoryEntry.path())) {
          OMPL_WARN("'%s' contains irregular file '%s' which is not loaded into the configuration.",
                    defaultContextConfigDirectory.c_str(), directoryEntry.path().c_str());
          continue;
        }

        // Make sure the file can be opened.
        std::ifstream configFile(directoryEntry.path().string());
        if (configFile.fail()) {
          OMPL_ERROR("File '%s' exists but cannot be opened.",
                     directoryEntry.path().string().c_str());
          throw std::ios_base::failure("Configuration error.");
        }

        OMPL_DEBUG("Loading configuration from '%s'", directoryEntry.path().c_str());

        // Load the file into a temporary config.
        json::json config;
        configFile >> config;

        // Make sure the config is a sane planner configuration.
        if (!config.contains("context")) {
          throw std::invalid_argument("Default context configuration at '"s +
                                      directoryEntry.path().string() +
                                      "' is not a valid context configuration. Valid context "
                                      "configurations must be encapsulated in a 'context' key."s);
        }
        if (config.size() > 1) {
          throw std::invalid_argument("Configuration at '"s + directoryEntry.path().string() +
                                      "' must only contain one context configuration."s);
        }
        if (!config.front().front().contains("type")) {
          throw std::invalid_argument(
              directoryEntry.path().string() +
              ": Context configurations must specify the type of the context in a field named "
              "\"type\".");
        }
        if (!config.front().front().contains("objective")) {
          throw std::invalid_argument(directoryEntry.path().string() +
                                      ": Context configurations must specify the optimization "
                                      "objective of the context in a "
                                      "field named \"objective\".");
        }
        if (!config.front().front().contains("dimensions")) {
          throw std::invalid_argument(
              directoryEntry.path().string() +
              ": Context configurations must specify the dimensions of the context in a "
              "field named \"dimensions\".");
        }
        if (!config.front().front().contains("collisionCheckResolution")) {
          throw std::invalid_argument(
              directoryEntry.path().string() +
              ": Context configurations must specify the collision check resolution of the context "
              "in a field named \"collisionCheckResolution\".");
        }

        // Check that the default configs don't specify the same parameter twice.
        for (const auto &entry : config.front().items()) {
          if (parameters_["context"].contains(entry.key())) {
            throw std::invalid_argument(directoryEntry.path().string() +
                                        ": Overwrites existing config for context '"s +
                                        entry.key().c_str() + "'."s);
          }
        }
        parameters_.merge_patch(config);
        OMPL_INFORM("Loaded configuration from '%s'", directoryEntry.path().c_str());
        // No need to close config, std::ifstreams are closed on destruction.
      }
    } else {
      // Cannot find default context config at default location.
      auto msg = "Default context config directory does not exist at '"s +
                 defaultContextConfigDirectory.string() + "'."s;
      throw std::ios_base::failure(msg.c_str());
    }
    OMPL_INFORM("Loaded default context configs.");
  } else {
    OMPL_INFORM("Not loading default context configs.");
  }

  // Load the default objective configs.
  if (loadDefaultObjectiveConfigs) {
    fs::path defaultObjectiveConfigDirectory(Directory::SOURCE / "parameters/defaults/objectives/");
    if (fs::exists(defaultObjectiveConfigDirectory)) {
      // Load all files in this directory as patches.
      for (auto &directoryEntry : fs::directory_iterator(defaultObjectiveConfigDirectory)) {
        // Make sure the file exists.
        if (!fs::exists(directoryEntry.path())) {
          OMPL_WARN("'%s' contains irregular file '%s' which is not loaded into the configuration.",
                    defaultObjectiveConfigDirectory.c_str(), directoryEntry.path().c_str());
          continue;
        }

        // Make sure the file can be opened.
        std::ifstream configFile(directoryEntry.path().string());
        if (configFile.fail()) {
          OMPL_ERROR("File '%s' exists but cannot be opened.",
                     directoryEntry.path().string().c_str());
          throw std::ios_base::failure("Configuration error.");
        }

        // Load the file into a temporary config.
        json::json config;
        configFile >> config;

        // Make sure the config is a sane planner configuration.
        if (!config.contains("objective")) {
          throw std::invalid_argument("Default objective configuration at '"s +
                                      directoryEntry.path().string() +
                                      "' is not a valid objective configuration. Valid objective "
                                      "configurations must be encapsulated in a 'objective' key."s);
        }
        if (config.size() > 1) {
          throw std::invalid_argument("Configuration at '"s + directoryEntry.path().string() +
                                      "' must only contain one objective configuration."s);
        }
        if (!config.front().front().contains("type")) {
          throw std::invalid_argument(
              directoryEntry.path().string() +
              ": Objective configurations must specify the type of the context in a field named "
              "\"type\".");
        }

        // Check that the default configs don't specify the same parameter twice.
        for (const auto &entry : config.front().items()) {
          if (parameters_["objective"].contains(entry.key())) {
            throw std::invalid_argument(directoryEntry.path().string() +
                                        ": Overwrites existing config for objective '"s +
                                        entry.key().c_str() + "'."s);
          }
        }
        parameters_.merge_patch(config);
        OMPL_INFORM("Loaded configuration from '%s'", directoryEntry.path().c_str());
        // No need to close config, std::ifstreams are closed on destruction.
      }
    } else {
      // Cannot find default objective config at default location.
      auto msg = "Default objective config directory does not exist at '"s +
                 defaultObjectiveConfigDirectory.string() + "'."s;
      throw std::ios_base::failure(msg.c_str());
    }
    OMPL_INFORM("Loaded default objective configs.");
  } else {
    OMPL_INFORM("Not loading default objective configs.");
  }
}

void Configuration::loadReportConfig(const std::experimental::filesystem::path &path) {
  if (fs::exists(path)) {
    // Make sure the file can be opened.
    std::ifstream configFile(path.string());
    if (configFile.fail()) {
      OMPL_ERROR("File '%s' exists but cannot be opened.", path.string().c_str());
      throw std::ios_base::failure("Configuration error.");
    }

    // Load the file into a temporary config.
    json::json config;
    configFile >> config;

    // Make sure the config is a sane report configuration.
    const std::vector<std::string> necessaryKeys = {"colors",
                                                    "successPlots",
                                                    "medianCostPlots",
                                                    "medianInitialSolutionPlots",
                                                    "initialSolutionScatterPlots",
                                                    "initialSolutionPlots",
                                                    "costPercentileEvolutionPlots",
                                                    "statistics"};
    const std::vector<std::string> necessarySubkeys = {"axisWidth",   "axisHeight",  "xminorgrids",
                                                       "xmajorgrids", "yminorgrids", "ymajorgrids",
                                                       "xlog",        "markSize"};
    for (const auto &key : necessaryKeys) {
      if (!config.contains(key)) {
        throw std::invalid_argument(
            "Report configuration at '"s + path.string() +
            "' is not a valid report configuration. Valid report configurations must contain a '"s +
            key + "' key."s);
      } else if (key != "colors"s && key != "statistics"s) {
        for (const auto &subKey : necessarySubkeys) {
          if (!config[key].contains(subKey)) {
            throw std::invalid_argument(
                "Report configuration at '"s + path.string() +
                "' is not a valid report configuration. Valid report configurations must contain a '"s +
                key + "' key with a '" + subKey + "' subkey."s);
          }
        }
      }
    }

    parameters_.merge_patch(config);
    OMPL_INFORM("Loaded configuration from '%s'", path.c_str());
    // No need to close config, std::ifstreams are closed on destruction.
  } else {
    throw std::invalid_argument("Could not load report config from '"s + path.string() + "'."s);
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

void Configuration::registerAsExperiment() {
  // Check the status of the working directory.
  if (Version::GIT_STATUS == "DIRTY"s) {
    OMPL_WARN("Working directory is dirty.");
  }

  // Make sure we're executing the right executable.
  if (parameters_.contains("experiment")) {
    // Check we're on the same commit.
    if (parameters_["experiment"].contains("version")) {
      if (parameters_["experiment"]["version"].contains("commit")) {
        auto commitHash = parameters_["experiment"]["version"]["commit"].get<std::string>();
        if (commitHash != "any"s) {
          if (commitHash != Version::GIT_SHA1) {
            OMPL_ERROR("Config specifies commit %s. You are currently on %s.", commitHash.c_str(),
                       Version::GIT_SHA1.c_str());
          }
        }
      }
    }

    // Check this is the correct executable.
    if (parameters_["experiment"].contains("executable")) {
      auto executable = parameters_["experiment"]["executable"].get<std::string>();
      if (executable != "any"s) {
        if (executable != executable_) {
          OMPL_ERROR("Config specifies executable '%s'. You are executing '%s'.",
                     executable.c_str(), executable_.c_str());
        }
      }

      // Handle executable specific configuration.
      if (executable == "benchmark"s) {
        if (parameters_["experiment"].contains("report")) {
          if (!parameters_["experiment"]["report"].contains("automatic")) {
            parameters_["experiment"]["report"]["automatic"] = false;
            parameters_["experiment"]["report"]["verboseCompilation"] = false;
          }
        } else {
          parameters_["experiment"]["report"]["automatic"] = false;
          parameters_["experiment"]["report"]["verboseCompilation"] = false;
        }
        accessedParameters_["experiment"]["report"]["automatic"] =
            parameters_["experiment"]["report"]["automatic"];
        accessedParameters_["experiment"]["report"]["verboseCompilation"] =
            parameters_["experiment"]["report"]["verboseCompilation"];
      }
      if (parameters_["experiment"]["report"].contains("config")) {
        OMPL_INFORM("Loading custom report config.");
        loadReportConfig(parameters_["experiment"]["report"]["config"].get<std::string>());
      } else {
        OMPL_INFORM("Loading default report config.");
        loadReportConfig(Directory::SOURCE / "parameters/defaults/esp_default_report_config.json");
      }
    }
  }

  // Store the executable name.
  accessedParameters_["experiment"]["executable"] = executable_;

  // Store the current version.
  accessedParameters_["experiment"]["version"]["commit"] = Version::GIT_SHA1;
  accessedParameters_["experiment"]["version"]["branch"] = Version::GIT_REFSPEC;
  accessedParameters_["experiment"]["version"]["status"] = Version::GIT_STATUS;

  // This ensures we don't load any additional context or planner config when rerunning this
  // experiment.
  accessedParameters_["experiment"]["loadDefaultContextConfig"] = false;
  accessedParameters_["experiment"]["loadDefaultPlannerConfig"] = false;

  // Handle seed specifications.
  handleSeedSpecification();
}

void Configuration::handleSeedSpecification() {
  if (parameters_["experiment"].contains("seed")) {
    auto seed = parameters_["experiment"]["seed"].get<unsigned long>();
    ompl::RNG::setSeed(seed);
    OMPL_WARN("Configuration set seed to be %lu", seed);
    accessedParameters_["experiment"]["seed"] = seed;
  } else {
    auto seed = ompl::RNG::getSeed();
    OMPL_INFORM("Seed is %lu", seed);
    parameters_["experiment"]["seed"] = seed;
    accessedParameters_["experiment"]["seed"] = seed;
  }
}

}  // namespace ompltools

}  // namespace esp
