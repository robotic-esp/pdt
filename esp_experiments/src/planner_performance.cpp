/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2017     University of Toronto
 *  Copyright (c) 2018-present  University of Oxford
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
 *   * Neither the names of the copyright holders nor the names of its
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

#include <functional>
#include <future>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

#include <experimental/filesystem>

#include <ompl/util/Console.h>

#include "esp_configuration/configuration.h"
#include "esp_factories/context_factory.h"
#include "esp_factories/planner_factory.h"
#include "esp_performance_loggers/performance_loggers.h"
#include "esp_planning_contexts/all_contexts.h"
#include "esp_time/time.h"
#include "esp_utilities/get_best_cost.h"

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

int main(int argc, char **argv) {
  // Read the config files.
  auto config = std::make_shared<esp::ompltools::Configuration>(argc, argv);
  config->registerAsExperiment();

  // Record the experiment start time.
  auto experimentStartTime = std::chrono::system_clock::now();
  auto experimentStartTimeString = esp::ompltools::time::toDateString(experimentStartTime);

  // Create the context for this experiment.
  esp::ompltools::ContextFactory contextFactory(config);
  auto context = contextFactory.create(config->get<std::string>("Experiment/context"));

  // Create a planner factory for planners in this context.
  esp::ompltools::PlannerFactory plannerFactory(config, context);

  // Report the planner names to the console.
  std::cout << '\n';
  for (const auto &plannerName : config->get<std::vector<std::string>>("Experiment/planners")) {
    std::cout << std::setw(7) << std::setfill(' ') << ' ' << std::setw(21) << std::setfill(' ')
              << std::left << std::string(plannerName);
  }
  std::cout << '\n';

  // Create a name for this experiment.
  std::string experimentName = experimentStartTimeString + '_' + context->getName();

  // Create the directory for the results of this experiment to live in.
  fs::path experimentDirectory(config->get<std::string>("Experiment/executable") + "_logs/"s +
                               experimentName);

  // Create the performance log.
  esp::ompltools::ResultLog<esp::ompltools::TimeCostLogger> results(
      experimentDirectory / experimentName += "_results.csv"s);

  // May the best planner win.
  for (std::size_t i = 0; i < config->get<std::size_t>("Experiment/numRuns"); ++i) {
    std::cout << '\n' << std::setw(4) << std::right << std::setfill(' ') << i << " | ";
    for (const auto &plannerName : config->get<std::vector<std::string>>("Experiment/planners")) {
      // Create the logger for this run.
      esp::ompltools::TimeCostLogger logger(context->getTargetDuration(),
                                            config->get<std::size_t>("Experiment/logFrequency"));

      // Allocate the planner.
      auto [planner, plannerType] = plannerFactory.create(plannerName);

      // Set it up.
      const auto setupStartTime = esp::ompltools::time::Clock::now();
      planner->setup();
      const auto setupDuration = esp::ompltools::time::Clock::now() - setupStartTime;

      // Compute the duration we have left for solving.
      const auto maxSolveDuration =
          esp::ompltools::time::seconds(context->getTargetDuration() - setupDuration);
      const std::chrono::microseconds idle(1000000u /
                                           config->get<std::size_t>("Experiment/logFrequency"));

      // Solve the problem on a separate thread.
      esp::ompltools::time::Clock::time_point addMeasurementStart;
      const auto solveStartTime = esp::ompltools::time::Clock::now();
      std::future<void> future = std::async(std::launch::async, [&planner, &maxSolveDuration]() {
        planner->solve(maxSolveDuration);
      });

      // Log the intermediate best costs.
      do {
        addMeasurementStart = esp::ompltools::time::Clock::now();
        logger.addMeasurement(setupDuration + (esp::ompltools::time::Clock::now() - solveStartTime),
                              esp::ompltools::utilities::getBestCost(planner, plannerType));
      } while (future.wait_until(addMeasurementStart + idle) != std::future_status::ready);
      future.get();

      // Get the final runtime.
      const auto totalDuration =
          setupDuration + (esp::ompltools::time::Clock::now() - solveStartTime);

      // Store the final cost.
      const auto problem = planner->getProblemDefinition();
      if (problem->hasExactSolution()) {
        logger.addMeasurement(
            totalDuration, problem->getSolutionPath()->cost(context->getOptimizationObjective()));
      } else {
        logger.addMeasurement(totalDuration,
                              ompl::base::Cost(std::numeric_limits<double>::infinity()));
      }

      // Add this run to the log and report it to the console.
      results.addResult(planner->getName(), logger);
      const auto result = logger.lastMeasurement();
      std::cout << std::setw(17) << std::left << result.first << std::setw(8) << std::fixed
                << result.second << " | " << std::flush;
    }
  }

  // Register the end time of the experiment.
  auto experimentEndTime = std::chrono::system_clock::now();
  auto experimentEndTimeString = esp::ompltools::time::toDateString(experimentEndTime);
  esp::ompltools::time::Duration experimentDuration = experimentEndTime - experimentStartTime;

  // Log some info to a log file.
  auto logPath = experimentDirectory / experimentName += "_log.txt"s;
  ompl::msg::OutputHandlerFile log(logPath.c_str());
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_INFO);
  ompl::msg::useOutputHandler(&log);
  OMPL_INFORM("Start of experiment: '%s'",
              esp::ompltools::time::toDateString(experimentStartTime).c_str());
  OMPL_INFORM("End of experiment: '%s'",
              esp::ompltools::time::toDateString(experimentEndTime).c_str());
  OMPL_INFORM("Duration of experiment: '%s'",
              esp::ompltools::time::toDurationString(experimentDuration).c_str());
  OMPL_INFORM("Wrote results to '%s'", results.getFilePath().c_str());

  // Dump the accessed parameters next to the results file.
  auto configPath = experimentDirectory / experimentName += "_config.json"s;
  config->add<std::string>("Experiment/results", results.getFilePath());
  config->dumpAccessed(fs::current_path().string() + '/' + configPath.string());
  OMPL_INFORM("Wrote configuration to '%s'", configPath.c_str());

  // Report success.
  std::cout << "\n\nExperiment ran for:\t" << (experimentEndTime - experimentStartTime) << "\t\t("
            << experimentStartTimeString << " -- " << experimentEndTimeString << ")\n"
            << "\nWrote results to:\t" << results.getFilePath() << "\nWrote config to:\t"
            << fs::current_path() / configPath << "\nWrote log to:\t\t"
            << fs::current_path() / logPath << "\n\n";

  return 0;
}
