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

/* Authors: Marlin Strub */

#include <functional>
#include <iomanip>
#include <iostream>
#include <vector>

#include <experimental/filesystem>

#include <ompl/util/Console.h>
#include <boost/thread/thread.hpp>

#include "esp_configuration/configuration.h"
#include "esp_factories/context_factory.h"
#include "esp_factories/planner_factory.h"
#include "esp_performance_loggers/performance_loggers.h"
#include "esp_planning_contexts/all_contexts.h"
#include "esp_time/time.h"
#include "esp_utilities/get_best_cost.h"

int main(int argc, char **argv) {
  // Read the config files.
  auto config = std::make_shared<esp::ompltools::Configuration>(argc, argv);

  // Record the experiment start time.
  auto experimentStartTime = std::chrono::system_clock::now();
  auto experimentStartTimeString = esp::ompltools::time::toDateString(experimentStartTime);
  config->addToMiscField("start", experimentStartTimeString);

  // Get the config for this experiment.
  auto experimentConfig = config->getExperimentConfig();

  // Create the context for this experiment.
  esp::ompltools::ContextFactory contextFactory(config);
  auto context = contextFactory.create(experimentConfig["context"]);

  // Create a planner factory for planners in this context.
  esp::ompltools::PlannerFactory plannerFactory(config, context);

  // Let's keep the console output for now, I can create a nicer pango visualization later.
  std::cout << '\n';
  for (const auto &plannerType : experimentConfig["planners"]) {
    std::cout << std::setw(7) << std::setfill(' ') << ' ' << std::setw(21) << std::setfill(' ')
              << std::left << std::string(plannerType);
  }
  std::cout << '\n';

  // Prepare the performance logger.
  esp::ompltools::PerformanceLog<esp::ompltools::TimeCostLogger> log(
      experimentConfig["executable"].get<std::string>() + std::string("_logs/") +
      experimentStartTimeString + '_' + context->getName() + std::string(".csv"));

  // May the best planner win.
  for (std::size_t i = 0; i < experimentConfig["numRuns"]; ++i) {
    std::cout << '\n' << std::setw(4) << std::right << std::setfill(' ') << i << " | ";
    for (const auto &plannerName : experimentConfig["planners"]) {
      // Create the logger for this run.
      esp::ompltools::TimeCostLogger logger(context->getTargetDuration(),
                                            experimentConfig["logFrequency"]);
      // Allocate the planner.
      auto [planner, plannerType] = plannerFactory.create(plannerName);

      // Set it up.
      auto setupStartTime = esp::ompltools::time::Clock::now();
      planner->setup();
      auto setupDuration = esp::ompltools::time::Clock::now() - setupStartTime;

      // Compute the duration we have left for solving.
      auto maxSolveDuration =
          esp::ompltools::time::seconds(context->getTargetDuration() - setupDuration);

      // Solve the problem on a separate thread.
      auto solveStartTime = esp::ompltools::time::Clock::now();
      boost::thread solveThread(
          [&planner, &maxSolveDuration]() { planner->solve(maxSolveDuration); });
      // Log the intermediate best costs.
      do {
        logger.addMeasurement(setupDuration + (esp::ompltools::time::Clock::now() - solveStartTime),
                              esp::ompltools::utilities::getBestCost(planner, plannerType));
      } while (!solveThread.try_join_for(
          boost::chrono::duration<double>(1.0 / experimentConfig["logFrequency"].get<double>())));

      // Get the final runtime.
      auto totalDuration = setupDuration + (esp::ompltools::time::Clock::now() - solveStartTime);

      // Store the final cost.
      auto problem = planner->getProblemDefinition();
      if (problem->hasExactSolution()) {
        logger.addMeasurement(
            totalDuration, problem->getSolutionPath()->cost(context->getOptimizationObjective()));
      } else {
        logger.addMeasurement(totalDuration,
                              ompl::base::Cost(std::numeric_limits<double>::infinity()));
      }

      // Add this run to the log and report it to the console.
      log.addResult(planner->getName(), logger);
      auto result = logger.lastMeasurement();
      std::cout << std::setw(17) << std::left << result.first << std::setw(8) << std::fixed
                << result.second << " | " << std::flush;
    }
  }

  // Register the end time of the experiment.
  auto experimentEndTime = std::chrono::system_clock::now();
  auto experimentEndTimeString = esp::ompltools::time::toDateString(experimentEndTime);
  config->addToMiscField("end", experimentEndTimeString);

  // Get the duration of the experiment.
  esp::ompltools::time::Duration experimentDuration = experimentEndTime - experimentStartTime;
  config->addToMiscField("duration", esp::ompltools::time::toDurationString(experimentDuration));

  // Dump all accessed parameters right next to the log file.
  auto logPath = std::experimental::filesystem::path(log.getFilename());
  auto configPath = logPath.parent_path() /= logPath.stem() += ".json";
  config->addToMiscField("resultFile", logPath.string());
  config->addToMiscField("configFile", configPath.string());
  config->dumpAccessed(configPath.string());

  // Report success.
  std::cout << "\n\nExperiment ran for: " << (experimentEndTime - experimentStartTime)
            << " (Start: " << experimentStartTimeString << ", End: " << experimentEndTimeString
            << ")\n"
            << "\nWrote results to: " << logPath.string()
            << "\nWrote config to: " << configPath.string() << "\n\n";

  return 0;
}
