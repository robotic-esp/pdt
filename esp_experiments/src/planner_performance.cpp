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

#include <ompl/util/Console.h>
#include <boost/thread/thread.hpp>

#include "esp_configuration/configuration.h"
#include "esp_factories/context_factory.h"
#include "esp_factories/planner_factory.h"
#include "esp_performance_loggers/performance_loggers.h"
#include "esp_planning_contexts/all_contexts.h"
#include "esp_utilities/time.h"

int main(int argc, char **argv) {
  // Read the config files.
  auto config = std::make_shared<esp::ompltools::Configuration>(argc, argv);

  // Get the config for this experiment.
  auto experimentConfig = config->getExperimentConfig();

  // Create the context for this experiment.
  esp::ompltools::ContextFactory contextFactory(config);
  auto context = contextFactory.create(experimentConfig["context"]);

  // Create a planner factory for planners in this context.
  esp::ompltools::PlannerFactory plannerFactory(config, context);

  // Let's keep the console output for now, I can create a nicer pango visualization later.
  for (const auto &plannerType : experimentConfig["planners"]) {
    std::cout << "      " << std::setw(22) << std::setfill(' ') << std::left
              << std::string(plannerType);
  }
  std::cout << '\n';

  // Let's dance.
  for (std::size_t i = 0; i < experimentConfig["numRuns"]; ++i) {
    std::cout << '\n' << std::setw(4) << std::right << std::setfill('0') << i << ": ";
    for (const auto &plannerType : experimentConfig["planners"]) {
      // Allocate the planner.
      auto planner = plannerFactory.create(plannerType);

      // Set it up.
      auto setupStartTime = esp::ompltools::time::Clock::now();
      planner->setup();
      auto setupDuration = esp::ompltools::time::Clock::now() - setupStartTime;

      // Compute the duration we have left for solving.
      auto maxSolveDuration =
          esp::ompltools::time::seconds(context->getTargetDuration() - setupDuration);

      // Make sure the clock is steady.
      if (!esp::ompltools::time::Clock::is_steady) {
        boost::this_thread::sleep_for(boost::chrono::microseconds(1));
      }

      // Solve the problem on a separate thread.
      auto solveStartTime = esp::ompltools::time::Clock::now();
      boost::thread solveThread(
          [&planner, &maxSolveDuration]() { planner->solve(maxSolveDuration); });
      do {
        // Log progress here.
      } while (!solveThread.try_join_for(boost::chrono::microseconds(1000)));
      auto solveDuration = esp::ompltools::time::Clock::now() - solveStartTime;

      auto result = std::make_pair(setupDuration + solveDuration,
                                   planner->getProblemDefinition()->getSolutionPath()->cost(
                                       context->getOptimizationObjective()));

      std::cout << std::setw(17) << std::left << result.first << ' ' << std::setw(8) << std::fixed
                << result.second << "  ";
    }
  }

  // config->dumpAccessed();

  return 0;
}
