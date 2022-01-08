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

#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/Console.h>

#include "esp_configuration/configuration.h"
#include "esp_factories/context_factory.h"
#include "esp_factories/planner_factory.h"
#include "esp_performance_loggers/performance_loggers.h"
#include "esp_planning_contexts/all_contexts.h"
#include "esp_statistics/statistics.h"
#include "esp_tikz/experiment_report.h"
#include "esp_time/time.h"
#include "esp_utilities/get_best_cost.h"

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

int main(const int argc, const char **argv) {
  // Read the config files.
  auto config = std::make_shared<esp::ompltools::Configuration>(argc, argv);
  config->registerAsExperiment();

  // Record the experiment start time.
  auto experimentStartTime = std::chrono::system_clock::now();
  auto experimentStartTimeString = esp::ompltools::time::toDateString(experimentStartTime);

  // Create the context for this experiment.
  esp::ompltools::ContextFactory contextFactory(config);
  auto context = contextFactory.create(config->get<std::string>("experiment/context"));

  // Create a planner factory for planners in this context.
  esp::ompltools::PlannerFactory plannerFactory(config, context);

  // Print some basic info about this benchmark.
  auto estimatedRuntime = config->get<std::size_t>("experiment/numRuns") *
                          config->get<std::vector<std::string>>("experiment/planners").size() *
                          context->getMaxSolveDuration();
  auto estimatedDoneBy =
      esp::ompltools::time::toDateString(std::chrono::time_point_cast<std::chrono::nanoseconds>(
          std::chrono::time_point_cast<esp::ompltools::time::Duration>(experimentStartTime) +
          estimatedRuntime));
  std::cout << "\nBenchmark parameters\n";
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
            << std::setfill('.') << "Number of planners" << std::setw(20) << std::right
            << config->get<std::vector<std::string>>("experiment/planners").size() << '\n';
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
            << std::setfill('.') << "Number of runs per planner" << std::setw(20) << std::right
            << config->get<std::size_t>("experiment/numRuns") << '\n';
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
            << std::setfill('.') << "Maximum time per run" << std::setw(20) << std::right
            << context->getMaxSolveDuration().count() << " s\n";
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
            << std::setfill('.') << "Cost log frequency" << std::setw(20) << std::right
            << config->get<std::size_t>("experiment/logFrequency") << " Hz\n";
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
            << std::setfill('.') << "Expected runtime no more than" << std::setw(20) << std::right
            << esp::ompltools::time::toDurationString(estimatedRuntime) << " HH:MM:SS\n";
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
            << std::setfill('.') << "Expected to be done before" << std::setw(20) << std::right
            << estimatedDoneBy << " YYYY-MM-DD_HH-MM-SS\n";
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
            << std::setfill('.') << "Random number seed" << std::setw(20) << std::right
            << ompl::RNG::getSeed() << '\n';

  // Print some info about the context of this benchmark.
  std::cout << "\nContext parameters\n";
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
            << std::setfill('.') << "Context name" << std::setw(20) << std::right
            << config->get<std::string>("experiment/context") << '\n';
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
            << std::setfill('.') << "Problem dimension" << std::setw(20) << std::right
            << context->getDimension() << '\n';
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
            << std::setfill('.') << "State space" << std::setw(20) << std::right
            << context->getStateSpace()->getName() << '\n';
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
            << std::setfill('.') << "State space measure" << std::setw(20) << std::right
            << context->getSpaceInformation()->getSpaceMeasure() << '\n';
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
            << std::setfill('.') << "State space maximum extent" << std::setw(20) << std::right
            << context->getSpaceInformation()->getMaximumExtent() << '\n';
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
            << std::setfill('.') << "Relative collision resolution" << std::setw(20) << std::right
            << context->getSpaceInformation()->getStateValidityCheckingResolution() << '\n';
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
            << std::setfill('.') << "Absolute collision resolution" << std::setw(20) << std::right
            << context->getStateSpace()->getLongestValidSegmentLength() << '\n';
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
            << std::setfill('.') << "Objective" << std::setw(20) << std::right
            << context->getObjective()->getDescription() << '\n';
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
            << std::setfill('.') << "Cost threshold" << std::setw(20) << std::right
            << context->getObjective()->getCostThreshold() << '\n';

  // Setup the results table.
  std::cout << "\nTesting planners\n";
  for (const auto &plannerName : config->get<std::vector<std::string>>("experiment/planners")) {
    std::cout << std::setw(2) << std::setfill(' ') << ' ' << plannerName << '\n';
  }
  std::cout << "\nBenchmark\n";

  // Create a name for this experiment.
  std::string experimentName = experimentStartTimeString + "_" + context->getName();
  config->add<std::string>("experiment/name", experimentName);

  // Create the directory for the results of this experiment to live in.
  fs::path experimentDirectory(config->get<std::string>("experiment/executable") + "s/"s +
                               experimentName);

  // Create the performance log.
  esp::ompltools::ResultLog<esp::ompltools::TimeCostLogger> results(experimentDirectory /
                                                                    "results.csv"s);

  // Add the result path to the experiment.
  config->add<std::string>("experiment/results", results.getFilePath());

  // Compute the total number of runs.
  const auto totalNumberOfRuns =
      config->get<std::size_t>("experiment/numRuns") *
      config->get<std::vector<std::string>>("experiment/planners").size();
  auto currentRun = 0u;

  // May the best planner win.
  for (std::size_t i = 0; i < config->get<std::size_t>("experiment/numRuns"); ++i) {
    // Randomly shuffle the planners.
    auto plannerNames = config->get<std::vector<std::string>>("experiment/planners");
    std::random_shuffle(plannerNames.begin(), plannerNames.end());

    for (const auto &plannerName : plannerNames) {
      // Create the logger for this run.
      esp::ompltools::TimeCostLogger logger(context->getMaxSolveDuration(),
                                            config->get<double>("experiment/logFrequency"));

      // The following results in more consistent measurements. I don't fully understand why, but it
      // seems to be connected to creating a separate thread.
      {
        auto reconciler =
            std::make_shared<ompl::geometric::RRTConnect>(context->getSpaceInformation());
        reconciler->setName("ReconcilingPlanner");
        reconciler->setProblemDefinition(context->instantiateNewProblemDefinition());
        auto hotpath = std::async(std::launch::async, [&reconciler]() {
          reconciler->solve(ompl::base::timedPlannerTerminationCondition(0.0));
        });
        hotpath.get();
      }

      // Allocate the planner.
      auto [planner, plannerType] = plannerFactory.create(plannerName);

      // Set it up.
      const auto setupStartTime = esp::ompltools::time::Clock::now();
      planner->setup();
      const auto setupDuration = esp::ompltools::time::Clock::now() - setupStartTime;

      // Compute the duration we have left for solving.
      const auto maxSolveDuration =
          esp::ompltools::time::seconds(context->getMaxSolveDuration() - setupDuration);
      const std::chrono::microseconds idle(1000000u /
                                           config->get<std::size_t>("experiment/logFrequency"));

      // Solve the problem on a separate thread.
      esp::ompltools::time::Clock::time_point addMeasurementStart;
      const auto solveStartTime = esp::ompltools::time::Clock::now();
      std::future<void> future = std::async(std::launch::async, [&planner, &maxSolveDuration]() {
        planner->solve(maxSolveDuration);
      });

      // Log the intermediate best costs.
      do {
        addMeasurementStart = esp::ompltools::time::Clock::now();
        logger.addMeasurement(setupDuration + (addMeasurementStart - solveStartTime),
                              esp::ompltools::utilities::getBestCost(planner, plannerType));

        // Stop logging intermediate best costs if the planner overshoots.
        if (esp::ompltools::time::seconds(addMeasurementStart - solveStartTime) >
            maxSolveDuration) {
          break;
        }
      } while (future.wait_until(addMeasurementStart + idle) != std::future_status::ready);

      // Wait until the planner returns.
      OMPL_DEBUG(
          "Stopped logging results for planner '%s' because it overshot the termination condition.",
          plannerName.c_str());
      future.get();

      // Get the final runtime.
      const auto totalDuration =
          setupDuration + (esp::ompltools::time::Clock::now() - solveStartTime);

      // Store the final cost.
      const auto problem = planner->getProblemDefinition();
      if (problem->hasExactSolution()) {
        logger.addMeasurement(totalDuration,
                              problem->getSolutionPath()->cost(context->getObjective()));
      } else {
        logger.addMeasurement(totalDuration,
                              ompl::base::Cost(std::numeric_limits<double>::infinity()));
      }

      // Add this run to the log and report it to the console.
      results.addResult(planner->getName(), logger);

      // Compute the progress.
      ++currentRun;
      const auto progress = static_cast<float>(currentRun) / static_cast<float>(totalNumberOfRuns);
      constexpr auto barWidth = 36;
      std::cout << '\r' << std::setw(2) << std::setfill(' ') << std::right << ' ' << "Progress"
                << (std::ceil(progress * barWidth) != barWidth
                    ? std::setw(static_cast<int>(std::ceil(progress * barWidth)))
                    : std::setw(static_cast<int>(std::ceil(progress * barWidth) - 1u)))
                << std::setfill('.') << (currentRun != totalNumberOfRuns ? '|' : '.') << std::right
                << std::setw(barWidth - static_cast<int>(std::ceil(progress * barWidth)))
                << std::setfill('.') << '.'
                << std::right << std::fixed << std::setw(6) << std::setfill(' ')
                << std::setprecision(2) << progress * 100.0f << " %" << std::flush;
    }
  }

  // dump the complete config to make sure that we can produce the report once we ran the experiment
  auto configPath = experimentDirectory / "config.json"s;
  config->dumpAll(fs::current_path().string() + '/' + configPath.string());

  // Register the end time of the experiment.
  auto experimentEndTime = std::chrono::system_clock::now();
  auto experimentEndTimeString = esp::ompltools::time::toDateString(experimentEndTime);
  esp::ompltools::time::Duration experimentDuration = experimentEndTime - experimentStartTime;

  // Report the elapsed time and some statistics.
  std::cout << '\n'
            << std::setw(2u) << std::setfill(' ') << ' ' << std::setw(30) << std::setfill('.')
            << std::left << "Elapsed time" << std::setw(20) << std::right
            << esp::ompltools::time::toDurationString(experimentEndTime - experimentStartTime)
            << " HH:MM:SS\n"
            << std::setw(2u) << std::setfill(' ') << ' ' << std::setw(30) << std::setfill('.')
            << std::left << "Number of checked motions" << std::setw(20) << std::right
            << context->getSpaceInformation()->getCheckedMotionCount() << '\n'
            << std::setw(2u) << std::setfill(' ') << ' ' << std::setw(30) << std::setfill('.')
            << std::left << "Percentage of valid motions" << std::setw(20) << std::right
            << context->getSpaceInformation()->getMotionValidator()->getValidMotionFraction() *
                   100.0f
            << " %\n";

  // Inform that the report is being compiled.
  std::cout << "\nReport\n"
            << std::setw(2u) << std::setfill(' ') << ' '
            << "Compiling (this may take a couple of minutes)" << std::flush;

  // Generate the statistic.
  esp::ompltools::Statistics stats(config, true);

  // Generate the report.
  esp::ompltools::ExperimentReport report(config, stats);
  report.generateReport();
  auto reportPath = report.compileReport();

  // Inform that we are done compiling the report.
  std::cout << '\r' << std::setw(47u) << std::setfill(' ') << ' ' << '\r' << std::setw(2u)
            << std::setfill(' ') << ' ' << "Compilation done\n"
            << std::flush;

  // Log some info to a log file.
  auto logPath = experimentDirectory / "log.txt"s;
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

  // Inform where we wrote the report to.
  std::cout << std::setw(2u) << std::setfill(' ') << ' ' << "Location " << reportPath << "\n\n";

  // Dump the accessed parameters next to the results file.
  // This overwrites the previously dumped config with one that only consists of the
  // accessed parameters.
  config->dumpAccessed(fs::current_path().string() + '/' + configPath.string());

  return 0;
}
