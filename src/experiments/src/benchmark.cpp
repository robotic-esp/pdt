/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014--2022
 *  Estimation, Search, and Planning (ESP) Research Group
 *  All rights reserved
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
 *   * Neither the names of the organizations nor the names of its
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
#include <tuple>

#include <experimental/filesystem>

#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/Console.h>

#include "pdt/config/configuration.h"
#include "pdt/factories/context_factory.h"
#include "pdt/factories/planner_factory.h"
#include "pdt/loggers/performance_loggers.h"
#include "pdt/planning_contexts/all_contexts.h"
#include "pdt/reports/multiquery_report.h"
#include "pdt/reports/single_query_report.h"
#include "pdt/statistics/planning_statistics.h"
#include "pdt/time/time.h"
#include "pdt/time/CumulativeTimer.h"
#include "pdt/utilities/get_best_cost.h"

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

constexpr auto barWidth = 36;

// Function to break a long message across multiple lines at spaces
std::string linebreak(const std::string &in, const size_t lineWidth)
{
  std::string out = in;
  std::string::size_type curLength = 0u;
  for (std::string::size_type i = 0u; i < in.size(); ++i) {
    if (curLength == lineWidth) {
      if (in[i] == ' ') {
        out[i] = '\n';
        curLength = 0u;
      } else {
        for (std::string::size_type j = i - 1u; j >= 1u; --j) {
          if (out[j] == ' ') {
            out[j] = '\n';
            curLength = i - j;
            break;
          }
        }
      }
    } else {
      ++curLength;
    }
  }
  return out;
}

// Function to check if the ProblemDefinitions defined by the Context is actually valid.
bool checkContextValidity(const std::shared_ptr<pdt::config::Configuration> &config, const std::shared_ptr<pdt::planning_contexts::BaseContext> &context) {
  std::cout << "\nContext validation\n";
  if (config->contains("experiment/validateProblemDefinitions") &&
      config->get<bool>("experiment/validateProblemDefinitions")) {
    const std::size_t numQueries = context->getNumQueries();
    double runtime = 10.0;
    if (config->contains("experiment/validateProblemDefinitionsDuration")){
      runtime = config->get<double>("experiment/validateProblemDefinitionsDuration");
    }
    std::string validPlannerName = "defaultRRTConnect";
    if (config->contains("experiment/validateProblemDefinitionsPlanner")){
      validPlannerName = config->get<std::string>("experiment/validateProblemDefinitionsPlanner");
    }

    std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
              << std::setfill('.') << "Planner" << std::setw(20) << std::right
              << validPlannerName << '\n';
    std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
              << std::setfill('.') << "Time per ProblemDefinition" << std::setw(20) << std::right
              << runtime << " s\n";
    std::cout << std::setw(2) << std::setfill(' ') << std::right << ' ' << "Progress"
              << std::left << std::setw(barWidth) << std::setfill('.') << '|' << std::right
              << std::fixed << std::setw(6) << std::setfill(' ') << std::setprecision(2) << 0.0
              << " %" << std::flush;

    pdt::factories::PlannerFactory plannerFactory(config, context);
    std::shared_ptr<ompl::base::Planner> validPlanner;
    std::tie(validPlanner, std::ignore, std::ignore) = plannerFactory.create(validPlannerName);

    for (auto i=0u; i<numQueries; ++i) {
      const auto problemDefinition = context->instantiateNthProblemDefinition(i);
      validPlanner->clear();
      validPlanner->setProblemDefinition(problemDefinition);

      validPlanner->solve(ompl::base::timedPlannerTerminationCondition(runtime));

      const auto progress = static_cast<float>(i + 1u) / static_cast<float>(numQueries);
      std::cout << '\r' << std::setw(2) << std::setfill(' ') << std::right << ' ' << "Progress"
                  << (std::ceil(progress * barWidth) != barWidth
                          ? std::setw(static_cast<int>(std::ceil(progress * barWidth)))
                          : std::setw(static_cast<int>(std::ceil(progress * barWidth) - 1u)))
                  << std::setfill('.') << (i + 1u != numQueries ? '|' : '.') << std::right
                  << std::setw(barWidth - static_cast<int>(std::ceil(progress * barWidth)))
                  << std::setfill('.') << '.' << std::right;
                  if (problemDefinition->hasExactSolution()) {
                     std::cout << std::fixed << std::setw(6) << std::setfill(' ')
                               << std::setprecision(2) << progress * 100.0f << " %";
                  } else {
                    std::cout << "Failed !";
                  }
                  std::cout << std::flush;
      if (!problemDefinition->hasExactSolution()) {
        auto msg = "This context may not be solveable since "s + validPlannerName
                   + " did not find a solution to the ompl::base::ProblemDefinition for query "s
                   + std::to_string(i) + " in "s + std::to_string(runtime) + "s. Please check "s
                   + "the start and goal states, use a different pseudorandom seed if the "s
                   + "problem was randomly generated, and/or increase the validation time "s
                   + "('experiment/validateProblemDefinitionsDuration'). You may also choose to "s
                   + "disable this validation if you know your context is valid "s
                   + "('experiment/validateProblemDefinitions')."s;
        std::cout << "\n\n" << linebreak(msg, 80u) << "\n\n";
        return false;
      }
    }
  } else {
    std::cout << '\r' << std::setw(2) << std::setfill(' ') << std::right << ' ' << "Progress"
                  << std::setw(static_cast<int>(std::ceil(barWidth) - 1u))
                  << std::setfill('.') << '.' << std::right << "Skipped";
  }
  std::cout << '\n';
  return true;
}

int main(const int argc, const char **argv) {
  // Read the config files.
  auto config = std::make_shared<pdt::config::Configuration>(argc, argv);
  config->registerAsExperiment();

  // Record the experiment start time.
  auto experimentStartTime = std::chrono::system_clock::now();
  auto experimentStartTimeString = pdt::time::toDateString(experimentStartTime);

  // Create the context for this experiment.
  pdt::factories::ContextFactory contextFactory(config);
  auto context = contextFactory.create(config->get<std::string>("experiment/context"));

  const std::size_t numQueries = context->getNumQueries();

  // Create a planner factory for planners in this context.
  pdt::factories::PlannerFactory plannerFactory(config, context);

  // Print some basic info about this benchmark.
  auto estimatedRuntime = config->get<std::size_t>("experiment/numRuns") *
                          config->get<std::vector<std::string>>("experiment/planners").size() *
                          context->getMaxSolveDuration() * 
                          numQueries;
  auto estimatedDoneBy =
      pdt::time::toDateString(std::chrono::time_point_cast<std::chrono::nanoseconds>(
          std::chrono::time_point_cast<pdt::time::Duration>(experimentStartTime) +
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
            << pdt::time::toDurationString(estimatedRuntime) << " HH:MM:SS\n";
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
            << std::setfill('.') << "Expected to be done before" << std::setw(20) << std::right
            << estimatedDoneBy << " YYYY-MM-DD_HH-MM-SS\n";
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
            << std::setfill('.') << "Random number seed" << std::setw(20) << std::right
            << ompl::RNG::getSeed() << '\n';

  // Print some info about the context of this benchmark.
  std::cout << "\nContext parameters\n";
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(20)
            << std::setfill('.') << "Context name" << std::setw(30) << std::right
            << config->get<std::string>("experiment/context") << '\n';
  std::cout << std::setw(2) << std::setfill(' ') << ' ' << std::left << std::setw(30)
            << std::setfill('.') << "Number of queries" << std::setw(20) << std::right
            << numQueries << '\n';
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

  if (!checkContextValidity(config, context)) {
      return 0;
  }

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
  fs::path experimentDirectory(fs::path(config->get<std::string>("experiment/baseDirectory")) /
                               experimentName);
  config->add<std::string>("experiment/experimentDirectory",
                           fs::absolute(experimentDirectory).string());

  std::vector<std::string> resultPaths;

  // Preallocate the paths of the files we are about to create
  for (auto i=0u; i<numQueries; ++i){
    const fs::path path = (fs::absolute(experimentDirectory) / ("raw/results_" + std::to_string(i) + ".csv"s));
    resultPaths.push_back(path.string());
  }
  
  // Add the result path to the experiment.
  config->add<std::vector<std::string>>("experiment/results", resultPaths);
  config->add<std::string>("experiment/experimentDirectory", fs::absolute(experimentDirectory).string());

  // Compute the total number of runs.
  const auto totalNumberOfRuns =
      config->get<std::size_t>("experiment/numRuns") *
      config->get<std::vector<std::string>>("experiment/planners").size() * 
      numQueries;
  auto currentRun = 0u;

  // May the best planner win.
  for (auto i = 0u; i < config->get<std::size_t>("experiment/numRuns"); ++i) {
    // Randomly shuffle the planners.
    auto plannerNames = config->get<std::vector<std::string>>("experiment/planners");
    std::random_shuffle(plannerNames.begin(), plannerNames.end());

    // In a multiquery setting: regenerate the start/goal pairs if so desired
    if (i > 0 && 
        config->contains("experiment/regenerateQueries") && 
        config->get<bool>("experiment/regenerateQueries")){
       context->regenerateQueries();
    }

    // If multiple starts/goal queries are defined (i.e. we evaluate a multiquery setting),
    // the planners run _all_ queries before the next planner runs the _same_ queries.
    for (const auto &plannerName : plannerNames) {
      // Allocate and run a dummy planner before allocating the actual planner.
      // This results in more consistent measurements. I don't fully understand why, but it
      // seems to be connected to running the planner in a separate thread.
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

      // Allocate the planner to be tested.
        std::shared_ptr<ompl::base::Planner> planner;
        pdt::common::PLANNER_TYPE plannerType;
        pdt::time::Duration factoryDuration;
        std::tie(planner, plannerType, factoryDuration) = plannerFactory.create(plannerName);

      for (auto j=0u; j<numQueries; ++j){
        pdt::time::CumulativeTimer configTimer;
        // Create the logger for this run.
        pdt::loggers::TimeCostLogger logger(context->getMaxSolveDuration(),
                                              config->get<double>("experiment/logFrequency"));

        // Prepare the planner for this query.
        pdt::time::Duration querySetupDuration = std::chrono::seconds{0};
        if (j == 0){
          // The PlannerFactory starts the planner with the 0th query.
          // Set the planner up.
          configTimer.start();
          planner->setup();
          configTimer.stop();

          // Time is construction (from PlannerFactory) and setup.
          querySetupDuration = factoryDuration + configTimer.duration();
        }
        else {
          // Clear the current query
          configTimer.start();
          planner->clearQuery();
          configTimer.stop();

          // get the problem setting for the nth query
          const auto problemDefinition = context->instantiateNthProblemDefinition(j);

          // Give it to the current planner
          configTimer.start();
          planner->setProblemDefinition(problemDefinition);
          configTimer.stop();

          // Time is just setup as constructed for previous query.
          querySetupDuration = configTimer.duration();
        }

        // Create the performance log:
        // If it's not the first time we run this query (i.e. not the first run, and not the first planner), 
        // tell the log to expect to append to the existing file.
        pdt::loggers::ResultLog<pdt::loggers::TimeCostLogger> results(resultPaths[j], i!=0u || plannerName != plannerNames.front());

        // Compute the duration we have left for solving.
        const auto maxSolveDuration =
            pdt::time::seconds(context->getMaxSolveDuration() - querySetupDuration);
        const std::chrono::microseconds idle(1000000u /
                                             config->get<std::size_t>("experiment/logFrequency"));

        // Solve the problem on a separate thread.
        pdt::time::Clock::time_point addMeasurementStart;
        const auto solveStartTime = pdt::time::Clock::now();
        std::future<void> future = std::async(std::launch::async, [&planner, &maxSolveDuration]() {
          planner->solve(maxSolveDuration);
        });

        // Log the intermediate best costs.
        do {
          addMeasurementStart = pdt::time::Clock::now();
          logger.addMeasurement(querySetupDuration + (addMeasurementStart - solveStartTime),
                                pdt::utilities::getBestCost(planner, plannerType));

          // Stop logging intermediate best costs if the planner overshoots.
          if (pdt::time::seconds(addMeasurementStart - solveStartTime) >
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
            querySetupDuration + (pdt::time::Clock::now() - solveStartTime);

        // Store the final cost.
        const auto problem = planner->getProblemDefinition();
        if (problem->hasExactSolution()) {
          logger.addMeasurement(totalDuration,
                                problem->getSolutionPath()->cost(context->getObjective()));
        } else {
          logger.addMeasurement(totalDuration,
                              ompl::base::Cost(context->getObjective()->infiniteCost()));
      }

      // Anytime planners can stop early, e.g. if they know that they found the optimal solution.
      // Thus, we need to add an additional final measurement point at the maximum runtime.
      const auto maxRunDuration = context->getMaxSolveDuration();
      if (totalDuration < maxRunDuration) {
        if (problem->hasExactSolution()) {
          logger.addMeasurement(maxRunDuration,
                                problem->getSolutionPath()->cost(context->getObjective()));
        } else {
          logger.addMeasurement(maxRunDuration,
                                ompl::base::Cost(context->getObjective()->infiniteCost()));
        }
        }

        // Add this run to the log and report it to the console.
        results.addResult(planner->getName(), logger);

        // Compute the progress.
        ++currentRun;
        const auto progress = static_cast<float>(currentRun) / static_cast<float>(totalNumberOfRuns);

        // estimate how much time is left by extrapolating from the time spent so far.
        // This is more accurate than subtracting the time used so far from the worst case total time
        // since it can take planners that terminate early into account.
        const auto timeSoFar = std::chrono::system_clock::now() - experimentStartTime;
        const auto extrapolatedRuntime = timeSoFar / progress;
        const auto estimatedTimeString =
            " (est. time left: "s +
            pdt::time::toDurationString(
                std::chrono::ceil<std::chrono::seconds>(extrapolatedRuntime - timeSoFar)) +
            ")"s;

        std::cout << '\r' << std::setw(2) << std::setfill(' ') << std::right << ' ' << "Progress"
                  << (std::ceil(progress * barWidth) != barWidth
                          ? std::setw(static_cast<int>(std::ceil(progress * barWidth)))
                          : std::setw(static_cast<int>(std::ceil(progress * barWidth) - 1u)))
                  << std::setfill('.') << (currentRun != totalNumberOfRuns ? '|' : '.') << std::right
                  << std::setw(barWidth - static_cast<int>(std::ceil(progress * barWidth)))
                  << std::setfill('.') << '.' << std::right << std::fixed << std::setw(6)
                  << std::setfill(' ') << std::setprecision(2) << progress * 100.0f << " %";
        if (currentRun != totalNumberOfRuns) {
          std::cout << estimatedTimeString;
        } else {
          std::cout << std::setfill(' ') << std::setw(static_cast<int>(estimatedTimeString.length()))
                    << " ";
        }
        std::cout << std::flush;
      }
    }
  }

  // dump the complete config to make sure that we can produce the report once we ran the experiment
  auto configPath = experimentDirectory / "config.json"s;
  config->dumpAll(configPath.string());

  // Register the end time of the experiment.
  auto experimentEndTime = std::chrono::system_clock::now();
  auto experimentEndTimeString = pdt::time::toDateString(experimentEndTime);
  pdt::time::Duration experimentDuration = experimentEndTime - experimentStartTime;

  // Report the elapsed time and some statistics.
  std::cout << '\n'
            << std::setw(2u) << std::setfill(' ') << ' ' << std::setw(30) << std::setfill('.')
            << std::left << "Elapsed time" << std::setw(20) << std::right
            << pdt::time::toDurationString(experimentEndTime - experimentStartTime)
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
  std::vector<pdt::statistics::PlanningStatistics> stats;

  for (const auto &path: resultPaths){
    stats.push_back(pdt::statistics::PlanningStatistics(config, path, false));
  }

  // Generate the report.
  fs::path reportPath;
  if (stats.size() == 0u){
    throw std::runtime_error(
        "No statistics were generated, thus no report can be compiled.");
  }
  else if(stats.size() == 1u){ // Single query report
    pdt::reports::SingleQueryReport report(config, stats[0u]);
    report.generateReport();
    reportPath = report.compileReport();
  }
  else{ // Multiquery report
    pdt::statistics::MultiqueryStatistics mqstats(config, stats, false);

    pdt::reports::MultiqueryReport report(config, mqstats);
    report.generateReport();
    reportPath = report.compileReport();
  }

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
              pdt::time::toDateString(experimentStartTime).c_str());
  OMPL_INFORM("End of experiment: '%s'",
              pdt::time::toDateString(experimentEndTime).c_str());
  OMPL_INFORM("Duration of experiment: '%s'",
              pdt::time::toDurationString(experimentDuration).c_str());
  OMPL_INFORM("Wrote results to '%s'", experimentDirectory.string());

  // Inform where we wrote the report to.
  std::cout << std::setw(2u) << std::setfill(' ') << ' ' << "Location " << reportPath << "\n\n";

  // Dump the accessed parameters next to the results file.
  // This overwrites the previously dumped config with one that only consists of the
  // accessed parameters.
  config->dumpAccessed(configPath.string());

  return 0;
}
