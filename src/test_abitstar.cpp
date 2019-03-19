/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Toronto
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
 *   * Neither the name of the University of Toronto nor the names of its
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

/* Authors: Jonathan Gammell */

// For SIGINT handling
#include <signal.h>
#include <unistd.h>

// For std::cout
#include <iostream>
// For std::ifstream and std::ofstream
#include <fstream>
// For std::setprecision
#include <iomanip>
// For std::stringstream
#include <sstream>
// For std::make_shared
#include <memory>
// For boost program options
#include <boost/program_options.hpp>
// For boost lexical cast
#include <boost/lexical_cast.hpp>
// For boost time
#include <boost/chrono/chrono.hpp>
// For boost thread:
#include <boost/thread/thread.hpp>

// OMPL:
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
//#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
//#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/tools/config/MagicConstants.h>  //For BETTER_PATH_COST_MARGIN
#include <ompl/util/Console.h>                 //For OMPL_INFORM et al.
#include <ompl/util/Exception.h>

// Our Experiments
#include "ExperimentDefinitions.h"

// The helper functions for general and plotting:
#include "tools/general_tools.h"
#include "tools/planner_tools.h"
#include "tools/plotting_tools.h"

#include "esp_planner_factory/planner_factory.h"

#define ASRL_DBL_INFINITY std::numeric_limits<double>::infinity()

// World:
const double CHECK_RESOLUTION = 0.001;

// Common
const double REWIRE_SCALE = 1.1;  // The factor scaling the RGG term
const bool K_NEAREST = false;
const double PRUNE_FRACTION = 0.1;

// Experiment:
const bool INITIAL_SOLN_ONLY = false;
const bool LOG_PROGRESS = true;
const bool LOG_ITERATIONS_AND_COST = false;
const unsigned int MICROSEC_SLEEP = 100u;  // Period for logging data, 1000us = 1ms

// BIT*
const unsigned int BITSTAR_BATCH_SIZE = 500;
const bool BITSTAR_K_NEAREST = K_NEAREST;
const double BITSTAR_REWIRE_SCALE = REWIRE_SCALE;
const bool BITSTAR_ENABLE_PRUNING = false;
const double BITSTAR_PRUNE_FRACTION = PRUNE_FRACTION;
const bool BITSTAR_STRICT_QUEUE = true;
const bool BITSTAR_DELAY_REWIRE = false;
const bool BITSTAR_JIT = false;
const bool BITSTAR_DROP_BATCH = false;

// ABIT*
const double ABITSTAR_INITIAL_INFLATION_FACTOR = 1000000.0;
const double ABITSTAR_INFLATION_FACTOR_PARAMETER = 50;
const double ABITSTAR_TRUNCATION_FACTOR_PARAMETER = 50;

// RRT
const bool RRT_K_NEAREST = K_NEAREST;
const double RRT_REWIRE_SCALE = REWIRE_SCALE;
const double RRT_PRUNE_FRACTION = PRUNE_FRACTION;
const double RRT_STEER_ETA_2D = 0.3;
const double RRT_STEER_ETA_4D = 0.5;
const double RRT_STEER_ETA_6D = 0.9;
const double RRT_STEER_ETA_8D = 1.25;  // 0.9;
const double RRT_STEER_ETA_16D = 3.0;

// LBTRRT
const double LBTRRT_EPSILON = 0.4;

// RRT SHARP
const bool RRTSHARP_REJECT = false;

// Others
const double GOAL_BIAS = 0.05;

// Plotting
const bool PLOT_VERTICES = true;
const bool PLOT_INDICES = false;
const bool PLOT_WORLD_ELLIPSE = true;
const bool PLOT_BITSTAR_ELLIPSE = false;
const bool PLOT_BITSTAR_EDGE = true;
const bool PLOT_BITSTAR_QUEUE = false;

bool argParse(int argc, char** argv, unsigned int* numObsPtr, double* obsRatioPtr,
              unsigned int* dimensionPtr, unsigned int* numTrialsPtr, double* runTimePtr,
                bool* animatePtr, std::string* plannerConfigFilePtr) {
  // Declare the supported options.
  boost::program_options::options_description desc("Allowed options");
  desc.add_options()("help,h", "Display this help message.")(
      "state,r", boost::program_options::value<unsigned int>(), "The state dimension.")(
      "planner-config,p", boost::program_options::value<std::string>(), "The planner config file.")(
      "experiments,e", boost::program_options::value<unsigned int>(),
      "The number of unique experiments to run on the random world.")(
      "number-obstacles,n", boost::program_options::value<unsigned int>(),
      "The number of obstacles used to calculate the max obstacle radius to give the desired "
      "obstacle coverage,"
      "[1, 2, ...). Note: radius is selected from 0.5x - 1x max radius, so the actual number of "
      "obstacles will vary.")(
      "obstacle-coverage,o", boost::program_options::value<double>(),
      "The mean percentage of volume taken up by obstacles in the random world, [0,1).")(
      "runtime,t", boost::program_options::value<double>(),
      "The CPU time in seconds for which to run the planners, (0,infty)")(
      "animate,a", boost::program_options::value<bool>()->zero_tokens(),
      "Create frame-by-frame animations.")(
      "log-level,l", boost::program_options::value<unsigned int>(),
      "Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG. Defaults to 0 if not set.");
  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return false;
  }

  if (vm.count("number-obstacles")) {
    *numObsPtr = vm["number-obstacles"].as<unsigned int>();
    if ((*numObsPtr < 1)) {
      std::cout << "number-obstacles must be >= 1" << std::endl << std::endl << desc << std::endl;
      return false;
    }
  } else {
    std::cout << "number-obstacles not set" << std::endl << std::endl << desc << std::endl;
    return false;
  }

  if (vm.count("obstacle-coverage")) {
    *obsRatioPtr = vm["obstacle-coverage"].as<double>();
    if ((*obsRatioPtr < 0) || (*obsRatioPtr >= 1)) {
      std::cout << "obstacle-coverage must be [0, 1)" << std::endl
                << std::endl
                << desc << std::endl;
      return false;
    }
  } else {
    std::cout << "obstacle-coverage not set" << std::endl << std::endl << desc << std::endl;
    return false;
  }

  if (vm.count("state")) {
    *dimensionPtr = vm["state"].as<unsigned int>();
    if ((*dimensionPtr != 2) && (*dimensionPtr != 4) && (*dimensionPtr != 6) &&
        (*dimensionPtr != 8) && (*dimensionPtr != 16)) {
      std::cout
          << "Due to hard-coded RRT* paramenters, state dimension must be 2, 4, 8, or 16 for now "
          << std::endl
          << std::endl
          << desc << std::endl;
      return false;
    }
  } else {
    std::cout << "state dimension not set" << std::endl << std::endl << desc << std::endl;
    return false;
  }

  if (vm.count("experiments")) {
    *numTrialsPtr = vm["experiments"].as<unsigned int>();
  } else {
    std::cout << "Number of experiments not set" << std::endl << std::endl << desc << std::endl;
    return false;
  }

  if (vm.count("runtime")) {
    *runTimePtr = vm["runtime"].as<double>();
    if ((*runTimePtr < 0)) {
      std::cout << "runtime must be greater than, or equal to, 0" << std::endl
                << std::endl
                << desc << std::endl;
      return false;
    }

    if (*runTimePtr == 0.0) {
      *runTimePtr = ASRL_DBL_INFINITY;
    }
  } else {
    std::cout << "runtime not set" << std::endl << std::endl << desc << std::endl;
    return false;
  }

  if (vm.count("animate")) {
    *animatePtr = true;
  } else {
    *animatePtr = false;
  }

  if (vm.count("log-level")) {
    unsigned int logLevel;

    logLevel = vm["log-level"].as<unsigned int>();
    if (logLevel == 0u) {
      ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
    } else if (logLevel == 1u) {
      ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
    } else if (logLevel == 2u) {
      ompl::msg::setLogLevel(ompl::msg::LOG_DEBUG);
    } else {
      std::cout << "invalid log-level" << std::endl << std::endl << desc << std::endl;
      return false;
    }
  } else {
    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
  }

  if (vm.count("planner-config")) {
    *plannerConfigFilePtr = vm["planner-config"].as<std::string>();
  } else {
    *plannerConfigFilePtr = "No config file supplied.";
  }

  return true;
}

ompl::base::PlannerPtr allocatePlanner(const PlannerType plnrType, const BaseExperimentPtr& expDefn,
                                       const unsigned int numSamples, double steerEta) {
  // Variables
  // The allocated planner
  ompl::base::PlannerPtr plnr;

  switch (plnrType) {
    case PLANNER_BITSTAR: {
      plnr =
          allocateBitStar(expDefn->getSpaceInformation(), BITSTAR_K_NEAREST, BITSTAR_REWIRE_SCALE,
                          numSamples, BITSTAR_ENABLE_PRUNING, BITSTAR_PRUNE_FRACTION, BITSTAR_JIT,
                          BITSTAR_DROP_BATCH, 1.0, 0.0, 0.0);
      break;
    }
    case PLANNER_SBITSTAR: {
      plnr = allocateBitStar(expDefn->getSpaceInformation(), BITSTAR_K_NEAREST,
                             BITSTAR_REWIRE_SCALE, numSamples, BITSTAR_ENABLE_PRUNING,
                             BITSTAR_PRUNE_FRACTION, BITSTAR_JIT, BITSTAR_DROP_BATCH,
                             ABITSTAR_INITIAL_INFLATION_FACTOR, ABITSTAR_INFLATION_FACTOR_PARAMETER,
                             ABITSTAR_TRUNCATION_FACTOR_PARAMETER);
      break;
    }
    case PLANNER_RRTCONNECT: {
      plnr = allocateRrtConnect(expDefn->getSpaceInformation(), steerEta);
      break;
    }
    case PLANNER_RRTSTAR: {
      plnr = allocateRrtStar(expDefn->getSpaceInformation(), steerEta, GOAL_BIAS, RRT_K_NEAREST,
                             REWIRE_SCALE);
      break;
    }
    case PLANNER_RRTSTAR_INFORMED: {
      plnr = allocateInformedRrtStar(expDefn->getSpaceInformation(), steerEta, GOAL_BIAS,
                                     RRT_K_NEAREST, RRT_REWIRE_SCALE, RRT_PRUNE_FRACTION);
      break;
    }
    case PLANNER_RRTSHARP: {
      // Abuse numSamples as the variant number.
      plnr = allocateRrtSharp(expDefn->getSpaceInformation(), steerEta, GOAL_BIAS, RRT_K_NEAREST,
                              REWIRE_SCALE, RRTSHARP_REJECT, false, numSamples);
      break;
    }
    case PLANNER_LBTRRT: {
      plnr = allocateLbtRrt(expDefn->getSpaceInformation(), steerEta, GOAL_BIAS, LBTRRT_EPSILON);
      break;
    }
#ifdef BITSTAR_REGRESSION
    case PLANNER_BITSTAR_REGRESSION: {
      plnr = allocateBitStarRegression(expDefn->getSpaceInformation(), BITSTAR_K_NEAREST,
                                       BITSTAR_REWIRE_SCALE, numSamples, BITSTAR_PRUNE_FRACTION,
                                       BITSTAR_STRICT_QUEUE, BITSTAR_DELAY_REWIRE, BITSTAR_JIT,
                                       BITSTAR_DROP_BATCH);
      break;
    }
#endif  // BITSTAR_REGRESSION
    default: { throw ompl::Exception("Unrecognized planner type in allocatePlanner()"); }
  }

  // Set the problem definition
  plnr->setProblemDefinition(expDefn->newProblemDefinition());

  // Return
  return plnr;
}

// Defining the experiment counter globally allows to gracefully finish an experiment and close the
// output file on SIGINT.
unsigned int experimentCounter = 0u;

// Handle SIGINT by setting the counter to maximum value, terminating the experiment loop.
void signalHandler(int signum) {
  std::cout
      << "\n=== Caught signal " << signum
      << ". Will clean up as soon as all planners have completed the same number of runs. ===\n"
      << std::endl;
  experimentCounter = std::numeric_limits<unsigned int>::max() - 1u;
}

void callSolve(asrl::time::point* startTime, const ompl::base::PlannerPtr& planner,
               const asrl::time::duration& solveDuration) {
  *startTime = asrl::time::now();
  planner->solve(asrl::time::seconds(solveDuration));
}

int main(int argc, char** argv) {
  signal(SIGINT, signalHandler);

  // Argument Variables
  // The dimension size:
  unsigned int N;
  // The number of experiments
  unsigned int numExperiments;
  // The (maximum) percent of space covered by obstacles
  double obsRatio;
  // The number of obstacles to create
  unsigned int numObs;
  // The time for which to run the planners
  double targetTime;
  // Whether to make frame-by-frame animations
  bool createAnimationFrames;

  std::string plannerConfigFile{};

  // Get the command line arguments
  if (argParse(argc, argv, &numObs, &obsRatio, &N, &numExperiments, &targetTime,
               &createAnimationFrames, &plannerConfigFile) == false) {
    return 1;
  }

  esp_ompl_tools::PlannerFactory plannerFactory(plannerConfigFile);
  plannerFactory.dumpParameters(std::cout);

  // Variables
  // ompl::RNG::setSeed(18439067297677225292u);    std::cout << std::endl << " ---------> Seed set!
  // <---------                   " << std::endl << std::endl;

  // Master seed:
  std::uint_fast32_t masterSeed = ompl::RNG::getSeed();
  // Convenience typedefs:
  typedef std::pair<asrl::time::duration, ompl::base::Cost> planner_result_t;
  //    typedef std::vector<planner_result_t> experiment_result_t;
  // The filename for progress
  std::stringstream fileName;
  // The world name
  std::stringstream worldName;
  // The experiment
  // BaseExperimentPtr expDefn = std::make_shared<CentreSquareExperiment>(N, 0.25, 2.8, targetTime,
  // CHECK_RESOLUTION); BaseExperimentPtr expDefn = std::make_shared<DeadEndExperiment>(0.4,
  // targetTime, CHECK_RESOLUTION); BaseExperimentPtr expDefn =
  // std::make_shared<SpiralExperiment>(0.4, targetTime, CHECK_RESOLUTION); BaseExperimentPtr
  // expDefn = std::make_shared<WallGapExperiment>(N, false, 0.03, 0.3, 0.3, targetTime,
  // CHECK_RESOLUTION);
  BaseExperimentPtr expDefn =
      std::make_shared<FlankingGapExperiment>(false, 0.01, targetTime, CHECK_RESOLUTION);
  // BaseExperimentPtr expDefn = std::make_shared<RandomRectanglesExperiment>(N, numObs, obsRatio,
  // targetTime, CHECK_RESOLUTION); BaseExperimentPtr expDefn =
  // std::make_shared<RegularRectanglesExperiment>(N, 1.0, 3, targetTime, CHECK_RESOLUTION);
  // BaseExperimentPtr expDefn = std::make_shared<DoubleEnclosureExperiment>(N, 1.0, 0.3, 0.05, 0.1,
  // targetTime, CHECK_RESOLUTION); //worldHalfWidth, insideWidth, wallThickness, gapWidth. Symmetry
  // when: worldHalfWidth = (3*insideWidth + 1)/2 BaseExperimentPtr expDefn =
  // std::make_shared<GoalEnclosureExperiment>(N, 1.0, 0.3, 0.1, 0.1, targetTime, CHECK_RESOLUTION);
  // // worldHalfWidth, insideWidth, wallThickness, gapWidth. BaseExperimentPtr expDefn =
  // std::make_shared<StartEnclosureExperiment>(N, 9.0, 3.0, 0.5, 3.0, targetTime,
  // CHECK_RESOLUTION); // worldHalfWidth, insideWidth, wallThickness, gapWidth. BaseExperimentPtr
  // expDefn = std::make_shared<DoubleFloorplanExperiment>(N, 3, 1.5, 0.2, 0.6, targetTime,
  // CHECK_RESOLUTION); BaseExperimentPtr expDefn = std::make_shared<FloorplanExperiment>(N,
  // 10, 3.0, 0.4, 1.5, targetTime, CHECK_RESOLUTION);

  if (INITIAL_SOLN_ONLY == true) {
    expDefn->setTarget(std::numeric_limits<double>::infinity());
  }

  double steerEta{0.0};
  if (N == 2u) {
    steerEta = RRT_STEER_ETA_2D;
  } else if (N == 4) {
    steerEta = RRT_STEER_ETA_4D;
  } else if (N == 6u) {
    steerEta = RRT_STEER_ETA_6D;
  } else if (N == 8u) {
    steerEta = RRT_STEER_ETA_8D;
  } else if (N == 16u) {
    steerEta = RRT_STEER_ETA_16D;
  } else {
    throw ompl::Exception("No recorded steer eta for this dimension.");
  }

  std::cout << "Seed: " << masterSeed << std::endl;
  expDefn->print();

  // The results output file:
  fileName << "R" << N << "S" << masterSeed << expDefn->getName() << ".csv";
  ResultsFile<TimeCostHistory> plannerProgress(fileName.str());
  ResultsFile<TimeIterationCostHistory> plannerProgressIters(fileName.str());

  for (; experimentCounter < numExperiments; ++experimentCounter) {
    // Variables:
    // The vector of planners:
    std::vector<std::pair<PlannerType, unsigned int> > plannersToTest;

    // Add the planners to test. Be careful, too large of FMT batch size (i.e., ~100000u) fucks up
    // wall time of other planners
    plannersToTest.push_back(std::make_pair(PLANNER_BITSTAR, BITSTAR_BATCH_SIZE));
    // plannersToTest.push_back(std::make_pair(PLANNER_BITSTAR_REGRESSION, BITSTAR_BATCH_SIZE));
    // plannersToTest.push_back(std::make_pair(PLANNER_SBITSTAR, BITSTAR_BATCH_SIZE));
    // plannersToTest.push_back(std::make_pair(PLANNER_RRTCONNECT, 0u));
    // plannersToTest.push_back(std::make_pair(PLANNER_LBTRRT, 0u));
    // plannersToTest.push_back(std::make_pair(PLANNER_RRTSTAR, 0u));
    // plannersToTest.push_back(std::make_pair(PLANNER_RRTSHARP, 0u));

    if (experimentCounter == 0u) {
      std::cout << "   ";
      for (unsigned int i = 0u; i < plannersToTest.size(); ++i) {
        std::cout << std::setw(26) << std::setfill(' ')
                  << allocatePlanner(plannersToTest.at(i).first, expDefn,
                                     plannersToTest.at(i).second, steerEta)
                         ->getName();
        if (i != plannersToTest.size() - 1u) {
          std::cout << "    ";
        }
      }
      std::cout << std::endl;
    }
    std::cout << std::setw(2) << experimentCounter << ": " << std::flush;

    // Iterate over the planners, using the seed and solution from the first planner:
    for (unsigned int i = 0u; i < plannersToTest.size(); ++i) {
      // Variables
      // The start time for a call, and the running time
      asrl::time::point startTime;
      asrl::time::duration initTime(0);
      asrl::time::duration runTime(0);
      // The current planner:
      ompl::base::PlannerPtr plnr;
      // The problem defintion used by this planner
      ompl::base::ProblemDefinitionPtr pdef;

      // Allocate a planner
      // plnr = allocatePlanner(plannersToTest.at(i).first, expDefn, plannersToTest.at(i).second,
      //                        steerEta);

      plnr = plannerFactory.create("BITstar", expDefn);

      // Get the problem definition
      pdef = plnr->getProblemDefinition();

      // Set up the planner
      startTime = asrl::time::now();
      plnr->setup();
      initTime = asrl::time::now() - startTime;

      // // Set the seed.
      // if (plannersToTest.at(i).first == PLANNER_BITSTAR || plannersToTest.at(i).first ==
      // PLANNER_SBITSTAR)
      // {
      //     plnr->as<ompl::geometric::BITstar>()->setLocalSeed(masterSeed);
      // }
      // if (plannersToTest.at(i).first == PLANNER_BITSTAR_REGRESSION)
      // {
      //     plnr->as<ompl::geometric::BITstarRegression>()->setLocalSeed(masterSeed);
      // }

      // Run the planner
      if (createAnimationFrames == true) {
        if (plannersToTest.at(i).first == PLANNER_RRT) {
          startTime = asrl::time::now();
          plnr->solve(asrl::time::seconds(expDefn->getTargetTime() - initTime));
          runTime = initTime + (asrl::time::now() - startTime);
          // runTime = initTime + createAnimation(expDefn plannersToTest.at(i), masterSeed,
          // expDefn->getTargetTime());
        } else {
          runTime =
              initTime + createAnimation(expDefn, plannersToTest.at(i).first, plnr, masterSeed,
                                         expDefn->getTargetTime() - initTime, PLOT_VERTICES,
                                         PLOT_INDICES, PLOT_WORLD_ELLIPSE, PLOT_BITSTAR_ELLIPSE,
                                         PLOT_BITSTAR_EDGE, PLOT_BITSTAR_QUEUE);
        }
      } else if (LOG_PROGRESS == true) {
        // Variables
        // A vector of the planner progress
        TimeCostHistory progressPair(expDefn->getTargetTime(), MICROSEC_SLEEP);
        TimeIterationCostHistory progressTuple(expDefn->getTargetTime(), MICROSEC_SLEEP);

        // Start solving in another thread. It will record startTime, but so we notice if it doesn't
        // we will clear startTime first.
        startTime = asrl::time::point();
        boost::thread solveThread(callSolve, &startTime, plnr, expDefn->getTargetTime() - initTime);

        // If the clock is not steady, sleep for 1us so that the upcoming first entry is not
        // backwards in time.
        if (asrl::time::clock::is_steady == false) {
          usleep(1u);
        }

        // Log data
        do {
          if (plannersToTest.at(i).first == PLANNER_RRT ||
              plannersToTest.at(i).first == PLANNER_RRTCONNECT) {
            // Do nothing, these do not have intermediate data
          } else if (plannersToTest.at(i).first == PLANNER_FMTSTAR) {
            throw ompl::Exception(
                "Whoops, FMT* planners arent supported for logging data. Feel free to implement "
                "(haha who am I kidding.)");
            if (LOG_ITERATIONS_AND_COST == true) {
              // If FMT* is modified, 1 of 2:
              // progressTuple.push_back( std::make_tuple(initTime + (asrl::time::now() -
              // startTime), plnr->as<ompl::geometric::FMT>()->iterationProgressProperty(),
              // boost::lexical_cast<std::string>(ASRL_DBL_INFINITY)) );
            } else {
            }
          } else if (isRrtStar(plannersToTest.at(i).first)) {
            // Store the runtime and the current cost
            if (LOG_ITERATIONS_AND_COST == true) {
              progressTuple.push_back(
                  std::make_tuple(initTime + (asrl::time::now() - startTime),
                                  plnr->as<ompl::geometric::RRTstar>()->numIterations(),
                                  plnr->as<ompl::geometric::RRTstar>()->bestCost().value()));
            } else {
              progressPair.push_back(
                  std::make_pair(initTime + (asrl::time::now() - startTime),
                                 plnr->as<ompl::geometric::RRTstar>()->bestCost().value()));
            }
          } else if (isRrtSharp(plannersToTest.at(i).first)) {
            // Store the runtime and the current cost
            if (LOG_ITERATIONS_AND_COST == true) {
              progressTuple.push_back(
                  std::make_tuple(initTime + (asrl::time::now() - startTime),
                                  plnr->as<ompl::geometric::RRTsharp>()->numIterations(),
                                  plnr->as<ompl::geometric::RRTsharp>()->bestCost().value()));
            } else {
              progressPair.push_back(
                  std::make_pair(initTime + (asrl::time::now() - startTime),
                                 plnr->as<ompl::geometric::RRTsharp>()->bestCost().value()));
            }
          } else if (isLbtRrt(plannersToTest.at(i).first)) {
            // Store the runtime and the current cost
            if (LOG_ITERATIONS_AND_COST == true) {
              progressTuple.push_back(std::make_tuple(
                  initTime + (asrl::time::now() - startTime),
                  std::stoi(plnr->as<ompl::geometric::LBTRRT>()->getIterationCount()),
                  std::stod(plnr->as<ompl::geometric::LBTRRT>()->getBestCost())));
            } else {
              progressPair.push_back(
                  std::make_pair(initTime + (asrl::time::now() - startTime),
                                 std::stod(plnr->as<ompl::geometric::LBTRRT>()->getBestCost())));
            }
          } else if (isBitStar(plannersToTest.at(i).first)) {
            if (LOG_ITERATIONS_AND_COST == true) {
              progressTuple.push_back(
                  std::make_tuple(initTime + (asrl::time::now() - startTime),
                                  plnr->as<ompl::geometric::BITstar>()->numIterations(),
                                  plnr->as<ompl::geometric::BITstar>()->bestCost().value()));
            } else {
              progressPair.push_back(
                  std::make_pair(initTime + (asrl::time::now() - startTime),
                                 plnr->as<ompl::geometric::BITstar>()->bestCost().value()));
            }
          }
#ifdef BITSTAR_REGRESSION
          else if (plannersToTest.at(i).first == PLANNER_BITSTAR_REGRESSION) {
            if (LOG_ITERATIONS_AND_COST == true) {
              progressTuple.push_back(std::make_tuple(
                  initTime + (asrl::time::now() - startTime),
                  plnr->as<ompl::geometric::BITstarRegression>()->numIterations(),
                  plnr->as<ompl::geometric::BITstarRegression>()->bestCost().value()));
            } else {
              progressPair.push_back(std::make_pair(
                  initTime + (asrl::time::now() - startTime),
                  plnr->as<ompl::geometric::BITstarRegression>()->bestCost().value()));
            }
          }
#endif  // BITSTAR_REGRESSION
          else {
            throw ompl::Exception("Planner not recognized for progress logging.");
          }
        } while (solveThread.try_join_for(boost::chrono::microseconds(MICROSEC_SLEEP)) == false);

        // Store the final run time
        runTime = initTime + (asrl::time::now() - startTime);

        if (LOG_ITERATIONS_AND_COST == true) {
          double finalCost;

          // Get the final solution
          if (pdef->hasExactSolution() == true) {
            finalCost = pdef->getSolutionPath()->cost(expDefn->getOptimizationObjective()).value();
          } else {
            finalCost = ASRL_DBL_INFINITY;
          }

          if (plannersToTest.at(i).first == PLANNER_RRT) {
            // Do nothing, these do not have intermediate data
          } else if (plannersToTest.at(i).first == PLANNER_FMTSTAR) {
            // If FMT* is modified, 2 of 2:
            // progressTuple.push_back( std::make_tuple(runTime,
            // plnr->as<ompl::geometric::FMT>()->iterationProgressProperty(), finalCost) );
          } else if (isRrtStar(plannersToTest.at(i).first)) {
            progressTuple.push_back(std::make_tuple(
                runTime, plnr->as<ompl::geometric::RRTstar>()->numIterations(), finalCost));
          } else if (isBitStar(plannersToTest.at(i).first)) {
            progressTuple.push_back(std::make_tuple(
                runTime, plnr->as<ompl::geometric::BITstar>()->numIterations(), finalCost));
          } else if (isLbtRrt(plannersToTest.at(i).first)) {
            progressTuple.push_back(std::make_tuple(
                runTime, std::stoi(plnr->as<ompl::geometric::LBTRRT>()->getIterationCount()),
                std::stod(plnr->as<ompl::geometric::LBTRRT>()->getBestCost())));
          } else {
            throw ompl::Exception("Planner not recognized for progress logging.");
          }

          // Save the progress:
          plannerProgressIters.addResult(plnr->getName(), progressTuple);
        } else {
          // Push back the final solution:
          if (pdef->hasExactSolution() == true) {
            progressPair.push_back(std::make_pair(
                runTime,
                pdef->getSolutionPath()->cost(expDefn->getOptimizationObjective()).value()));
          } else {
            progressPair.push_back(std::make_pair(runTime, ASRL_DBL_INFINITY));
          }

          // Save the progress:
          plannerProgress.addResult(plnr->getName(), progressPair);
        }
      } else {
        startTime = asrl::time::now();
        plnr->solve(asrl::time::seconds(expDefn->getTargetTime() - initTime));
        runTime = initTime + (asrl::time::now() - startTime);
      }

      // Store the information:
      planner_result_t myResult;
      if (pdef->hasExactSolution() == true) {
        myResult = std::make_pair(
            runTime, pdef->getSolutionPath()->cost(expDefn->getOptimizationObjective()));
      } else {
        myResult = std::make_pair(runTime, ompl::base::Cost(ASRL_DBL_INFINITY));
      }

      // Save the map?
      writeMatlabMap(expDefn, plannersToTest.at(i).first, plnr, masterSeed,
                     asrl::time::seconds(runTime), PLOT_VERTICES, PLOT_INDICES, PLOT_WORLD_ELLIPSE,
                     PLOT_BITSTAR_ELLIPSE, PLOT_BITSTAR_EDGE, PLOT_BITSTAR_QUEUE);

      std::cout << myResult.first << ", " << std::setw(8) << myResult.second;
      if (i != plannersToTest.size() - 1u) {
        std::cout << ";    " << std::flush;
      }

      // Explicitly destroy the planner.
      plnr.reset();
    }
    std::cout << std::endl;
  }

  return 0;
}
