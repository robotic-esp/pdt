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

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <ompl/geometric/PathGeometric.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Woverflow"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <openrave-core.h>
#include <openrave/viewer.h>
#pragma GCC diagnostic pop

#include "pdt/common/context_type.h"
#include "pdt/common/planner_type.h"
#include "pdt/config/configuration.h"
#include "pdt/factories/context_factory.h"
#include "pdt/factories/planner_factory.h"
#include "pdt/open_rave/open_rave_base_context.h"
#include "pdt/open_rave/open_rave_manipulator.h"
#include "pdt/open_rave/open_rave_manipulator_validity_checker.h"
#include "pdt/open_rave/open_rave_r3.h"
#include "pdt/open_rave/open_rave_r3_validity_checker.h"
#include "pdt/open_rave/open_rave_r3xso2.h"
#include "pdt/open_rave/open_rave_r3xso2_validity_checker.h"
#include "pdt/open_rave/open_rave_se3.h"
#include "pdt/open_rave/open_rave_se3_validity_checker.h"
#include "pdt/planning_contexts/all_contexts.h"
#include "pdt/planning_contexts/context_visitor.h"

using namespace std::string_literals;
using namespace std::chrono_literals;

void planManipulator(std::shared_ptr<pdt::config::Configuration> config,
                     std::shared_ptr<pdt::open_rave::OpenRaveManipulator> context) {
  pdt::factories::PlannerFactory plannerFactory(config, context);
  std::shared_ptr<ompl::base::Planner> planner;
  std::tie(planner, std::ignore, std::ignore) =
      plannerFactory.create(config->get<std::string>("experiment/planner"));

  // Setup the planner.
  planner->setup();

  // Get the environment.
  auto environment = std::dynamic_pointer_cast<pdt::open_rave::OpenRaveManipulatorValidityChecker>(
                         context->getSpaceInformation()->getStateValidityChecker())
                         ->getOpenRaveEnvironment();

  // Get the robot.
  auto robot =
      environment->GetRobot(config->get<std::string>("context/" + context->getName() + "/robot"));

  // Create the vector to hold the current state.
  std::vector<double> openRaveState =
      config->get<std::vector<double>>("context/" + context->getName() + "/start");

  double timePerQuery = 0.0;
  if (config->contains("experiment/time")) {
    timePerQuery = config->get<double>("experiment/time");
  }

  double totalSolveDuration = 0.0;
  std::size_t queryNumber = 0u;
  double currentIterationStartTime = 0.0;

  // Work it.
  while (true) {
    if (queryNumber + 1 < context->getNumQueries() &&
        ((planner->getProblemDefinition()->hasExactSolution() && timePerQuery <= 0.0) ||
         totalSolveDuration - currentIterationStartTime > timePerQuery)) {
      planner->clearQuery();
      ++queryNumber;

      const auto problemDefinition = context->instantiateNthProblemDefinition(queryNumber);
      planner->setProblemDefinition(problemDefinition);
      currentIterationStartTime = totalSolveDuration;

      std::cout << "Query: " << queryNumber << std::endl;
    }

    // Work on the problem.
    planner->solve(config->get<double>("experiment/visualizationInterval"));

    // Update the total solve duration.
    totalSolveDuration += config->get<double>("experiment/visualizationInterval");

    // Check if the planner found a solution yet.
    if (planner->getProblemDefinition()->hasExactSolution()) {
      // Get the solution of the planner.
      auto solution =
          planner->getProblemDefinition()->getSolutionPath()->as<ompl::geometric::PathGeometric>();

      // Report the cost of the solution.
      std::cout << "[ " << totalSolveDuration << "s ] "
                << config->get<std::string>("experiment/planner") << " found a solution of cost "
                << solution->cost(planner->getProblemDefinition()->getOptimizationObjective())
                << '\n';

      // Interpolate ("approx to collision checking resolution").
      solution->interpolate();

      // Get the solution states.
      auto solutionStates = solution->getStates();

      // Visualize the solution.
      for (const auto solutionState : solutionStates) {
        for (auto i = 0u; i < openRaveState.size(); ++i) {
          openRaveState[i] =
              solutionState->as<ompl::base::RealVectorStateSpace::StateType>()->operator[](i);
        }
        OpenRAVE::EnvironmentMutex::scoped_lock lock(environment->GetMutex());
        robot->SetActiveDOFValues(openRaveState);
        std::this_thread::sleep_for(0.1s);
      }
    } else {
      std::cout << "[ " << totalSolveDuration << "s ] "
                << config->get<std::string>("experiment/planner")
                << " did not find a solution yet.\n";
    }
  }
}

void planMover(std::shared_ptr<pdt::config::Configuration> config,
               std::shared_ptr<pdt::open_rave::OpenRaveSE3> context) {
  pdt::factories::PlannerFactory plannerFactory(config, context);
  std::shared_ptr<ompl::base::Planner> planner;
  std::tie(planner, std::ignore, std::ignore) =
      plannerFactory.create(config->get<std::string>("experiment/planner"));

  // Setup the planner.
  planner->setup();

  // Get the environment.
  auto environment = std::dynamic_pointer_cast<pdt::open_rave::OpenRaveSE3ValidityChecker>(
                         context->getSpaceInformation()->getStateValidityChecker())
                         ->getOpenRaveEnvironment();

  // Get the robot.
  auto robot =
      environment->GetRobot(config->get<std::string>("context/" + context->getName() + "/robot"));

  // Create the vector to hold the current state.
  OpenRAVE::Transform raveState;

  double timePerQuery = 0.0;
  if (config->contains("experiment/time")) {
    timePerQuery = config->get<double>("experiment/time");
  }

  double totalSolveDuration = 0.0;
  std::size_t queryNumber = 0u;
  double currentIterationStartTime = 0.0;

  // Work it.
  while (true) {
    if (queryNumber + 1 < context->getNumQueries() &&
        ((planner->getProblemDefinition()->hasExactSolution() && timePerQuery <= 0.0) ||
         totalSolveDuration - currentIterationStartTime > timePerQuery)) {
      planner->clearQuery();
      ++queryNumber;

      const auto problemDefinition = context->instantiateNthProblemDefinition(queryNumber);
      planner->setProblemDefinition(problemDefinition);
      currentIterationStartTime = totalSolveDuration;

      std::cout << "Query: " << queryNumber << std::endl;
    }

    // Work on the problem.
    planner->solve(config->get<double>("experiment/visualizationInterval"));

    // Update the total solve duration.
    totalSolveDuration += config->get<double>("experiment/visualizationInterval");

    // Check if the planner found a solution yet.
    if (planner->getProblemDefinition()->hasExactSolution()) {
      // Get the solution of the planner.
      auto solution =
          planner->getProblemDefinition()->getSolutionPath()->as<ompl::geometric::PathGeometric>();

      // Report the cost of the solution.
      std::cout << "[ " << totalSolveDuration << "s ] "
                << config->get<std::string>("experiment/planner") << " found a solution of cost "
                << solution->cost(planner->getProblemDefinition()->getOptimizationObjective())
                << '\n';

      // Interpolate ("approx to collision checking resolution").
      solution->interpolate();

      // Get the solution states.
      auto solutionStates = solution->getStates();

      // Visualize the solution.
      for (const auto solutionState : solutionStates) {
        auto se3State = solutionState->as<ompl::base::SE3StateSpace::StateType>();
        raveState.trans.Set3(se3State->getX(), se3State->getY(), se3State->getZ());
        raveState.rot.x = se3State->rotation().x;
        raveState.rot.y = se3State->rotation().y;
        raveState.rot.z = se3State->rotation().z;
        raveState.rot.w = se3State->rotation().w;

        OpenRAVE::EnvironmentMutex::scoped_lock lock(environment->GetMutex());
        robot->SetTransform(raveState);
        std::this_thread::sleep_for(0.01s);
      }
    } else {
      std::cout << "[ " << totalSolveDuration << "s ] "
                << config->get<std::string>("experiment/planner")
                << " did not find a solution yet.\n";
    }
  }
}

void planR3(std::shared_ptr<pdt::config::Configuration> config,
            std::shared_ptr<pdt::open_rave::OpenRaveR3> context) {
  pdt::factories::PlannerFactory plannerFactory(config, context);
  std::shared_ptr<ompl::base::Planner> planner;
  std::tie(planner, std::ignore, std::ignore) =
      plannerFactory.create(config->get<std::string>("experiment/planner"));

  // Setup the planner.
  planner->setup();

  // Get the environment.
  auto environment = std::dynamic_pointer_cast<pdt::open_rave::OpenRaveR3ValidityChecker>(
                         context->getSpaceInformation()->getStateValidityChecker())
                         ->getOpenRaveEnvironment();

  // Get the robot.
  auto robot =
      environment->GetRobot(config->get<std::string>("context/" + context->getName() + "/robot"));

  // Create the vector to hold the current state.
  OpenRAVE::Transform raveState;
  raveState.identity();

  double timePerQuery = 0.0;
  if (config->contains("experiment/time")) {
    timePerQuery = config->get<double>("experiment/time");
  }

  double totalSolveDuration = 0.0;
  std::size_t queryNumber = 0u;
  double currentIterationStartTime = 0.0;

  // Work it.
  while (true) {
    if (queryNumber + 1 < context->getNumQueries() &&
        ((planner->getProblemDefinition()->hasExactSolution() && timePerQuery <= 0.0) ||
         totalSolveDuration - currentIterationStartTime > timePerQuery)) {
      planner->clearQuery();
      ++queryNumber;

      const auto problemDefinition = context->instantiateNthProblemDefinition(queryNumber);
      planner->setProblemDefinition(problemDefinition);
      currentIterationStartTime = totalSolveDuration;

      std::cout << "Query: " << queryNumber << std::endl;
    }

    // Work on the problem.
    planner->solve(config->get<double>("experiment/visualizationInterval"));

    // Update the total solve duration.
    totalSolveDuration += config->get<double>("experiment/visualizationInterval");

    // Check if the planner found a solution yet.
    if (planner->getProblemDefinition()->hasExactSolution()) {
      // Get the solution of the planner.
      auto solution =
          planner->getProblemDefinition()->getSolutionPath()->as<ompl::geometric::PathGeometric>();

      // Report the cost of the solution.
      std::cout << "[ " << totalSolveDuration << "s ] "
                << config->get<std::string>("experiment/planner") << " found a solution of cost "
                << solution->cost(planner->getProblemDefinition()->getOptimizationObjective())
                << '\n';

      // Interpolate ("approx to collision checking resolution").
      solution->interpolate();

      // Get the solution states.
      auto solutionStates = solution->getStates();

      // Visualize the solution.
      for (const auto solutionState : solutionStates) {
        auto r3State = solutionState->as<ompl::base::RealVectorStateSpace::StateType>();
        raveState.trans.Set3((*r3State)[0u], (*r3State)[1u], (*r3State)[2u]);

        OpenRAVE::EnvironmentMutex::scoped_lock lock(environment->GetMutex());
        robot->SetTransform(raveState);
        std::this_thread::sleep_for(0.01s);
      }
    } else {
      std::cout << "[ " << totalSolveDuration << "s ] "
                << config->get<std::string>("experiment/planner")
                << " did not find a solution yet.\n";
    }
  }
}

void planR3xSO2(std::shared_ptr<pdt::config::Configuration> config,
                std::shared_ptr<pdt::open_rave::OpenRaveR3xSO2> context) {
  pdt::factories::PlannerFactory plannerFactory(config, context);
  std::shared_ptr<ompl::base::Planner> planner;
  std::tie(planner, std::ignore, std::ignore) =
      plannerFactory.create(config->get<std::string>("experiment/planner"));

  // Setup the planner.
  planner->setup();

  // Get the environment.
  auto environment = std::dynamic_pointer_cast<pdt::open_rave::OpenRaveR3xSO2ValidityChecker>(
                         context->getSpaceInformation()->getStateValidityChecker())
                         ->getOpenRaveEnvironment();

  // Get the robot.
  auto robot =
      environment->GetRobot(config->get<std::string>("context/" + context->getName() + "/robot"));

  // Create the vector to hold the current state.
  OpenRAVE::Transform raveState;
  raveState.identity();

  double timePerQuery = 0.0;
  if (config->contains("experiment/time")) {
    timePerQuery = config->get<double>("experiment/time");
  }

  double totalSolveDuration = 0.0;
  std::size_t queryNumber = 0u;
  double currentIterationStartTime = 0.0;

  // Work it.
  while (true) {
    if (queryNumber + 1 < context->getNumQueries() &&
        ((planner->getProblemDefinition()->hasExactSolution() && timePerQuery <= 0.0) ||
         totalSolveDuration - currentIterationStartTime > timePerQuery)) {
      planner->clearQuery();
      ++queryNumber;

      const auto problemDefinition = context->instantiateNthProblemDefinition(queryNumber);
      planner->setProblemDefinition(problemDefinition);
      currentIterationStartTime = totalSolveDuration;

      std::cout << "Query: " << queryNumber << std::endl;
    }

    // Work on the problem.
    planner->solve(config->get<double>("experiment/visualizationInterval"));

    // Update the total solve duration.
    totalSolveDuration += config->get<double>("experiment/visualizationInterval");

    // Check if the planner found a solution yet.
    if (planner->getProblemDefinition()->hasExactSolution()) {
      // Get the solution of the planner.
      auto solution =
          planner->getProblemDefinition()->getSolutionPath()->as<ompl::geometric::PathGeometric>();

      // Report the cost of the solution.
      std::cout << "[ " << totalSolveDuration << "s ] "
                << config->get<std::string>("experiment/planner") << " found a solution of cost "
                << solution->cost(planner->getProblemDefinition()->getOptimizationObjective())
                << '\n';

      // Interpolate ("approx to collision checking resolution").
      solution->interpolate();

      // Get the solution states.
      auto solutionStates = solution->getStates();

      // Visualize the solution.
      for (const auto solutionState : solutionStates) {
        // Fill the R3 part of the state.
        auto r3State = solutionState->as<ompl::base::CompoundStateSpace::StateType>()
                           ->as<ompl::base::RealVectorStateSpace::StateType>(0u);
        raveState.trans.Set3((*r3State)[0u], (*r3State)[1u], (*r3State)[2u]);

        // Fill the SO2 part of the state
        auto so2State = solutionState->as<ompl::base::CompoundStateSpace::StateType>()
                            ->as<ompl::base::SO2StateSpace::StateType>(1u);
        raveState.rot.Set4(std::sin(so2State->value / 2.0), 0.0, 0.0,
                           std::cos(so2State->value / 2.0));

        OpenRAVE::EnvironmentMutex::scoped_lock lock(environment->GetMutex());
        robot->SetTransform(raveState);
        std::this_thread::sleep_for(0.01s);
      }
    } else {
      std::cout << "[ " << totalSolveDuration << "s ] "
                << config->get<std::string>("experiment/planner")
                << " did not find a solution yet.\n";
    }
  }
}

int main(const int argc, const char** argv) {
  // Instantiate the config.
  auto config = std::make_shared<pdt::config::Configuration>(argc, argv);
  config->registerAsExperiment();

  // Create the context.
  auto contextFactory = std::make_shared<pdt::factories::ContextFactory>(config);
  auto context = contextFactory->create(config->get<std::string>("experiment/context"));

  if (std::dynamic_pointer_cast<pdt::open_rave::OpenRaveManipulator>(context)) {
    // Get the environment.
    auto environment =
        std::dynamic_pointer_cast<pdt::open_rave::OpenRaveManipulatorValidityChecker>(
            context->getSpaceInformation()->getStateValidityChecker())
            ->getOpenRaveEnvironment();

    // Create the viewer.
    auto viewer =
        OpenRAVE::RaveCreateViewer(environment, config->get<std::string>("experiment/viewer"));

    auto planThread =
        std::thread(&planManipulator, config,
                    std::dynamic_pointer_cast<pdt::open_rave::OpenRaveManipulator>(context));

    viewer->main(true);

    while (!planThread.joinable()) {
      std::this_thread::sleep_for(0.5s);
    }

    planThread.join();
  } else if (std::dynamic_pointer_cast<pdt::open_rave::OpenRaveSE3>(context)) {
    // Get the environment.
    auto environment = std::dynamic_pointer_cast<pdt::open_rave::OpenRaveSE3ValidityChecker>(
                           context->getSpaceInformation()->getStateValidityChecker())
                           ->getOpenRaveEnvironment();

    // Create the viewer.
    auto viewer =
        OpenRAVE::RaveCreateViewer(environment, config->get<std::string>("experiment/viewer"));

    auto planThread = std::thread(&planMover, config,
                                  std::dynamic_pointer_cast<pdt::open_rave::OpenRaveSE3>(context));

    viewer->main(true);

    while (!planThread.joinable()) {
      std::this_thread::sleep_for(0.5s);
    }

    planThread.join();
  } else if (std::dynamic_pointer_cast<pdt::open_rave::OpenRaveR3>(context)) {
    // Get the environment.
    auto environment = std::dynamic_pointer_cast<pdt::open_rave::OpenRaveR3ValidityChecker>(
                           context->getSpaceInformation()->getStateValidityChecker())
                           ->getOpenRaveEnvironment();

    // Create the viewer.
    auto viewer =
        OpenRAVE::RaveCreateViewer(environment, config->get<std::string>("experiment/viewer"));

    auto planThread = std::thread(&planR3, config,
                                  std::dynamic_pointer_cast<pdt::open_rave::OpenRaveR3>(context));

    viewer->main(true);

    while (!planThread.joinable()) {
      std::this_thread::sleep_for(0.5s);
    }

    planThread.join();
  } else if (std::dynamic_pointer_cast<pdt::open_rave::OpenRaveR3xSO2>(context)) {
    // Get the environment.
    auto environment = std::dynamic_pointer_cast<pdt::open_rave::OpenRaveR3xSO2ValidityChecker>(
                           context->getSpaceInformation()->getStateValidityChecker())
                           ->getOpenRaveEnvironment();

    // Create the viewer.
    auto viewer =
        OpenRAVE::RaveCreateViewer(environment, config->get<std::string>("experiment/viewer"));

    auto planThread = std::thread(
        &planR3xSO2, config, std::dynamic_pointer_cast<pdt::open_rave::OpenRaveR3xSO2>(context));

    viewer->main(true);

    while (!planThread.joinable()) {
      std::this_thread::sleep_for(0.5s);
    }

    planThread.join();
  } else {
    throw std::runtime_error("Cannot process non-openrave context.");
  }

  return 0;
}
