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

// Authors: Marlin Strub

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <dbg.h>

#include <ompl/geometric/PathGeometric.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#include <openrave-core.h>
#include <openrave/viewer.h>
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop

#include "esp_common/context_type.h"
#include "esp_common/planner_type.h"
#include "esp_configuration/configuration.h"
#include "esp_factories/context_factory.h"
#include "esp_factories/planner_factory.h"
#include "esp_open_rave/open_rave_context.h"
#include "esp_open_rave/open_rave_validity_checker.h"
#include "esp_planning_contexts/all_contexts.h"

using namespace std::string_literals;
using namespace std::chrono_literals;

void plan(std::shared_ptr<esp::ompltools::Configuration> config,
          std::shared_ptr<esp::ompltools::BaseContext> context) {
  esp::ompltools::PlannerFactory plannerFactory(config, context);
  auto [planner, plannerType] = plannerFactory.create("ABITstar");
  (void)plannerType;

  planner->setup();
  planner->solve(20.0);

  // Get the solution of the planner.
  auto solution =
      planner->getProblemDefinition()->getSolutionPath()->as<ompl::geometric::PathGeometric>();

  // Interpolate ("approx to collision checking resolution").
  solution->interpolate();

  // Get the solution states.
  auto solutionStates = solution->getStates();

  // Get the environment.
  auto environment = std::dynamic_pointer_cast<esp::ompltools::OpenRaveValidityChecker>(
                         context->getSpaceInformation()->getStateValidityChecker())
                         ->getOpenRaveEnvironment();

  auto bounds = context->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getBounds();

  dbg(bounds.low);
  dbg(bounds.high);

  // Get the robot.
  auto robot =
      environment->GetRobot(config->get<std::string>("Contexts/" + context->getName() + "/robot"));
  robot->SetActiveDOFValues(
      config->get<std::vector<double>>("Contexts/" + context->getName() + "/start"));

  // Create the vector to hold the current state.
  std::vector<double> openRaveState =
      config->get<std::vector<double>>("Contexts/" + context->getName() + "/start");

  {
    OpenRAVE::EnvironmentMutex::scoped_lock lock(environment->GetMutex());
    robot->SetActiveDOFValues(openRaveState);
  }

  while (true) {
    for (const auto solutionState : solutionStates) {
      for (std::size_t i = 0u; i < openRaveState.size(); ++i) {
        openRaveState[i] =
            solutionState->as<ompl::base::RealVectorStateSpace::StateType>()->operator[](i);
      }
      OpenRAVE::EnvironmentMutex::scoped_lock lock(environment->GetMutex());
      robot->SetActiveDOFValues(openRaveState);
      if (environment->CheckCollision(robot) || robot->CheckSelfCollision()) {
        OMPL_ERROR("Solution path contains collision.");
      }
      std::this_thread::sleep_for(0.01s);
    }
  }
}

int main(int argc, char** argv) {
  // Instantiate the config.
  auto config = std::make_shared<esp::ompltools::Configuration>(argc, argv);
  config->registerAsExperiment();

  // Create the context.
  auto contextFactory = std::make_shared<esp::ompltools::ContextFactory>(config);
  auto context = contextFactory->create(config->get<std::string>("Experiment/context"));

  // Get the environment.
  auto environment = std::dynamic_pointer_cast<esp::ompltools::OpenRaveValidityChecker>(
                         context->getSpaceInformation()->getStateValidityChecker())
                         ->getOpenRaveEnvironment();

  // Set some joint values of the robot.
  auto robot =
      environment->GetRobot(config->get<std::string>("Contexts/" + context->getName() + "/robot"));
  robot->SetActiveDOFValues(
      config->get<std::vector<double>>("Contexts/" + context->getName() + "/start"));

  // Create the viewer.
  auto viewer = OpenRAVE::RaveCreateViewer(environment, "qtcoin");

  auto planThread = std::thread(&plan, config, context);

  viewer->main(true);

  while (!planThread.joinable()) {
    std::this_thread::sleep_for(0.5s);
  }

  planThread.join();

  return 0;
}
