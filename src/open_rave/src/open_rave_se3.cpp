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

#include "pdt/open_rave/open_rave_se3.h"

#include <algorithm>

#include <boost/smart_ptr.hpp>

#include <ompl/base/goals/GoalSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <openrave/environment.h>

#include "pdt/config/directory.h"
#include "pdt/open_rave/open_rave_knee_goal.h"
#include "pdt/open_rave/open_rave_se3_validity_checker.h"

using namespace std::string_literals;

namespace pdt {

namespace open_rave {

OpenRaveSE3::OpenRaveSE3(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                         const std::shared_ptr<const config::Configuration>& config,
                         const std::string& name) :
    OpenRaveBaseContext(spaceInfo, config, name) {
  // Initialize rave.
  OpenRAVE::RaveInitialize(true, OpenRAVE::Level_Warn);

  // Create a rave environment.
  auto environment = OpenRAVE::RaveCreateEnvironment();

  // Create a collision checker.
  OpenRAVE::CollisionCheckerBasePtr collisionChecker = OpenRAVE::RaveCreateCollisionChecker(
      environment, config->get<std::string>("context/" + name + "/collisionChecker"));
  const auto boundingVolumeHierarchyRepresentation =
      config->get<std::string>("context/" + name + "/boundingVolumeHierarchyRepresentation");
  std::string input = "SetBVHRepresentation " + boundingVolumeHierarchyRepresentation;
  std::string output = "";
  collisionChecker->SendCommand(output, input);
  environment->SetCollisionChecker(collisionChecker);

  // Load the specified environment.
  environment->Load(std::string(config::Directory::SOURCE) + "/"s +
                    config_->get<std::string>("context/" + name + "/environment"));

  // Load the robot.
  auto robot = environment->GetRobot(config_->get<std::string>("context/" + name + "/robot"));

  // In this context, there are no active dimensions.
  robot->SetActiveDOFs({});

  auto se3space = spaceInfo_->getStateSpace()->as<ompl::base::SE3StateSpace>();
  ompl::base::RealVectorBounds bounds(3u);
  bounds.high = config_->get<std::vector<double>>("context/"s + name + "/upperBounds"s);  // x y z
  bounds.low = config_->get<std::vector<double>>("context/"s + name + "/lowerBounds"s);   // x y z
  se3space->setBounds(bounds);

  // Create the validity checker.
  auto validityChecker =
      std::make_shared<OpenRaveSE3ValidityChecker>(spaceInfo_, environment, robot, config_);

  OpenRAVE::Transform raveState;
  raveState.identity();

  // Set the validity checker and check resolution.
  spaceInfo_->setStateValidityChecker(validityChecker);
  spaceInfo_->setStateValidityCheckingResolution(
      config_->get<double>("context/" + name + "/collisionCheckResolution"));

  // Setup the space info.
  spaceInfo_->setup();

  startGoalPairs_ = makeStartGoalPair();
}

OpenRaveSE3::~OpenRaveSE3() {
  OpenRAVE::RaveDestroy();
}

std::vector<planning_contexts::StartGoalPair> OpenRaveSE3::makeStartGoalPair() const {
  if (config_->contains("context/" + name_ + "/starts")) {
    OMPL_ERROR("OpenRaveSE3 context does not support multiple queries.");
    throw std::runtime_error("Context error.");
  }

  auto startPosition = config_->get<std::vector<double>>("context/" + name_ + "/start");

  ompl::base::ScopedState<ompl::base::SE3StateSpace> startState(spaceInfo_);
  startState.get()->setXYZ(startPosition.at(0u), startPosition.at(1u), startPosition.at(2u));
  startState.get()->rotation().x = startPosition.at(3u);
  startState.get()->rotation().y = startPosition.at(4u);
  startState.get()->rotation().z = startPosition.at(5u);
  startState.get()->rotation().w = startPosition.at(6u);

  planning_contexts::StartGoalPair pair;
  pair.start = {startState};
  pair.goal = createGoal();

  return {pair};
}

std::shared_ptr<ompl::base::Goal> OpenRaveSE3::createGoal() const {
  // Instantiate the goal.
  switch (goalType_) {
    case ompl::base::GoalType::GOAL_STATE: {
      // Get the goal position.
      const auto goalPosition = config_->get<std::vector<double>>("context/" + name_ + "/goal");

      // Check dimensionality of the goal state position.
      if (goalPosition.size() - 1u != dimensionality_) {
        OMPL_ERROR("%s: Dimensionality of problem and of goal specification does not match.",
                   name_.c_str());
        throw std::runtime_error("Context error.");
      }

      // Allocate a goal state and set the position.
      ompl::base::ScopedState<ompl::base::SE3StateSpace> goalState(spaceInfo_);

      // Fill the goal state's coordinates.
      goalState->setXYZ(goalPosition.at(0u), goalPosition.at(1u), goalPosition.at(2u));
      goalState->rotation().x = goalPosition.at(3u);
      goalState->rotation().y = goalPosition.at(4u);
      goalState->rotation().z = goalPosition.at(5u);
      goalState->rotation().w = goalPosition.at(6u);

      // Register the goal state with the goal.
      auto goal = std::make_shared<ompl::base::GoalState>(spaceInfo_);
      goal->as<ompl::base::GoalState>()->setState(goalState);
      return goal;
    }
    case ompl::base::GoalType::GOAL_STATES: {
      const auto numGoals = config_->get<unsigned>("context/" + name_ + "/numGoals");
      ompl::base::ScopedState<ompl::base::SE3StateSpace> goalState(spaceInfo_);
      auto goal = std::make_shared<ompl::base::GoalStates>(spaceInfo_);
      for (auto i = 0u; i < numGoals; ++i) {
        goalState.random();
        goal->addState(goalState);
      }
      return goal;
    }
    case ompl::base::GoalType::GOAL_SPACE: {
      // Get the goal bounds.
      ompl::base::RealVectorBounds goalBounds(static_cast<unsigned>(dimensionality_));
      goalBounds.low = config_->get<std::vector<double>>("context/" + name_ + "/goalLowerBounds");
      goalBounds.high = config_->get<std::vector<double>>("context/" + name_ + "/goalUpperBounds");

      // Generate a goal space.
      auto goalSpace = std::make_shared<ompl::base::SE3StateSpace>();

      // Set the goal bounds.
      goalSpace->setBounds(goalBounds);

      // Let the goal know about the goal space.
      auto goal = std::make_shared<ompl::base::GoalSpace>(spaceInfo_);
      goal->as<ompl::base::GoalSpace>()->setSpace(goalSpace);
      return goal;
      break;
    }
    default: { throw std::runtime_error("Goal type not implemented."); }
  }
}

void OpenRaveSE3::accept(const planning_contexts::ContextVisitor& visitor) const {
  visitor.visit(*this);
}

}  // namespace open_rave

}  // namespace pdt
