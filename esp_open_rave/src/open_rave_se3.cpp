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

#include "esp_open_rave/open_rave_se3.h"

#include <algorithm>

#include <boost/smart_ptr.hpp>

#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <openrave/environment.h>

#include "esp_open_rave/open_rave_se3_validity_checker.h"

using namespace std::string_literals;

namespace esp {

namespace ompltools {

OpenRaveSE3::OpenRaveSE3(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                         const std::shared_ptr<const Configuration>& config,
                         const std::string& name) :
    OpenRaveBaseContext(spaceInfo, config, name),
    startState_(spaceInfo),
    goalState_(spaceInfo) {
  // Get the start and goal positions.
  auto startPosition = config->get<std::vector<double>>("Contexts/" + name + "/start");
  auto goalPosition = config->get<std::vector<double>>("Contexts/" + name + "/goal");

  // Get the upper and lower bounds and the state scales.
  auto raveLowerBounds = config_->get<std::vector<double>>("Contexts/"s + name + "/lowerBounds"s);
  auto raveUpperBounds = config_->get<std::vector<double>>("Contexts/"s + name + "/upperBounds"s);
  assert(raveLowerBounds.size() == raveUpperBounds.size());
  std::vector<double> raveStateScales;
  raveStateScales.reserve(raveLowerBounds.size());
  for (std::size_t i = 0u; i < raveLowerBounds.size(); ++i) {
    assert(raveUpperBounds[i] > raveLowerBounds[i]);
    raveStateScales.emplace_back(raveUpperBounds[i] - raveLowerBounds[i]);
  }

  startState_.get()->setXYZ(
      (startPosition.at(0u) - raveLowerBounds.at(0u)) / raveStateScales.at(0u),
      (startPosition.at(1u) - raveLowerBounds.at(1u)) / raveStateScales.at(1u),
      (startPosition.at(2u) - raveLowerBounds.at(2u)) / raveStateScales.at(2u));
  startState_.get()->rotation().x = startPosition.at(3u);
  startState_.get()->rotation().y = startPosition.at(4u);
  startState_.get()->rotation().z = startPosition.at(5u);
  startState_.get()->rotation().w = startPosition.at(6u);

  goalState_.get()->setXYZ((goalPosition.at(0u) - raveLowerBounds.at(0u)) / raveStateScales.at(0u),
                           (goalPosition.at(1u) - raveLowerBounds.at(1u)) / raveStateScales.at(1u),
                           (goalPosition.at(2u) - raveLowerBounds.at(2u)) / raveStateScales.at(2u));
  goalState_.get()->rotation().x = goalPosition.at(3u);
  goalState_.get()->rotation().y = goalPosition.at(4u);
  goalState_.get()->rotation().z = goalPosition.at(5u);
  goalState_.get()->rotation().w = goalPosition.at(6u);

  // Initialize rave.
  OpenRAVE::RaveInitialize(true, OpenRAVE::Level_Warn);

  // Create a rave environment.
  auto environment = OpenRAVE::RaveCreateEnvironment();

  // Create a collision checker.
  OpenRAVE::CollisionCheckerBasePtr collisionChecker = OpenRAVE::RaveCreateCollisionChecker(
      environment, config->get<std::string>("Contexts/" + name + "/collisionChecker"));
  environment->SetCollisionChecker(collisionChecker);

  // Load the specified environment.
  environment->Load(config_->get<std::string>("Contexts/" + name + "/environment"));

  // Load the robot.
  auto robot = environment->GetRobot(config_->get<std::string>("Contexts/" + name + "/robot"));

  // In this context, there are no active dimensions.
  robot->SetActiveDOFs({});

  auto se3space = spaceInfo_->getStateSpace()->as<ompl::base::SE3StateSpace>();
  ompl::base::RealVectorBounds bounds(3u);
  bounds.setLow(0.0);
  bounds.setHigh(1.0);
  se3space->setBounds(bounds);

  // Create the validity checker.
  auto validityChecker =
      std::make_shared<OpenRaveSE3ValidityChecker>(spaceInfo_, environment, robot, config_);

  OpenRAVE::Transform raveState;
  raveState.identity();

  // Set the validity checker and check resolution.
  spaceInfo_->setStateValidityChecker(validityChecker);
  spaceInfo_->setStateValidityCheckingResolution(
      config_->get<double>("Contexts/" + name + "/collisionCheckResolution"));

  // Setup the space info.
  spaceInfo_->setup();
}

OpenRaveSE3::~OpenRaveSE3() {
  OpenRAVE::RaveDestroy();
}

ompl::base::ProblemDefinitionPtr OpenRaveSE3::instantiateNewProblemDefinition() const {
  // Instantiate a new problem definition.
  auto problemDefinition = std::make_shared<ompl::base::ProblemDefinition>(spaceInfo_);

  // Set the objective.
  problemDefinition->setOptimizationObjective(objective_);

  // Set the start state in the problem definition.
  problemDefinition->addStartState(startState_);

  // Create a goal for the problem definition.
  auto goal = std::make_shared<ompl::base::GoalState>(spaceInfo_);
  goal->setState(goalState_);
  problemDefinition->setGoal(goal);

  // Return the new definition.
  return problemDefinition;
}

ompl::base::ScopedState<ompl::base::SE3StateSpace> OpenRaveSE3::getStartState() const {
  return startState_;
}

ompl::base::ScopedState<ompl::base::SE3StateSpace> OpenRaveSE3::getGoalState() const {
  return goalState_;
}

void OpenRaveSE3::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

}  // namespace ompltools

}  // namespace esp
