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

#include "esp_open_rave/open_rave_r3.h"

#include <algorithm>

#include <boost/smart_ptr.hpp>

#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <openrave/environment.h>

#include "esp_configuration/directory.h"
#include "esp_open_rave/open_rave_r3_validity_checker.h"

using namespace std::string_literals;

namespace esp {

namespace ompltools {

OpenRaveR3::OpenRaveR3(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                       const std::shared_ptr<const Configuration>& config,
                       const std::string& name) :
    OpenRaveBaseContext(spaceInfo, config, name),
    startState_(spaceInfo),
    goalState_(spaceInfo) {
  // Get the start and goal positions.
  auto startPosition = config->get<std::vector<double>>("context/" + name + "/start");
  auto goalPosition = config->get<std::vector<double>>("context/" + name + "/goal");

  // Set the start position.
  (*startState_)[0] = startPosition.at(0u);
  (*startState_)[1] = startPosition.at(1u);
  (*startState_)[2] = startPosition.at(2u);

  // Set the goal position.
  (*goalState_)[0] = goalPosition.at(0u);
  (*goalState_)[1] = goalPosition.at(1u);
  (*goalState_)[2] = goalPosition.at(2u);

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
  environment->Load(std::string(Directory::SOURCE) + "/"s +
                    config_->get<std::string>("context/" + name + "/environment"));

  // Load the robot.
  auto robot = environment->GetRobot(config_->get<std::string>("context/" + name + "/robot"));

  // In this context, there are no active dimensions.
  robot->SetActiveDOFs({});

  // Set the R3 bounds.
  auto r3space = spaceInfo_->getStateSpace()->as<ompl::base::RealVectorStateSpace>();
  ompl::base::RealVectorBounds bounds(3u);
  bounds.high = config_->get<std::vector<double>>("context/"s + name + "/upperBounds"s);  // x y z
  bounds.low = config_->get<std::vector<double>>("context/"s + name + "/lowerBounds"s);   // x y z
  r3space->setBounds(bounds);

  // Create the validity checker.
  auto validityChecker =
      std::make_shared<OpenRaveR3ValidityChecker>(spaceInfo_, environment, robot, config_);

  OpenRAVE::Transform raveState;
  raveState.identity();

  // Set the validity checker and check resolution.
  spaceInfo_->setStateValidityChecker(validityChecker);
  spaceInfo_->setStateValidityCheckingResolution(
      config_->get<double>("context/" + name + "/collisionCheckResolution"));

  // Setup the space info.
  spaceInfo_->setup();
}

OpenRaveR3::~OpenRaveR3() {
  OpenRAVE::RaveDestroy();
}

ompl::base::ProblemDefinitionPtr OpenRaveR3::instantiateNewProblemDefinition() const {
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

ompl::base::ScopedState<ompl::base::RealVectorStateSpace> OpenRaveR3::getStartState() const {
  return startState_;
}

ompl::base::ScopedState<ompl::base::RealVectorStateSpace> OpenRaveR3::getGoalState() const {
  return goalState_;
}

void OpenRaveR3::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

}  // namespace ompltools

}  // namespace esp