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

#include "esp_open_rave/open_rave_context.h"

#include <algorithm>

#include <boost/smart_ptr.hpp>

#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <openrave/environment.h>

#include "esp_open_rave/open_rave_validity_checker.h"

using namespace std::string_literals;

namespace esp {

namespace ompltools {

OpenRave::OpenRave(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                   const std::shared_ptr<const Configuration>& config, const std::string& name) :
    BaseContext(spaceInfo, config, name),
    startState_(spaceInfo),
    goalState_(spaceInfo) {
  // Get the start and goal positions.
  auto startPosition = config->get<std::vector<double>>("Contexts/" + name + "/start");
  auto goalPosition = config->get<std::vector<double>>("Contexts/" + name + "/goal");

  // Initialize rave.
  OpenRAVE::RaveInitialize(true /* TODO: this loads all plugins, is this necessary? */,
                           OpenRAVE::Level_Warn);

  // Create a rave environment.
  auto environment = OpenRAVE::RaveCreateEnvironment();

  // Load the specified environment.
  environment->Load(config_->get<std::string>("Contexts/" + name + "/environment"));

  // Load the robot.
  auto robot = environment->GetRobot(config_->get<std::string>("Contexts/" + name + "/robot"));

  if (static_cast<std::size_t>(robot->GetDOF()) != spaceInfo->getStateDimension()) {
    throw std::runtime_error(
        "The degrees of freedom of the robot and the dimensionality of the state space do not "
        "match.");
  }

  // Get the upper and lower bounds for each dimension.
  std::vector<double> lowerBounds, upperBounds;
  robot->GetDOFLimits(lowerBounds, upperBounds);

  // Set the bounds of the state space.
  ompl::base::RealVectorBounds bounds(robot->GetDOF());
  bounds.low = lowerBounds;
  bounds.high = upperBounds;
  spaceInfo_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

  // Set all dimensions as active.
  std::vector<int> activeDimensions(robot->GetDOF());
  std::iota(activeDimensions.begin(), activeDimensions.end(), 0);
  robot->SetActiveDOFs(activeDimensions);

  OMPL_WARN("Dimensions: %zu", robot->GetDOF());

  // Create the validity checker.
  auto validityChecker = std::make_shared<OpenRaveValidityChecker>(spaceInfo_, environment, robot);

  // Set the validity checker and check resolution.
  spaceInfo_->setStateValidityChecker(validityChecker);
  spaceInfo_->setStateValidityCheckingResolution(
      config_->get<double>("Contexts/" + name + "/collisionCheckResolution"));

  // Setup the space info.
  spaceInfo_->setup();

  // Fill the start and goal states' coordinates.
  for (std::size_t i = 0u; i < spaceInfo_->getStateDimension(); ++i) {
    startState_[i] = startPosition.at(i);
    goalState_[i] = goalPosition.at(i);
  }
}

OpenRave::~OpenRave() {
  OpenRAVE::RaveDestroy();
}

ompl::base::ProblemDefinitionPtr OpenRave::instantiateNewProblemDefinition() const {
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

ompl::base::ScopedState<ompl::base::RealVectorStateSpace> OpenRave::getStartState() const {
  return startState_;
}

ompl::base::ScopedState<ompl::base::RealVectorStateSpace> OpenRave::getGoalState() const {
  return goalState_;
}

void OpenRave::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

}  // namespace ompltools

}  // namespace esp
