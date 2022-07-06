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

#include "esp_open_rave/open_rave_manipulator.h"

#include <algorithm>

#include <boost/smart_ptr.hpp>

#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <openrave/environment.h>

#include "esp_configuration/directory.h"
#include "esp_open_rave/open_rave_manipulator_validity_checker.h"

using namespace std::string_literals;

namespace esp {

namespace ompltools {

OpenRaveManipulator::OpenRaveManipulator(
    const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
    const std::shared_ptr<const Configuration>& config, const std::string& name) :
    OpenRaveBaseContext(spaceInfo, config, name){
  // Initialize rave.
  OpenRAVE::RaveInitialize(true, OpenRAVE::Level_Warn);

  // Create a rave environment.
  auto environment = OpenRAVE::RaveCreateEnvironment();

  // Create a collision checker.
  OpenRAVE::CollisionCheckerBasePtr collisionChecker = OpenRAVE::RaveCreateCollisionChecker(
      environment, config->get<std::string>("context/" + name + "/collisionChecker"));
  const auto boundingVolumeHierarchyRepresentation =
      config->get<std::string>("context/" + name + "/boundingVolumeHierarchyRepresentation");
  std::string cmd = "SetBVHRepresentation " + boundingVolumeHierarchyRepresentation;
  std::string bhv = "";
  collisionChecker->SendCommand(bhv, cmd);
  environment->SetCollisionChecker(collisionChecker);

  // Load the specified environment.
  environment->Load(std::string(Directory::SOURCE) + "/"s +
                    config_->get<std::string>("context/" + name + "/environment"));

  // Load the robot.
  auto robot = environment->GetRobot(config_->get<std::string>("context/" + name + "/robot"));

  // Set the active dimensions.
  robot->SetActiveDOFs(config_->get<std::vector<int>>("context/" + name + "/activeDofIndices"));

  // Set the bounds of the state space.
  // Get the upper and lower bounds of the state dimensions.
  std::vector<double> raveLowerBounds, raveUpperBounds;
  robot->GetActiveDOFLimits(raveLowerBounds, raveUpperBounds);
  assert(raveLowerBounds.size() == raveUpperBounds.size());
  ompl::base::RealVectorBounds bounds(static_cast<unsigned>(raveLowerBounds.size()));
  bounds.low = raveLowerBounds;
  bounds.high = raveUpperBounds;
  spaceInfo_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

  // Create the validity checker.
  auto validityChecker =
      std::make_shared<OpenRaveManipulatorValidityChecker>(spaceInfo_, environment, robot, config_);

  // Set the validity checker and check resolution.
  spaceInfo_->setStateValidityChecker(validityChecker);
  spaceInfo_->setStateValidityCheckingResolution(
      config_->get<double>("context/" + name + "/collisionCheckResolution"));

  // Setup the space info.
  spaceInfo_->setup();

  startGoalPair_ = makeStartGoalPair();
}

OpenRaveManipulator::~OpenRaveManipulator() {
  OpenRAVE::RaveDestroy();
}

StartGoalPair OpenRaveManipulator::makeStartGoalPair() const{
  // Get the start and goal positions.
  const auto startPosition = config_->get<std::vector<double>>("context/" + name_ + "/start");
  const auto goalPosition = config_->get<std::vector<double>>("context/" + name_ + "/goal");

  ompl::base::ScopedState<ompl::base::CompoundStateSpace> startState(spaceInfo_);
  ompl::base::ScopedState<ompl::base::CompoundStateSpace> goalState(spaceInfo_);

  // Fill the start and goal states' coordinates.
  for (auto i = 0u; i < spaceInfo_->getStateDimension(); ++i) {
    startState[i] = startPosition.at(i);
    goalState[i] = goalPosition.at(i);
  }
  
  StartGoalPair pair;
  pair.start = {startState};

  const auto goal = std::make_shared<ompl::base::GoalState>(spaceInfo_);
  goal->setState(goalState);
  pair.goal = goal;

  return pair;
}

ompl::base::ScopedState<ompl::base::RealVectorStateSpace> OpenRaveManipulator::getStartState()
    const {
  return startGoalPair_.start.at(0);
}


void OpenRaveManipulator::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

}  // namespace ompltools

}  // namespace esp
