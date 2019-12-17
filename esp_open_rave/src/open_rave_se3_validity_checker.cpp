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

#include "esp_open_rave/open_rave_se3_validity_checker.h"

#include <ompl/base/spaces/SE3StateSpace.h>

using namespace std::string_literals;

namespace esp {

namespace ompltools {

OpenRaveSE3ValidityChecker::OpenRaveSE3ValidityChecker(
    const ompl::base::SpaceInformationPtr& spaceInfo,
    const OpenRAVE::EnvironmentBasePtr& environment, const OpenRAVE::RobotBasePtr& robot,
    const std::shared_ptr<const Configuration>& config) :
    OpenRaveBaseValidityChecker(spaceInfo, environment, robot, config),
    raveState_() {
  raveState_.identity();
  const auto contextName = config_->get<std::string>("Experiment/context");
  raveLowerBounds_ =
      config_->get<std::vector<double>>("Contexts/"s + contextName + "/lowerBounds"s);
  raveUpperBounds_ =
      config_->get<std::vector<double>>("Contexts/"s + contextName + "/upperBounds"s);
  assert(raveLowerBounds_.size() == raveUpperBounds_.size());
  raveStateScales_.reserve(raveLowerBounds_.size());
  for (std::size_t i = 0u; i < raveLowerBounds_.size(); ++i) {
    assert(raveUpperBounds_[i] > raveLowerBounds_[i]);
    raveStateScales_.emplace_back(raveUpperBounds_[i] - raveLowerBounds_[i]);
  }
}

bool OpenRaveSE3ValidityChecker::isValid(const ompl::base::State* state) const {
  // Check the states is within the bounds of the state space.
  if (!stateSpace_->satisfiesBounds(state)) {
    return false;
  }

  // Fill the rave state with the ompl state values.
  auto se3State = state->as<ompl::base::SE3StateSpace::StateType>();
  raveState_.trans.Set3(raveLowerBounds_[0] + (raveStateScales_[0] * se3State->getX()),
                        raveLowerBounds_[1] + (raveStateScales_[1] * se3State->getY()),
                        raveLowerBounds_[2] + (raveStateScales_[2] * se3State->getZ()));
  raveState_.rot.x = se3State->rotation().x;
  raveState_.rot.y = se3State->rotation().y;
  raveState_.rot.z = se3State->rotation().z;
  raveState_.rot.w = se3State->rotation().w;

  // Lock the environment mutex.
  OpenRAVE::EnvironmentMutex::scoped_lock lock(environment_->GetMutex());

  // Set the robot to the requested state.
  robot_->SetTransform(raveState_);

  // Set the option to measure distance.
  environment_->GetCollisionChecker()->SetCollisionOptions(OpenRAVE::CO_Contacts);

  // Check for collisions.
  if (environment_->CheckCollision(robot_)) {
    return false;
  } else {
    return true;
  }
}

double OpenRaveSE3ValidityChecker::clearance(const ompl::base::State* state) const {
  // Fill the rave state with the ompl state values.
  auto se3State = state->as<ompl::base::SE3StateSpace::StateType>();
  raveState_.trans.Set3(raveLowerBounds_[0] + (raveStateScales_[0] * se3State->getX()),
                        raveLowerBounds_[1] + (raveStateScales_[1] * se3State->getY()),
                        raveLowerBounds_[2] + (raveStateScales_[2] * se3State->getZ()));
  raveState_.rot.x = se3State->rotation().x;
  raveState_.rot.y = se3State->rotation().y;
  raveState_.rot.z = se3State->rotation().z;
  raveState_.rot.w = se3State->rotation().w;

  // Lock the environment mutex.
  OpenRAVE::EnvironmentMutex::scoped_lock lock(environment_->GetMutex());

  // Set the robot to the requested state.
  robot_->SetTransform(raveState_);

  // Set the option to measure distance.
  environment_->GetCollisionChecker()->SetCollisionOptions(OpenRAVE::CO_Distance);

  // Compute the distance.
  environment_->CheckCollision(robot_, collisionReport_);

  // Report the distance.
  return collisionReport_->minDistance;
}

}  // namespace ompltools

}  // namespace esp
