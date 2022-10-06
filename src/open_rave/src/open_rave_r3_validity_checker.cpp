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

#include "pdt/open_rave/open_rave_r3_validity_checker.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace std::string_literals;

namespace pdt {

namespace open_rave {

OpenRaveR3ValidityChecker::OpenRaveR3ValidityChecker(
    const ompl::base::SpaceInformationPtr& spaceInfo,
    const OpenRAVE::EnvironmentBasePtr& environment, const OpenRAVE::RobotBasePtr& robot,
    const std::shared_ptr<const config::Configuration>& config) :
    OpenRaveBaseValidityChecker(spaceInfo, environment, robot, config),
    raveState_() {
  raveState_.identity();
}

bool OpenRaveR3ValidityChecker::isValid(const ompl::base::State* state) const {
  // Check the states is within the bounds of the state space.
  if (!stateSpace_->satisfiesBounds(state)) {
    return false;
  }

  // Fill the rave state with the ompl state values.
  auto r3State = state->as<ompl::base::RealVectorStateSpace::StateType>();
  raveState_.trans.Set3((*r3State)[0u], (*r3State)[1u], (*r3State)[2u]);

  // Lock the environment mutex.
  OpenRAVE::EnvironmentMutex::scoped_lock lock(environment_->GetMutex());

  // Set the robot to the requested state.
  robot_->SetTransform(raveState_);

  // Set the option to measure distance.
  environment_->GetCollisionChecker()->SetCollisionOptions(OpenRAVE::CO_Contacts);

  // Check for collisions.
  return !environment_->CheckCollision(robot_);
}

double OpenRaveR3ValidityChecker::clearance(const ompl::base::State* state) const {
  // Fill the rave state with the ompl state values.
  auto r3State = state->as<ompl::base::RealVectorStateSpace::StateType>();
  raveState_.trans.Set3((*r3State)[0u], (*r3State)[1u], (*r3State)[2u]);

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

}  // namespace open_rave

}  // namespace pdt
