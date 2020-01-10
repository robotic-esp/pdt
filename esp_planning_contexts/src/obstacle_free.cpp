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

// Authors: Jonathan Gammell, Marlin Strub

#include "esp_planning_contexts/obstacle_free.h"

#include <cmath>
#include <functional>
#include <memory>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>

#include "esp_obstacles/base_obstacle.h"
#include "esp_obstacles/hyperrectangle.h"
#include "esp_planning_contexts/context_validity_checker.h"

namespace esp {

namespace ompltools {

ObstacleFree::ObstacleFree(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                           const std::shared_ptr<const Configuration>& config,
                           const std::string& name) :
    RealVectorGeometricContext(spaceInfo, config, name),
    startState_(spaceInfo),
    goalState_(spaceInfo) {
  // Get the start and goal positions.
  auto startPosition = config_->get<std::vector<double>>("context/" + name + "/start");
  auto goalPosition = config_->get<std::vector<double>>("context/" + name + "/goal");

  // Assert configuration sanity.
  if (startPosition.size() != spaceInfo_->getStateDimension()) {
    OMPL_ERROR("%s: Dimensionality of problem and of start specification does not match.",
               name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (goalPosition.size() != spaceInfo_->getStateDimension()) {
    OMPL_ERROR("%s: Dimensionality of problem and of goal specification does not match.",
               name.c_str());
    throw std::runtime_error("Context error.");
  }
  // Create the validity checker without any obstacles and anti obstacles.
  auto validityChecker = std::make_shared<ContextValidityChecker>(spaceInfo_);

  // Set the validity checker and the check resolution.
  spaceInfo_->setStateValidityChecker(validityChecker);
  spaceInfo_->setStateValidityCheckingResolution(
      config->get<double>("context/" + name + "/collisionCheckResolution"));

  // Set up the space info.
  spaceInfo_->setup();

  // Fill the start and goal states' coordinates.
  for (std::size_t i = 0u; i < spaceInfo_->getStateDimension(); ++i) {
    startState_[i] = startPosition.at(i);
    goalState_[i] = goalPosition.at(i);
  }
}

ompl::base::ProblemDefinitionPtr ObstacleFree::instantiateNewProblemDefinition() const {
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

ompl::base::ScopedState<ompl::base::RealVectorStateSpace> ObstacleFree::getStartState() const {
  return startState_;
}

ompl::base::ScopedState<ompl::base::RealVectorStateSpace> ObstacleFree::getGoalState() const {
  return goalState_;
}

void ObstacleFree::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

}  // namespace ompltools

}  // namespace esp
