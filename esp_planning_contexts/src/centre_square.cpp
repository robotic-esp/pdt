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

#include "esp_planning_contexts/centre_square.h"

#include <cmath>
#include <functional>
#include <memory>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include "esp_obstacles/hyperrectangle.h"
#include "esp_planning_contexts/context_validity_checker.h"

namespace esp {

namespace ompltools {

CentreSquare::CentreSquare(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                           const std::shared_ptr<const Configuration>& config,
                           const std::string& name) :
    RealVectorGeometricContext(spaceInfo, config, name),
    startState_(spaceInfo),
    goalState_(spaceInfo) {
  // Get the start and goal positions.
  auto startPosition = config_->get<std::vector<double>>("context/" + name + "/start");
  auto goalPosition = config_->get<std::vector<double>>("context/" + name + "/goal");

  // Get the dimensionality of the problem.
  auto dimensionality = spaceInfo_->getStateDimension();

  // Get the obstacle widths.
  std::vector<double> widths(dimensionality,
                             config_->get<double>("context/" + name + "/obstacleWidth"));

  // Assert configuration sanity.
  if (startPosition.size() != dimensionality) {
    OMPL_ERROR("%s: Dimensionality of problem and of start specification does not match.",
               name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (goalPosition.size() != dimensionality) {
    OMPL_ERROR("%s: Dimensionality of problem and of goal specification does not match.",
               name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (widths.at(0u) < 0.0) {
    OMPL_ERROR("%s: Obstacle width must be positive.", name.c_str());
    throw std::runtime_error("Context error.");
  }

  // Fill the start and goal states' coordinates.
  for (std::size_t i = 0u; i < spaceInfo_->getStateDimension(); ++i) {
    startState_[i] = startPosition.at(i);
    goalState_[i] = goalPosition.at(i);
  }

  // Compute the midpoint of the obstacle.
  auto midpoint = std::make_unique<ompl::base::ScopedState<>>(spaceInfo_);
  for (std::size_t i = 0u; i < dimensionality; ++i) {
    (*midpoint)[i] = (startPosition.at(i) + goalPosition.at(i)) / 2.0;
  }

  // Create the obstacle.
  obstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseObstacle>>(spaceInfo_, *midpoint, widths));

  // Create the validity checker and add the obstacle.
  auto validityChecker = std::make_shared<ContextValidityChecker>(spaceInfo_);
  validityChecker->addObstacles(obstacles_);

  // Set the validity checker and the check resolution.
  spaceInfo_->setStateValidityChecker(
      static_cast<ompl::base::StateValidityCheckerPtr>(validityChecker));
  spaceInfo_->setStateValidityCheckingResolution(
      config_->get<double>("context/" + name + "/collisionCheckResolution"));

  // Set up the space info.
  spaceInfo_->setup();
}

ompl::base::ProblemDefinitionPtr CentreSquare::instantiateNewProblemDefinition() const {
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

ompl::base::ScopedState<ompl::base::RealVectorStateSpace> CentreSquare::getStartState() const {
  return startState_;
}

ompl::base::ScopedState<ompl::base::RealVectorStateSpace> CentreSquare::getGoalState() const {
  return goalState_;
}

void CentreSquare::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

}  // namespace ompltools

}  // namespace esp
