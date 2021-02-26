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

#include "esp_planning_contexts/start_enclosure.h"

#include <vector>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>

#include "esp_obstacles/base_obstacle.h"
#include "esp_obstacles/hyperrectangle.h"
#include "esp_planning_contexts/context_validity_checker.h"

namespace esp {

namespace ompltools {

StartEnclosure::StartEnclosure(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                               const std::shared_ptr<const Configuration>& config,
                               const std::string& name) :
    RealVectorGeometricContext(spaceInfo, config, name),
    dimensionality_(spaceInfo_->getStateDimension()),
    startOutsideWidth_(config->get<double>("context/" + name + "/startOutsideWidth")),
    startInsideWidth_(config->get<double>("context/" + name + "/startInsideWidth")),
    startGapWidth_(config->get<double>("context/" + name + "/startGapWidth")),
    startState_(spaceInfo),
    goalState_(spaceInfo) {
  // Get the start and goal positions.
  auto startPosition = config_->get<std::vector<double>>("context/" + name + "/start");
  auto goalPosition = config_->get<std::vector<double>>("context/" + name + "/goal");

  // Assert configuration sanity.
  if (config->get<std::vector<double>>("context/" + name + "/start").size() != dimensionality_) {
    OMPL_ERROR("%s: Dimensionality of problem and of start specification does not match.",
               name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (config->get<std::vector<double>>("context/" + name + "/goal").size() != dimensionality_) {
    OMPL_ERROR("%s: Dimensionality of problem and of goal specification does not match.",
               name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (startInsideWidth_ > startOutsideWidth_) {
    OMPL_ERROR("%s: Start inside width is greater than start outside width.", name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (startGapWidth_ > startOutsideWidth_) {
    OMPL_ERROR("%s: Start gap width is greater than start outside width.", name.c_str());
    throw std::runtime_error("Context error.");
  }

  // Create the validity checker.
  auto validityChecker = std::make_shared<ContextValidityChecker>(spaceInfo_);

  // Create the obstacles and add them to the validity checker.
  createObstacles();
  validityChecker->addObstacles(obstacles_);

  // Create the anti obstacles and add them to the validity checker.
  createAntiObstacles();
  validityChecker->addAntiObstacles(antiObstacles_);

  // Set the validity checker and the check resolution.
  spaceInfo_->setStateValidityChecker(validityChecker);
  spaceInfo_->setStateValidityCheckingResolution(
      config->get<double>("context/" + name + "/collisionCheckResolution"));

  // Set up the space info.
  spaceInfo_->setup();

  // Fill the start and goal states' coordinates.
  for (auto i = 0u; i < spaceInfo_->getStateDimension(); ++i) {
    startState_[i] = startPosition.at(i);
    goalState_[i] = goalPosition.at(i);
  }
}

ompl::base::ProblemDefinitionPtr StartEnclosure::instantiateNewProblemDefinition() const {
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

ompl::base::ScopedState<ompl::base::RealVectorStateSpace> StartEnclosure::getStartState() const {
  return startState_;
}

ompl::base::ScopedState<ompl::base::RealVectorStateSpace> StartEnclosure::getGoalState() const {
  return goalState_;
}

void StartEnclosure::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

void StartEnclosure::createObstacles() {
  ompl::base::ScopedState<> startOutsideAnchor(spaceInfo_);
  for (auto i = 0u; i < dimensionality_; ++i) {
    startOutsideAnchor[i] = config_->get<std::vector<double>>("context/" + name_ + "/start").at(i);
  }
  std::vector<double> startWidths(dimensionality_, startOutsideWidth_);
  obstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseObstacle>>(spaceInfo_, startOutsideAnchor, startWidths));
}

void StartEnclosure::createAntiObstacles() {
  // Create the inside.
  ompl::base::ScopedState<> startInsideAnchor(spaceInfo_);
  for (auto i = 0u; i < dimensionality_; ++i) {
    startInsideAnchor[i] = config_->get<std::vector<double>>("context/" + name_ + "/start").at(i);
  }
  std::vector<double> startWidths(dimensionality_, startInsideWidth_);
  antiObstacles_.emplace_back(std::make_shared<Hyperrectangle<BaseAntiObstacle>>(
      spaceInfo_, startInsideAnchor, startWidths));

  // Create the gap.
  ompl::base::ScopedState<> startGapAnchor(spaceInfo_);
  startGapAnchor[0u] = config_->get<std::vector<double>>("context/" + name_ + "/start").at(0u) -
                       startInsideWidth_ / 2.0 - (startOutsideWidth_ - startInsideWidth_) / 4.0;
  for (auto i = 1u; i < dimensionality_; ++i) {
    startGapAnchor[i] = 0.0;
  }
  std::vector<double> startGapWidths(dimensionality_, startGapWidth_);
  startGapWidths.at(0u) =
      (startOutsideWidth_ - startInsideWidth_) / 2.0 + std::numeric_limits<double>::epsilon();
  antiObstacles_.emplace_back(std::make_shared<Hyperrectangle<BaseAntiObstacle>>(
      spaceInfo_, startGapAnchor, startGapWidths));
}

}  // namespace ompltools

}  // namespace esp
