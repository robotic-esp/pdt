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

#include "esp_planning_contexts/narrow_passage.h"

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>

namespace esp {

namespace ompltools {

NarrowPassage::NarrowPassage(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                             const std::shared_ptr<const Configuration>& config,
                             const std::string& name) :
    RealVectorGeometricContext(spaceInfo, config, name),
    dimensionality_(spaceInfo->getStateDimension()),
    wallThickness_(config->get<double>("context/" + name + "/wallThickness")),
    passageWidth_(config->get<double>("context/" + name + "/passageWidth")),
    passageOffset_(config->get<double>("context/" + name + "/passageOffset")),
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
  if (wallThickness_ < 0.0) {
    OMPL_ERROR("%s: Wall thickness is negative.", name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (passageWidth_ < 0.0) {
    OMPL_ERROR("%s: Gap width is negative.", name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (passageOffset_ == 0.0) {
    OMPL_WARN("%s: Passage offset is 0, straight-line connection is valid.");
  }

  // Fill the start and goal states' coordinates.
  for (std::size_t i = 0u; i < spaceInfo_->getStateDimension(); ++i) {
    startState_[i] = startPosition.at(i);
    goalState_[i] = goalPosition.at(i);
  }

  // Create the validity checker.
  auto validityChecker = std::make_shared<ContextValidityChecker>(spaceInfo_);

  // Create the obstacles and add them to the validity checker.
  createObstacles();
  validityChecker->addObstacles(obstacles_);

  // Set the validity checker and the check resolution.
  spaceInfo_->setStateValidityChecker(validityChecker);
  spaceInfo_->setStateValidityCheckingResolution(
      config->get<double>("context/" + name + "/collisionCheckResolution"));

  // Set up the space info.
  spaceInfo_->setup();
}

ompl::base::ProblemDefinitionPtr NarrowPassage::instantiateNewProblemDefinition() const {
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

ompl::base::ScopedState<ompl::base::RealVectorStateSpace> NarrowPassage::getStartState() const {
  return startState_;
}

ompl::base::ScopedState<ompl::base::RealVectorStateSpace> NarrowPassage::getGoalState() const {
  return goalState_;
}

void NarrowPassage::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

void NarrowPassage::createObstacles() {
  // Get the state space bounds.
  auto bounds = spaceInfo_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getBounds();

  // Get the midpoint between the start and goal positions.
  ompl::base::ScopedState<> midpoint(spaceInfo_);
  for (std::size_t j = 0u; j < dimensionality_; ++j) {
    midpoint[j] = (startState_[j] + goalState_[j]) / 2.0;
  }

  // Create an anchor point for the lower obstacle.
  ompl::base::ScopedState<> lowerAnchor(midpoint);
  lowerAnchor[1] = (midpoint[1] + passageOffset_ - (passageWidth_ / 2.0) + bounds.low[1]) / 2.0;

  // Create the widths of the lower wall.
  std::vector<double> lowerWidths(dimensionality_, 0.0);

  // Set the thickness of the lower wall.
  lowerWidths.at(0) = wallThickness_;

  // Set the height of the lower wall.
  lowerWidths.at(1) = midpoint[1] + passageOffset_ - (passageWidth_ / 2.0) - bounds.low[1];

  // The wall extends to the boundaries in all other dimensions.
  for (std::size_t j = 2u; j < dimensionality_; ++j) {
    lowerWidths.at(j) = bounds.high.at(j) - bounds.low.at(j);
  }

  // Create an anchor point for the upper obstacle.
  ompl::base::ScopedState<> upperAnchor(midpoint);
  upperAnchor[1] =
      midpoint[1] + passageOffset_ + (bounds.high[1] - bounds.low[1] - lowerWidths.at(1)) / 2.0;

  // Create the widths of the upper wall.
  std::vector<double> upperWidths(dimensionality_, 0.0);

  // Set the thickness of the upper wall.
  upperWidths.at(0) = wallThickness_;

  // Set the height of the upper wall.
  upperWidths.at(1) = bounds.high.at(1) - bounds.low.at(1) - lowerWidths.at(1) - passageWidth_;

  // The wall extends to the boundaries in all other dimensions.
  for (std::size_t j = 2u; j < dimensionality_; ++j) {
    upperWidths.at(j) = bounds.high.at(j) - bounds.low.at(j);
  }

  // Add both obstacles.
  obstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseObstacle>>(spaceInfo_, lowerAnchor, lowerWidths));
  obstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseObstacle>>(spaceInfo_, upperAnchor, upperWidths));
}

}  // namespace ompltools

}  // namespace esp