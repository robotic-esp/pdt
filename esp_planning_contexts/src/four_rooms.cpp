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

#include "esp_planning_contexts/four_rooms.h"

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>

namespace esp {

namespace ompltools {

FourRooms::FourRooms(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                     const std::shared_ptr<const Configuration>& config, const std::string& name) :
    RealVectorGeometricContext(spaceInfo, config, name),
    dimensionality_(spaceInfo_->getStateDimension()),
    wallThickness_(config->get<double>("Contexts/" + name + "/wallThickness")),
    gapWidth_(config->get<double>("Contexts/" + name + "/gapWidth")),
    startState_(spaceInfo),
    goalState_(spaceInfo) {
  // Get the start and goal positions.
  auto startPosition = config_->get<std::vector<double>>("Contexts/" + name + "/start");
  auto goalPosition = config_->get<std::vector<double>>("Contexts/" + name + "/goal");

  // Assert configuration sanity.
  if (dimensionality_ != 2u) {
    OMPL_ERROR("%s: Currently only implemented for two dimensional state spaces.", name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (startPosition.size() != dimensionality_) {
    OMPL_ERROR("%s: Dimensionality of problem and of start specification does not match.",
               name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (goalPosition.size() != dimensionality_) {
    OMPL_ERROR("%s: Dimensionality of problem and of goal specification does not match.",
               name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (wallThickness_ < 0.0) {
    OMPL_ERROR("%s: Wall thickness is negative.", name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (gapWidth_ < 0.0) {
    OMPL_ERROR("%s: Gap width is negative.", name.c_str());
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
      config->get<double>("Contexts/" + name + "/collisionCheckResolution"));

  // Set up the space info.
  spaceInfo_->setup();

  // Fill the start and goal states' coordinates.
  for (std::size_t i = 0u; i < spaceInfo_->getStateDimension(); ++i) {
    startState_[i] = startPosition.at(i);
    goalState_[i] = goalPosition.at(i);
  }
}

ompl::base::ProblemDefinitionPtr FourRooms::instantiateNewProblemDefinition() const {
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

ompl::base::ScopedState<ompl::base::RealVectorStateSpace> FourRooms::getStartState() const {
  return startState_;
}

ompl::base::ScopedState<ompl::base::RealVectorStateSpace> FourRooms::getGoalState() const {
  return goalState_;
}

void FourRooms::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

void FourRooms::createObstacles() {
  // Get the state space bounds.
  auto bounds = spaceInfo_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getBounds();

  // Create the obstacle that divides the space parallel to the x axis.
  ompl::base::ScopedState<> midpointX(spaceInfo_);
  for (std::size_t i = 0u; i < dimensionality_; ++i) {
    midpointX[i] = (bounds.low.at(i) + bounds.high.at(i)) / 2.0;
  }

  // Create the widths.
  std::vector<double> widthsX(dimensionality_, 0.0);
  widthsX.at(0u) = bounds.high.at(0u) - bounds.low.at(0u);
  widthsX.at(1u) = wallThickness_;

  // Emplace this obstacle.
  obstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseObstacle>>(spaceInfo_, midpointX, widthsX));

  // Create the obstacle that divides the space parallel to the x axis.
  ompl::base::ScopedState<> midpointY(spaceInfo_);
  for (std::size_t i = 0u; i < dimensionality_; ++i) {
    midpointY[i] = (bounds.low.at(i) + bounds.high.at(i)) / 2.0;
  }

  // Create the widths.
  std::vector<double> widthsY(dimensionality_, 0.0);
  widthsY.at(0u) = wallThickness_;
  widthsY.at(1u) = bounds.high.at(1u) - bounds.low.at(1u);

  // Emplace this obstacle.
  obstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseObstacle>>(spaceInfo_, midpointY, widthsY));
}

void FourRooms::createAntiObstacles() {
  // Get the state space bounds.
  auto bounds = spaceInfo_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getBounds();

  // Create the gap in the south.
  ompl::base::ScopedState<> midpointSouth(spaceInfo_);
  for (std::size_t i = 0u; i < dimensionality_; ++i) {
    midpointSouth[i] = (bounds.low.at(i) + bounds.high.at(i)) / 2.0;
  }
  midpointSouth[1u] = bounds.low.at(1u) + (gapWidth_ / 2.0);

  std::vector<double> widthsSouth(dimensionality_, 0.0);
  widthsSouth.at(0u) = wallThickness_ + std::numeric_limits<double>::epsilon();
  widthsSouth.at(1u) = gapWidth_;

  antiObstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseAntiObstacle>>(spaceInfo_, midpointSouth, widthsSouth));

  // Create the gap in the north.
  ompl::base::ScopedState<> midpointNorth(spaceInfo_);
  for (std::size_t i = 0u; i < dimensionality_; ++i) {
    midpointNorth[i] = (bounds.low.at(i) + bounds.high.at(i)) / 2.0;
  }
  midpointNorth[1u] = bounds.high.at(1u) - (gapWidth_ / 2.0);

  std::vector<double> widthsNorth(dimensionality_, 0.0);
  widthsNorth.at(0u) = wallThickness_ + std::numeric_limits<double>::epsilon();
  widthsNorth.at(1u) = gapWidth_;

  antiObstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseAntiObstacle>>(spaceInfo_, midpointNorth, widthsNorth));

  // Create the gap in the west.
  ompl::base::ScopedState<> midpointWest(spaceInfo_);
  for (std::size_t i = 0u; i < dimensionality_; ++i) {
    midpointWest[i] = (bounds.low.at(i) + bounds.high.at(i)) / 2.0;
  }
  midpointWest[0u] = bounds.low.at(0u) + (gapWidth_ / 2.0);

  std::vector<double> widthsWest(dimensionality_, 0.0);
  widthsWest.at(0u) = gapWidth_;
  widthsWest.at(1u) = wallThickness_ + std::numeric_limits<double>::epsilon();

  antiObstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseAntiObstacle>>(spaceInfo_, midpointWest, widthsWest));
}

}  // namespace ompltools

}  // namespace esp
