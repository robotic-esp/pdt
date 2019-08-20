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

#include <cmath>
#include <functional>
#include <memory>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace esp {

namespace ompltools {

FourRooms::FourRooms(const std::shared_ptr<const Configuration>& config, const std::string& name) :
    BaseContext(config, name),
    wallThickness_(config->get<double>("Contexts/" + name + "/wallThickness")),
    gapWidth_(config->get<double>("Contexts/" + name + "/gapWidth")),
    startPos_(config->get<std::vector<double>>("Contexts/" + name + "/start")),
    goalPos_(config->get<std::vector<double>>("Contexts/" + name + "/goal")) {
  // Assert configuration sanity.
  if (dimensionality_ != 2u) {
    OMPL_ERROR("%s: Currently only implemented for two dimensional state spaces.", name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (startPos_.size() != dimensionality_) {
    OMPL_ERROR("%s: Dimensionality of problem and of start specification does not match.",
               name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (goalPos_.size() != dimensionality_) {
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

  // Create a state space and set the bounds.
  auto stateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(dimensionality_);
  stateSpace->setBounds(bounds_.at(0u).first, bounds_.at(0u).second);

  // Create the space information class:
  spaceInfo_ = std::make_shared<ompl::base::SpaceInformation>(stateSpace);

  // Create the validity checker.
  validityChecker_ = std::make_shared<ContextValidityChecker>(spaceInfo_);

  // Create the obstacles and anti obstacles.
  createObstacles();
  createAntiObstacles();

  // Add them to the validity checker.
  validityChecker_->addObstacles(obstacles_);
  validityChecker_->addAntiObstacles(antiObstacles_);

  // Set the validity checker and the check resolution.
  spaceInfo_->setStateValidityChecker(validityChecker_);
  spaceInfo_->setStateValidityCheckingResolution(
      config->get<double>("Contexts/" + name + "/collisionCheckResolution"));

  // Set up the space info.
  spaceInfo_->setup();

  // Allocate the optimization objective
  optimizationObjective_ =
      std::make_shared<ompl::base::PathLengthOptimizationObjective>(spaceInfo_);

  // Set the heuristic to the default:
  optimizationObjective_->setCostToGoHeuristic(
      std::bind(&ompl::base::goalRegionCostToGo, std::placeholders::_1, std::placeholders::_2));

  // Create a start state.
  addStartState(startPos_);

  // Create a goal state.
  addGoalState(goalPos_);
  goalPtr_ = std::make_shared<ompl::base::GoalState>(spaceInfo_);
  goalPtr_->as<ompl::base::GoalState>()->setState(goalStates_.back());

  // Specify the optimization target.
  optimizationObjective_->setCostThreshold(computeMinPossibleCost());
}

bool FourRooms::knowsOptimum() const {
  return false;
}

ompl::base::Cost FourRooms::computeOptimum() const {
  throw ompl::Exception("The global optimum is unknown, though it could be", BaseContext::name_);
}

void FourRooms::setTarget(double targetSpecifier) {
  optimizationObjective_->setCostThreshold(ompl::base::Cost(targetSpecifier));
}

std::string FourRooms::lineInfo() const {
  std::stringstream rval;

  return rval.str();
}

std::string FourRooms::paraInfo() const {
  std::stringstream rval;
  rval << lineInfo();
  return rval.str();
}

void FourRooms::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

void FourRooms::createObstacles() {
  // Create the obstacle that divides the space parallel to the x axis.
  ompl::base::ScopedState<> midpointX(spaceInfo_);
  for (std::size_t i = 0u; i < dimensionality_; ++i) {
    midpointX[i] = (bounds_.at(i).first + bounds_.at(i).second) / 2.0;
  }

  // Create the widths.
  std::vector<double> widthsX(dimensionality_, 0.0);
  widthsX.at(0u) = bounds_.at(0u).second - bounds_.at(0u).first;
  widthsX.at(1u) = wallThickness_;

  // Emplace this obstacle.
  obstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseObstacle>>(spaceInfo_, midpointX, widthsX));

  // Create the obstacle that divides the space parallel to the x axis.
  ompl::base::ScopedState<> midpointY(spaceInfo_);
  for (std::size_t i = 0u; i < dimensionality_; ++i) {
    midpointY[i] = (bounds_.at(i).first + bounds_.at(i).second) / 2.0;
  }

  // Create the widths.
  std::vector<double> widthsY(dimensionality_, 0.0);
  widthsY.at(0u) = wallThickness_;
  widthsY.at(1u) = bounds_.at(1u).second - bounds_.at(1u).first;

  // Emplace this obstacle.
  obstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseObstacle>>(spaceInfo_, midpointY, widthsY));
}

void FourRooms::createAntiObstacles() {
  // Create the gap in the south.
  ompl::base::ScopedState<> midpointSouth(spaceInfo_);
  for (std::size_t i = 0u; i < dimensionality_; ++i) {
    midpointSouth[i] = (bounds_.at(i).first + bounds_.at(i).second) / 2.0;
  }
  midpointSouth[1u] = bounds_.at(1u).first + (gapWidth_ / 2.0);

  std::vector<double> widthsSouth(dimensionality_, 0.0);
  widthsSouth.at(0u) = wallThickness_ + std::numeric_limits<double>::epsilon();
  widthsSouth.at(1u) = gapWidth_;

  antiObstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseAntiObstacle>>(spaceInfo_, midpointSouth, widthsSouth));

  // Create the gap in the north.
  ompl::base::ScopedState<> midpointNorth(spaceInfo_);
  for (std::size_t i = 0u; i < dimensionality_; ++i) {
    midpointNorth[i] = (bounds_.at(i).first + bounds_.at(i).second) / 2.0;
  }
  midpointNorth[1u] = bounds_.at(1u).second - (gapWidth_ / 2.0);

  std::vector<double> widthsNorth(dimensionality_, 0.0);
  widthsNorth.at(0u) = wallThickness_ + std::numeric_limits<double>::epsilon();
  widthsNorth.at(1u) = gapWidth_;

  antiObstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseAntiObstacle>>(spaceInfo_, midpointNorth, widthsNorth));

  // Create the gap in the west.
  ompl::base::ScopedState<> midpointWest(spaceInfo_);
  for (std::size_t i = 0u; i < dimensionality_; ++i) {
    midpointWest[i] = (bounds_.at(i).first + bounds_.at(i).second) / 2.0;
  }
  midpointWest[0u] = bounds_.at(0u).first + (gapWidth_ / 2.0);

  std::vector<double> widthsWest(dimensionality_, 0.0);
  widthsWest.at(0u) = gapWidth_;
  widthsWest.at(1u) = wallThickness_ + std::numeric_limits<double>::epsilon();

  antiObstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseAntiObstacle>>(spaceInfo_, midpointWest, widthsWest));
}

}  // namespace ompltools

}  // namespace esp
