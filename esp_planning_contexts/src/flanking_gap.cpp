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

#include "esp_planning_contexts/flanking_gap.h"

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

FlankingGap::FlankingGap(const std::shared_ptr<const Configuration>& config,
                         const std::string& name) :
    BaseObstacleContext(config, name),
    wallWidth_(config->get<double>("Contexts/" + name + "/wallWidth")),
    wallThickness_(config->get<double>("Contexts/" + name + "/wallThickness")),
    gapWidth_(config->get<double>("Contexts/" + name + "/gapWidth")),
    gapOffset_(config->get<double>("Contexts/" + name + "/gapOffset")),
    startPos_(config->get<std::vector<double>>("Contexts/" + name + "/start")),
    goalPos_(config->get<std::vector<double>>("Contexts/" + name + "/goal")) {
  // Assert configuration sanity.
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
  if (wallWidth_ < 0.0) {
    OMPL_ERROR("%s: Wall width is negative.", name.c_str());
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

  // Create the validity checker and add the obstacle.
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

bool FlankingGap::knowsOptimum() const {
  return false;
}

ompl::base::Cost FlankingGap::computeOptimum() const {
  throw ompl::Exception("The global optimum is unknown, though it could be", BaseObstacleContext::name_);
}

void FlankingGap::setTarget(double targetSpecifier) {
  optimizationObjective_->setCostThreshold(ompl::base::Cost(targetSpecifier));
}

std::string FlankingGap::lineInfo() const {
  std::stringstream rval;

  return rval.str();
}

std::string FlankingGap::paraInfo() const {
  std::stringstream rval;
  rval << lineInfo();
  return rval.str();
}

void FlankingGap::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

void FlankingGap::createObstacles() {
  ompl::base::ScopedState<> midpoint(spaceInfo_);
  // Set the obstacle midpoint in the middle of the state space.
  for (std::size_t j = 0u; j < dimensionality_; ++j) {
    midpoint[j] = (bounds_.at(j).first + bounds_.at(j).second) / 2.0;
  }
  // Create the widths of this wall.
  std::vector<double> widths(dimensionality_, 0.0);
  // Set the obstacle width in the first dimension.
  widths.at(0) = wallThickness_;
  // The wall has the specified width in all other dimensions.
  for (std::size_t j = 1u; j < dimensionality_; ++j) {
    widths.at(j) = wallWidth_;
  }
  obstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseObstacle>>(spaceInfo_, midpoint, widths));
}

void FlankingGap::createAntiObstacles() {
  ompl::base::ScopedState<> midpoint(spaceInfo_);
  // Set the obstacle midpoint in the second dimension.
  midpoint[1u] = gapOffset_ + gapWidth_ / 2.0;
  // Set the obstacle midpoint in the remaining dimension.
  for (std::size_t j = 0; j < dimensionality_; ++j) {
    if (j != 1u) {
      midpoint[j] = (bounds_.at(j).first + bounds_.at(j).second) / 2.0;
    }
  }
  // Create the widths of gap.
  std::vector<double> widths(dimensionality_, 0.0);
  // Set the obstacle width in the first dimension.
  widths.at(0) = wallThickness_;
  // The wall spans all other dimensions.
  for (std::size_t j = 1u; j < dimensionality_; ++j) {
    widths.at(j) = gapWidth_;
  }
  antiObstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseAntiObstacle>>(spaceInfo_, midpoint, widths));
}

}  // namespace ompltools

}  // namespace esp
