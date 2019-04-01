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

#include "esp_planning_contexts/double_enclosure.h"

#include <cmath>
#include <functional>
#include <memory>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "esp_obstacles/hyperrectangle.h"

namespace esp {

namespace ompltools {

DoubleEnclosure::DoubleEnclosure(const std::shared_ptr<const Configuration>& config,
                                 const std::string& name) :
    BaseContext(config, name),
    startOutsideWidth_(config->get<double>("Contexts/" + name + "/startOutsideWidth")),
    startInsideWidth_(config->get<double>("Contexts/" + name + "/startInsideWidth")),
    startGapWidth_(config->get<double>("Contexts/" + name + "/startGapWidth")),
    goalOutsideWidth_(config->get<double>("Contexts/" + name + "/goalOutsideWidth")),
    goalInsideWidth_(config->get<double>("Contexts/" + name + "/goalInsideWidth")),
    goalGapWidth_(config->get<double>("Contexts/" + name + "/goalGapWidth")),
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
  if (startInsideWidth_ > startOutsideWidth_) {
    OMPL_ERROR("%s: Start inside width is greater than start outside width.", name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (startGapWidth_ > startOutsideWidth_) {
    OMPL_ERROR("%s: Start gap width is greater than start outside width.", name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (goalInsideWidth_ > goalOutsideWidth_) {
    OMPL_ERROR("%s: Goal inside width is greater than goal outside width.", name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (goalGapWidth_ > goalOutsideWidth_) {
    OMPL_ERROR("%s: Goal gap width is greater than goal outside width.", name.c_str());
    throw std::runtime_error("Context error.");
  }
  // Create a state space and set the bounds.
  auto stateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(dimensionality_);
  stateSpace->setBounds(bounds_.at(0u).first, bounds_.at(0u).second);

  // Create the space information class:
  spaceInfo_ = std::make_shared<ompl::base::SpaceInformation>(stateSpace);

  // Create the obstacles.
  createObstacles();
  createAntiObstacles();

  // Create the validity checker and add the obstacle.
  validityChecker_ = std::make_shared<ContextValidityChecker>(spaceInfo_);
  validityChecker_->addObstacles(obstacles_);
  validityChecker_->addAntiObstacles(antiObstacles_);

  // Set the validity checker and the check resolution.
  spaceInfo_->setStateValidityChecker(
      static_cast<ompl::base::StateValidityCheckerPtr>(validityChecker_));
  spaceInfo_->setStateValidityCheckingResolution(
      config->get<double>("Contexts/" + name + "/collisionCheckResolution"));

  // Set up the space info.
  spaceInfo_->setup();

  // Allocate the optimization objective.
  optimizationObjective_ =
      std::make_shared<ompl::base::PathLengthOptimizationObjective>(spaceInfo_);

  // Set the heuristic to the default.
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

bool DoubleEnclosure::knowsOptimum() const {
  return false;
}

ompl::base::Cost DoubleEnclosure::computeOptimum() const {
  throw ompl::Exception("The global optimum is unknown, though it could be", BaseContext::name_);
}

void DoubleEnclosure::setTarget(double targetSpecifier) {
  optimizationObjective_->setCostThreshold(
      ompl::base::Cost(targetSpecifier * this->computeOptimum().value()));
}

std::string DoubleEnclosure::lineInfo() const {
  std::stringstream rval;

  return rval.str();
}

std::string DoubleEnclosure::paraInfo() const {
  return std::string();
}

void DoubleEnclosure::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

void DoubleEnclosure::createObstacles() {
  // Create the start obstacle.
  ompl::base::ScopedState<> startState(spaceInfo_);
  for (std::size_t i = 0u; i < dimensionality_; ++i) {
    startState[i] = startPos_[i];
  }
  std::vector<double> startWidths(dimensionality_, startOutsideWidth_);
  obstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseObstacle>>(spaceInfo_, startState, startWidths));
  // Create the goal obstacle.
  ompl::base::ScopedState<> goalState(spaceInfo_);
  for (std::size_t i = 0u; i < dimensionality_; ++i) {
    goalState[i] = goalPos_[i];
  }
  std::vector<double> goalWidths(dimensionality_, goalOutsideWidth_);
  obstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseObstacle>>(spaceInfo_, goalState, goalWidths));
}

void DoubleEnclosure::createAntiObstacles() {
  // Create the start inside.
  ompl::base::ScopedState<> startState(spaceInfo_);
  for (std::size_t i = 0u; i < dimensionality_; ++i) {
    startState[i] = startPos_[i];
  }
  std::vector<double> startWidths(dimensionality_, startInsideWidth_);
  antiObstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseAntiObstacle>>(spaceInfo_, startState, startWidths));

  // Create the goal inside.
  ompl::base::ScopedState<> goalState(spaceInfo_);
  for (std::size_t i = 0u; i < dimensionality_; ++i) {
    goalState[i] = goalPos_[i];
  }
  std::vector<double> goalWidths(dimensionality_, goalInsideWidth_);
  antiObstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseAntiObstacle>>(spaceInfo_, goalState, goalWidths));

  // Create the start gap.
  ompl::base::ScopedState<> startGapMidpoint(spaceInfo_);
  startGapMidpoint[0u] =
      startPos_[0u] - startInsideWidth_ / 2.0 - (startOutsideWidth_ - startInsideWidth_) / 4.0;
  for (std::size_t i = 1u; i < dimensionality_; ++i) {
    startGapMidpoint[i] = 0.0;
  }
  std::vector<double> startGapWidths(dimensionality_, startGapWidth_);
  startGapWidths.at(0u) =
      (startOutsideWidth_ - startInsideWidth_) / 2.0 + std::numeric_limits<double>::epsilon();
  antiObstacles_.emplace_back(std::make_shared<Hyperrectangle<BaseAntiObstacle>>(
      spaceInfo_, startGapMidpoint, startGapWidths));

  // Create the goal gap.
  ompl::base::ScopedState<> goalGapMidpoint(spaceInfo_);
  goalGapMidpoint[0u] =
      goalPos_[0u] + goalInsideWidth_ / 2.0 + (goalOutsideWidth_ - goalInsideWidth_) / 4.0;
  for (std::size_t i = 1u; i < dimensionality_; ++i) {
    goalGapMidpoint[i] = 0.0;
  }
  std::vector<double> goalGapWidths(dimensionality_, goalGapWidth_);
  goalGapWidths.at(0u) =
      (goalOutsideWidth_ - goalInsideWidth_) / 2.0 + std::numeric_limits<double>::epsilon();
  antiObstacles_.emplace_back(std::make_shared<Hyperrectangle<BaseAntiObstacle>>(
      spaceInfo_, goalGapMidpoint, goalGapWidths));
}

}  // namespace ompltools

}  // namespace esp
