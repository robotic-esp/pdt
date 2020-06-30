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

#include <vector>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>

#include "esp_obstacles/base_obstacle.h"
#include "esp_obstacles/hyperrectangle.h"
#include "esp_planning_contexts/context_validity_checker.h"

namespace esp {

namespace ompltools {

DoubleEnclosure::DoubleEnclosure(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                                 const std::shared_ptr<const Configuration>& config,
                                 const std::string& name) :
    RealVectorGeometricContext(spaceInfo, config, name),
    dimensionality_(spaceInfo_->getStateDimension()),
    startOutsideWidth_(config->get<double>("context/" + name + "/startOutsideWidth")),
    startInsideWidth_(config->get<double>("context/" + name + "/startInsideWidth")),
    startGapWidth_(config->get<double>("context/" + name + "/startGapWidth")),
    goalOutsideWidth_(config->get<double>("context/" + name + "/goalOutsideWidth")),
    goalInsideWidth_(config->get<double>("context/" + name + "/goalInsideWidth")),
    goalGapWidth_(config->get<double>("context/" + name + "/goalGapWidth")),
    startState_(spaceInfo) {
  // Get the start and goal positions.
  auto startPosition = config_->get<std::vector<double>>("context/" + name + "/start");

  // Assert configuration sanity.
  if (startPosition.size() != dimensionality_) {
    OMPL_ERROR("%s: Dimensionality of problem and of start specification does not match.",
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
  for (std::size_t i = 0u; i < spaceInfo_->getStateDimension(); ++i) {
    startState_[i] = startPosition.at(i);
  }
}

ompl::base::ProblemDefinitionPtr DoubleEnclosure::instantiateNewProblemDefinition() const {
  // Instantiate a new problem definition.
  auto problemDefinition = std::make_shared<ompl::base::ProblemDefinition>(spaceInfo_);

  // Set the objective.
  problemDefinition->setOptimizationObjective(objective_);

  // Set the start state in the problem definition.
  problemDefinition->addStartState(startState_);

  // Set the goal for the problem definition.
  problemDefinition->setGoal(goal_);

  // Return the new definition.
  return problemDefinition;
}

ompl::base::ScopedState<ompl::base::RealVectorStateSpace> DoubleEnclosure::getStartState() const {
  return startState_;
}

void DoubleEnclosure::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

void DoubleEnclosure::createObstacles() {
  // Create the anchor for the start enclosure obstacle.
  ompl::base::ScopedState<> startAnchor(spaceInfo_);
  for (std::size_t i = 0u; i < dimensionality_; ++i) {
    startAnchor[i] = config_->get<std::vector<double>>("context/" + name_ + "/start").at(i);
  }

  // Get the widths of the start enclosure.
  std::vector<double> startWidths(dimensionality_, startOutsideWidth_);

  // Create the start enclosure obstacle.
  obstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseObstacle>>(spaceInfo_, startAnchor, startWidths));

  // Create the anchor for the goal enclosure obstacle.
  ompl::base::ScopedState<> goalAnchor(spaceInfo_);
  for (std::size_t i = 0u; i < dimensionality_; ++i) {
    goalAnchor[i] = config_->get<std::vector<double>>("context/" + name_ + "/goal").at(i);
  }

  // Get the widths of the goal enclosure.
  std::vector<double> goalWidths(dimensionality_, goalOutsideWidth_);

  // Create the goal enclosure obstacle.
  obstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseObstacle>>(spaceInfo_, goalAnchor, goalWidths));
}

void DoubleEnclosure::createAntiObstacles() {
  // Create the anchor for the start enclosure anti obstacle.
  ompl::base::ScopedState<> startAnchor(spaceInfo_);
  for (std::size_t i = 0u; i < dimensionality_; ++i) {
    startAnchor[i] = config_->get<std::vector<double>>("context/" + name_ + "/start").at(i);
  }

  // Get the widths of the start enclosure anti obstacle.
  std::vector<double> startWidths(dimensionality_, startInsideWidth_);

  // Create the start enclosure anti obstacle.
  antiObstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseAntiObstacle>>(spaceInfo_, startAnchor, startWidths));

  // Create the anchor for the goal enclosure anti obstacle.
  ompl::base::ScopedState<> goalAnchor(spaceInfo_);
  for (std::size_t i = 0u; i < dimensionality_; ++i) {
    goalAnchor[i] = config_->get<std::vector<double>>("context/" + name_ + "/goal").at(i);
  }

  // Get the widhts of the goal enclosre anti obstacle.
  std::vector<double> goalWidths(dimensionality_, goalInsideWidth_);

  // Create the goal enclosure anti obstacle.
  antiObstacles_.emplace_back(
      std::make_shared<Hyperrectangle<BaseAntiObstacle>>(spaceInfo_, goalAnchor, goalWidths));

  // Create the start gap.
  ompl::base::ScopedState<> startGapMidpoint(spaceInfo_);
  startGapMidpoint[0u] = config_->get<std::vector<double>>("context/" + name_ + "/start").at(0u) -
                         startInsideWidth_ / 2.0 - (startOutsideWidth_ - startInsideWidth_) / 4.0;
  for (std::size_t i = 1u; i < dimensionality_; ++i) {
    startGapMidpoint[i] = 0.0;
  }
  std::vector<double> startGapWidths(dimensionality_, startGapWidth_);
  startGapWidths.at(0u) = (startOutsideWidth_ - startInsideWidth_) / 2.0 + 1e-3;
  antiObstacles_.emplace_back(std::make_shared<Hyperrectangle<BaseAntiObstacle>>(
      spaceInfo_, startGapMidpoint, startGapWidths));

  // Create the goal gap.
  ompl::base::ScopedState<> goalGapMidpoint(spaceInfo_);
  goalGapMidpoint[0u] = config_->get<std::vector<double>>("context/" + name_ + "/goal").at(0u) +
                        goalInsideWidth_ / 2.0 + (goalOutsideWidth_ - goalInsideWidth_) / 4.0;
  for (std::size_t i = 1u; i < dimensionality_; ++i) {
    goalGapMidpoint[i] = 0.0;
  }
  std::vector<double> goalGapWidths(dimensionality_, goalGapWidth_);
  goalGapWidths.at(0u) = (goalOutsideWidth_ - goalInsideWidth_) / 2.0 + 1e-3;
  antiObstacles_.emplace_back(std::make_shared<Hyperrectangle<BaseAntiObstacle>>(
      spaceInfo_, goalGapMidpoint, goalGapWidths));
}

}  // namespace ompltools

}  // namespace esp
