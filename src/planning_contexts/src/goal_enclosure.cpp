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

// Authors: Jonathan Gammell, Marlin Strub

#include "pdt/planning_contexts/goal_enclosure.h"

#include <vector>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>

#include "pdt/obstacles/base_obstacle.h"
#include "pdt/obstacles/hyperrectangle.h"
#include "pdt/planning_contexts/context_validity_checker.h"

namespace pdt {

namespace planning_contexts {

GoalEnclosure::GoalEnclosure(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                             const std::shared_ptr<const config::Configuration>& config,
                             const std::string& name) :
    RealVectorGeometricContext(spaceInfo, config, name),
    goalOutsideWidth_(config->get<double>("context/" + name + "/goalOutsideWidth")),
    goalInsideWidth_(config->get<double>("context/" + name + "/goalInsideWidth")),
    goalGapWidth_(config->get<double>("context/" + name + "/goalGapWidth")) {
  if (config->get<std::vector<double>>("context/" + name + "/goal").size() != dimensionality_) {
    OMPL_ERROR("%s: Dimensionality of problem and of goal specification does not match.",
               name.c_str());
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

  startGoalPairs_ = makeStartGoalPair();
}

void GoalEnclosure::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

void GoalEnclosure::createObstacles() {
  ompl::base::ScopedState<> goalAnchor(spaceInfo_);
  for (auto i = 0u; i < dimensionality_; ++i) {
    goalAnchor[i] = config_->get<std::vector<double>>("context/" + name_ + "/goal").at(i);
  }
  std::vector<double> goalWidths(dimensionality_, goalOutsideWidth_);
  obstacles_.push_back(std::make_shared<obstacles::Hyperrectangle<obstacles::BaseObstacle>>(
      spaceInfo_, goalAnchor, goalWidths));
}

void GoalEnclosure::createAntiObstacles() {
  // Create the inside.
  ompl::base::ScopedState<> goalState(spaceInfo_);
  for (auto i = 0u; i < dimensionality_; ++i) {
    goalState[i] = config_->get<std::vector<double>>("context/" + name_ + "/goal").at(i);
  }
  std::vector<double> goalWidths(dimensionality_, goalInsideWidth_);
  antiObstacles_.push_back(std::make_shared<obstacles::Hyperrectangle<obstacles::BaseAntiObstacle>>(
      spaceInfo_, goalState, goalWidths));

  // Create the gap.
  ompl::base::ScopedState<> goalGapMidpoint(spaceInfo_);
  goalGapMidpoint[0u] = config_->get<std::vector<double>>("context/" + name_ + "/goal").at(0u) +
                        goalInsideWidth_ / 2.0 + (goalOutsideWidth_ - goalInsideWidth_) / 4.0;
  for (auto i = 1u; i < dimensionality_; ++i) {
    goalGapMidpoint[i] = 0.0;
  }
  std::vector<double> goalGapWidths(dimensionality_, goalGapWidth_);
  goalGapWidths.at(0u) =
      (goalOutsideWidth_ - goalInsideWidth_) / 2.0 + std::numeric_limits<double>::epsilon();
  antiObstacles_.push_back(std::make_shared<obstacles::Hyperrectangle<obstacles::BaseAntiObstacle>>(
      spaceInfo_, goalGapMidpoint, goalGapWidths));
}

}  // namespace planning_contexts

}  // namespace pdt
