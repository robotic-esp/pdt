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

#include "pdt/planning_contexts/random_rectangles_multi_start_goal.h"

#include <vector>

#include <ompl/base/GoalTypes.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>

#include "pdt/obstacles/base_obstacle.h"
#include "pdt/obstacles/hyperrectangle.h"
#include "pdt/planning_contexts/context_validity_checker_gnat.h"

namespace pdt {

namespace planning_contexts {

RandomRectanglesMultiStartGoal::RandomRectanglesMultiStartGoal(
    const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
    const std::shared_ptr<const config::Configuration>& config, const std::string& name) :
    RealVectorGeometricContext(spaceInfo, config, name),
    numRectangles_(config->get<std::size_t>("context/" + name + "/numObstacles")),
    minSideLength_(config->get<double>("context/" + name + "/minSideLength")),
    maxSideLength_(config->get<double>("context/" + name + "/maxSideLength")),
    numStarts_(config->get<std::size_t>("context/" + name + "/numStarts")),
    numGoals_(config->get<std::size_t>("context/" + name + "/numGoals")) {
  startGoalPairs_ = makeStartGoalPair();

  // Assert configuration sanity.
  if (numStarts_ == 0u) {
    OMPL_ERROR("%s: Must at least have one start.", name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (numGoals_ == 0u) {
    OMPL_ERROR("%s: Must at least have one goal.", name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (minSideLength_ > maxSideLength_) {
    OMPL_ERROR("%s: Specified min side length is greater than specified max side length.",
               name.c_str());
    throw std::runtime_error("Context error.");
  }

  // Create the validity checker.
  auto validityChecker = std::make_shared<ContextValidityCheckerGNAT>(spaceInfo_);

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

std::vector<StartGoalPair> RandomRectanglesMultiStartGoal::makeStartGoalPair() const {
  if (config_->contains("context/" + name_ + "/starts")) {
    OMPL_ERROR("MultiStartGoal context does not support multiple queries.");
    throw std::runtime_error("Context error.");
  }

  // Fill the start and goal states' coordinates.
  std::vector<ompl::base::ScopedState<>> startStates;

  // Create the start states.
  for (std::size_t i = 0u; i < numStarts_; ++i) {
    startStates.emplace_back(spaceInfo_);
    startStates.back().random();
  }

  StartGoalPair pair;
  pair.start = startStates;
  pair.goal = createGoal();

  return {pair};
}

void RandomRectanglesMultiStartGoal::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

void RandomRectanglesMultiStartGoal::createObstacles() {
  // Instantiate obstacles.
  for (int i = 0; i < static_cast<int>(numRectangles_); ++i) {
    // Create a random anchor (uniform).
    ompl::base::ScopedState<> anchor(spaceInfo_);
    anchor.random();
    // Create random widths (uniform).
    std::vector<double> widths(dimensionality_, 0.0);
    for (std::size_t j = 0; j < dimensionality_; ++j) {
      widths[j] = rng_.uniformReal(minSideLength_, maxSideLength_);
    }
    bool invalidates = false;
    auto obstacle = std::make_shared<obstacles::Hyperrectangle<obstacles::BaseObstacle>>(
        spaceInfo_, anchor, widths);
    // Add this to the obstacles if it doesn't invalidate the start or goal states.
    for (const auto& start : startGoalPairs_[0].start) {
      if (obstacle->invalidates(start)) {
        invalidates = true;
        break;
      }
    }
    if (!invalidates && (goalType_ == ompl::base::GoalType::GOAL_STATE ||
                         goalType_ == ompl::base::GoalType::GOAL_STATES)) {
      const auto& goal = startGoalPairs_[0].goal;
      if (goalType_ == ompl::base::GoalType::GOAL_STATE) {
        invalidates = obstacle->invalidates(goal->as<ompl::base::GoalState>()->getState());
        break;
      }
      if (goalType_ == ompl::base::GoalType::GOAL_STATES) {
        for (auto i = 0u; i < goal->as<ompl::base::GoalStates>()->getStateCount(); ++i) {
          if (obstacle->invalidates(goal->as<ompl::base::GoalStates>()->getState(i))) {
            invalidates = true;
            break;
          }
        }
      }
    }
    if (!invalidates) {
      obstacles_.push_back(obstacle);
    } else {
      --i;
    }
  }
}

}  // namespace planning_contexts

}  // namespace pdt
