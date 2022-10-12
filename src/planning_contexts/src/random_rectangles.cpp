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

#include "pdt/planning_contexts/random_rectangles.h"

#include <vector>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>

#include "pdt/obstacles/base_obstacle.h"
#include "pdt/obstacles/hyperrectangle.h"
#include "pdt/planning_contexts/context_validity_checker_gnat.h"

using namespace std::string_literals;

namespace pdt {

namespace planning_contexts {

RandomRectangles::RandomRectangles(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                                   const std::shared_ptr<const config::Configuration>& config,
                                   const std::string& name) :
    RealVectorGeometricContext(spaceInfo, config, name),
    numRectangles_(config->get<std::size_t>("context/" + name + "/numObstacles")),
    minSideLength_(config->get<double>("context/" + name + "/minSideLength")),
    maxSideLength_(config->get<double>("context/" + name + "/maxSideLength")) {
  if (minSideLength_ > maxSideLength_) {
    OMPL_ERROR("%s: Specified min side length is greater than specified max side length.",
               name.c_str());
    throw std::runtime_error("Context error.");
  }

  bool generateQueriesBeforeObstacles = false;

  // If we only specify one single start, we first place that start/goal pair, and then generate
  // valid obstacles around them.
  if (config_->contains("context/" + name_ + "/starts")) {
    if (config_->get<std::string>("context/" + name + "/starts/type") == "specified") {
      if (config_->get<std::size_t>("context/" + name + "/starts/numGenerated") == 1) {
        generateQueriesBeforeObstacles = true;
      } else {
        throw std::runtime_error(
            "Context error. Multiple specified starts/goals are not supported for this context at "
            "the moment.");
      }
    } else if (config_->get<std::string>("context/" + name + "/starts/type") == "generated") {
      generateQueriesBeforeObstacles = false;
    } else {
      throw std::runtime_error(
          "Context error. The only start types that are currently supported are 'specified' and "
          "'generated'.");
    }
  } else if (config_->contains("context/" + name_ + "/start")) {
    generateQueriesBeforeObstacles = true;
  } else {
    throw std::runtime_error("Context error. Neither 'start' nor 'starts' specified.");
  }

  if (generateQueriesBeforeObstacles) {
    startGoalPairs_ = makeStartGoalPair();
  }

  // Create the obstacles and add them to the validity checker.
  createObstacles();

  // Create the validity checker.
  std::shared_ptr<ContextValidityChecker> validityChecker;
  if (numRectangles_ < 500) {
    validityChecker = std::make_shared<ContextValidityChecker>(spaceInfo_);
  } else {
    validityChecker = std::make_shared<ContextValidityCheckerGNAT>(spaceInfo_);
  }

  // Add the obstacles to the validity checker.
  validityChecker->addObstacles(obstacles_);

  // Set the validity checker and the check resolution.
  spaceInfo_->setStateValidityChecker(validityChecker);
  spaceInfo_->setStateValidityCheckingResolution(
      config->get<double>("context/" + name + "/collisionCheckResolution"));

  // Set up the space info.
  spaceInfo_->setup();

  if (!generateQueriesBeforeObstacles) {
    startGoalPairs_ = makeStartGoalPair();
  }
}

void RandomRectangles::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

void RandomRectangles::createObstacles() {
  // Instantiate obstacles.
  while (obstacles_.size() < numRectangles_) {
    // Create a random anchor (uniform).
    ompl::base::ScopedState<> anchor(spaceInfo_);
    anchor.random();

    // Create random widths (uniform).
    std::vector<double> widths(dimensionality_, 0.0);
    for (std::size_t j = 0; j < dimensionality_; ++j) {
      widths[j] = rng_.uniformReal(minSideLength_, maxSideLength_);
    }

    auto obstacle = std::make_shared<obstacles::Hyperrectangle<obstacles::BaseObstacle>>(
        spaceInfo_, anchor, widths);

    // Add this to the obstacles if it doesn't invalidate the start or goal state.
    bool invalidates = false;
    for (const auto& startGoalPair : startGoalPairs_) {
      for (const auto& start : startGoalPair.start) {
        if (obstacle->invalidates(start)) {
          invalidates = true;
          // Break out of inner for loop over starts
          break;
        }
      }
      if (invalidates) {
        // Break out of outer for loop over start-goal pairs
        break;
      }

      if (goalType_ == ompl::base::GoalType::GOAL_STATE) {
        if (obstacle->invalidates(startGoalPair.goal->as<ompl::base::GoalState>()->getState())) {
          invalidates = true;
        }
      } else if (goalType_ == ompl::base::GoalType::GOAL_STATES) {
        for (auto i = 0u; i < startGoalPair.goal->as<ompl::base::GoalStates>()->getStateCount();
             ++i) {
          if (obstacle->invalidates(
                  startGoalPair.goal->as<ompl::base::GoalStates>()->getState(i))) {
            invalidates = true;
            // Break out of inner for loop over goals
            break;
          }
        }
      }

      if (invalidates) {
        // Break out of outer for loop over start-goal pairs
        break;
      }
    }

    if (!invalidates) {
      obstacles_.push_back(obstacle);
    }
  }
}

}  // namespace planning_contexts

}  // namespace pdt
