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

#include "esp_planning_contexts/random_rectangles_multi_start_goal.h"

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

RandomRectanglesMultiStartGoal::RandomRectanglesMultiStartGoal(const std::shared_ptr<const Configuration>& config,
                                   const std::string& name) :
    BaseContext(config, name),
    numRectangles_(config->get<std::size_t>("Contexts/" + name + "/numObstacles")),
    minSideLength_(config->get<double>("Contexts/" + name + "/minSideLength")),
    maxSideLength_(config->get<double>("Contexts/" + name + "/maxSideLength")),
    numStarts_(config->get<std::size_t>("Contexts/" + name + "/numStarts")),
    numGoals_(config->get<std::size_t>("Contexts/" + name + "/numGoals")) {
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
  // Create a state space and set the bounds.
  auto stateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(dimensionality_);
  stateSpace->setBounds(bounds_.at(0u).first, bounds_.at(0u).second);

  // Create the space information class:
  spaceInfo_ = std::make_shared<ompl::base::SpaceInformation>(stateSpace);

  // Create the start states.
  for (std::size_t i = 0u; i < numStarts_; ++i) {
    startStates_.emplace_back(spaceInfo_);
    startStates_.back().random();
  }

  // Create the goal states.
  for (std::size_t i = 0u; i < numGoals_; ++i) {
    goalStates_.emplace_back(spaceInfo_);
    goalStates_.back().random();
  }

  if (numGoals_ > 1u) {
    goalPtr_ = std::make_shared<ompl::base::GoalStates>(spaceInfo_);
    for (std::size_t i = 0u; i < numGoals_; ++i) {
      goalPtr_->as<ompl::base::GoalStates>()->addState(goalStates_.at(i));
    }
  } else {
    goalPtr_ = std::make_shared<ompl::base::GoalState>(spaceInfo_);
    goalPtr_->as<ompl::base::GoalState>()->setState(goalStates_.back());
  }

  // Create the obstacles.
  createObstacles();

  // Create the validity checker and add the obstacle.
  validityChecker_ = std::make_shared<ContextValidityCheckerGNAT>(spaceInfo_);
  validityChecker_->addObstacles(obstacles_);

  // Set the validity checker and the check resolution.
  spaceInfo_->setStateValidityChecker(
      static_cast<ompl::base::StateValidityCheckerPtr>(validityChecker_));
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

  // Specify the optimization target.
  optimizationObjective_->setCostThreshold(computeMinPossibleCost());
}

bool RandomRectanglesMultiStartGoal::knowsOptimum() const {
  return false;
}

ompl::base::Cost RandomRectanglesMultiStartGoal::computeOptimum() const {
  throw ompl::Exception("The global optimum is unknown.", BaseContext::name_);
}

void RandomRectanglesMultiStartGoal::setTarget(double targetSpecifier) {
  optimizationObjective_->setCostThreshold(
      ompl::base::Cost(targetSpecifier * this->computeOptimum().value()));
}

std::string RandomRectanglesMultiStartGoal::lineInfo() const {
  std::stringstream rval;

  return rval.str();
}

std::string RandomRectanglesMultiStartGoal::paraInfo() const {
  return std::string();
}

void RandomRectanglesMultiStartGoal::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

void RandomRectanglesMultiStartGoal::createObstacles() {
  // Instantiate obstacles.
  for (int i = 0; i < static_cast<int>(numRectangles_); ++i) {
    // Create a random midpoint (uniform).
    ompl::base::ScopedState<> midpoint(spaceInfo_);
    midpoint.random();
    // Create random widths (uniform).
    std::vector<double> widths(dimensionality_, 0.0);
    for (std::size_t j = 0; j < dimensionality_; ++j) {
      widths[j] = rng_.uniformReal(minSideLength_, maxSideLength_);
    }
    bool invalidates = false;
    auto obstacle = std::make_shared<Hyperrectangle<BaseObstacle>>(spaceInfo_, midpoint, widths);
    // Add this to the obstacles if it doesn't invalidate the start or goal states.
    for (const auto& start : startStates_) {
      if (obstacle->invalidates(start.get())) {
        invalidates = true;
        break;
      }
    }
    if (!invalidates) {
      for (const auto& goal : goalStates_) {
        if (obstacle->invalidates(goal.get())) {
          invalidates = true;
          break;
        }
      }
    }
    if (!invalidates) {
      obstacles_.emplace_back(obstacle);
    } else {
      --i;
    }
  }
}

}  // namespace ompltools

}  // namespace esp