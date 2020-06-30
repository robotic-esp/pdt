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

#include "esp_planning_contexts/random_rectangles.h"

#include <vector>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>

#include "esp_obstacles/base_obstacle.h"
#include "esp_obstacles/hyperrectangle.h"
#include "esp_planning_contexts/context_validity_checker_gnat.h"

using namespace std::string_literals;

namespace esp {

namespace ompltools {

RandomRectangles::RandomRectangles(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                                   const std::shared_ptr<const Configuration>& config,
                                   const std::string& name) :
    RealVectorGeometricContext(spaceInfo, config, name),
    numRectangles_(config->get<std::size_t>("context/" + name + "/numObstacles")),
    minSideLength_(config->get<double>("context/" + name + "/minSideLength")),
    maxSideLength_(config->get<double>("context/" + name + "/maxSideLength")),
    startState_(spaceInfo) {
  // Get the start position.
  auto startPosition = config_->get<std::vector<double>>("context/" + name + "/start");

  // Assert configuration sanity.
  if (startPosition.size() != dimensionality_) {
    OMPL_ERROR("%s: Dimensionality of problem and of start specification does not match.",
               name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (minSideLength_ > maxSideLength_) {
    OMPL_ERROR("%s: Specified min side length is greater than specified max side length.",
               name.c_str());
    throw std::runtime_error("Context error.");
  }

  // Fill the start state's coordinates.
  for (std::size_t i = 0u; i < dimensionality_; ++i) {
    startState_[i] = startPosition.at(i);
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
}

ompl::base::ProblemDefinitionPtr RandomRectangles::instantiateNewProblemDefinition() const {
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

ompl::base::ScopedState<ompl::base::RealVectorStateSpace> RandomRectangles::getStartState() const {
  return startState_;
}

void RandomRectangles::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

void RandomRectangles::createObstacles() {
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
    auto obstacle = std::make_shared<Hyperrectangle<BaseObstacle>>(spaceInfo_, anchor, widths);

    // Add this to the obstacles if it doesn't invalidate the start or goal state.
    if (!obstacle->invalidates(startState_)) {
      if (goal_->getType() == ompl::base::GoalType::GOAL_STATE) {
        if (!obstacle->invalidates(goal_->as<ompl::base::GoalState>()->getState())) {
          obstacles_.emplace_back(obstacle);
        } else {
          --i;
        }
      } else if (goal_->getType() == ompl::base::GoalType::GOAL_STATES) {
        auto invalidates = false;
        for (auto i = 0u; i < goal_->as<ompl::base::GoalStates>()->getStateCount(); ++i) {
          if (obstacle->invalidates(goal_->as<ompl::base::GoalStates>()->getState(i))) {
            invalidates = true;
            break;
          }
        }
        if (!invalidates) {
          obstacles_.emplace_back(obstacle);
        } else {
          --i;
        }
      }
    } else {
      --i;
    }
  }
}

}  // namespace ompltools

}  // namespace esp
