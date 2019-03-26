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

/* Authors: Jonathan Gammell */

#include "esp_planning_contexts/obstacle_free.h"

#include <functional>
#include <memory>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "esp_obstacles/random_hyperrectangles.h"

namespace esp {

namespace ompltools {

ObstacleFree::ObstacleFree(const unsigned int dim, const unsigned int maxNumStarts,
                           const unsigned int maxNumGoals, const double runSeconds) :
    BaseContext(dim,
                std::vector<std::pair<double, double>>(dim, std::pair<double, double>(-1.0, 1.0)),
                runSeconds, "Free") {
  // Variables
  // The state space
  std::shared_ptr<ompl::base::RealVectorStateSpace> ss;
  // The problem bounds
  ompl::base::RealVectorBounds problemBounds(BaseContext::dim_);
  // The validity checker:
  ompl::base::StateValidityCheckerPtr vc;
  // The number of starts and goals we're actually creating
  unsigned int numStarts;
  unsigned int numGoals;
  // The elemental position of the start/goal
  double startPos;
  double goalPos;

  // Specify the start/goal "position" term.
  if (maxNumStarts == 1u && maxNumGoals == 1u) {
    centrePos_ = -0.5;
    outsidePos_ = 0.5;
  } else {
    centrePos_ = 0.0;
    outsidePos_ = 0.5;
  }

  // Make the state space Rn:
  ss = std::make_shared<ompl::base::RealVectorStateSpace>(BaseContext::dim_);

  // Create the space information class:
  BaseContext::si_ = std::make_shared<ompl::base::SpaceInformation>(ss);

  // Make an empty obstacle pointer:
  BaseContext::obs_ = std::make_shared<RandomHyperrectangles>(BaseContext::si_, false);

  // Make the validity checker all-true
  vc = std::make_shared<ompl::base::AllValidStateValidityChecker>(BaseContext::si_);

  // Set the problem bounds:
  problemBounds.setLow(BaseContext::limits_.at(0u).first);
  problemBounds.setHigh(BaseContext::limits_.at(0u).second);

  // Store the problem bounds:
  ss->setBounds(problemBounds);

  // Set the validity checker and checking resolution
  BaseContext::si_->setStateValidityChecker(vc);
  BaseContext::si_->setStateValidityCheckingResolution(0.1);

  // Call setup!
  BaseContext::si_->setup();

  // Allocate the optimization objective
  BaseContext::opt_ =
      std::make_shared<ompl::base::PathLengthOptimizationObjective>(BaseContext::si_);

  // Set the heuristic to the default:
  BaseContext::opt_->setCostToGoHeuristic(
      std::bind(&ompl::base::goalRegionCostToGo, std::placeholders::_1, std::placeholders::_2));

  // Given the way we define goals, We can only have 2 per dimension (i.e., 4 in 2D, 6 in 3D, etc)
  numStarts = std::min(maxNumStarts, 2u * BaseContext::dim_);
  numGoals = std::min(maxNumGoals, 2u * BaseContext::dim_);

  // Allocate the goal:
  if (numGoals == 1u) {
    BaseContext::goalPtr_ = std::make_shared<ompl::base::GoalState>(BaseContext::si_);
  } else {
    BaseContext::goalPtr_ = std::make_shared<ompl::base::GoalStates>(BaseContext::si_);
  }

  // Assign positions
  if (numStarts > numGoals) {
    goalPos = centrePos_;
    startPos = outsidePos_;
  } else {
    startPos = centrePos_;
    goalPos = outsidePos_;
  }

  // Create my starts:
  for (unsigned int i = 0u; i < numStarts; ++i) {
    // Create a start state on the vector:
    BaseContext::startStates_.push_back(ompl::base::ScopedState<>(ss));

    // Assign to each component
    for (unsigned int j = 0u; j < BaseContext::dim_; ++j) {
      // Start
      if (((i == 0u) || (i - 1u <= 2u * j)) && (2u * j <= i)) {
        if (i % 2u == 0u) {
          BaseContext::startStates_.back()[j] = startPos;
        } else {
          BaseContext::startStates_.back()[j] = -startPos;
        }
      } else {
        BaseContext::startStates_.back()[j] = 0.0;
      }
    }
  }

  // and the goal(s)
  for (unsigned int i = 0u; i < numGoals; ++i) {
    // Create a start state on the vector:
    BaseContext::goalStates_.push_back(ompl::base::ScopedState<>(ss));

    // Assign to each component
    for (unsigned int j = 0u; j < BaseContext::dim_; ++j) {
      // Goal
      if (((i == 0u) || (i - 1u <= 2u * j)) && (2u * j <= i)) {
        if (i % 2u == 0u) {
          BaseContext::goalStates_.back()[j] = goalPos;
        } else {
          BaseContext::goalStates_.back()[j] = -goalPos;
        }
      } else {
        BaseContext::goalStates_.back()[j] = 0.0;
      }
    }

    // Store
    if (numGoals == 1u) {
      BaseContext::goalPtr_->as<ompl::base::GoalState>()->setState(BaseContext::goalStates_.back());
    } else {
      BaseContext::goalPtr_->as<ompl::base::GoalStates>()->addState(
          BaseContext::goalStates_.back());
    }
  }

  // Now specify the optimization target:
  BaseContext::opt_->setCostThreshold(this->getOptimum());
}

bool ObstacleFree::knowsOptimum() const {
  return true;
}

ompl::base::Cost ObstacleFree::getOptimum() const {
  return BaseContext::getMinimum();
}

void ObstacleFree::setTarget(double targetSpecifier) {
  BaseContext::opt_->setCostThreshold(
      ompl::base::Cost(targetSpecifier * this->getOptimum().value()));
}

std::string ObstacleFree::lineInfo() const {
  return std::string();
}

std::string ObstacleFree::paraInfo() const {
  return std::string();
}

}  // namespace ompltools

}  // namespace esp
