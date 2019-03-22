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

#include "esp_planning_contexts/dead_end.h"

#include <cmath>
#include <memory>
#include <functional>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

DeadEnd::DeadEnd(const double distFraction, const double runSeconds,
                                     const double checkResolution)
    : BaseContext(
          2u, std::vector<std::pair<double, double>>(2u, std::pair<double, double>(-1.0, 1.0)),
          runSeconds, "DeadEnd"),
      topHorizontalWidths_(2u, 0.0),     
      sideVerticalWidths_(2u, 0.0),      
      bottomHorizontalWidths_(2u, 0.0),  
      obsThickness_(0.1) {
  // Variable
  // The state space
  std::shared_ptr<ompl::base::RealVectorStateSpace> ss;
  // The problem bounds
  ompl::base::RealVectorBounds problemBounds(BaseContext::dim_);
  // The "characteristic width" of the problem
  double goalDist;
  // The start and goal position
  double startPos;
  double goalPos;

  // Make the state space Rn:
  ss = std::make_shared<ompl::base::RealVectorStateSpace>(BaseContext::dim_);

  // Create the space information class:
  BaseContext::si_ = std::make_shared<ompl::base::SpaceInformation>(ss);

  // Allocate the obstacle world
  rectObs_ = std::make_shared<Hyperrectangle>(BaseContext::si_, false);
  BaseContext::obs_ = rectObs_;

  // Set the problem bounds:
  problemBounds.setLow(BaseContext::limits_.at(0u).first);
  problemBounds.setHigh(BaseContext::limits_.at(0u).second);

  // Store the problem bounds:
  ss->setBounds(problemBounds);

  // Set the validity checker and checking resolution
  BaseContext::si_->setStateValidityChecker(
      static_cast<ompl::base::StateValidityCheckerPtr>(rectObs_));
  BaseContext::si_->setStateValidityCheckingResolution(checkResolution);

  // Call setup!
  BaseContext::si_->setup();

  // Allocate the optimization objective
  BaseContext::opt_ =
      std::make_shared<ompl::base::PathLengthOptimizationObjective>(BaseContext::si_);

  // Set the heuristic to the default:
  BaseContext::opt_->setCostToGoHeuristic(
      std::bind(&ompl::base::goalRegionCostToGo, std::placeholders::_1, std::placeholders::_2));

  // Calculate the characteristic width for the problem
  goalDist =
      distFraction * (BaseContext::limits_.at(0u).second - BaseContext::limits_.at(0u).first);

  // Calculate the start and goal position:
  startPos = -0.5 * goalDist;
  goalPos = 0.5 * goalDist;

  // Create my start:
  // Create a start state on the vector:
  BaseContext::startStates_.push_back(ompl::base::ScopedState<>(ss));

  // Assign to each component
  for (unsigned int j = 0u; j < BaseContext::dim_; ++j) {
    if (j == 0u) {
      BaseContext::startStates_.back()[j] = startPos;
    } else {
      BaseContext::startStates_.back()[j] = 0.0;
    }
  }

  // Create my goal:
  // Create a goal state on the vector:
  BaseContext::goalStates_.push_back(ompl::base::ScopedState<>(ss));

  // Assign to each component
  for (unsigned int j = 0u; j < BaseContext::dim_; ++j) {
    if (j == 0u) {
      BaseContext::goalStates_.back()[j] = goalPos;
    } else {
      BaseContext::goalStates_.back()[j] = 0.0;
    }
  }

  // Allocate the goal:
  BaseContext::goalPtr_ = std::make_shared<ompl::base::GoalState>(BaseContext::si_);

  // Add
  BaseContext::goalPtr_->as<ompl::base::GoalState>()->setState(
      BaseContext::goalStates_.back());

  // Allocate the obstacles' lower-left corners:
  topHorizontal_ = std::make_shared<ompl::base::ScopedState<>>(ss);
  sideVertical_ = std::make_shared<ompl::base::ScopedState<>>(ss);
  bottomHorizontal_ = std::make_shared<ompl::base::ScopedState<>>(ss);

  // Specify the obstacles' lower-left corners:
  (*topHorizontal_)[0] = -1.0 * goalDist;
  (*topHorizontal_)[1] = 0.5 * goalDist;

  (*sideVertical_)[0] = 0;
  (*sideVertical_)[1] = -0.5 * goalDist;

  (*bottomHorizontal_)[0] = (*topHorizontal_)[0];
  (*bottomHorizontal_)[1] = -0.5 * goalDist + -1.0 * obsThickness_;

  // The widths of the obstacles
  topHorizontalWidths_.at(0) = goalDist + obsThickness_;
  topHorizontalWidths_.at(1) = obsThickness_;

  sideVerticalWidths_.at(0) = obsThickness_;
  sideVerticalWidths_.at(1) = goalDist;

  bottomHorizontalWidths_.at(0) = topHorizontalWidths_.at(0);
  bottomHorizontalWidths_.at(1) = topHorizontalWidths_.at(1);

  // Add the obstacles
  rectObs_->addObstacle(std::make_pair(topHorizontal_->get(), topHorizontalWidths_));
  rectObs_->addObstacle(std::make_pair(sideVertical_->get(), sideVerticalWidths_));
  rectObs_->addObstacle(std::make_pair(bottomHorizontal_->get(), bottomHorizontalWidths_));

  // Finally specify the optimization target:
  BaseContext::opt_->setCostThreshold(this->getOptimum());
}

bool DeadEnd::knowsOptimum() const {
  return true;
}

ompl::base::Cost DeadEnd::getOptimum() const {
  ompl::base::Cost startToCorner(std::sqrt(
      std::pow((*bottomHorizontal_)[0u] - BaseContext::startStates_.front()[0u], 2.0) +
      std::pow((*bottomHorizontal_)[1u] - BaseContext::startStates_.front()[1u], 2.0)));
  ompl::base::Cost obsEdge(bottomHorizontalWidths_.at(0u) + sideVerticalWidths_.at(0u));
  ompl::base::Cost otherCornerToGoal(
      std::sqrt(std::pow(BaseContext::goalStates_.front()[0u] -
                             ((*sideVertical_)[0u] + sideVerticalWidths_.at(0u)),
                         2.0) +
                std::pow(BaseContext::goalStates_.front()[1u] - (*sideVertical_)[1u], 2.0)));

  // Combine and return:
  return BaseContext::opt_->combineCosts(
      BaseContext::opt_->combineCosts(startToCorner, obsEdge), otherCornerToGoal);
}

void DeadEnd::setTarget(double targetSpecifier) {
  BaseContext::opt_->setCostThreshold(
      ompl::base::Cost(targetSpecifier * this->getOptimum().value()));
}

std::string DeadEnd::lineInfo() const {
  return std::string();
}

std::string DeadEnd::paraInfo() const {
  return std::string();
}
