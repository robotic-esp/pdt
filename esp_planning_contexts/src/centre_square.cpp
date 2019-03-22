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

#include "esp_planning_contexts/centre_square.h"

#include <cmath>
#include <functional>
#include <memory>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace esp {

namespace ompltools {

CentreSquare::CentreSquare(const unsigned int dim, const double obsWidth, const double worldWidth,
                           const double runSeconds, const double checkResolution) :
    BaseContext(dim,
                std::vector<std::pair<double, double>>(
                    dim, std::pair<double, double>(-0.5 * worldWidth, 0.5 * worldWidth)),
                runSeconds, "CentreSquare"),
    obsWidth_(obsWidth),
    startPos_(-0.5),
    goalPos_(0.5) {
  // Variable
  // The state space
  std::shared_ptr<ompl::base::RealVectorStateSpace> ss;
  // The problem bounds
  ompl::base::RealVectorBounds problemBounds(BaseContext::dim_);

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

  // Create my start:
  // Create a start state on the vector:
  BaseContext::startStates_.push_back(ompl::base::ScopedState<>(ss));

  // Assign to each component
  for (unsigned int j = 0u; j < BaseContext::dim_; ++j) {
    if (j == 0u) {
      BaseContext::startStates_.back()[j] = startPos_;
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
      BaseContext::goalStates_.back()[j] = goalPos_;
    } else {
      BaseContext::goalStates_.back()[j] = 0.0;
    }
  }

  // Allocate the goal:
  BaseContext::goalPtr_ = std::make_shared<ompl::base::GoalState>(BaseContext::si_);

  // Add
  BaseContext::goalPtr_->as<ompl::base::GoalState>()->setState(BaseContext::goalStates_.back());

  // Set the obstacle's lower-left corner:
  sightLineObs_ = std::make_shared<ompl::base::ScopedState<>>(ss);
  for (unsigned int i = 0u; i < BaseContext::dim_; ++i) {
    (*sightLineObs_)[i] =
        (BaseContext::goalStates_.back()[i] + BaseContext::startStates_.back()[i]) / 2.0 -
        0.5 * obsWidth_;
  }

  // Add the obstacle:
  rectObs_->addObstacle(
      std::make_pair(sightLineObs_->get(), std::vector<double>(BaseContext::dim_, obsWidth_)));

  // Finally specify the optimization target:
  BaseContext::opt_->setCostThreshold(this->getOptimum());
}

bool CentreSquare::knowsOptimum() const {
  return true;
}

ompl::base::Cost CentreSquare::getOptimum() const {
  ompl::base::Cost startToCorner(
      std::sqrt(std::pow((*sightLineObs_)[0u] - BaseContext::startStates_.front()[0u], 2.0) +
                std::pow((*sightLineObs_)[1u] - BaseContext::startStates_.front()[1u], 2.0)));
  ompl::base::Cost obsEdge(obsWidth_);
  ompl::base::Cost otherCornerToGoal(std::sqrt(
      std::pow(BaseContext::goalStates_.front()[0u] - ((*sightLineObs_)[0u] + obsWidth_), 2.0) +
      std::pow(BaseContext::goalStates_.front()[1u] - (*sightLineObs_)[1u], 2.0)));

  // Combine and return:
  return BaseContext::opt_->combineCosts(BaseContext::opt_->combineCosts(startToCorner, obsEdge),
                                         otherCornerToGoal);
}

void CentreSquare::setTarget(double targetSpecifier) {
  BaseContext::opt_->setCostThreshold(
      ompl::base::Cost(targetSpecifier * this->getOptimum().value()));
}

std::string CentreSquare::lineInfo() const {
  std::stringstream rval;

  rval << "Obstacle width: " << obsWidth_ << ".";

  return rval.str();
}

std::string CentreSquare::paraInfo() const {
  return std::string();
}

}  // namespace ompltools

}  // namespace esp
