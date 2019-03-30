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

#include "esp_planning_contexts/regular_rectangles.h"

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

RegularRectangles::RegularRectangles(const unsigned int dim, const double worldHalfWidth,
                                     unsigned int numObsBetween, const double runSeconds,
                                     const double checkResolution) :
    BaseContext(dim,
                std::vector<std::pair<double, double>>(
                    dim, std::pair<double, double>(-1.0 * worldHalfWidth, worldHalfWidth)),
                runSeconds, "RegularRects") {
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
  obsWidths_.reserve(BaseContext::dim_);
  blankWidths_.reserve(BaseContext::dim_);
  origin_.reserve(BaseContext::dim_);

  for (unsigned int i = 0u; i < BaseContext::dim_; ++i) {
    obsWidths_.push_back((goalPos_ - startPos_) / (3.0 * numObsBetween));
    blankWidths_.push_back(2.0 * obsWidths_.at(i));

    if (numObsBetween % 2 == 0) {
      origin_.push_back(0.5 * blankWidths_.at(0u));
    } else {
      origin_.push_back(-0.5 * obsWidths_.at(0u));
    }
  }

  regObs_ = std::make_shared<RepeatingHyperrectangles>(BaseContext::si_, obsWidths_, blankWidths_,
                                                       origin_);

  // Set the problem bounds:
  problemBounds.setLow(BaseContext::limits_.at(0u).first);
  problemBounds.setHigh(BaseContext::limits_.at(0u).second);

  // Store the problem bounds:
  ss->setBounds(problemBounds);

  // Set the validity checker and checking resolution
  BaseContext::si_->setStateValidityChecker(
      static_cast<ompl::base::StateValidityCheckerPtr>(regObs_));
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
  BaseContext::startStates_.back()[0u] = startPos_;
  for (unsigned int j = 1u; j < BaseContext::dim_; ++j) {
    BaseContext::startStates_.back()[j] = 0.0;
  }

  // Create my goal:
  // Create a goal state on the vector:
  BaseContext::goalStates_.push_back(ompl::base::ScopedState<>(ss));

  // Assign to each component
  BaseContext::goalStates_.back()[0u] = goalPos_;
  for (unsigned int j = 1u; j < BaseContext::dim_; ++j) {
    BaseContext::goalStates_.back()[j] = 0.0;
  }

  // Allocate the goal:
  BaseContext::goalPtr_ = std::make_shared<ompl::base::GoalState>(BaseContext::si_);

  // Add
  BaseContext::goalPtr_->as<ompl::base::GoalState>()->setState(BaseContext::goalStates_.back());

  // Finally specify the optimization target:
  BaseContext::opt_->setCostThreshold(this->getMinimum());
}

bool RegularRectangles::knowsOptimum() const {
  return false;
}

ompl::base::Cost RegularRectangles::getOptimum() const {
  throw ompl::Exception("The global optimum is unknown", BaseContext::name_);
}

void RegularRectangles::setTarget(double targetSpecifier) {
  BaseContext::opt_->setCostThreshold(ompl::base::Cost(targetSpecifier));
}

std::string RegularRectangles::lineInfo() const {
  std::stringstream rval;

  rval << "Obstacles starting at [";
  for (unsigned int i = 0u; i < BaseContext::dim_; ++i) {
    rval << origin_.at(i);
    if (i != BaseContext::dim_ - 1u) {
      rval << ", ";
    }
  }
  rval << "] with widths [";
  for (unsigned int i = 0u; i < BaseContext::dim_; ++i) {
    rval << obsWidths_.at(i);
    if (i != BaseContext::dim_ - 1u) {
      rval << ", ";
    }
  }
  rval << "] and gaps [";
  for (unsigned int i = 0u; i < BaseContext::dim_; ++i) {
    rval << blankWidths_.at(i);
    if (i != BaseContext::dim_ - 1u) {
      rval << ", ";
    }
  }
  rval << "]." << std::flush;

  return rval.str();
}

std::string RegularRectangles::paraInfo() const {
  return std::string();
}

void RegularRectangles::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

}  // namespace ompltools

}  // namespace esp
