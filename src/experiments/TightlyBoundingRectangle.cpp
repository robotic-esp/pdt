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

// Me!
#include "experiments/TightlyBoundingRectangle.h"

// An Obstacle-World
#include "obstacles/HyperrectangleObstacles.h"

// For std::shared_ptr, etc.
#include <memory>
// For std::bind
#include <functional>

// OMPL
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/goals/GoalStates.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/util/GeometricEquations.h"

TightlyBoundingRectangle::TightlyBoundingRectangle(const unsigned int dim,
                                                   const double fociDistance,
                                                   const double transverseDiameter,
                                                   const double runSeconds)
    : BaseExperiment(
          dim, std::vector<std::pair<double, double>>(dim, std::pair<double, double>(-1.0, 1.0)),
          runSeconds,
          "TightRect"),  // We will have to manually overwrite the limits in our constructor
      dFoci_(fociDistance),
      dTrans_(transverseDiameter) {
  // Variables
  // The state space
  std::shared_ptr<ompl::base::RealVectorStateSpace> ss;
  // The problem bounds
  ompl::base::RealVectorBounds problemBounds(BaseExperiment::dim_);
  // The validity checker:
  ompl::base::StateValidityCheckerPtr vc;
  // The conjugate diameter:
  double dConj;

  // Make the state space Rn:
  ss = std::make_shared<ompl::base::RealVectorStateSpace>(BaseExperiment::dim_);

  // Create the space information class:
  BaseExperiment::si_ = std::make_shared<ompl::base::SpaceInformation>(ss);

  // Make an empty obstacle pointer:
  BaseExperiment::obs_ = std::make_shared<HyperrectangleObstacles>(BaseExperiment::si_, false);

  // Make the validity checker all-true
  vc = std::make_shared<ompl::base::AllValidStateValidityChecker>(BaseExperiment::si_);

  // Calculate the conjugate diameter:
  dConj = std::sqrt(dTrans_ * dTrans_ - dFoci_ * dFoci_);

  // Correct the limits and set the problem bounds:
  for (unsigned int i = 0u; i < BaseExperiment::dim_; ++i) {
    // Variable
    // The half limit for this dimension:
    double halfLimit;

    // Transverse or conjugate?
    if (i == 0u) {
      // First dimension is transverse diameter
      halfLimit = 0.5 * dTrans_;
    } else {
      // Rest are conjugate
      halfLimit = 0.5 * dConj;
    }

    // Set the limits:
    BaseExperiment::limits_.at(i).first = -halfLimit;
    BaseExperiment::limits_.at(i).second = halfLimit;

    // Then set the problem bounds:
    problemBounds.setLow(i, BaseExperiment::limits_.at(i).first);
    problemBounds.setHigh(i, BaseExperiment::limits_.at(i).second);
  }

  // Store the problem bounds:
  ss->setBounds(problemBounds);

  // Set the validity checker and checking resolution
  BaseExperiment::si_->setStateValidityChecker(vc);
  BaseExperiment::si_->setStateValidityCheckingResolution(0.1);

  // Call setup!
  BaseExperiment::si_->setup();

  // Allocate the optimization objective
  BaseExperiment::opt_ =
      std::make_shared<ompl::base::PathLengthOptimizationObjective>(BaseExperiment::si_);

  // Set the heuristic to the default:
  BaseExperiment::opt_->setCostToGoHeuristic(
      std::bind(&ompl::base::goalRegionCostToGo, std::placeholders::_1, std::placeholders::_2));

  // Allocate the goal:
  BaseExperiment::goalPtr_ = std::make_shared<ompl::base::GoalState>(BaseExperiment::si_);

  // Create my start and goal states on the vector:
  BaseExperiment::startStates_.push_back(ompl::base::ScopedState<>(ss));
  BaseExperiment::goalStates_.push_back(ompl::base::ScopedState<>(ss));

  // Assign to each component
  for (unsigned int i = 0u; i < BaseExperiment::dim_; ++i) {
    // Start
    if (i == 0u) {
      BaseExperiment::startStates_.back()[i] = -0.5 * dFoci_;
      BaseExperiment::goalStates_.back()[i] = 0.5 * dFoci_;
    } else {
      BaseExperiment::startStates_.back()[i] = 0.0;
      BaseExperiment::goalStates_.back()[i] = 0.0;
    }
  }

  // Store the goal:
  BaseExperiment::goalPtr_->as<ompl::base::GoalState>()->setState(
      BaseExperiment::goalStates_.back());

  // Now specify the optimization target:
  BaseExperiment::opt_->setCostThreshold(this->getOptimum());
}

bool TightlyBoundingRectangle::knowsOptimum() const {
  return true;
}

ompl::base::Cost TightlyBoundingRectangle::getOptimum() const {
  return BaseExperiment::getMinimum();
}

void TightlyBoundingRectangle::setTarget(double targetSpecifier) {
  BaseExperiment::opt_->setCostThreshold(
      ompl::base::Cost(targetSpecifier * this->getOptimum().value()));
}

std::string TightlyBoundingRectangle::lineInfo() const {
  std::stringstream rval;

  rval << "... (" << BaseExperiment::limits_.at(1u).first << ", "
       << BaseExperiment::limits_.at(1u).second << "). PHS/X: "
       << ompl::prolateHyperspheroidMeasure(BaseExperiment::dim_, dFoci_, dTrans_) /
              BaseExperiment::si_->getSpaceMeasure();

  return rval.str();
}

std::string TightlyBoundingRectangle::paraInfo() const {
  return std::string();
}
