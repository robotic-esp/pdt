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
#include "experiments/RandomRectanglesExperiment.h"

// STL
#include <cmath>
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

RandomRectanglesExperiment::RandomRectanglesExperiment(const unsigned int dim,
                                                       const unsigned int numObs,
                                                       const double obsRatio,
                                                       const double runSeconds,
                                                       const double checkResolution)
    : BaseExperiment(
          dim, std::vector<std::pair<double, double>>(dim, std::pair<double, double>(-1.0, 1.0)),
          runSeconds, "RandRect") {
  // Variable
  // The state space
  std::shared_ptr<ompl::base::RealVectorStateSpace> ss;
  // The problem bounds
  ompl::base::RealVectorBounds problemBounds(BaseExperiment::dim_);
  // The width of the sightline obstacle
  double sightLineWidth;
  // The measure of obstacles
  double obsMeasure;

  // Make the state space Rn:
  ss = std::make_shared<ompl::base::RealVectorStateSpace>(BaseExperiment::dim_);

  // Create the space information class:
  BaseExperiment::si_ = std::make_shared<ompl::base::SpaceInformation>(ss);

  // Allocate the obstacle world
  rectObs_ = std::make_shared<HyperrectangleObstacles>(BaseExperiment::si_, false);
  BaseExperiment::obs_ = rectObs_;

  // Set the problem bounds:
  problemBounds.setLow(BaseExperiment::limits_.at(0u).first);
  problemBounds.setHigh(BaseExperiment::limits_.at(0u).second);

  // Store the problem bounds:
  ss->setBounds(problemBounds);

  // Set the validity checker and checking resolution
  BaseExperiment::si_->setStateValidityChecker(
      static_cast<ompl::base::StateValidityCheckerPtr>(rectObs_));
  BaseExperiment::si_->setStateValidityCheckingResolution(checkResolution);

  // Call setup!
  BaseExperiment::si_->setup();

  // Allocate the optimization objective
  BaseExperiment::opt_ =
      std::make_shared<ompl::base::PathLengthOptimizationObjective>(BaseExperiment::si_);

  // Set the heuristic to the default:
  BaseExperiment::opt_->setCostToGoHeuristic(
      std::bind(&ompl::base::goalRegionCostToGo, std::placeholders::_1, std::placeholders::_2));

  // Create my start:
  // Create a start state on the vector:
  BaseExperiment::startStates_.push_back(ompl::base::ScopedState<>(ss));

  // Assign to each component
  BaseExperiment::startStates_.back()[0u] = startPos_;
  for (unsigned int j = 1u; j < BaseExperiment::dim_; ++j) {
    BaseExperiment::startStates_.back()[j] = 0.0;
  }

  // Create my goal:
  // Create a goal state on the vector:
  BaseExperiment::goalStates_.push_back(ompl::base::ScopedState<>(ss));

  // Assign to each component
  BaseExperiment::goalStates_.back()[0u] = goalPos_;
  for (unsigned int j = 1u; j < BaseExperiment::dim_; ++j) {
    BaseExperiment::goalStates_.back()[j] = 0.0;
  }

  // Allocate the goal:
  BaseExperiment::goalPtr_ = std::make_shared<ompl::base::GoalState>(BaseExperiment::si_);

  // Add
  BaseExperiment::goalPtr_->as<ompl::base::GoalState>()->setState(
      BaseExperiment::goalStates_.back());

  // Calculate the minimum and maximum radius of the obstacles:
  // First, calculate the desired obstacle volume of the problem:
  obsMeasure = obsRatio * BaseExperiment::si_->getSpaceMeasure();

  // Then, calculate the mean radius necessary to get the desired obstacle volume with the desired
  // number of obstacles:
  meanObsWidth_ = std::pow(obsMeasure / static_cast<double>(numObs), 1.0 / BaseExperiment::dim_);

  // And then the sightline width.  The min is here to make sure we don't swallow either the start
  // or goal
  sightLineWidth = std::min(BaseExperiment::getMinimum().value() / 7.5, meanObsWidth_);

  // Set the sight-line obstacle's lower-left corner:
  sightLineObs_ = std::make_shared<ompl::base::ScopedState<>>(ss);
  for (unsigned int i = 0u; i < BaseExperiment::dim_; ++i) {
    (*sightLineObs_)[i] =
        (BaseExperiment::goalStates_.back()[i] + BaseExperiment::startStates_.back()[i]) / 2.0 -
        0.5 * sightLineWidth;
  }

  // Add the obstacle.:
  rectObs_->addObstacle(std::make_pair(sightLineObs_->get(),
                                       std::vector<double>(BaseExperiment::dim_, sightLineWidth)));

  // Create a random set of obstacles
  if (obsRatio > 0.0) {
    // A temporary vector
    std::vector<ompl::base::ScopedState<>> tVec;

    // Copy into
    tVec.insert(tVec.end(), BaseExperiment::startStates_.begin(),
                BaseExperiment::startStates_.end());
    tVec.insert(tVec.end(), BaseExperiment::goalStates_.begin(), BaseExperiment::goalStates_.end());

    rectObs_->randomize(0.50 * meanObsWidth_, 1.5 * meanObsWidth_, obsRatio, tVec);
  }

  // Finally specify the optimization target:
  BaseExperiment::opt_->setCostThreshold(BaseExperiment::getMinimum());
}

bool RandomRectanglesExperiment::knowsOptimum() const {
  return false;
}

ompl::base::Cost RandomRectanglesExperiment::getOptimum() const {
  throw ompl::Exception("The global optimum is unknown", BaseExperiment::name_);
}

void RandomRectanglesExperiment::setTarget(double targetSpecifier) {
  BaseExperiment::opt_->setCostThreshold(ompl::base::Cost(targetSpecifier));
}

std::string RandomRectanglesExperiment::lineInfo() const {
  std::stringstream rval;

  rval << " #Obs: " << rectObs_->getObstacles().size() << ". Widths ~ [" << 0.5 * meanObsWidth_
       << ", " << 1.5 * meanObsWidth_ << "].";

  return rval.str();
}

std::string RandomRectanglesExperiment::paraInfo() const {
  return std::string();
}
