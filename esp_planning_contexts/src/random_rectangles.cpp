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

#include "esp_planning_contexts/random_rectangles.h"

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

RandomRectangles::RandomRectangles(const unsigned int dim, const unsigned int numObs,
                                   const double obsRatio, const double runSeconds,
                                   const double checkResolution) :
    BaseContext(dim,
                std::vector<std::pair<double, double>>(dim, std::pair<double, double>(-1.0, 1.0)),
                runSeconds, "RandRect") {
  // Variable
  // The width of the sightline obstacle
  double sightLineWidth;
  // The measure of obstacles
  double obsMeasure;

  // Create a state space with appropriate bounds.
  auto stateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(dim_);
  stateSpace->setBounds(bounds_.at(0u).first, bounds_.at(0u).second);

  // Reset the space information to this new space.
  si_ = std::make_shared<ompl::base::SpaceInformation>(stateSpace);

  // Setup the nearest neighbor structure to hold obstacles.
  obstacles_ = std::make_shared<ompl::NearestNeighborsGNAT<Hyperrectangle>>();


  // Allocate the obstacle world
  rectObs_ = std::make_shared<Hyperrectangles>(si_, false);

  // Set the validity checker and checking resolution
  si_->setStateValidityChecker(
      static_cast<ompl::base::StateValidityCheckerPtr>(rectObs_));
  si_->setStateValidityCheckingResolution(checkResolution);

  // Call setup!
  si_->setup();

  // Allocate the optimization objective
  opt_ =
      std::make_shared<ompl::base::PathLengthOptimizationObjective>(si_);

  // Set the heuristic to the default:
  opt_->setCostToGoHeuristic(
      std::bind(&ompl::base::goalRegionCostToGo, std::placeholders::_1, std::placeholders::_2));

  // Create my start:
  // Create a start state on the vector:
  startStates_.push_back(ompl::base::ScopedState<>(stateSpace));

  // Assign to each component
  startStates_.back()[0u] = startPos_;
  for (unsigned int j = 1u; j < dim_; ++j) {
    startStates_.back()[j] = 0.0;
  }

  // Create my goal:
  // Create a goal state on the vector:
  goalStates_.push_back(ompl::base::ScopedState<>(stateSpace));

  // Assign to each component
  goalStates_.back()[0u] = goalPos_;
  for (unsigned int j = 1u; j < dim_; ++j) {
    goalStates_.back()[j] = 0.0;
  }

  // Allocate the goal:
  goalPtr_ = std::make_shared<ompl::base::GoalState>(si_);

  // Add
  goalPtr_->as<ompl::base::GoalState>()->setState(goalStates_.back());

  // Calculate the minimum and maximum radius of the obstacles:
  // First, calculate the desired obstacle volume of the problem:
  obsMeasure = obsRatio * si_->getSpaceMeasure();

  // Then, calculate the mean radius necessary to get the desired obstacle volume with the desired
  // number of obstacles:
  meanObsWidth_ = std::pow(obsMeasure / static_cast<double>(numObs), 1.0 / dim_);

  // And then the sightline width.  The min is here to make sure we don't swallow either the start
  // or goal
  sightLineWidth = std::min(getMinimum().value() / 7.5, meanObsWidth_);

  // Set the sight-line obstacle's lower-left corner:
  sightLineObs_ = std::make_shared<ompl::base::ScopedState<>>(stateSpace);
  for (unsigned int i = 0u; i < dim_; ++i) {
    (*sightLineObs_)[i] =
        (goalStates_.back()[i] + startStates_.back()[i]) / 2.0 -
        0.5 * sightLineWidth;
  }

  // Add the obstacle.:
  rectObs_->addObstacle(
      std::make_pair(sightLineObs_->get(), std::vector<double>(dim_, sightLineWidth)));

  // Create a random set of obstacles
  if (obsRatio > 0.0) {
    // A temporary vector
    std::vector<ompl::base::ScopedState<>> tVec;

    // Copy into
    tVec.insert(tVec.end(), startStates_.begin(), startStates_.end());
    tVec.insert(tVec.end(), goalStates_.begin(), goalStates_.end());

    rectObs_->randomize(0.50 * meanObsWidth_, 1.5 * meanObsWidth_, obsRatio, tVec);
  }

  // Finally specify the optimization target:
  opt_->setCostThreshold(getMinimum());
}

bool RandomRectangles::knowsOptimum() const {
  return false;
}

ompl::base::Cost RandomRectangles::getOptimum() const {
  throw ompl::Exception("The global optimum is unknown", name_);
}

void RandomRectangles::setTarget(double targetSpecifier) {
  opt_->setCostThreshold(ompl::base::Cost(targetSpecifier));
}

std::string RandomRectangles::lineInfo() const {
  std::stringstream rval;

  rval << " #Obs: " << rectObs_->getObstacles().size() << ". Widths ~ [" << 0.5 * meanObsWidth_
       << ", " << 1.5 * meanObsWidth_ << "].";

  return rval.str();
}

std::string RandomRectangles::paraInfo() const {
  return std::string();
}

void RandomRectangles::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

}  // namespace ompltools

}  // namespace esp
