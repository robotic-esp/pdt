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

#include "esp_planning_contexts/multi_start_goal.h"

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

MultiStartGoal::MultiStartGoal(const unsigned int dim, const unsigned int numObs,
                               const double obsRatio, const double runSeconds,
                               const double checkResolution) :
    BaseContext(dim,
                std::vector<std::pair<double, double>>(dim, std::pair<double, double>(-1.0, 1.0)),
                runSeconds, "MultiStartGoal") {
  // Variable
  // The state space
  std::shared_ptr<ompl::base::RealVectorStateSpace> ss;
  // The problem bounds
  ompl::base::RealVectorBounds problemBounds(BaseContext::dim_);
  // The mean width of the obstacles:
  double meanObsWidth;
  // The measure of obstacles
  double obsMeasure;

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

  // Set the starts and goals (http://www.mathopenref.com/coordpolycalc.html):
  // Create my starts:
  // Create each one
  for (unsigned int i = 0u; i < 2u; ++i) {
    // Create a start state on the vector:
    BaseContext::startStates_.push_back(ompl::base::ScopedState<>(ss));

    if (i == 0u) {
      BaseContext::startStates_.back()[0u] = -0.29;
      BaseContext::startStates_.back()[1u] = -0.40;
      for (unsigned int j = 2u; j < BaseContext::dim_; ++j) {
        BaseContext::startStates_.back()[j] = 0.0;
      }
    } else if (i == 1u) {
      BaseContext::startStates_.back()[0u] = 0.29;
      BaseContext::startStates_.back()[1u] = -0.40;
      for (unsigned int j = 2u; j < BaseContext::dim_; ++j) {
        BaseContext::startStates_.back()[j] = 0.0;
      }
    } else {
      throw ompl::Exception("Incorrect number of starts");
    }
  }

  // Create my goals:
  // Allocate the goal pointer:
  BaseContext::goalPtr_ = std::make_shared<ompl::base::GoalStates>(BaseContext::si_);

  // Create each one
  for (unsigned int i = 0u; i < 3u; ++i) {
    // Create a goal state on the vector:
    BaseContext::goalStates_.push_back(ompl::base::ScopedState<>(ss));

    if (i == 0u) {
      BaseContext::goalStates_.back()[0u] = 0.0;
      BaseContext::goalStates_.back()[1u] = 0.5;
      for (unsigned int j = 2u; j < BaseContext::dim_; ++j) {
        BaseContext::goalStates_.back()[j] = 0.0;
      }
    } else if (i == 1u) {
      BaseContext::goalStates_.back()[0u] = -0.48;
      BaseContext::goalStates_.back()[1u] = 0.15;
      for (unsigned int j = 2u; j < BaseContext::dim_; ++j) {
        BaseContext::goalStates_.back()[j] = 0.0;
      }
    } else if (i == 2u) {
      BaseContext::goalStates_.back()[0u] = 0.48;
      BaseContext::goalStates_.back()[1u] = 0.15;
      for (unsigned int j = 2u; j < BaseContext::dim_; ++j) {
        BaseContext::goalStates_.back()[j] = 0.0;
      }
    } else if (i == 3u) {
      BaseContext::goalStates_.back()[0u] = 0.0;
      BaseContext::goalStates_.back()[1u] = 0.0;
      for (unsigned int j = 2u; j < BaseContext::dim_; ++j) {
        BaseContext::goalStates_.back()[j] = 0.0;
      }
    } else {
      throw ompl::Exception("Incorrect number of goals");
    }

    // Add
    BaseContext::goalPtr_->as<ompl::base::GoalStates>()->addState(BaseContext::goalStates_.back());
  }

  // Calculate the minimum and maximum radius of the obstacles:
  // First, calculate the desired obstacle volume of the problem:
  obsMeasure = obsRatio * BaseContext::si_->getSpaceMeasure();

  // Then, calculate the mean radius necessary to get the desired obstacle volume with the desired
  // number of obstacles:
  meanObsWidth = std::pow(obsMeasure / static_cast<double>(numObs), 1.0 / BaseContext::dim_);

  // Create a random set of obstacles
  if (obsRatio > 0.0) {
    // A temporary vector
    std::vector<ompl::base::ScopedState<>> tVec;

    // Copy into
    tVec.insert(tVec.end(), BaseContext::startStates_.begin(), BaseContext::startStates_.end());
    tVec.insert(tVec.end(), BaseContext::goalStates_.begin(), BaseContext::goalStates_.end());

    rectObs_->randomize(0.50 * meanObsWidth, 1.5 * meanObsWidth, obsRatio, tVec);
  }

  // Finally specify the optimization target:
  BaseContext::opt_->setCostThreshold(BaseContext::getMinimum());
}

bool MultiStartGoal::knowsOptimum() const {
  return false;
}

ompl::base::Cost MultiStartGoal::getOptimum() const {
  throw ompl::Exception("The global optimum is unknown", BaseContext::name_);
}

void MultiStartGoal::setTarget(double targetSpecifier) {
  BaseContext::opt_->setCostThreshold(ompl::base::Cost(targetSpecifier));
}

std::string MultiStartGoal::lineInfo() const {
  std::stringstream rval;

  rval << " #Obs: " << rectObs_->getObstacles().size() << ".";

  return rval.str();
}

std::string MultiStartGoal::paraInfo() const {
  return std::string();
}

}  // namespace ompltools

}  // namespace esp