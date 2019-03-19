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
#include "experiments/RRTsharpResponseExperiment.h"

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

namespace {

std::string ResponseName(unsigned int num) {
  if (num == 1u) {
    return "RRTsharpResponse1";
  } else if (num == 2u) {
    return "RRTsharpResponse2";
  } else {
    throw ompl::Exception("Unknown experiment number.");
  }
}

}  // namespace

RRTsharpResponseExperiment::RRTsharpResponseExperiment(const unsigned int expNum,
                                                       const double runSeconds,
                                                       const double checkResolution)
    : BaseExperiment(2,
                     std::vector<std::pair<double, double>>(2, std::pair<double, double>(0.0, 1.0)),
                     runSeconds, ResponseName(expNum)),
      expNum_(expNum) {
  // Variable
  // The state space
  std::shared_ptr<ompl::base::RealVectorStateSpace> ss;
  // The problem bounds
  ompl::base::RealVectorBounds problemBounds(BaseExperiment::dim_);

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
  BaseExperiment::startStates_.back()[0] = posX_;
  BaseExperiment::startStates_.back()[1] = startPosY_;

  // Create my goal:
  // Create a goal state on the vector:
  BaseExperiment::goalStates_.push_back(ompl::base::ScopedState<>(ss));

  // Assign to each component
  BaseExperiment::goalStates_.back()[0] = posX_;
  BaseExperiment::goalStates_.back()[1] = goalPosY_;

  // Allocate the goal:
  BaseExperiment::goalPtr_ = std::make_shared<ompl::base::GoalState>(BaseExperiment::si_);

  // Add
  BaseExperiment::goalPtr_->as<ompl::base::GoalState>()->setState(
      BaseExperiment::goalStates_.back());

  if (expNum_ == 1) {
    // Set the lower-left corner
    obs_.push_back(std::make_shared<ompl::base::ScopedState<>>(ss));
    (*obs_.back())[0] = 0.1;
    (*obs_.back())[1] = 0.47;

    // Set the size
    std::vector<double> widths;
    widths.push_back(0.8);
    widths.push_back(0.06);

    // Add the obstacle:
    rectObs_->addObstacle(std::make_pair(obs_.back()->get(), widths));
  } else if (expNum_ == 2) {
    // Variables
    std::vector<double> widths1;
    std::vector<double> widths2;
    std::vector<double> widths3;

    // Set the lower-left corner of the centre obstacle
    obs_.push_back(std::make_shared<ompl::base::ScopedState<>>(ss));
    (*obs_.back())[0] = 0.1;
    (*obs_.back())[1] = 0.47;

    // Set the size
    widths1.push_back(0.8);
    widths1.push_back(0.06);

    // Add the obstacle:
    rectObs_->addObstacle(std::make_pair(obs_.back()->get(), widths1));

    // Set the lower-left corner of the left obstacle: center: [0.18, 0.29], size: [0.16, 0.36]
    obs_.push_back(std::make_shared<ompl::base::ScopedState<>>(ss));
    (*obs_.back())[0] = 0.1;
    (*obs_.back())[1] = 0.11;

    // Set the size
    widths2.push_back(0.16);
    widths2.push_back(0.36);

    // Add the obstacle:
    rectObs_->addObstacle(std::make_pair(obs_.back()->get(), widths2));

    // Set the lower-left corner of the right obstacle: center: [0.82, 0.29], size: [0.16, 0.36]
    obs_.push_back(std::make_shared<ompl::base::ScopedState<>>(ss));
    (*obs_.back())[0] = 0.74;
    (*obs_.back())[1] = 0.11;

    // Set the size
    widths3.push_back(0.16);
    widths3.push_back(0.36);

    // Add the obstacle:
    rectObs_->addObstacle(std::make_pair(obs_.back()->get(), widths3));
  } else {
    throw ompl::Exception("Unknown experiment number.", BaseExperiment::name_);
  }

  // Finally specify the optimization target:
  BaseExperiment::opt_->setCostThreshold(this->getOptimum());
}

bool RRTsharpResponseExperiment::knowsOptimum() const {
  return true;
}

ompl::base::Cost RRTsharpResponseExperiment::getOptimum() const {
  double length;

  if (expNum_ == 1) {
    double centreXY = 0.5;
    double height = 0.06;
    double width = 0.8;
    length =
        std::sqrt(std::pow(goalPosY_ - (centreXY + height / 2.0), 2.0) +
                  std::pow(width / 2.0, 2.0)) +
        height +
        std::sqrt(std::pow(centreXY - height / 2.0 - startPosY_, 2.0) + std::pow(width / 2.0, 2.0));
  } else if (expNum_ == 2) {
    double centreXY1 = 0.5;
    double height1 = 0.06;
    double width1 = 0.8;
    double centreX2 = 0.18;
    double centreY2 = 0.29;
    double height2 = 0.36;
    double width2 = 0.16;
    length = std::sqrt(std::pow(centreXY1 - height1 / 2.0 - startPosY_, 2.0) +
                       std::pow(width1 / 2.0, 2.0)) +
             width2 + height2 + height1 +
             std::sqrt(std::pow(centreY2 - height2 / 2.0 - startPosY_, 2.0) +
                       std::pow(centreX2 + width2 / 2.0 - posX_, 2.0));
  } else {
    throw ompl::Exception("Unknown experiment number.", BaseExperiment::name_);
  }

  // Return:
  return ompl::base::Cost(length);
}

void RRTsharpResponseExperiment::setTarget(double targetSpecifier) {
  BaseExperiment::opt_->setCostThreshold(
      ompl::base::Cost(targetSpecifier * this->getOptimum().value()));
}

std::string RRTsharpResponseExperiment::lineInfo() const {
  std::stringstream rval;

  rval << "Experiment number: " << expNum_ << ".";

  return rval.str();
}

std::string RRTsharpResponseExperiment::paraInfo() const {
  return std::string();
}
