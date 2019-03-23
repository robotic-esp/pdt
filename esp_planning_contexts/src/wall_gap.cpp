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

#include "esp_planning_contexts/wall_gap.h"

#include <cmath>
#include <functional>
#include <memory>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "esp_obstacles/hyperrectangle.h"

namespace esp {

namespace ompltools {

WallGap::WallGap(const unsigned int dim, const bool onlyFindGap, const double gapWidth,
                 const double gapOffset, const double flankWidth, const double runSeconds,
                 const double checkResolution) :
    BaseContext(dim,
                std::vector<std::pair<double, double>>(dim, std::pair<double, double>(-1.0, 1.0)),
                runSeconds, "WallGap"),
    stopClassSwitch_(onlyFindGap),
    gapWidth_(gapWidth),
    gapOffset_(gapOffset),
    flankWidth_(flankWidth) {
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
  std::shared_ptr<Hyperrectangle> obs;
  std::shared_ptr<Hyperrectangle> anti;

  rectObs_ = std::make_shared<CutoutObstacles>(BaseContext::si_);
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

  // Allocate the temporary variable for the obstacles, antiobstacles, and the lower-left corners
  obs = std::make_shared<Hyperrectangle>(BaseContext::si_, false);
  anti = std::make_shared<Hyperrectangle>(BaseContext::si_, false);

  // Allocate the obstacles lower-left corners:
  obstacleLowerLeftCorner_ = std::make_shared<ompl::base::ScopedState<>>(ss);

  // Specify the obstacle
  // Position
  (*obstacleLowerLeftCorner_)[0u] = (goalPos_ + startPos_) / 2.0 - 0.5 * obsThickness_;  // x
  (*obstacleLowerLeftCorner_)[1u] = BaseContext::limits_.at(1u).first;                   // y
  for (unsigned int i = 2u; i < BaseContext::dim_; ++i) {
    (*obstacleLowerLeftCorner_)[i] = BaseContext::limits_.at(i).first;  // z
  }

  // Obstacle widths
  obstacleWidths_ = std::vector<double>(BaseContext::dim_, 0.0);
  obstacleWidths_.at(0u) = obsThickness_;  // x width
  obstacleWidths_.at(1u) =
      (BaseContext::limits_.at(1u).second - BaseContext::limits_.at(1u).first) -
      flankWidth_;  // y width
  for (unsigned int i = 2u; i < BaseContext::dim_; ++i) {
    obstacleWidths_.at(i) = BaseContext::limits_.at(i).second - BaseContext::limits_.at(i).first;
  }

  obs->addObstacle(std::make_pair(obstacleLowerLeftCorner_->get(), obstacleWidths_));

  // Gap lower left corner.
  gapLowerLeftCorner_ = std::make_shared<ompl::base::ScopedState<>>(ss);

  // Specify the gap (as anti obstacle)
  (*gapLowerLeftCorner_)[0u] = (goalPos_ + startPos_) / 2.0 - 0.5 * obsThickness_;  // x
  (*gapLowerLeftCorner_)[1u] = gapOffset_ + gapWidth_;
  for (unsigned int i = 2u; i < BaseContext::dim_; ++i) {
    (*gapLowerLeftCorner_)[i] = BaseContext::limits_.at(i).first;  // z
  }

  // Gap widths
  gapWidths_ = std::vector<double>(BaseContext::dim_, 0.0);
  gapWidths_.at(0u) = obsThickness_;  // x width
  gapWidths_.at(1u) = gapWidth_;      // y width
  for (unsigned int i = 2u; i < BaseContext::dim_; ++i) {
    gapWidths_.at(i) = BaseContext::limits_.at(i).second - BaseContext::limits_.at(i).first;
  }

  anti->addObstacle(std::make_pair(gapLowerLeftCorner_->get(), gapWidths_));

  rectObs_->addObstacle(obs);
  rectObs_->addAntiObstacle(anti);

  //     //Finally specify the optimization target:
  //     if (stopClassSwitch_ == true)
  //     {
  //         //Stop if we find a cost better than the max non flanking cost
  //         BaseExperiment::opt_->setCostThreshold(this->minFlankingCost());
  //     }
  //     else
  //     {
  //         //The optimum:
  //         BaseExperiment::opt_->setCostThreshold(this->getOptimum());
  //     }

  //     //Make sure this is a sane problem:
  //     if (this->minFlankingCost().value() < this->maxGapCost().value())
  //     {
  //         throw ompl::Exception("For the given gap width, a path through the gap can be worse
  //         than a flanking path.");
  //     }
  //     //No else
}

bool WallGap::knowsOptimum() const {
  return false;
}

ompl::base::Cost WallGap::getOptimum() const {
  // Variables
  ompl::base::Cost startToCorner;
  ompl::base::Cost obsEdge;
  ompl::base::Cost otherCornerToGoal;

  // The optimum goes exactly under *upperObs_:
  startToCorner = ompl::base::Cost(
      std::sqrt(std::pow((*gapLowerLeftCorner_)[0u] - BaseContext::startStates_.front()[0u], 2.0) +
                std::pow((*gapLowerLeftCorner_)[1u] - BaseContext::startStates_.front()[1u], 2.0)));
  obsEdge = ompl::base::Cost(gapWidths_.at(0u));
  otherCornerToGoal = ompl::base::Cost(
      std::sqrt(std::pow(BaseContext::goalStates_.front()[0u] -
                             ((*gapLowerLeftCorner_)[0u] + gapWidths_.at(0u)),
                         2.0) +
                std::pow(BaseContext::goalStates_.front()[1u] - (*gapLowerLeftCorner_)[1u], 2.0)));

  // Combine and return:
  return BaseContext::opt_->combineCosts(BaseContext::opt_->combineCosts(startToCorner, obsEdge),
                                         otherCornerToGoal);
}

ompl::base::Cost WallGap::minFlankingCost() const {
  // Variables
  ompl::base::Cost startToCorner;
  ompl::base::Cost obsEdge;
  ompl::base::Cost otherCornerToGoal;

  // The minimum cost not through the gap goes exactly over *upperObs_:
  startToCorner = ompl::base::Cost(
      std::sqrt(std::pow((*gapLowerLeftCorner_)[0u] - BaseContext::startStates_.front()[0u], 2.0) +
                std::pow(((*gapLowerLeftCorner_)[1u] + gapWidths_.at(1u)) -
                             BaseContext::startStates_.front()[1u],
                         2.0)));
  obsEdge = ompl::base::Cost(gapWidths_.at(0u));
  otherCornerToGoal =
      ompl::base::Cost(std::sqrt(std::pow(BaseContext::goalStates_.front()[0u] -
                                              ((*gapLowerLeftCorner_)[0u] + gapWidths_.at(0u)),
                                          2.0) +
                                 std::pow(BaseContext::goalStates_.front()[1u] -
                                              ((*gapLowerLeftCorner_)[1u] + gapWidths_.at(1u)),
                                          2.0)));

  // Combine and return:
  return BaseContext::opt_->combineCosts(BaseContext::opt_->combineCosts(startToCorner, obsEdge),
                                         otherCornerToGoal);
}

ompl::base::Cost WallGap::maxGapCost() const {
  // Variables
  ompl::base::Cost startToCorner;
  ompl::base::Cost obsEdge;
  ompl::base::Cost otherCornerToGoal;

  // The worst-case straight-line-path through the gap goes exactly over *lowerObs_:
  startToCorner = ompl::base::Cost(std::sqrt(
      std::pow((*obstacleLowerLeftCorner_)[0u] - BaseContext::startStates_.front()[0u], 2.0) +
      std::pow(((*obstacleLowerLeftCorner_)[1u] + obstacleWidths_.at(1u)) -
                   BaseContext::startStates_.front()[1u],
               2.0)));
  obsEdge = ompl::base::Cost(obstacleWidths_.at(0u));
  otherCornerToGoal = ompl::base::Cost(
      std::sqrt(std::pow(BaseContext::goalStates_.front()[0u] -
                             ((*obstacleLowerLeftCorner_)[0u] + obstacleWidths_.at(0u)),
                         2.0) +
                std::pow(BaseContext::goalStates_.front()[1u] -
                             ((*obstacleLowerLeftCorner_)[1u] + obstacleWidths_.at(1u)),
                         2.0)));

  // Combine and return:
  return BaseContext::opt_->combineCosts(BaseContext::opt_->combineCosts(startToCorner, obsEdge),
                                         otherCornerToGoal);
}

void WallGap::setTarget(double targetSpecifier) {
  if (stopClassSwitch_ == true) {
    throw ompl::Exception("Cannot set target when searching for class switch");
  }
  // No else

  BaseContext::opt_->setCostThreshold(
      ompl::base::Cost(targetSpecifier * this->getOptimum().value()));
}

std::string WallGap::lineInfo() const {
  std::stringstream rval;

  rval << "(min flanking: " << this->minFlankingCost() << ", max gap: " << this->maxGapCost()
       << ".";

  return rval.str();
}

std::string WallGap::paraInfo() const {
  std::stringstream rval;

  rval << "bottom obstacle: [";
  for (unsigned int i = 0u; i < BaseContext::dim_; ++i) {
    rval << (*obstacleLowerLeftCorner_)[i];
    if (i != BaseContext::dim_ - 1u) {
      rval << ", ";
    }
  }
  rval << "], [";
  for (unsigned int i = 0u; i < BaseContext::dim_; ++i) {
    rval << (*obstacleLowerLeftCorner_)[i] + obstacleWidths_.at(i);
    if (i != BaseContext::dim_ - 1u) {
      rval << ", ";
    }
  }
  rval << "]" << std::endl;
  rval << "upper obstacle: [";
  for (unsigned int i = 0u; i < BaseContext::dim_; ++i) {
    rval << (*gapLowerLeftCorner_)[i];
    if (i != BaseContext::dim_ - 1u) {
      rval << ", ";
    }
  }
  rval << "], [";
  for (unsigned int i = 0u; i < BaseContext::dim_; ++i) {
    rval << (*gapLowerLeftCorner_)[i] + gapWidths_.at(i);
    if (i != BaseContext::dim_ - 1u) {
      rval << ", ";
    }
  }
  rval << "]" << std::endl;

  return rval.str();
}

}  // namespace ompltools

}  // namespace esp