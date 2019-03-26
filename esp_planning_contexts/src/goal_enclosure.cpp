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

#include "esp_planning_contexts/goal_enclosure.h"

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

GoalEnclosure::GoalEnclosure(const unsigned int dim, const double worldHalfWidth,
                             const double insideWidth, const double wallThickness,
                             const double gapWidth, const double runSeconds,
                             const double checkResolution) :
    BaseContext(dim,
                std::vector<std::pair<double, double>>(
                    dim, std::pair<double, double>(-worldHalfWidth, worldHalfWidth)),
                runSeconds, "GoalEnc"),
    insideWidth_(insideWidth),
    wallThickness_(wallThickness),
    gapWidth_(gapWidth) {
  // Variables
  // The state space
  std::shared_ptr<ompl::base::RealVectorStateSpace> ss;
  // Temporary variables for the obstacle and antiobstacle
  std::shared_ptr<Hyperrectangle> obs;
  std::shared_ptr<Hyperrectangle> anti;

  if (gapWidth_ <= 0 || insideWidth_ <= 0 || wallThickness_ <= 0) {
    throw ompl::Exception("Widths must be > 0.");
  }
  if (gapWidth_ > insideWidth_ && gapWidth_ != (insideWidth_ + 2.0 * wallThickness_)) {
    throw ompl::Exception(
        "Gap can only be wider than the interior width if it is as wide as the entire obstacle.");
  }
  if (gapWidth_ > (insideWidth_ + 2.0 * wallThickness_)) {
    throw ompl::Exception("Gap is wider than the enclosure.");
  }

  // The problem bounds
  ompl::base::RealVectorBounds problemBounds(BaseContext::dim_);

  // Make the state space Rn:
  ss = std::make_shared<ompl::base::RealVectorStateSpace>(BaseContext::dim_);

  // Create the space information class:
  BaseContext::si_ = std::make_shared<ompl::base::SpaceInformation>(ss);

  // Allocate the obstacle world
  enclObs_ = std::make_shared<CutoutObstacles>(BaseContext::si_);
  BaseContext::obs_ = enclObs_;

  // Set the problem bounds:
  problemBounds.setLow(BaseContext::limits_.at(0u).first);
  problemBounds.setHigh(BaseContext::limits_.at(0u).second);

  // Store the problem bounds:
  ss->setBounds(problemBounds);

  // Set the validity checker and checking resolution
  BaseContext::si_->setStateValidityChecker(
      static_cast<ompl::base::StateValidityCheckerPtr>(enclObs_));
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

  //*****Obstacle*****//
  goalEnclCorners_.push_back(std::make_shared<ompl::base::ScopedState<>>(ss));

  // All widths are the same
  goalEnclWidths_.push_back(
      std::vector<double>(BaseContext::dim_, insideWidth_ + 2 * wallThickness_));

  // X-position is unique
  (*goalEnclCorners_.back())[0u] = goalPos_ - 0.5 * goalEnclWidths_.back().at(0u);

  // All the other dimensions are the same
  for (unsigned int i = 1u; i < BaseContext::dim_; ++i) {
    (*goalEnclCorners_.back())[i] = 0.0 - 0.5 * goalEnclWidths_.back().at(i);
  }

  // Add the obstacle.
  obs->addObstacle(std::make_pair(goalEnclCorners_.back()->get(), goalEnclWidths_.back()));

  //*****Internal Space*****//
  goalEnclCorners_.push_back(std::make_shared<ompl::base::ScopedState<>>(ss));

  // All widths are the same
  goalEnclWidths_.push_back(std::vector<double>(BaseContext::dim_, insideWidth_));

  // X-position is unique
  (*goalEnclCorners_.back())[0u] = goalPos_ - 0.5 * goalEnclWidths_.back().at(0u);

  // All the other dimensions are the same
  for (unsigned int i = 1u; i < BaseContext::dim_; ++i) {
    (*goalEnclCorners_.back())[i] = 0.0 - 0.5 * goalEnclWidths_.back().at(i);
  }

  // Add to the anti obstacle.
  anti->addObstacle(std::make_pair(goalEnclCorners_.back()->get(), goalEnclWidths_.back()));

  //*****Right opening*****//
  startEnclCorners_.push_back(std::make_shared<ompl::base::ScopedState<>>(ss));
  startEnclWidths_.push_back(std::vector<double>());

  // All the other dimensions are the same
  for (unsigned int i = 0u; i < BaseContext::dim_; ++i) {
    // X is unique (it's the outside of the wall on the left)
    if (i == 0u) {
      (*startEnclCorners_.back())[i] = goalPos_ + (0.5 * insideWidth_);
      startEnclWidths_.back().push_back(wallThickness_);
    } else {
      (*startEnclCorners_.back())[i] = 0.0 - 0.5 * gapWidth_;
      startEnclWidths_.back().push_back(gapWidth_);
    }
  }

  // Add to the anti obstacle.
  anti->addObstacle(std::make_pair(startEnclCorners_.back()->get(), startEnclWidths_.back()));

  // Add the obstacles and antiobstacles into the CutoutObstacles class
  enclObs_->addObstacle(obs);
  enclObs_->addAntiObstacle(anti);

  // Finally specify the optimization target as the minimum:
  BaseContext::opt_->setCostThreshold(BaseContext::getMinimum());
}

bool GoalEnclosure::knowsOptimum() const {
  return true;
}

ompl::base::Cost GoalEnclosure::getOptimum() const {
  /*
  The optimal solution has 5 components and is constrained to the xy plane:
    - diagonally from the start to the corner of the opening
    - vertically to the far obstacle corner
    - horizontal across to the other obstacle's far corner
    - vertically to the corner of the opening
    - diagonally from the corner of the opening to the goal
  but the 1 & 5 and 2 & 3 are the same (as we're symmetric)
  */

  // Variables
  ompl::base::Cost startToOpening;
  ompl::base::Cost openingToCorner;
  ompl::base::Cost cornerToCorner;

  // Calculate the distances They depend on whether the gap is wider than the interior or not
  if (gapWidth_ <= insideWidth_) {
    // It is not, we must go to the top-left corner of the passage
    startToOpening = ompl::base::Cost(std::sqrt(std::pow(0.5 * insideWidth_ + wallThickness_, 2.0) +
                                                std::pow(0.5 * gapWidth_, 2.0)));

    // and then to the top-left corner of the obstacle
    openingToCorner = ompl::base::Cost(0.5 * (insideWidth_ + 2 * wallThickness_ - gapWidth_));

    // and then across to the other
    cornerToCorner =
        ompl::base::Cost((goalPos_ - startPos_) + 2 * (0.5 * insideWidth_ + wallThickness_));
  } else {
    // It is, it is therefore restricted to being the whole obstacle width, we go to the top-left
    // corner of the interior
    startToOpening = ompl::base::Cost(
        std::sqrt(std::pow(0.5 * insideWidth_, 2.0) + std::pow(0.5 * insideWidth_, 2.0)));

    // and then to the top of the passage
    openingToCorner = ompl::base::Cost(wallThickness_);

    // and then across to the other
    cornerToCorner = ompl::base::Cost((goalPos_ - startPos_) + 2 * (0.5 * insideWidth_));
  }

  // Combine and return:
  return BaseContext::opt_->combineCosts(
      startToOpening,
      BaseContext::opt_->combineCosts(
          openingToCorner,
          BaseContext::opt_->combineCosts(
              cornerToCorner, BaseContext::opt_->combineCosts(openingToCorner, startToOpening))));
}

void GoalEnclosure::setTarget(double targetSpecifier) {
  BaseContext::opt_->setCostThreshold(ompl::base::Cost(targetSpecifier));
}

std::string GoalEnclosure::lineInfo() const {
  return std::string();
}

std::string GoalEnclosure::paraInfo() const {
  std::stringstream rval;

  rval << "Start obstacle "
       << this->printRectangle(startEnclCorners_.at(0u), startEnclWidths_.at(0u));
  rval << "Start cavity "
       << this->printRectangle(startEnclCorners_.at(1u), startEnclWidths_.at(1u));
  rval << "Start passage "
       << this->printRectangle(startEnclCorners_.at(2u), startEnclWidths_.at(2u));

  rval << "Goal obstacle " << this->printRectangle(goalEnclCorners_.at(0u), goalEnclWidths_.at(0u));
  rval << "Goal cavity " << this->printRectangle(goalEnclCorners_.at(1u), goalEnclWidths_.at(1u));
  rval << "Goal passage " << this->printRectangle(goalEnclCorners_.at(2u), goalEnclWidths_.at(2u));

  return rval.str();
}

std::string GoalEnclosure::printRectangle(std::shared_ptr<ompl::base::ScopedState<>> llCorner,
                                          std::vector<double> widths) const {
  std::stringstream rval;

  rval << "[";
  for (unsigned int i = 0u; i < BaseContext::dim_; ++i) {
    rval << (*llCorner)[i];
    if (i != BaseContext::dim_ - 1u) {
      rval << ", ";
    }
  }
  rval << "], [";
  for (unsigned int i = 0u; i < BaseContext::dim_; ++i) {
    rval << (*llCorner)[i] + widths.at(i);
    if (i != BaseContext::dim_ - 1u) {
      rval << ", ";
    }
  }
  rval << "]" << std::endl;

  return rval.str();
}

void GoalEnclosure::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

}  // namespace ompltools

}  // namespace esp
