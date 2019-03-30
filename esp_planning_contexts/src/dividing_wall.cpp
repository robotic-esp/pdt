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

#include "esp_planning_contexts/dividing_wall.h"

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

DividingWall::DividingWall(const unsigned int dim, const double wallThickness,
                           const unsigned int numGaps, const double gapWidth,
                           const double runSeconds, const double checkResolution) :
    BaseContext(dim,
                std::vector<std::pair<double, double>>(dim, std::pair<double, double>(-1.0, 1.0)),
                runSeconds, "DividingWall") {
  common_constructor(std::vector<double>(1u, wallThickness), std::vector<unsigned int>(1u, numGaps),
                     std::vector<double>(1u, gapWidth), std::vector<double>(), checkResolution);
}

DividingWall::DividingWall(const unsigned int dim, const std::vector<double> wallThicknesses,
                           const std::vector<unsigned int> numGaps,
                           const std::vector<double> gapWidths,
                           const std::vector<double> wallSpacings, const double runSeconds,
                           const double checkResolution) :
    BaseContext(dim,
                std::vector<std::pair<double, double>>(dim, std::pair<double, double>(-1.0, 1.0)),
                runSeconds, "DividingWall") {
  common_constructor(wallThicknesses, numGaps, gapWidths, wallSpacings, checkResolution);
}

void DividingWall::common_constructor(const std::vector<double> wallThickness,
                                      const std::vector<unsigned int> numGaps,
                                      const std::vector<double> gapWidth,
                                      const std::vector<double> wallSpacings,
                                      const double checkResolution) {
  // Variable
  // The state space
  std::shared_ptr<ompl::base::RealVectorStateSpace> ss;
  // The problem bounds
  ompl::base::RealVectorBounds problemBounds(BaseContext::dim_);
  // Whether one wall has an odd number of wall segments.
  bool hasOddNumObs;
  // A token for the x-position of the wall's left corner
  double leftXPos;

  // Store things:
  wallThicknesses_ = wallThickness;
  numGaps_ = numGaps;
  gapWidths_ = gapWidth;
  wallSpacings_ = wallSpacings;
  startPos_ = -0.5;
  goalPos_ = 0.5;

  // Make the state space Rn:
  ss = std::make_shared<ompl::base::RealVectorStateSpace>(BaseContext::dim_);

  // Create the space information class:
  BaseContext::si_ = std::make_shared<ompl::base::SpaceInformation>(ss);

  // Allocate the obstacle world
  rectObs_ = std::make_shared<Hyperrectangles>(BaseContext::si_, false);

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

  // Calculate the obstacle parameters:

  // The number of walls:
  numWalls_ = wallThicknesses_.size();

  // Check that all the given vectors are sane:
  if (numGaps_.size() != numWalls_) {
    throw ompl::Exception("Number of gaps does not match number of walls.");
  } else if (gapWidths_.size() != numWalls_) {
    throw ompl::Exception("Gap widths does not match number of walls.");
  } else if (wallSpacings_.size() != numWalls_ - 1u) {
    throw ompl::Exception("Wall spacings is not one less than the number of walls.");
  }

  // Allocate the other vectors:
  numObs_ = std::vector<unsigned int>(numWalls_, 0u);
  wallWidths_ = std::vector<double>(numWalls_, 0.0);
  allObsWidths_ =
      std::vector<std::vector<double>>(numWalls_, std::vector<double>(BaseContext::dim_, 0.0));
  obsCorners_ = std::vector<std::vector<std::shared_ptr<ompl::base::ScopedState<>>>>(
      numWalls_, std::vector<std::shared_ptr<ompl::base::ScopedState<>>>());

  // Now, iterate through each wall. We have to do this multiple times to be cleaner. First
  // calculate parameters:
  hasOddNumObs = false;
  for (unsigned int w = 0u; w < numWalls_; ++w) {
    // Calculate the number of obs for this wall:
    numObs_.at(w) = numGaps_.at(w) + 1u;

    // Track whether we have a nontrivial problem (i.e., one even number of gaps:)
    hasOddNumObs = hasOddNumObs || (numObs_.at(w) % 2u == 1u);

    // And the width:
    wallWidths_.at(w) = (BaseContext::limits_.at(1u).second - BaseContext::limits_.at(1u).first -
                         static_cast<double>(numGaps_.at(w)) * gapWidths_.at(w)) /
                        static_cast<double>(numObs_.at(w));

    // Sanity check
    if (wallWidths_.at(w) <= 0.0) {
      throw ompl::Exception("There is no wall! Too many gaps and/or gaps that are too large.");
    }

    // Fill in the obstacle widths:
    allObsWidths_.at(w).at(0u) = wallThicknesses_.at(w);  // x width;
    allObsWidths_.at(w).at(1u) = wallWidths_.at(w);       // y width;
    for (unsigned int i = 2u; i < BaseContext::dim_; ++i) {
      allObsWidths_.at(w).at(i) =
          BaseContext::limits_.at(i).second - BaseContext::limits_.at(i).first;  // z width
    }
  }

  // Assert sanity
  if (hasOddNumObs == false) {
    throw ompl::Exception(
        "All the walls have an odd number of gaps! This will be a trivial problem.");
  }

  // Calculate the x-position of the first wall:
  // Find the centre between start and goal
  leftXPos = (goalPos_ + startPos_) / 2.0;
  for (unsigned int w = 0u; w < numWalls_; ++w) {
    // Update the x-position, which will be: centre - 0.5*(sum(wallthickness) + sum(wallspacings)):
    // Half this wall's thickness
    leftXPos = leftXPos - 0.5 * wallThicknesses_.at(w);

    // Half the spacing to the right of this wall
    if (w != wallSpacings_.size()) {
      leftXPos = leftXPos - 0.5 * wallSpacings_.at(w);
    }
  }

  // Assert sanity
  if (leftXPos <= startPos_) {
    throw ompl::Exception("Walls will start left of the start position");
  }

  // Now iterate over the walls to specify the obstacle locations:
  for (unsigned int w = 0u; w < numWalls_; ++w) {
    // Iterate through the number of obstacles
    for (unsigned int i = 0u; i < numObs_.at(w); ++i) {
      // Variable
      // The obstacle number as a double
      double obsNum = static_cast<double>(i);

      // Allocate the obstacle
      obsCorners_.at(w).push_back(std::make_shared<ompl::base::ScopedState<>>(ss));

      // Specify it's lower-left corner:
      (*(obsCorners_.at(w).back()))[0u] = leftXPos;  // x
      (*(obsCorners_.at(w).back()))[1u] =
          BaseContext::limits_.at(1u).first + obsNum * (wallWidths_.at(w) + gapWidths_.at(w));  // y
      for (unsigned int i = 2u; i < BaseContext::dim_; ++i) {
        (*(obsCorners_.at(w).back()))[i] = BaseContext::limits_.at(i).first;  // z
      }

      // Add the obstacle
      rectObs_->addObstacle(std::make_pair(obsCorners_.at(w).back()->get(), allObsWidths_.at(w)));
    }

    // Move the leftXPos token, first by the amount of the wall:
    leftXPos = leftXPos + wallThicknesses_.at(w);

    // And then, if relevant, by the amount of the gap to the right:
    if (w != wallSpacings_.size()) {
      leftXPos = leftXPos + wallSpacings_.at(w);
    }
  }

  // Finally specify the optimization target:
  BaseContext::opt_->setCostThreshold(BaseContext::getMinimum());
}

bool DividingWall::knowsOptimum() const {
  return false;
}

ompl::base::Cost DividingWall::getOptimum() const {
  throw ompl::Exception("The global optimum is unknown, though it could be", BaseContext::name_);
}

void DividingWall::setTarget(double targetSpecifier) {
  BaseContext::opt_->setCostThreshold(ompl::base::Cost(targetSpecifier));
}

std::string DividingWall::lineInfo() const {
  std::stringstream rval;

  for (unsigned w = 0u; w < numWalls_; ++w) {
    rval << " gap_width/obs_width: " << gapWidths_.at(w) << "/" << wallWidths_.at(w) << " = "
         << gapWidths_.at(w) / wallWidths_.at(w);
    if (w != wallSpacings_.size()) {
      rval << " ( then" << wallSpacings_.at(w) << ")";
    }
  }
  rval << ".";

  return rval.str();
}

std::string DividingWall::paraInfo() const {
  std::stringstream rval;

  rval << "obstacles = " << std::endl;
  for (unsigned int w = 0u; w < numWalls_; ++w) {
    for (unsigned int i = 0u; i < numObs_.at(w); ++i) {
      rval << w << "." << i << ": [";

      for (unsigned int j = 0u; j < BaseContext::dim_; ++j) {
        rval << (*(obsCorners_.at(w).at(i)))[j];
        if (j != BaseContext::dim_ - 1u) {
          rval << ", ";
        }
      }
      rval << "], [";
      for (unsigned int j = 0u; j < BaseContext::dim_; ++j) {
        rval << (*(obsCorners_.at(w).at(i)))[j] + allObsWidths_.at(w).at(j);
        if (j != BaseContext::dim_ - 1u) {
          rval << ", ";
        }
      }
      rval << "]" << std::endl;
    }
  }

  return rval.str();
}

void DividingWall::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

}  // namespace ompltools

}  // namespace esp
