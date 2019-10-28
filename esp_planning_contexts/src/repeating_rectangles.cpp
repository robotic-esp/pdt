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

// Authors: Jonathan Gammell, Marlin Strub

#include "esp_planning_contexts/repeating_rectangles.h"

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

RepeatingRectangles::RepeatingRectangles(const std::shared_ptr<const Configuration>& config,
                                         const std::string& name) :
    BaseObstacleContext(config, name),
    numObsPerDim_(config->get<std::size_t>("Contexts/" + name + "/numObstaclesPerDim")),
    obsWidth_(config->get<double>("Contexts/" + name + "/obstacleWidth")),
    startPos_(config->get<std::vector<double>>("Contexts/" + name + "/start")),
    goalPos_(config->get<std::vector<double>>("Contexts/" + name + "/goal")) {
  // Assert configuration sanity.
  if (startPos_.size() != dimensionality_) {
    OMPL_ERROR("%s: Dimensionality of problem and of start specification does not match.",
               name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (goalPos_.size() != dimensionality_) {
    OMPL_ERROR("%s: Dimensionality of problem and of goal specification does not match.",
               name.c_str());
    throw std::runtime_error("Context error.");
  }
  for (std::size_t i = 1u; i < dimensionality_; ++i) {
    if ((bounds_.at(i).second - bounds_.at(i).first) !=
        bounds_.at(0u).second - bounds_.at(0u).first) {
      OMPL_ERROR("%s: Repeating rectangles assumes a (hyper)square.");
      throw std::runtime_error("Context error.");
    }
  }
  // Create a state space and set the bounds.
  auto stateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(dimensionality_);
  stateSpace->setBounds(bounds_.at(0u).first, bounds_.at(0u).second);

  // Create the space information class:
  spaceInfo_ = std::make_shared<ompl::base::SpaceInformation>(stateSpace);

  // Create the obstacles.
  createObstacles();

  // Create the validity checker and add the obstacle.
  validityChecker_ = std::make_shared<ContextValidityCheckerGNAT>(spaceInfo_);
  validityChecker_->addObstacles(obstacles_);

  // Set the validity checker and the check resolution.
  spaceInfo_->setStateValidityChecker(
      static_cast<ompl::base::StateValidityCheckerPtr>(validityChecker_));
  spaceInfo_->setStateValidityCheckingResolution(
      config->get<double>("Contexts/" + name + "/collisionCheckResolution"));

  // Set up the space info.
  spaceInfo_->setup();

  // Allocate the optimization objective
  optimizationObjective_ =
      std::make_shared<ompl::base::PathLengthOptimizationObjective>(spaceInfo_);

  // Set the heuristic to the default:
  optimizationObjective_->setCostToGoHeuristic(
      std::bind(&ompl::base::goalRegionCostToGo, std::placeholders::_1, std::placeholders::_2));

  // Create a start state.
  addStartState(startPos_);

  // Create a goal state.
  addGoalState(goalPos_);
  goalPtr_ = std::make_shared<ompl::base::GoalState>(spaceInfo_);
  goalPtr_->as<ompl::base::GoalState>()->setState(goalStates_.back());

  // Specify the optimization target.
  optimizationObjective_->setCostThreshold(computeMinPossibleCost());
}

bool RepeatingRectangles::knowsOptimum() const {
  return false;
}

ompl::base::Cost RepeatingRectangles::computeOptimum() const {
  throw ompl::Exception("The global optimum is unknown.", BaseObstacleContext::name_);
}

void RepeatingRectangles::setTarget(double targetSpecifier) {
  optimizationObjective_->setCostThreshold(
      ompl::base::Cost(targetSpecifier * this->computeOptimum().value()));
}

std::string RepeatingRectangles::lineInfo() const {
  std::stringstream rval;

  return rval.str();
}

std::string RepeatingRectangles::paraInfo() const {
  return std::string();
}

void RepeatingRectangles::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

void RepeatingRectangles::createObstacles() {
  // Let's try to keep this general for any number of dimensions.

  // Generate evenly spaced coordinates.
  for (std::size_t i = 1u; i < dimensionality_; ++i) {
    if ((bounds_.at(i).second - bounds_.at(i).first) !=
        bounds_.at(0u).second - bounds_.at(0u).first) {
      OMPL_ERROR("%s: Repeating rectangles assumes a (hyper)square.");
      throw std::runtime_error("Context error.");
    }
  }
  std::vector<double> coordinates{};
  coordinates.reserve(numObsPerDim_ * dimensionality_);
  for (std::size_t i = 0u; i < numObsPerDim_; ++i) {
    coordinates.emplace_back(
        ((i + 1u) * (bounds_.at(0).second - bounds_.at(0).first) / (numObsPerDim_ + 1u)) +
        bounds_.at(0u).first);
  }
  // Let d be the dimensionality. We need all combinations of d elements from the above coordinates
  // with replacement. The easiest way I know how to do this is to add each element d times to the
  // collection of possible coordinates and then use a all permutations of a bitmask that chooses
  // elements from this collection.
  for (std::size_t i = 1u; i < dimensionality_; ++i) {
    std::copy_n(coordinates.begin(), numObsPerDim_, std::back_inserter(coordinates));
  }
  std::vector<std::uint8_t> bitmask(dimensionality_, 1u);
  bitmask.resize(numObsPerDim_ * dimensionality_, 0u);
  do {
    // Take the coordinates of the bitmask for the midpoint.
    ompl::base::ScopedState<> midpoint(spaceInfo_);
    std::size_t dim{0u};
    for (std::size_t i = 0u; i < bitmask.size(); ++i) {
      if (bitmask[i]) {
        midpoint[dim++] = coordinates[i];
      }
    }
    // The widths are a parameter, so just take this.
    std::vector<double> widths(dimensionality_, obsWidth_);

    // Add the obstacle.
    obstacles_.emplace_back(
        std::make_shared<Hyperrectangle<BaseObstacle>>(spaceInfo_, midpoint, widths));
  } while (std::prev_permutation(bitmask.begin(), bitmask.end()));
}

}  // namespace ompltools

}  // namespace esp
