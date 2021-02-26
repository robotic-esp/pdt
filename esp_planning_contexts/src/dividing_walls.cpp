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

#include "esp_planning_contexts/dividing_walls.h"

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>

#include "esp_obstacles/base_obstacle.h"
#include "esp_obstacles/hyperrectangle.h"
#include "esp_planning_contexts/context_validity_checker.h"

namespace esp {

namespace ompltools {

DividingWalls::DividingWalls(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                             const std::shared_ptr<const Configuration>& config,
                             const std::string& name) :
    RealVectorGeometricContext(spaceInfo, config, name),
    dimensionality_(spaceInfo->getStateDimension()),
    numWalls_(config->get<std::size_t>("context/" + name + "/numWalls")),
    wallThicknesses_(config->get<std::vector<double>>("context/" + name + "/wallThicknesses")),
    numGaps_(config->get<std::size_t>("context/" + name + "/numGaps")),
    gapWidths_(config->get<std::vector<double>>("context/" + name + "/gapWidths")),
    startState_(spaceInfo) {
  // Get the start and goal positions.
  auto startPosition = config->get<std::vector<double>>("context/" + name + "/start");

  // Assert configuration sanity.
  if (startPosition.size() != dimensionality_) {
    OMPL_ERROR("%s: Dimensionality of problem and of start specification does not match.",
               name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (numWalls_ != wallThicknesses_.size()) {
    OMPL_ERROR("%s: Number of walls number of wall thicknesses do not match.", name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (numGaps_ != gapWidths_.size()) {
    OMPL_ERROR("%s: Number of gaps number of gap widths do not match.", name.c_str());
    throw std::runtime_error("Context error.");
  }

  // Create the validity checker.
  auto validityChecker = std::make_shared<ContextValidityChecker>(spaceInfo_);

  // Create the obstacles and add them to the validity checker.
  createObstacles();
  validityChecker->addObstacles(obstacles_);

  // Create the anti obstacles and add them to the validity checker.
  createAntiObstacles();
  validityChecker->addAntiObstacles(antiObstacles_);

  // Set the validity checker and the check resolution.
  spaceInfo_->setStateValidityChecker(validityChecker);
  spaceInfo_->setStateValidityCheckingResolution(
      config->get<double>("context/" + name + "/collisionCheckResolution"));

  // Set up the space info.
  spaceInfo_->setup();

  // Fill the start and goal states' coordinates.
  for (auto i = 0u; i < spaceInfo_->getStateDimension(); ++i) {
    startState_[i] = startPosition.at(i);
  }
}

ompl::base::ProblemDefinitionPtr DividingWalls::instantiateNewProblemDefinition() const {
  // Instantiate a new problem definition.
  auto problemDefinition = std::make_shared<ompl::base::ProblemDefinition>(spaceInfo_);

  // Set the objective.
  problemDefinition->setOptimizationObjective(objective_);

  // Set the start state in the problem definition.
  problemDefinition->addStartState(startState_);

  // Set the goal for the problem definition.
  problemDefinition->setGoal(createGoal());

  // Return the new definition.
  return problemDefinition;
}

ompl::base::ScopedState<ompl::base::RealVectorStateSpace> DividingWalls::getStartState() const {
  return startState_;
}

void DividingWalls::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

void DividingWalls::createObstacles() {
  // Create the walls.
  for (auto i = 0u; i < numWalls_; ++i) {
    // Create an obstacle midpoint for this wall.
    ompl::base::ScopedState<> midpoint(spaceInfo_);

    // Set the obstacle midpoint in the first dimension.
    midpoint[0u] = ((i + 1u) * (bounds_.high.at(0u) - bounds_.low.at(0u)) /
                    static_cast<double>((numWalls_ + 1u))) +
                   bounds_.low.at(0u);
    // Set the obstacle midpoint in the remaining dimension.
    for (auto j = 1u; j < dimensionality_; ++j) {
      midpoint[j] = (bounds_.low.at(j) + bounds_.high.at(j)) / 2.0;
    }
    // Create the widths of this wall.
    std::vector<double> widths(dimensionality_, 0.0);
    // Set the obstacle width in the first dimension.
    widths.at(0) = wallThicknesses_.at(i);
    // The wall spans all other dimensions.
    for (std::size_t j = 1; j < dimensionality_; ++j) {
      widths.at(j) = bounds_.high.at(j) - bounds_.low.at(j);
    }

    // Add this wall to the obstacles.
    obstacles_.emplace_back(
        std::make_shared<Hyperrectangle<BaseObstacle>>(spaceInfo_, midpoint, widths));
  }
}

void DividingWalls::createAntiObstacles() {
  // Create the gaps in the walls.
  for (std::size_t i = 0; i < numGaps_; ++i) {
    // Create an obstacle midpoint for this gap.
    ompl::base::ScopedState<> midpoint(spaceInfo_);

    // Set the obstacle midpoint in the second dimension.
    midpoint[1u] =
      static_cast<double>((i + 1u)) *
      (bounds_.high.at(1u) - bounds_.low.at(1u)) /
      static_cast<double>((numGaps_ + 1u)) + bounds_.low.at(1);

    // Set the obstacle midpoint in the remaining dimension.
    for (auto j = 0u; j < dimensionality_; ++j) {
      if (j != 1u) {
        midpoint[j] = (bounds_.low.at(j) + bounds_.high.at(j)) / 2.0;
      }
    }

    // Create the widths of this wall.
    std::vector<double> widths(dimensionality_, 0.0);

    // Set the obstacle width in the first dimension.
    widths.at(1) = gapWidths_.at(i);

    // The wall spans all other dimensions.
    for (std::size_t j = 0; j < dimensionality_; ++j) {
      if (j != 1) {
        widths.at(j) = (bounds_.high.at(j) - bounds_.low.at(j));
      }
    }

    // Add this gap to the anti obstacles.
    antiObstacles_.emplace_back(
        std::make_shared<Hyperrectangle<BaseAntiObstacle>>(spaceInfo_, midpoint, widths));
  }
}

}  // namespace ompltools

}  // namespace esp
