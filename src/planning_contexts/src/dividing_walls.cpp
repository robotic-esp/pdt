/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014--2022
 *  Estimation, Search, and Planning (ESP) Research Group
 *  All rights reserved
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
 *   * Neither the names of the organizations nor the names of its
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

#include "pdt/planning_contexts/dividing_walls.h"

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>

#include "pdt/obstacles/base_obstacle.h"
#include "pdt/obstacles/hyperrectangle.h"
#include "pdt/planning_contexts/context_validity_checker.h"

namespace pdt {

namespace planning_contexts {

DividingWalls::DividingWalls(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                             const std::shared_ptr<const config::Configuration>& config,
                             const std::string& name) :
    RealVectorGeometricContext(spaceInfo, config, name),
    numWalls_(config->get<std::size_t>("context/" + name + "/numWalls")),
    wallThicknesses_(config->get<std::vector<double>>("context/" + name + "/wallThicknesses")),
    numGaps_(config->get<std::size_t>("context/" + name + "/numGaps")),
    gapWidths_(config->get<std::vector<double>>("context/" + name + "/gapWidths")) {
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

  startGoalPairs_ = makeStartGoalPair();
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
    obstacles_.push_back(std::make_shared<obstacles::Hyperrectangle<obstacles::BaseObstacle>>(
        spaceInfo_, midpoint, widths));
  }
}

void DividingWalls::createAntiObstacles() {
  // Create the gaps in the walls.
  for (std::size_t i = 0; i < numGaps_; ++i) {
    // Create an obstacle midpoint for this gap.
    ompl::base::ScopedState<> midpoint(spaceInfo_);

    // Set the obstacle midpoint in the second dimension.
    midpoint[1u] = static_cast<double>((i + 1u)) * (bounds_.high.at(1u) - bounds_.low.at(1u)) /
                       static_cast<double>((numGaps_ + 1u)) +
                   bounds_.low.at(1);

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
    antiObstacles_.push_back(
        std::make_shared<obstacles::Hyperrectangle<obstacles::BaseAntiObstacle>>(spaceInfo_,
                                                                                 midpoint, widths));
  }
}

}  // namespace planning_contexts

}  // namespace pdt
