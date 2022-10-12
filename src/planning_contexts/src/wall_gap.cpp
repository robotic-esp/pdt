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

#include "pdt/planning_contexts/wall_gap.h"

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>

namespace pdt {

namespace planning_contexts {

WallGap::WallGap(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                 const std::shared_ptr<const config::Configuration>& config,
                 const std::string& name) :
    RealVectorGeometricContext(spaceInfo, config, name),
    wallWidth_(config->get<double>("context/" + name + "/wallWidth")),
    wallThickness_(config->get<double>("context/" + name + "/wallThickness")),
    gapWidth_(config->get<double>("context/" + name + "/gapWidth")),
    gapOffset_(config->get<double>("context/" + name + "/gapOffset")) {
  if (wallWidth_ < 0.0) {
    OMPL_ERROR("%s: Wall width is negative.", name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (wallThickness_ < 0.0) {
    OMPL_ERROR("%s: Wall thickness is negative.", name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (gapWidth_ < 0.0) {
    OMPL_ERROR("%s: Gap width is negative.", name.c_str());
    throw std::runtime_error("Context error.");
  }

  // Create the validity checker.
  auto validityChecker = std::make_shared<ContextValidityChecker>(spaceInfo_);

  // Create the obstacles and add them to the validity checker.
  createObstacles();
  validityChecker->addObstacles(obstacles_);

  // Set the validity checker and the check resolution.
  spaceInfo_->setStateValidityChecker(validityChecker);
  spaceInfo_->setStateValidityCheckingResolution(
      config->get<double>("context/" + name + "/collisionCheckResolution"));

  // Set up the space info.
  spaceInfo_->setup();

  startGoalPairs_ = makeStartGoalPair();
}

void WallGap::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

void WallGap::createObstacles() {
  // Get the state space bounds.
  auto bounds = spaceInfo_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getBounds();

  // Create an anchor for the lower obstacle.
  ompl::base::ScopedState<> anchor_low(spaceInfo_);

  // Set the obstacle anchor in the middle of the state space.
  for (auto j = 0u; j < dimensionality_; ++j) {
    anchor_low[j] = (bounds.low.at(j) + bounds.high.at(j)) / 2.0;
  }

  // Get the extent of the state space in the second dimension.
  auto extent2d = (bounds.high.at(1u) - bounds.low.at(1u));

  // Move it down in second dimension.
  anchor_low[1u] = bounds.low.at(1u) + (extent2d / 2.0 + gapOffset_ - gapWidth_ / 2.0) / 2.0;

  // Create the widths of this wall.
  std::vector<double> widths_low(dimensionality_, 0.0);

  // Set the thickness of the wall.
  widths_low.at(0) = wallThickness_;

  // Set the width.
  widths_low.at(1) = extent2d / 2.0 + gapOffset_ - gapWidth_ / 2.0;

  // The wall extends to the boundaries in all other dimensions.
  for (std::size_t j = 2u; j < dimensionality_; ++j) {
    widths_low.at(j) = bounds.high.at(j) - bounds.low.at(j);
  }
  obstacles_.push_back(std::make_shared<obstacles::Hyperrectangle<obstacles::BaseObstacle>>(
      spaceInfo_, anchor_low, widths_low));

  // Create an anchor for the upper obstacle.
  ompl::base::ScopedState<> anchor_up(spaceInfo_);

  // Set the obstacle anchor in the middle of the state space.
  for (auto j = 0u; j < dimensionality_; ++j) {
    anchor_up[j] = (bounds.low.at(j) + bounds.high.at(j)) / 2.0;
  }

  // Move it up in second dimension.
  anchor_up[1u] =
      bounds.low.at(1u) + extent2d / 4.0 + gapOffset_ / 2.0 + gapWidth_ / 4.0 + wallWidth_ / 2.0;

  // Create the widths of this wall.
  std::vector<double> widths_up(dimensionality_, 0.0);

  // Set the thickness of the wall.
  widths_up.at(0) = wallThickness_;

  // Set the width.
  widths_up.at(1u) = wallWidth_ - (extent2d / 2.0 + gapOffset_ + gapWidth_ / 2.0);

  // The wall extends to the boundaries in all other dimensions.
  for (std::size_t j = 2u; j < dimensionality_; ++j) {
    widths_up.at(j) = bounds.high.at(j) - bounds.low.at(j);
  }
  obstacles_.push_back(std::make_shared<obstacles::Hyperrectangle<obstacles::BaseObstacle>>(
      spaceInfo_, anchor_up, widths_up));
}

}  // namespace planning_contexts

}  // namespace pdt
