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

#include "pdt/planning_contexts/narrow_passage.h"

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>

namespace pdt {

namespace planning_contexts {

NarrowPassage::NarrowPassage(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                             const std::shared_ptr<const config::Configuration>& config,
                             const std::string& name) :
    RealVectorGeometricContext(spaceInfo, config, name),
    wallThickness_(config->get<double>("context/" + name + "/wallThickness")),
    wallOffsetX_(config->get<double>("context/" + name + "/wallOffset")),
    passageWidth_(config->get<double>("context/" + name + "/passageWidth")),
    passageOffset_(config->get<double>("context/" + name + "/passageOffset")) {
  if (wallThickness_ < 0.0) {
    OMPL_ERROR("%s: Wall thickness is negative.", name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (passageWidth_ < 0.0) {
    OMPL_ERROR("%s: Gap width is negative.", name.c_str());
    throw std::runtime_error("Context error.");
  }
  if (passageOffset_ == 0.0) {
    OMPL_WARN("%s: Passage offset is 0, straight-line connection is valid.");
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

void NarrowPassage::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

void NarrowPassage::createObstacles() {
  // Get the state space bounds.
  auto bounds = spaceInfo_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getBounds();

  // Get the midpoint of the state space and add the offset in X direction.
  ompl::base::ScopedState<> midpoint(spaceInfo_);
  for (auto j = 0u; j < dimensionality_; ++j) {
    midpoint[j] = (bounds.low.at(j) + bounds.high.at(j)) / 2.0;
  }
  midpoint[0] += wallOffsetX_;

  // Create an anchor point for the lower obstacle.
  ompl::base::ScopedState<> lowerAnchor(midpoint);
  lowerAnchor[1] = (midpoint[1] + passageOffset_ - (passageWidth_ / 2.0) + bounds.low[1]) / 2.0;

  // Create the widths of the lower wall.
  std::vector<double> lowerWidths(dimensionality_, 0.0);

  // Set the thickness of the lower wall.
  lowerWidths.at(0) = wallThickness_;

  // Set the height of the lower wall.
  lowerWidths.at(1) = midpoint[1] + passageOffset_ - (passageWidth_ / 2.0) - bounds.low[1];

  // The wall extends to the boundaries in all other dimensions.
  for (std::size_t j = 2u; j < dimensionality_; ++j) {
    lowerWidths.at(j) = bounds.high.at(j) - bounds.low.at(j);
  }

  // Create an anchor point for the upper obstacle.
  ompl::base::ScopedState<> upperAnchor(midpoint);
  upperAnchor[1] =
      midpoint[1] + passageOffset_ + (bounds.high[1] - bounds.low[1] - lowerWidths.at(1)) / 2.0;

  // Create the widths of the upper wall.
  std::vector<double> upperWidths(dimensionality_, 0.0);

  // Set the thickness of the upper wall.
  upperWidths.at(0) = wallThickness_;

  // Set the height of the upper wall.
  upperWidths.at(1) = bounds.high.at(1) - bounds.low.at(1) - lowerWidths.at(1) - passageWidth_;

  // The wall extends to the boundaries in all other dimensions.
  for (std::size_t j = 2u; j < dimensionality_; ++j) {
    upperWidths.at(j) = bounds.high.at(j) - bounds.low.at(j);
  }

  // Add both obstacles.
  obstacles_.push_back(std::make_shared<obstacles::Hyperrectangle<obstacles::BaseObstacle>>(
      spaceInfo_, lowerAnchor, lowerWidths));
  obstacles_.push_back(std::make_shared<obstacles::Hyperrectangle<obstacles::BaseObstacle>>(
      spaceInfo_, upperAnchor, upperWidths));
}

}  // namespace planning_contexts

}  // namespace pdt
