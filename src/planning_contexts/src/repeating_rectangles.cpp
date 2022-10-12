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

#include "pdt/planning_contexts/repeating_rectangles.h"

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>

#include "pdt/obstacles/base_obstacle.h"
#include "pdt/obstacles/hyperrectangle.h"
#include "pdt/planning_contexts/context_validity_checker_gnat.h"

namespace pdt {

namespace planning_contexts {

ContextValidityCheckerRepeatingRectangles::ContextValidityCheckerRepeatingRectangles(
    const ompl::base::SpaceInformationPtr& spaceInfo, const std::size_t numObsPerDim,
    const double obsWidth) :
    ContextValidityChecker(spaceInfo),
    numObsPerDim_(numObsPerDim),
    obsWidth_(obsWidth) {
  const auto bounds = si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getBounds();
  coordinates_.resize(numObsPerDim_);
  for (auto i = 0u; i < numObsPerDim_; ++i) {
    // The context assumes a hyperrectangle.
    // Thus, all bounds are the same, and we can stick to the 0th index of the bounds.
    coordinates_[i] = (static_cast<double>((i + 1u)) * (bounds.high.at(0u) - bounds.low.at(0u)) /
                       static_cast<double>((numObsPerDim_ + 1u))) +
                      bounds.low.at(0u);
  }
}

bool ContextValidityCheckerRepeatingRectangles::isValid(const ompl::base::State* state) const {
  if (!state) {
    throw std::runtime_error("ContextValidityCheckerRepeatingRectangles recieved nullptr state.");
  }
  // If the state does not satisfy the space bounds, it is not valid.
  if (!si_->satisfiesBounds(state)) {
    return false;
  }

  ompl::base::ScopedState<> scopedState(si_->getStateSpace(), state);
  auto bounds = si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getBounds();

  for (auto i = 0u; i < si_->getStateDimension(); ++i) {
    // figure out closest anchor pt
    const auto tmp = ((scopedState[i] - bounds.low.at(i)) / (bounds.high.at(i) - bounds.low.at(i)) *
                      static_cast<double>((numObsPerDim_ + 1u))) -
                     1u;

    std::size_t interval = 0u;
    if (tmp > 0u) {
      interval = static_cast<std::size_t>(std::round(tmp));
    }

    // clamp the ends
    if (scopedState[i] < coordinates_[0u]) {
      interval = 0u;
    } else if (scopedState[i] > coordinates_.back()) {
      interval = numObsPerDim_ - 1u;
    }

    if (scopedState[i] <
            coordinates_[interval] - obsWidth_ / 2.0 - std::numeric_limits<double>::epsilon() ||
        scopedState[i] >
            coordinates_[interval] + obsWidth_ / 2.0 + std::numeric_limits<double>::epsilon()) {
      return true;
    }
  }

  return false;
}

RepeatingRectangles::RepeatingRectangles(
    const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
    const std::shared_ptr<const config::Configuration>& config, const std::string& name) :
    RealVectorGeometricContext(spaceInfo, config, name),
    numObsPerDim_(config->get<std::size_t>("context/" + name + "/numObstaclesPerDim")),
    obsWidth_(config->get<double>("context/" + name + "/obstacleWidth")) {
  // Get the state space bounds.
  auto bounds = spaceInfo_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getBounds();
  for (auto i = 1u; i < dimensionality_; ++i) {
    if ((bounds.high.at(i) - bounds.low.at(i)) != bounds.high.at(0u) - bounds.low.at(0u)) {
      OMPL_ERROR("%s: Repeating rectangles assumes a (hyper)square.");
      throw std::runtime_error("Context error.");
    }
  }

  // Create the validity checker.
  auto validityChecker = std::make_shared<ContextValidityCheckerRepeatingRectangles>(
      spaceInfo_, numObsPerDim_, obsWidth_);

  // We still create the obstacles for 2d and 3d problems to be able to visualize the problem
  if (dimensionality_ <= 3) {
    createObstacles();
  }

  // Set the validity checker and the check resolution.
  spaceInfo_->setStateValidityChecker(validityChecker);
  spaceInfo_->setStateValidityCheckingResolution(
      config->get<double>("context/" + name + "/collisionCheckResolution"));

  // Set up the space info.
  spaceInfo_->setup();

  startGoalPairs_ = makeStartGoalPair();
}

void RepeatingRectangles::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

void RepeatingRectangles::createObstacles() {
  // Let's try to keep this general for any number of dimensions.

  // Generate evenly spaced coordinates.
  auto bounds = spaceInfo_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getBounds();
  for (auto i = 1u; i < dimensionality_; ++i) {
    if ((bounds.high.at(i) - bounds.low.at(i)) != bounds.high.at(0u) - bounds.low.at(0u)) {
      OMPL_ERROR("%s: Repeating rectangles assumes a (hyper)square.");
      throw std::runtime_error("Context error.");
    }
  }
  std::vector<double> coordinates{};
  coordinates.reserve(numObsPerDim_ * dimensionality_);
  for (auto i = 0u; i < numObsPerDim_; ++i) {
    // The context assumes a hyperrectangle.
    // Thus, all bounds are the same, and we can stick to the 0th index of the bounds.
    coordinates.push_back(
        (static_cast<double>((i + 1u)) * (bounds.high.at(0u) - bounds.low.at(0u)) /
         static_cast<double>((numObsPerDim_ + 1u))) +
        bounds.low.at(0u));
  }
  // We need all combinations of d elements from the above coordinates
  // with replacement. We achieve this, by essentially implementing a counter in
  // base numObsPerDim, and are counting through until we arrive at numObs^dim.
  std::vector<std::size_t> bins(dimensionality_, 0u);
  for (auto k = 0u; k < std::pow(numObsPerDim_, dimensionality_); ++k) {
    // Take the coordinates of the bitmask for the midpoint.
    ompl::base::ScopedState<> midpoint(spaceInfo_);
    for (auto dim = 0u; dim < dimensionality_; ++dim) {
      midpoint[dim] = coordinates[bins[dim]];
    }

    for (auto i = 0u; i < dimensionality_; ++i) {
      if (i == 0u) {
        bins[i] += 1;
      } else if (bins[i - 1] >= numObsPerDim_) {
        bins[i] += 1;
        bins[i - 1] = 0u;
      }
    }

    // The widths are a parameter, so just take this.
    std::vector<double> widths(dimensionality_, obsWidth_);

    // Add the obstacle.
    obstacles_.push_back(std::make_shared<obstacles::Hyperrectangle<obstacles::BaseObstacle>>(
        spaceInfo_, midpoint, widths));
  }
}

}  // namespace planning_contexts

}  // namespace pdt
