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

#include "pdt/planning_contexts/center_square.h"

#include <cmath>
#include <functional>
#include <memory>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include "pdt/obstacles/hyperrectangle.h"
#include "pdt/planning_contexts/context_validity_checker.h"

namespace pdt {

namespace planning_contexts {

CenterSquare::CenterSquare(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                           const std::shared_ptr<const config::Configuration>& config,
                           const std::string& name) :
    RealVectorGeometricContext(spaceInfo, config, name) {
  // Get the obstacle widths.
  std::vector<double> widths(dimensionality_,
                             config_->get<double>("context/" + name + "/obstacleWidth"));

  if (widths.at(0u) < 0.0) {
    OMPL_ERROR("%s: Obstacle width must be positive.", name.c_str());
    throw std::runtime_error("Context error.");
  }

  // Compute the midpoint of the obstacle.
  auto bounds = spaceInfo_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getBounds();
  auto midpoint = std::make_unique<ompl::base::ScopedState<>>(spaceInfo_);
  for (auto i = 0u; i < dimensionality_; ++i) {
    (*midpoint)[i] = (bounds.low.at(i) + bounds.high.at(i)) / 2.0;
  }

  // Create the obstacle.
  obstacles_.push_back(std::make_shared<obstacles::Hyperrectangle<obstacles::BaseObstacle>>(
      spaceInfo_, *midpoint, widths));

  // Create the validity checker and add the obstacle.
  auto validityChecker = std::make_shared<ContextValidityChecker>(spaceInfo_);
  validityChecker->addObstacles(obstacles_);

  // Set the validity checker and the check resolution.
  spaceInfo_->setStateValidityChecker(
      static_cast<ompl::base::StateValidityCheckerPtr>(validityChecker));
  spaceInfo_->setStateValidityCheckingResolution(
      config_->get<double>("context/" + name + "/collisionCheckResolution"));

  // Set up the space info.
  spaceInfo_->setup();

  startGoalPairs_ = makeStartGoalPair();
}

void CenterSquare::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

}  // namespace planning_contexts

}  // namespace pdt
