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

/* Authors: Jonathan Gammell */

#include "pdt/planning_contexts/context_validity_checker.h"

namespace pdt {

namespace planning_contexts {

ContextValidityChecker::ContextValidityChecker(const ompl::base::SpaceInformationPtr& spaceInfo) :
    ompl::base::StateValidityChecker(spaceInfo) {
}

bool ContextValidityChecker::isValid(const ompl::base::State* state) const {
  // If the state does not satisfy the space bounds, it is not valid.
  if (!si_->satisfiesBounds(state)) {
    return false;
  }

  // A state is valid if it collides with an anti obstacle. This overrides collisions with
  // obstacles.
  for (const auto& anti : antiObstacles_) {
    if (anti->validates(state)) {
      return true;
    }
  }

  // A state is not valid if it collides with an obstacle.
  for (const auto& obs : obstacles_) {
    if (obs->invalidates(state)) {
      return false;
    }
  }

  // The state satisfies the bounds and is not invalidated by any obstacles.
  return true;
}

double ContextValidityChecker::clearance(const ompl::base::State* state) const {
  // Compute the distance to all obstacles and take the minimum.
  double minDistance = std::numeric_limits<double>::infinity();
  for (const auto& obstacle : obstacles_) {
    double distance = obstacle->clearance(state);
    minDistance = distance < minDistance ? distance : minDistance;
  }
  return minDistance;
}

void ContextValidityChecker::addObstacle(const std::shared_ptr<obstacles::BaseObstacle>& obstacle) {
  obstacles_.push_back(obstacle);
}

void ContextValidityChecker::addObstacles(
    const std::vector<std::shared_ptr<obstacles::BaseObstacle>>& obstacles) {
  obstacles_.insert(obstacles_.end(), obstacles.begin(), obstacles.end());
}

void ContextValidityChecker::addAntiObstacle(
    const std::shared_ptr<obstacles::BaseAntiObstacle>& anti) {
  antiObstacles_.push_back(anti);
}

void ContextValidityChecker::addAntiObstacles(
    const std::vector<std::shared_ptr<obstacles::BaseAntiObstacle>>& antis) {
  antiObstacles_.insert(antiObstacles_.end(), antis.begin(), antis.end());
}

std::vector<std::shared_ptr<obstacles::BaseObstacle>> ContextValidityChecker::getObstacles() const {
  return obstacles_;
}

std::vector<std::shared_ptr<obstacles::BaseAntiObstacle>> ContextValidityChecker::getAntiObstacles()
    const {
  return antiObstacles_;
}

}  // namespace planning_contexts

}  // namespace pdt
