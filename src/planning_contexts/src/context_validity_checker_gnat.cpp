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

#include "pdt/planning_contexts/context_validity_checker_gnat.h"

namespace pdt {

namespace planning_contexts {

using namespace std::string_literals;

ContextValidityCheckerGNAT::ContextValidityCheckerGNAT(
    const ompl::base::SpaceInformationPtr& spaceInfo) :
    ContextValidityChecker(spaceInfo) {
  obstacleAnchors_.setDistanceFunction(
      [this](const std::pair<std::size_t, const ompl::base::State*>& a,
             const std::pair<std::size_t, const ompl::base::State*>& b) {
        return si_->distance(a.second, b.second);
      });
  antiObstacleAnchors_.setDistanceFunction(
      [this](const std::pair<std::size_t, const ompl::base::State*>& a,
             const std::pair<std::size_t, const ompl::base::State*>& b) {
        return si_->distance(a.second, b.second);
      });
}

bool ContextValidityCheckerGNAT::isValid(const ompl::base::State* state) const {
  if (!state) {
    throw std::runtime_error("ContextValidityCheckerGNAT recieved nullptr state.");
  }
  // If the state does not satisfy the space bounds, it is not valid.
  if (!si_->satisfiesBounds(state)) {
    return false;
  }

  // Convert this state to an indexed state for nearest neighbour lookup.
  auto indexedState = std::make_pair(0u, state);

  // A state is valid if it collides with an anti obstacle. This overrides collisions with
  // obstacles.
  std::vector<std::pair<std::size_t, const ompl::base::State*>> antiNeighbours{};
  antiObstacleAnchors_.nearestR(indexedState, maxAntiObstacleRadius_, antiNeighbours);
  for (const auto& anti : antiNeighbours) {
    try {
      if (antiObstacles_.at(anti.first)->validates(state)) {
        return true;
      }
    } catch (const std::out_of_range& e) {
      auto msg =
          "Nearest neighbour lookup for antiobstacles returned invalid index:\n    "s + e.what();
      throw std::out_of_range(msg);
    }
  }

  // A state is not valid if it collides with an obstacle.
  std::vector<std::pair<std::size_t, const ompl::base::State*>> obsNeighbours{};
  obstacleAnchors_.nearestR(indexedState, maxObstacleRadius_, obsNeighbours);
  for (const auto& obs : obsNeighbours) {
    try {
      if (obstacles_.at(obs.first)->invalidates(state)) {
        return false;
      }
    } catch (const std::out_of_range& e) {
      auto msg = "Nearest neighbour lookup for obstacles returned invalid index:\n    "s + e.what();
      throw std::out_of_range(msg);
    }
  }

  // The state satisfies the bounds and is not invalidated by any obstacles.
  return true;
}

void ContextValidityCheckerGNAT::addObstacle(
    const std::shared_ptr<obstacles::BaseObstacle>& obstacle) {
  if (maxObstacleRadius_ < obstacle->getCircumradius()) {
    maxObstacleRadius_ = obstacle->getCircumradius();
  }
  obstacles_.push_back(obstacle);
  obstacleAnchors_.add(std::make_pair(obstacles_.size() - 1u, obstacle->getState()));
}

void ContextValidityCheckerGNAT::addObstacles(
    const std::vector<std::shared_ptr<obstacles::BaseObstacle>>& obstacles) {
  // We could insert multiple elements at once, but have to be careful about indices and this is not
  // going to happen frequently, so better safe than fast.
  obstacles_.reserve(obstacles_.size() + obstacles.size());
  for (const auto& obstacle : obstacles) {
    if (maxObstacleRadius_ < obstacle->getCircumradius()) {
      maxObstacleRadius_ = obstacle->getCircumradius();
    }
    obstacles_.push_back(obstacle);
    obstacleAnchors_.add(std::make_pair(obstacles_.size() - 1u, obstacle->getState()));
  }
}

void ContextValidityCheckerGNAT::addAntiObstacle(
    const std::shared_ptr<obstacles::BaseAntiObstacle>& anti) {
  if (maxAntiObstacleRadius_ < anti->getCircumradius()) {
    maxAntiObstacleRadius_ = anti->getCircumradius();
  }
  antiObstacles_.push_back(anti);
  antiObstacleAnchors_.add(std::make_pair(antiObstacles_.size() - 1u, anti->getState()));
}

void ContextValidityCheckerGNAT::addAntiObstacles(
    const std::vector<std::shared_ptr<obstacles::BaseAntiObstacle>>& antis) {
  antiObstacles_.reserve(antiObstacles_.size() + antis.size());
  for (const auto& anti : antis) {
    if (maxAntiObstacleRadius_ < anti->getCircumradius()) {
      maxAntiObstacleRadius_ = anti->getCircumradius();
    }
    antiObstacles_.push_back(anti);
    antiObstacleAnchors_.add(std::make_pair(antiObstacles_.size() - 1u, anti->getState()));
  }
}

}  // namespace planning_contexts

}  // namespace pdt
