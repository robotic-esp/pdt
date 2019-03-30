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

#include "esp_obstacles/cutout_obstacles.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/util/Exception.h>

namespace esp {

namespace ompltools {

CutoutObstacles::CutoutObstacles(ompl::base::SpaceInformation* si) : BaseObstacle(si) {
}

CutoutObstacles::CutoutObstacles(const ompl::base::SpaceInformationPtr& si) : BaseObstacle(si) {
}

bool CutoutObstacles::isValid(const ompl::base::State* state) const {
  // Check that the state satisfies the problem bounds.
  if (!StateValidityChecker::si_->satisfiesBounds(state)) {
    return false;
  }

  // Check if the state is in an anti-obstacle.
  for (const auto& obs : antiObstaclePtrs_) {
    if (!obs->isValid(state)) {
      return true;
    }
  }

  // Check if the state is in an obstacle.
  for (const auto& obs : obstaclePtrs_) {
    if (!obs->isValid(state)) {
      return false;
    }
  }

  // If the state is in neither anti- nor regular obstacles, it's valid.
  return true;
}

void CutoutObstacles::addObstacle(const std::shared_ptr<BaseObstacle>& newObstaclePtr) {
  obstaclePtrs_.push_back(newObstaclePtr);
}

void CutoutObstacles::addAntiObstacle(const std::shared_ptr<BaseObstacle>& newAntiObstaclePtr) {
  antiObstaclePtrs_.push_back(newAntiObstaclePtr);
}

void CutoutObstacles::accept(const ObstacleVisitor& visitor) const {
  visitor.visit(*this);
}

}  // namespace ompltools

}  // namespace esp
