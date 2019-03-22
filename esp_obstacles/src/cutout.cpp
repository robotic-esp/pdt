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

#include "esp_obstacles/cutout.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/util/Exception.h>

CutoutObstacles::CutoutObstacles(ompl::base::SpaceInformation* si)
  : BaseObstacle(si) {}

CutoutObstacles::CutoutObstacles(const ompl::base::SpaceInformationPtr& si)
  : BaseObstacle(si) {}

bool CutoutObstacles::isValid(const ompl::base::State* state) const {
  // Variable
  // The return value
  bool validState;

  // Check if the state satisfies the bounds
  validState = StateValidityChecker::si_->satisfiesBounds(state);

  // Only continue if valid
  if (validState == true) {
    // Variable
    // Whether it's definitely valid (i.e., inside an anti obstacle)
    bool definitelyValid;

    // Iterate through the list of anti-obstacles. If the state is in any of them, it's free no
    // questions asked.
    definitelyValid = false;
    for (unsigned int i = 0u; i < antiObstaclePtrs_.size() && definitelyValid == false; ++i) {
      // Note the inversion, as anti obstacles are actually just obstacles (and therefore invalid
      // when inside them).
      definitelyValid = (antiObstaclePtrs_.at(i)->isValid(state) == false);
    }

    // If it's not definitely valid, it may be invalid
    if (definitelyValid == false) {
      // Iterate through the list of obstacles, stop if one is in collision
      for (unsigned int i = 0u; i < obstaclePtrs_.size() && validState == true; ++i) {
        validState = obstaclePtrs_.at(i)->isValid(state);
      }
    }
    // No else, it's _definitely_ valid and validState == true.
  }
  // No else, we're done

  return validState;
}

void CutoutObstacles::addObstacle(const std::shared_ptr<BaseObstacle>& newObstaclePtr) {
  obstaclePtrs_.push_back(newObstaclePtr);
}

void CutoutObstacles::addAntiObstacle(const std::shared_ptr<BaseObstacle>& newAntiObstaclePtr) {
  antiObstaclePtrs_.push_back(newAntiObstaclePtr);
}

std::string CutoutObstacles::mfile(const std::string& obsColour,
                                   const std::string& spaceColour) const {
  // Variables
  // The string stream:
  std::stringstream rval;

  for (const auto& obs : obstaclePtrs_) {
    rval << obs->mfile(obsColour);
  }

  for (const auto& anti : antiObstaclePtrs_) {
    rval << anti->mfile(spaceColour);
  }

  return rval.str();
}
