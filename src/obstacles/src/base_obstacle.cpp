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

#include "pdt/obstacles/base_obstacle.h"

namespace pdt {

namespace obstacles {

GeometricShape::GeometricShape(const ompl::base::SpaceInformationPtr& spaceInfo) :
    spaceInfo_(spaceInfo),
    anchor_(spaceInfo) {
}

void GeometricShape::setAnchor(const ompl::base::ScopedState<>& state) {
  anchor_ = state;
}

ompl::base::ScopedState<> GeometricShape::getAnchor() const {
  return anchor_;
}

const ompl::base::State* GeometricShape::getState() const {
  return anchor_.get();
}

std::vector<double> GeometricShape::getAnchorCoordinates() const {
  // return anchor_.reals() ?
  if (auto spaceInfo = spaceInfo_.lock()) {
    std::vector<double> coordinates(spaceInfo->getStateDimension(), 0.0);
    for (auto i = 0u; i < coordinates.size(); ++i) {
      coordinates[i] = anchor_[i];
    }
    return coordinates;
  } else {
    throw std::runtime_error("Space Information expired. This should not happen.");
  }
}

BaseObstacle::BaseObstacle(const ompl::base::SpaceInformationPtr& spaceInfo) :
    GeometricShape(spaceInfo) {
}

BaseAntiObstacle::BaseAntiObstacle(const ompl::base::SpaceInformationPtr& spaceInfo) :
    GeometricShape(spaceInfo) {
}

bool BaseObstacle::invalidates(const ompl::base::State* state) const {
  return isInside(state);
}

bool BaseObstacle::invalidates(const ompl::base::ScopedState<> state) const {
  return invalidates(state.get());
}

bool BaseAntiObstacle::validates(const ompl::base::State* state) const {
  return isInside(state);
}

bool BaseAntiObstacle::validates(const ompl::base::ScopedState<> state) const {
  return validates(state.get());
}

}  // namespace obstacles

}  // namespace pdt
