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

#include "esp_obstacles/hyperrectangle.h"

#include <functional>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/util/Exception.h>

namespace esp {

namespace ompltools {

Hyperrectangle::Hyperrectangle(ompl::base::SpaceInformation* si) :
    BaseObstacle(si),
    corner(BaseObstacle::stateSpace_),
    sideLengths_(BaseObstacle::stateSpace_->getDimension(), 0.0) {
  if (BaseObstacle::stateSpace_->getType() != ompl::base::StateSpaceType::STATE_SPACE_REAL_VECTOR) {
    throw std::runtime_error(
        "Not sure what a hyperrectangle looks like in any other space than a real vector state "
        "space.");
  }
}

Hyperrectangle::Hyperrectangle(const ompl::base::SpaceInformationPtr& si) :
    BaseObstacle(si),
    corner(BaseObstacle::stateSpace_),
    sideLengths_(BaseObstacle::stateSpace_->getDimension(), 0.0) {
}

void Hyperrectangle::setSideLengths(const std::vector<double>& sideLengths) {
  sideLengths_ = sideLengths;
}

std::vector<double> Hyperrectangle::getSideLengths() const {
  return sideLengths_;
}

void Hyperrectangle::setCornerCoordinates(const std::vector<double>& coordinates) {
  if (coordinates.size() != BaseObstacle::dimension_) {
    throw std::runtime_error("Coordinate dimensions do not match hyperrectangle dimensions.");
  } else {
    for (std::size_t i = 0; i < coordinates.size(); ++i) {
      corner[i] = coordinates[i];
    }
  }
}

std::vector<double> Hyperrectangle::getCornerCoordinates() const {
  std::vector<double> coordinates(BaseObstacle::dimension_, 0.0);
  for (std::size_t i = 0; i < coordinates.size(); ++i) {
    coordinates[i] = corner[i];
  }
  return coordinates;
}

bool Hyperrectangle::isValid(const ompl::base::State* state) const {
  if (!StateValidityChecker::si_->satisfiesBounds(state)) {
    return false;
  }
  const auto* rs = static_cast<const ompl::base::RealVectorStateSpace::StateType*>(state);
  for (std::size_t i = 0; i < BaseObstacle::dimension_; ++i) {
    if ((*rs)[i] - std::numeric_limits<double>::epsilon() > corner[i] + sideLengths_[i] &&
        (*rs)[i] + std::numeric_limits<double>::epsilon() < corner[i]) {
      return true;
    }
  }
  return false;
}

void Hyperrectangle::accept(const ObstacleVisitor& visitor) const {
  visitor.visit(*this);
}

}  // namespace ompltools

}  // namespace esp
