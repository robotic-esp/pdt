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

#include "obstacles/RepeatingHyperrectangleObstacles.h"

#include <cmath>

#include "ompl/util/Exception.h"

#include "ompl/base/spaces/RealVectorStateSpace.h"

RepeatingHyperrectangleObstacles::RepeatingHyperrectangleObstacles(
    ompl::base::SpaceInformation* si, double obsWidth, double blankWidth,
    const std::vector<double>& origin /*= std::vector<double>()*/)
    : BaseObstacle(si),
      dim_(StateValidityChecker::si_->getStateDimension()),
      obsWidths_(std::vector<double>(dim_, obsWidth)),
      blankWidths_(std::vector<double>(dim_, blankWidth)),
      origin_(origin) {
  this->construct();
}
RepeatingHyperrectangleObstacles::RepeatingHyperrectangleObstacles(
    ompl::base::SpaceInformation* si, const std::vector<double>& obsWidths,
    const std::vector<double>& blankWidths,
    const std::vector<double>& origin /*= std::vector<double>()*/)
    : BaseObstacle(si),
      dim_(StateValidityChecker::si_->getStateDimension()),
      obsWidths_(obsWidths),
      blankWidths_(blankWidths),
      origin_(origin) {
  this->construct();
}
RepeatingHyperrectangleObstacles::RepeatingHyperrectangleObstacles(
    const ompl::base::SpaceInformationPtr& si, double obsWidth, double blankWidth,
    const std::vector<double>& origin /*= std::vector<double>()*/)
    : BaseObstacle(si),
      dim_(StateValidityChecker::si_->getStateDimension()),
      obsWidths_(std::vector<double>(dim_, obsWidth)),
      blankWidths_(std::vector<double>(dim_, blankWidth)),
      origin_(origin) {
  this->construct();
}
RepeatingHyperrectangleObstacles::RepeatingHyperrectangleObstacles(
    const ompl::base::SpaceInformationPtr& si, const std::vector<double>& obsWidths,
    const std::vector<double>& blankWidths,
    const std::vector<double>& origin /*= std::vector<double>()*/)
    : BaseObstacle(si),
      dim_(StateValidityChecker::si_->getStateDimension()),
      obsWidths_(obsWidths),
      blankWidths_(blankWidths),
      origin_(origin) {
  this->construct();
}

void RepeatingHyperrectangleObstacles::construct() {
  if (origin_.empty() == true) {
    origin_ = std::vector<double>(dim_, 0.0);
  } else if (origin_.size() != dim_) {
    throw ompl::Exception("The provided origin is the wrong dimension");
  }

  for (unsigned int i = 0u; i < dim_; ++i) {
    periods_.push_back(blankWidths_.at(i) + obsWidths_.at(i));
  }
}

RepeatingHyperrectangleObstacles::~RepeatingHyperrectangleObstacles() {}

void RepeatingHyperrectangleObstacles::clear() {}

bool RepeatingHyperrectangleObstacles::isValid(const ompl::base::State* state) const {
  // Variable
  // The return value
  bool validState;

  // Check if the state satisfies the bounds
  validState = StateValidityChecker::si_->satisfiesBounds(state);

  if (validState == true) {
    // Variable
    // Whether we're free of the obstacle in the dimensions checked to date. To be valid, we only
    // need to be free in one dimension.
    bool freeState = false;

    // If it does, check each dimension against the regular grid.
    for (unsigned int i = 0u; i < dim_ && freeState == false; ++i) {
      // Variable
      // The coordinate value in this dimension:
      double coordValue;
      // The position inside a repeating period
      double periodDist;

      // Get the position in this dimension and subtract the origin to place it in the "grid" frame:
      coordValue =
          state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] - origin_.at(i);

      // Calculate the position inside the repeating segement, which will be the distance from the
      // nearest obstacle
      periodDist = std::fmod(coordValue, periods_.at(i));

      // If it's negative, then that's the distance from the "end" of the sequence, so subtract:
      if (periodDist < 0.0) {
        periodDist = periodDist + periods_.at(i);
      }

      // We're free if we're further away then the width of the obstacle.
      // No need to and/or as the loop will end as soon as this is true.
      freeState = (periodDist > obsWidths_.at(i));
    }

    // Calculate whether the state is valid or not, i.e., if it's free
    validState = freeState;
  }
  // No else, we're done

  return validState;
}

std::string RepeatingHyperrectangleObstacles::mfile(const std::string& obsColour,
                                                    const std::string& /*spaceColour*/) const {
  // Variables
  // The string stream:
  std::stringstream rval;

  rval << "origin = [" << origin_.at(0u) << "; " << origin_.at(1u) << "];" << std::endl;
  rval << "xPeriod = " << periods_.at(0u) << ";" << std::endl;
  rval << "yPeriod = " << periods_.at(1u) << ";" << std::endl;
  rval << "xWidth = " << obsWidths_.at(0u) << ";" << std::endl;
  rval << "yWidth = " << obsWidths_.at(1u) << ";" << std::endl;
  rval << "for x = "
          "sign(minX/xPeriod)*ceil(abs(minX/xPeriod)):sign(maxX/xPeriod)*ceil(abs(maxX/xPeriod))"
       << std::endl;
  rval << "    for y = "
          "sign(minY/yPeriod)*ceil(abs(minY/yPeriod)):sign(maxY/yPeriod)*ceil(abs(maxY/yPeriod))"
       << std::endl;
  rval << "        x_ll = x*xPeriod + origin(1,1);" << std::endl;
  rval << "        x_lr = x_ll + xWidth;" << std::endl;
  rval << "        x_ul = x_ll;" << std::endl;
  rval << "        x_ur = x_lr;" << std::endl;
  rval << std::endl;
  rval << "        y_ll = y*yPeriod + origin(2,1);" << std::endl;
  rval << "        y_lr = y_ll;" << std::endl;
  rval << "        y_ur = y_ll + yWidth;" << std::endl;
  rval << "        y_ul = y_ur;" << std::endl;
  rval << "        fill([x_ll, x_lr, x_ur, x_ul], [y_ll, y_lr, y_ur, y_ul], " << obsColour << ");"
       << std::endl;
  rval << "    end" << std::endl;
  rval << "end" << std::endl;

  return rval.str();
}
