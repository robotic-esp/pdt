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

#pragma once

#include <memory>
#include <vector>

#include <ompl/base/ScopedState.h>

#include "esp_obstacles/base_obstacle.h"
#include "esp_obstacles/obstacle_visitor.h"

namespace esp {

namespace ompltools {

/** \brief A world consisting of random hyperrectangular obstacles.*/
class Hyperrectangle : public BaseObstacle {
 public:
  // To create a hyperrectangle, we need to have some information about the underlying space.
  Hyperrectangle(ompl::base::SpaceInformation* si);
  Hyperrectangle(const ompl::base::SpaceInformationPtr& si);

  // Disable copy for now.
  Hyperrectangle(const Hyperrectangle&) = delete;
  Hyperrectangle& operator=(const Hyperrectangle&) = delete;

  // We need to free the corner state (OMPL does not believe in RAII).
  virtual ~Hyperrectangle();

  // Set the side lengths.
  void setSideLengths(const std::vector<double>& sideLength);

  // Get the side lengths.
  std::vector<double> getSideLengths() const;

  // Set the corner with a state.
  void setCorner(const ompl::base::ScopedState<> &state);

  // Get a copy of the current corner.
  ompl::base::ScopedState<> getCorner() const;

  // Set the corner coordinates.
  void setCornerCoordinates(const std::vector<double>& coordinates);

  // Get the corner coordinates.
  std::vector<double> getCornerCoordinates() const;

  /** \brief Check for state validity */
  virtual bool isValid(const ompl::base::State* state) const;

  // Accept a visitor.
  virtual void accept(const ObstacleVisitor& visitor) const override;

 private:
  // This is the corner with the smallest value in each dimension. If we're in [0, X]^2 it's the lower-left corner.
  ompl::base::ScopedState<> corner_;

  // These are the sidelengths of this hypercube.
  std::vector<double> sideLengths_{};
};

}  // namespace ompltools

}  // namespace esp
