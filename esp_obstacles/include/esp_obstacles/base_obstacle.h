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

// Authors: Jonathan Gammell, Marlin Strub

#pragma once

#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>

#include "esp_obstacles/obstacle_visitor.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Weffc++"

namespace esp {

namespace ompltools {

// Obstacles are geometric primitives.
class GeometricShape {
 public:
  GeometricShape(const ompl::base::SpaceInformationPtr& spaceInfo);
  virtual ~GeometricShape() = default;

  // Set the anchor point for this shape.
  virtual void setAnchor(const ompl::base::ScopedState<>& state);

  // Get the anchor point of this shape.
  virtual ompl::base::ScopedState<> getAnchor() const;

  // Get the measure of this obstacle.
  virtual double computeMeasure() const = 0;

  // The radius of the smallest circumscribing hypersphere with center at the anchor.
  virtual double computeMinCircumscribingRadius() const = 0;

  // Accept a visitor.
  virtual void accept(const ObstacleVisitor& visitor) const = 0;

 protected:
  // Whether a state is inside this shape.
  virtual bool isInside(const ompl::base::State* state) const = 0;

  // Information about the space this shape lives in.
  ompl::base::SpaceInformationPtr spaceInfo_{};

  // The anchor point.
  ompl::base::ScopedState<> anchor_;
};

class BaseObstacle : public GeometricShape {
 public:
  BaseObstacle(const ompl::base::SpaceInformationPtr& spaceInfo);
  virtual ~BaseObstacle() = default;

  // Return whether this obstacle invalidates a state.
  virtual bool invalidates(const ompl::base::State* state) const final;
};

class BaseAntiObstacle : public GeometricShape {
 public:
  BaseAntiObstacle(const ompl::base::SpaceInformationPtr& spaceInfo);
  virtual ~BaseAntiObstacle() = default;

  // Return whether this antiobstacle validates a state.
  virtual bool validates(const ompl::base::State* state) const final;
};

}  // namespace ompltools

}  // namespace esp

#pragma GCC diagnostic pop
