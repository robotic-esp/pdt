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

// Authors: Marlin Strub

#pragma once

#include <memory>
#include <utility>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

#include "pdt/obstacles/base_obstacle.h"
#include "pdt/obstacles/hyperrectangle.h"
#include "pdt/obstacles/obstacle_visitor.h"

namespace pdt {

namespace planning_contexts {

class ReedsSheppValidityChecker : public ompl::base::StateValidityChecker {
 public:
  explicit ReedsSheppValidityChecker(const ompl::base::SpaceInformationPtr& spaceInfo);
  virtual ~ReedsSheppValidityChecker() = default;

  // Check if a state is valid.
  virtual bool isValid(const ompl::base::State* state) const override;

  // Return the minimum distance of a point to any obstacle.
  virtual double clearance(const ompl::base::State* state) const override;

  // Add obstacles.
  virtual void addObstacle(const std::shared_ptr<obstacles::BaseObstacle>& obstacle);
  virtual void addObstacles(const std::vector<std::shared_ptr<obstacles::BaseObstacle>>& obstacles);

  // Make obstacles accessible.
  virtual std::vector<std::shared_ptr<obstacles::BaseObstacle>> getObstacles() const;

 protected:
  // The normalized width and length of the Reeds-Shepp car.
  double width_{0.02};
  double length_{0.04};
  double circumradius_;

  using Point = std::array<double, 2u>;
  using Axis = std::array<double, 2u>;

  Point project(const Point& point, const Axis& axis) const;
  double dotProduct(const Point& point, const Axis& axis) const;

  ompl::base::RealVectorStateSpace* vectorSpace_{};
  ompl::base::SO2StateSpace* so2Space_{};

  std::vector<std::shared_ptr<obstacles::BaseObstacle>> obstacles_{};
};

}  // namespace planning_contexts

}  // namespace pdt
