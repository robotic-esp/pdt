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
#include <ompl/datastructures/NearestNeighborsGNAT.h>

#include "pdt/obstacles/base_obstacle.h"
#include "pdt/obstacles/hyperrectangle.h"
#include "pdt/obstacles/obstacle_visitor.h"

namespace pdt {

namespace planning_contexts {

class ContextValidityChecker : public ompl::base::StateValidityChecker {
 public:
  ContextValidityChecker(const ompl::base::SpaceInformationPtr& spaceInfo);
  virtual ~ContextValidityChecker() = default;

  // Check if a state is valid.
  virtual bool isValid(const ompl::base::State* state) const override;

  // Return the minimum distance of a point to any obstacle.
  virtual double clearance(const ompl::base::State* state) const override;

  // Add obstacles.
  virtual void addObstacle(const std::shared_ptr<obstacles::BaseObstacle>& obstacle);
  virtual void addObstacles(const std::vector<std::shared_ptr<obstacles::BaseObstacle>>& obstacles);

  // Add antiobstacles.
  virtual void addAntiObstacle(const std::shared_ptr<obstacles::BaseAntiObstacle>& anti);
  virtual void addAntiObstacles(
      const std::vector<std::shared_ptr<obstacles::BaseAntiObstacle>>& antis);

  // Make obstacles accessible.
  virtual std::vector<std::shared_ptr<obstacles::BaseObstacle>> getObstacles() const;
  virtual std::vector<std::shared_ptr<obstacles::BaseAntiObstacle>> getAntiObstacles() const;

 protected:
  std::vector<std::shared_ptr<obstacles::BaseObstacle>> obstacles_{};
  std::vector<std::shared_ptr<obstacles::BaseAntiObstacle>> antiObstacles_{};
};

}  // namespace planning_contexts

}  // namespace pdt
