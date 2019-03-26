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

#include "esp_obstacles/base_obstacle.h"

namespace esp {

namespace ompltools {

/** \brief A world consisting of obstacles defined by obstacles and "anti-obstacles". A state is in
 * collision if it is in an obstacle and not in an anti-obstacle. */
class CutoutObstacles : public BaseObstacle {
 public:
  CutoutObstacles(ompl::base::SpaceInformation* si);
  CutoutObstacles(const ompl::base::SpaceInformationPtr& si);
  virtual ~CutoutObstacles() = default;

  /** \brief Check for state validity */
  bool isValid(const ompl::base::State* state) const override;

  /** \brief Add an obstacle to the obstacle space */
  void addObstacle(const std::shared_ptr<BaseObstacle>& newObstaclePtr);

  /** \brief Add an anti-obstacle to the obstacle space */
  void addAntiObstacle(const std::shared_ptr<BaseObstacle>& newAntiObstaclePtr);

  /** \brief The obstacle map as a series of Matlab plot functions */
  std::string mfile(const std::string& obsColour, const std::string& spaceColour) const override;

  // Accept a visitor.
  virtual void accept(const ObstacleVisitor& visitor) const override;

 private:
  /** \brief A vector of obstacles */
  std::vector<std::shared_ptr<BaseObstacle>> obstaclePtrs_{};

  /** \brief A vector of anti-obstacles */
  std::vector<std::shared_ptr<BaseObstacle>> antiObstaclePtrs_{};
};

}  // namespace ompltools

}  // namespace esp
