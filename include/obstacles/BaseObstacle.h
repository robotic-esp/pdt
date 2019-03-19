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

#ifndef OBSTACLES_BASE_OBSTACLE
#define OBSTACLES_BASE_OBSTACLE

#include <string>
#include <vector>

#include <ompl/base/StateValidityChecker.h>

/** \brief The base class for a random world that derives from StateValidityChecker*/
class BaseObstacle : public ompl::base::StateValidityChecker {
 public:
  BaseObstacle(ompl::base::SpaceInformation* si);
  BaseObstacle(const ompl::base::SpaceInformationPtr& si);
  virtual ~BaseObstacle() = default;

  /** \brief Clear the obstacle space */
  virtual void clear() {};

  /** \brief Check for state validity */
  virtual bool isValid(const ompl::base::State* state) const = 0;

  /** \brief Check a vector of states for validity */
  virtual bool isValid(const std::vector<const ompl::base::State*>& states) const;

  /** \brief The obstacle map as a series of Matlab plot functions */
  virtual std::string mfile(const std::string& obsColour = "k",
                            const std::string& spaceColour = "w") const = 0;
};

#endif  // OBSTACLES_BASE_OBSTACLE
