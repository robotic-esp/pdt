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

#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/util/RandomNumbers.h>

#include "esp_obstacles/base_obstacle.h"

/** \brief A world consisting of random hyperrectangular obstacles.*/
class RepeatingHyperrectangleObstacles : public BaseObstacle {
 public:
  /** \brief Constructor. */
  RepeatingHyperrectangleObstacles(ompl::base::SpaceInformation* si, double obsWidth,
                                   double blankWidth,
                                   const std::vector<double>& origin = std::vector<double>());
  /** \brief Constructor. */
  RepeatingHyperrectangleObstacles(ompl::base::SpaceInformation* si,
                                   const std::vector<double>& obsWidths,
                                   const std::vector<double>& blankWidths,
                                   const std::vector<double>& origin = std::vector<double>());
  /** \brief Constructor. */
  RepeatingHyperrectangleObstacles(const ompl::base::SpaceInformationPtr& si, double obsWidth,
                                   double blankWidth,
                                   const std::vector<double>& origin = std::vector<double>());
  /** \brief Constructor. */
  RepeatingHyperrectangleObstacles(const ompl::base::SpaceInformationPtr& si,
                                   const std::vector<double>& obsWidths,
                                   const std::vector<double>& blankWidths,
                                   const std::vector<double>& origin = std::vector<double>());
  /** \brief Destructor */
  ~RepeatingHyperrectangleObstacles();

  /** \brief Clear the obstacle space */
  virtual void clear();

  /** \brief Check for state validity */
  virtual bool isValid(const ompl::base::State* state) const;

  /** \brief The obstacle map as a series of Matlab plot functions. (Discard the space-colour
   * argument) */
  virtual std::string mfile(const std::string& obsColour, const std::string& /*spaceColour*/) const;

 protected:
 private:
  /** \brief Common constructor */
  void construct();

  // Variables
  /** \brief The dimension of the planning problem*/
  unsigned int dim_ { 0u };
  /** \brief The widths of the obstacle "columns"*/
  std::vector<double> obsWidths_ { };
  /** \brief The widths of the blank "columns" */
  std::vector<double> blankWidths_ { };
  /** \brief The origin of the regularized grid */
  std::vector<double> origin_ { };
  /** \brief The period of "columns" (i.e., the width of obstacle plus the width of the blank) */
  std::vector<double> periods_ { };
};
