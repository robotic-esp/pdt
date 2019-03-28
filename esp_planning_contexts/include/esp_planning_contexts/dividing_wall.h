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

#include "esp_obstacles/random_hyperrectangles.h"
#include "esp_planning_contexts/base_context.h"

namespace esp {

namespace ompltools {

/** \brief A single wall diving the problem space in two, with multiple narrow passages. Results in
 * a multiple homotopy-class experiment that scales to N dimensions. */
class DividingWall : public BaseContext {
 public:
  /** \brief Constructor for a single wall. Must be an even number of gaps to assure there is no
   * straight-line solution */
  DividingWall(const unsigned int dim, const double wallThickness, const unsigned int numGaps,
               const double gapWidth, const double runSeconds, const double checkResolution);

  /** \brief Constructor for multiple walls. All the vectors must be the same size, except for
   * wall-spacing which should be one element shorter than the others */
  DividingWall(const unsigned int dim, const std::vector<double> wallThicknesses,
               const std::vector<unsigned int> numGaps, const std::vector<double> gapWidths,
               const std::vector<double> wallSpacings, const double runSeconds,
               const double checkResolution);

  /** \brief This problem could knows its optimum, but doesn't at the moment */
  virtual bool knowsOptimum() const;

  /** \brief As the optimum isn't implemented, throw. */
  virtual ompl::base::Cost getOptimum() const;

  /** \brief Set the optimization target as the specified cost. */
  virtual void setTarget(double targetSpecifier);

  /** \brief Derived class specific information to include in the title line. */
  virtual std::string lineInfo() const;

  /** \brief Derived class specific information to include at the end. */
  virtual std::string paraInfo() const;

  virtual void accept(const ContextVisitor& visitor) const;

 protected:
  // Variables
  /** \brief The obstacle world */
  std::shared_ptr<RandomHyperrectangles> rectObs_{};
  /** \brief The number of walls */
  unsigned int numWalls_{0u};
  /** \brief The separation between walls */
  std::vector<double> wallSpacings_{};
  /** \brief The basic thickness of the wall. */
  std::vector<double> wallThicknesses_{};
  /** \brief The number of gaps. */
  std::vector<unsigned int> numGaps_{};
  /** \brief The resulting number of obstacles. */
  std::vector<unsigned int> numObs_{};
  /** \brief The gap width. */
  std::vector<double> gapWidths_{};
  /** \brief The resulting obstacle width. */
  std::vector<double> wallWidths_{};
  /** \brief The lower-left corners of the obstacles*/
  std::vector<std::vector<std::shared_ptr<ompl::base::ScopedState<>>>> obsCorners_{};
  /** The widths of an obstacles */
  std::vector<std::vector<double>> allObsWidths_{};

  // Constant Parameters
  /** \brief The start and goal positions */
  double startPos_{-0.5};
  double goalPos_{0.5};

  // The common constructor function:
  void common_constructor(const std::vector<double> wallThickness,
                          const std::vector<unsigned int> numGaps,
                          const std::vector<double> gapWidth,
                          const std::vector<double> wallSpacings, const double checkResolution);
};

}  // namespace ompltools

}  // namespace esp
