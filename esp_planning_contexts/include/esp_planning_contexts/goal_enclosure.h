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

#include "esp_obstacles/cutout_obstacles.h"
#include "esp_planning_contexts/base_context.h"

namespace esp {

namespace ompltools {

/** \brief A homotopy-breaking, "bug trap" style experiment that scales to N dimensions. I.e., The
 * start and goal are both enclosed in a box with a opening away from the other. 2D is a slice of
 * higher-D */
class GoalEnclosure : public BaseContext {
 public:
  /** \brief Constructor */
  GoalEnclosure(const unsigned int dim, const double worldHalfWidth, const double insideWidth,
                const double wallThickness, const double gapWidth, const double runSeconds,
                const double checkResolution);

  /** \brief This problem could knows its optimum, but doesn't right now */
  virtual bool knowsOptimum() const;

  /** \brief Could return the optimum, but instead throws. */
  virtual ompl::base::Cost getOptimum() const;

  /** \brief Set the target cost as the specified cost. */
  virtual void setTarget(double targetSpecifier);

  /** \brief Derived class specific information to include in the title line. */
  virtual std::string lineInfo() const;

  /** \brief Derived class specific information to include at the end. */
  virtual std::string paraInfo() const;

  virtual void accept(const ContextVisitor& visitor) const;

 protected:
  // Variables
  /** \brief The actual enclosures */
  std::shared_ptr<CutoutObstacles> enclObs_{};

  /** \brief Construction variables: The lower-left corners (1 obs, 2 anti) */
  std::vector<std::shared_ptr<ompl::base::ScopedState<> > > startEnclCorners_{};
  std::vector<std::shared_ptr<ompl::base::ScopedState<> > > goalEnclCorners_{};
  /** \brief Construction variables: The associated widths */
  std::vector<std::vector<double> > startEnclWidths_{};
  std::vector<std::vector<double> > goalEnclWidths_{};

  // Constant Parameters
  /** \brief The inside-width of the enclosure. */
  double insideWidth_{0.0};
  /** \brief The enclosure-wall thickness. */
  double wallThickness_{0.0};
  /** \brief The width of the opening in directions perpendicular to the wall thickness. */
  double gapWidth_{0.0};
  /** \brief The start and goal positions */
  double startPos_{-0.5};
  double goalPos_{0.5};

  // Helper function
  std::string printRectangle(std::shared_ptr<ompl::base::ScopedState<> > llCorner,
                             std::vector<double> widths) const;
};

}  // namespace ompltools

}  // namespace esp
