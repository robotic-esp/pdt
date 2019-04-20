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

#include "esp_configuration/configuration.h"
#include "esp_obstacles/base_obstacle.h"
#include "esp_obstacles/hyperrectangle.h"
#include "esp_planning_contexts/base_context.h"
#include "esp_planning_contexts/context_validity_checker.h"
#include "esp_planning_contexts/context_visitor.h"

namespace esp {

namespace ompltools {

/** \brief An experiment with a singularly placed square obstacle*/
class DoubleEnclosure : public BaseContext {
 public:
  DoubleEnclosure(const std::shared_ptr<const Configuration>& config, const std::string& name);
  virtual ~DoubleEnclosure() = default;

  /** \brief Whether the problem has an exact expression for the optimum */
  virtual bool knowsOptimum() const override;

  /** \brief This problem knows its optimum */
  virtual ompl::base::Cost computeOptimum() const override;

  /** \brief Set the target cost as the specified multiplier of the optimum. */
  virtual void setTarget(double targetSpecifier) override;

  /** \brief Derived class specific information to include in the title line. */
  virtual std::string lineInfo() const override;

  /** \brief Derived class specific information to include at the end. */
  virtual std::string paraInfo() const override;

  // Accept a context visitor.
  virtual void accept(const ContextVisitor& visitor) const override;

 protected:
  // Create obstacles and antiobstacles.
  void createObstacles();
  void createAntiObstacles();

  // The validity checker.
  std::shared_ptr<ContextValidityChecker> validityChecker_{};

  // Direct access to obstacle information.
  double startOutsideWidth_{0.0};
  double startInsideWidth_{0.0};
  double startGapWidth_{0.0};
  double goalOutsideWidth_{0.0};
  double goalInsideWidth_{0.0};
  double goalGapWidth_{0.0};

  // The start and goal positions.
  std::vector<double> startPos_{};
  std::vector<double> goalPos_{};
};

}  // namespace ompltools

}  // namespace esp