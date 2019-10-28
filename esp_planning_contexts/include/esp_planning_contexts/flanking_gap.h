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
#include "esp_planning_contexts/base_obstacle_context.h"
#include "esp_planning_contexts/context_validity_checker.h"

namespace esp {

namespace ompltools {

/** \brief A single wall diving the problem space in two, with multiple narrow passages. Results in
 * a multiple homotopy-class experiment that scales to N dimensions. */
class FlankingGap : public BaseObstacleContext {
 public:
  // An odd number of gaps results in a straight line solution.
  FlankingGap(const std::shared_ptr<const Configuration>& config, const std::string& name);

  /** \brief This problem could knows its optimum, but doesn't at the moment */
  virtual bool knowsOptimum() const override;

  /** \brief As the optimum isn't implemented, throw. */
  virtual ompl::base::Cost computeOptimum() const override;

  /** \brief Set the optimization target as the specified cost. */
  virtual void setTarget(double targetSpecifier) override;

  /** \brief Derived class specific information to include in the title line. */
  virtual std::string lineInfo() const override;

  /** \brief Derived class specific information to include at the end. */
  virtual std::string paraInfo() const override;

  // Accept a context visitor.
  virtual void accept(const ContextVisitor& visitor) const override;

 protected:
  // Create the obstacles.
  void createObstacles();
  
  // Create the obstacles.
  void createAntiObstacles();
  
  // Direct access to obstacle information.
  double wallWidth_;
  double wallThickness_;
  double gapWidth_;
  double gapOffset_;

  // The validity checker.
  std::shared_ptr<ContextValidityChecker> validityChecker_{};

  // The start and goal positions.
  std::vector<double> startPos_;
  std::vector<double> goalPos_;
};

}  // namespace ompltools

}  // namespace esp
