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

// Authors: Marlin Strub

#pragma once

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>

#include "esp_optimization_objectives/base_optimization_objective.h"
#include "esp_optimization_objectives/optimization_objective_visitor.h"

namespace esp {

namespace ompltools {

// Obstacles are geometric primitives.
class MaxMinClearanceOptimizationObjective : public ompl::base::OptimizationObjective,
                                             public BaseOptimizationObjective {
 public:
  MaxMinClearanceOptimizationObjective(
      const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo);
  virtual ~MaxMinClearanceOptimizationObjective();

  ompl::base::Cost stateCost(const ompl::base::State* state) const override;

  bool isCostBetterThan(ompl::base::Cost cost1, ompl::base::Cost cost2) const override;

  ompl::base::Cost identityCost() const override;

  ompl::base::Cost infiniteCost() const override;

  ompl::base::Cost combineCosts(ompl::base::Cost cost1, ompl::base::Cost cost2) const override;

  ompl::base::Cost motionCost(const ompl::base::State* state1,
                              const ompl::base::State* state2) const override;

  ompl::base::Cost motionCostHeuristic(const ompl::base::State* state1,
                                       const ompl::base::State* state2) const override;

  bool isSatisfied(ompl::base::Cost cost) const override;

  void accept(const ObjectiveVisitor& visitor) const override;

 private:
  ompl::base::State* testState_;
  const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo_ = OptimizationObjective::si_;
};

}  // namespace ompltools

}  // namespace esp