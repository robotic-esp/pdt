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

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include "pdt/config/configuration.h"
#include "pdt/objectives/base_optimization_objective.h"
#include "pdt/objectives/optimization_objective_visitor.h"

namespace pdt {

namespace objectives {

// Obstacles are geometric primitives.
class PotentialFieldOptimizationObjective : public ompl::base::StateCostIntegralObjective,
                                            public BaseOptimizationObjective {
 public:
  PotentialFieldOptimizationObjective(
      const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
      const std::shared_ptr<const config::Configuration> config);
  virtual ~PotentialFieldOptimizationObjective();

  ompl::base::Cost stateCost(const ompl::base::State* state) const override;

  ompl::base::Cost motionCostHeuristic(const ompl::base::State* state1,
                                       const ompl::base::State* state2) const override;

  void accept(const ObjectiveVisitor& visitor) const override;

 private:
  const std::shared_ptr<const config::Configuration> config_;
  std::vector<ompl::base::State*> pointSources_;
  const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo_ = OptimizationObjective::si_;
};

}  // namespace objectives

}  // namespace pdt
