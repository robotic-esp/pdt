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

#include "pdt/objectives/max_min_clearance_optimization_objective.h"

#include <memory>

#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace pdt {

namespace objectives {

MaxMinClearanceOptimizationObjective::MaxMinClearanceOptimizationObjective(
    const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo) :
    ompl::base::OptimizationObjective(spaceInfo),
    testState_(spaceInfo->allocState()),
    spaceInfo_(spaceInfo) {
  // Optimization objectives have descriptions. (...)
  description_ = "Maximum Minimal Clearance";

  // Set the default cost-to-go heuristic.
  setCostToGoHeuristic(ompl::base::goalRegionCostToGo);
}

MaxMinClearanceOptimizationObjective::~MaxMinClearanceOptimizationObjective() {
  spaceInfo_->freeState(testState_);
}

ompl::base::Cost MaxMinClearanceOptimizationObjective::stateCost(
    const ompl::base::State* state) const {
  return ompl::base::Cost(spaceInfo_->getStateValidityChecker()->clearance(state));
}

bool MaxMinClearanceOptimizationObjective::isCostBetterThan(ompl::base::Cost cost1,
                                                            ompl::base::Cost cost2) const {
  // As suggested in the ompl optimization objective tutorial.
  return cost1.value() > cost2.value() + std::numeric_limits<double>::epsilon();
}

ompl::base::Cost MaxMinClearanceOptimizationObjective::identityCost() const {
  return ompl::base::Cost(std::numeric_limits<double>::infinity());
}

ompl::base::Cost MaxMinClearanceOptimizationObjective::infiniteCost() const {
  return ompl::base::Cost(0.0);
}

ompl::base::Cost MaxMinClearanceOptimizationObjective::combineCosts(ompl::base::Cost cost1,
                                                                    ompl::base::Cost cost2) const {
  // Return the worse of the two.
  if (isCostBetterThan(cost1, cost2)) {
    return cost2;
  } else {
    return cost1;
  }
}

ompl::base::Cost MaxMinClearanceOptimizationObjective::motionCost(
    const ompl::base::State* state1, const ompl::base::State* state2) const {
  // Get the number of segments between the two states.
  std::size_t segmentCount = spaceInfo_->getStateSpace()->validSegmentCount(state1, state2);

  // If there is only a single segment, just combine the costs.
  if (segmentCount <= 1) {
    return combineCosts(stateCost(state1), stateCost(state2));
  }

  // The motion cost is the combination of all costs along the edge (i.e., the worst cost of any
  // state along the edge).
  ompl::base::Cost totalCost = stateCost(state1);

  // Check all states along the edge.
  for (std::size_t i = 1; i < segmentCount; ++i) {
    spaceInfo_->getStateSpace()->interpolate(
        state1, state2, static_cast<double>(i) / static_cast<double>(segmentCount), testState_);
    totalCost = combineCosts(stateCost(testState_), totalCost);
  }

  return totalCost;
}

ompl::base::Cost MaxMinClearanceOptimizationObjective::motionCostHeuristic(
    const ompl::base::State* state1, const ompl::base::State* state2) const {
  return combineCosts(stateCost(state1), stateCost(state2));
}

bool MaxMinClearanceOptimizationObjective::isSatisfied(ompl::base::Cost /* cost */) const {
  return false;
}

void MaxMinClearanceOptimizationObjective::accept(const ObjectiveVisitor& visitor) const {
  visitor.visit(*this);
}

}  // namespace objectives

}  // namespace pdt
