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

#include "esp_optimization_objectives/reciprocal_clearance_optimization_objective.h"

#include <cmath>
#include <memory>

#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace esp {

namespace ompltools {

ReciprocalClearanceOptimizationObjective::ReciprocalClearanceOptimizationObjective(
  const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
  const std::vector<double>& sampleFactors) :
    ompl::base::StateCostIntegralObjective(spaceInfo, true),
    spaceInfo_(spaceInfo),
    sampleFactors_(sampleFactors) {
  // Optimization objectives have descriptions. (...)
  description_ = "Reciprocal Clearance";
}

ompl::base::Cost ReciprocalClearanceOptimizationObjective::stateCost(
    const ompl::base::State* state) const {
  const auto clearance = spaceInfo_->getStateValidityChecker()->clearance(state);
  if (clearance > 1e-6) {
    return ompl::base::Cost(1.0 / clearance);
  } else {
    // Returning a really (really) high number, like infinity, here is not a good idea. The problem
    // is that this cost is sometimes used for further computation (such as for distance computation
    // in a nearest neighbor struct in FMT*) and overflows/infs/nans can result in segfaults because
    // these cases aren't handled propperly.
    return ompl::base::Cost(1.0 / 1e-6);
  }
}

  ompl::base::Cost ReciprocalClearanceOptimizationObjective::motionCostHeuristic(const ompl::base::State* s1, const ompl::base::State* s2) const {
    // Compute the distance between the states.
    const auto d  = spaceInfo_->distance(s1, s2);
    
    // Allocate state to test clearance.
    auto s = spaceInfo_->allocState();

    // Get the clearance of all samples along edge.
    constexpr auto percent = 0.5;
    const auto segments = spaceInfo_->getStateSpace()->validSegmentCount(s1, s2);
    const auto numSamples = std::max(2u, static_cast<unsigned>(std::ceil(percent * (segments + 1u))));
    std::vector<double> ts(numSamples, 0.0);
    std::vector<double> ds(numSamples, 0.0);
    std::vector<double> cs(numSamples, 0.0);
    for (auto i = 0u; i < numSamples; ++i) {
      ts[i] = 1.0 / static_cast<double>(numSamples) * static_cast<double>(i);
      spaceInfo_->getStateSpace()->interpolate(s1, s2, ts[i], s);
      cs[i] = std::max(spaceInfo_->getStateValidityChecker()->clearance(s), 1e-6);
      ds[i] = d * ts[i];
    }
    
    // Free the allocated state.
    spaceInfo_->freeState(s);
    
    // Compute first term.
    ompl::base::Cost cost(std::log((ds[0u] + cs[0u]) / cs[0u]));

    // Compute the middle terms.
    for (auto i = 1u; i < numSamples - 2u; ++i) {
      const auto seg = ompl::base::Cost(
          std::log(std::pow(cs[i] + cs[i + 1u] - (ds[i] - ds[i + 1u]), 2.0) /
                   (4.0 * cs[i] * cs[i + 1u])));
      cost = combineCosts(cost, seg);
    }

    // add the last term.
    return combineCosts(cost,
                        ompl::base::Cost(std::log((d - ds[numSamples - 1u] + cs[numSamples - 1u]) /
                                                  cs[numSamples - 1u])));
  }

void ReciprocalClearanceOptimizationObjective::accept(const ObjectiveVisitor& visitor) const {
  visitor.visit(*this);
}

}  // namespace ompltools

}  // namespace esp
