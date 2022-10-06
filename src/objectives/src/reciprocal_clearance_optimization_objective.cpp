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

#include "pdt/objectives/reciprocal_clearance_optimization_objective.h"

#include <cmath>
#include <memory>
#include <string>

#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace std::string_literals;

namespace pdt {

namespace objectives {

ReciprocalClearanceOptimizationObjective::ReciprocalClearanceOptimizationObjective(
    const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
    const double heuristicSampleFraction) :
    ompl::base::StateCostIntegralObjective(spaceInfo, true),
    spaceInfo_(spaceInfo),
    heuristicSampleFraction_(heuristicSampleFraction) {
  description_ = "Reciprocal clearance objective that samples "s +
                 std::to_string(heuristicSampleFraction) +
                 " times as many states as necessary to validate an edge to compute" +
                 " an admissible heuristic.";
}

ReciprocalClearanceOptimizationObjective::ReciprocalClearanceOptimizationObjective(
    const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
    const std::vector<double>& heuristicSampleFactors) :
    ompl::base::StateCostIntegralObjective(spaceInfo, true),
    spaceInfo_(spaceInfo),
    heuristicSampleFactors_(heuristicSampleFactors) {
  description_ = "Reciprocal clearance objective that samples "s +
                 std::to_string(heuristicSampleFactors.size()) +
                 " states along each edge to compute an admissible heuristic.";
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

ompl::base::Cost ReciprocalClearanceOptimizationObjective::motionCostHeuristic(
    const ompl::base::State* s1, const ompl::base::State* s2,
    const std::vector<double>& factors) const {
  // Compute the distance between the states.
  const auto d = spaceInfo_->distance(s1, s2);

  // Allocate state to test clearance.
  auto s = spaceInfo_->allocState();

  // Get the clearance of all samples along edge.
  const auto numSamples = factors.size();
  std::vector<double> cs(numSamples, 0.0);
  for (auto i = 0u; i < numSamples; ++i) {
    spaceInfo_->getStateSpace()->interpolate(s1, s2, factors[i], s);
    cs[i] = std::max(spaceInfo_->getStateValidityChecker()->clearance(s), 1e-6);
  }

  // Free the allocated state.
  spaceInfo_->freeState(s);

  // Compute the first half-term, i.e., the component left of the first sample.
  ompl::base::Cost cost(std::log((d * factors[0u] + cs[0u]) / cs[0u]));

  // Compute terms between the samples.
  for (auto i = 0u; i < numSamples - 1u; ++i) {
    const auto seg = ompl::base::Cost(
        std::log(std::pow(cs[i] + cs[i + 1u] + d * (factors[i + 1u] - factors[i]), 2.0) /
                 (4.0 * cs[i] * cs[i + 1u])));
    cost = combineCosts(cost, seg);
  }

  // Add the last half-term, i.e., the component right of the last sample.
  return combineCosts(
      cost, ompl::base::Cost(std::log((d * (1 - factors[numSamples - 1u]) + cs[numSamples - 1u]) /
                                      cs[numSamples - 1u])));
}

ompl::base::Cost ReciprocalClearanceOptimizationObjective::motionCostHeuristic(
    const ompl::base::State* s1, const ompl::base::State* s2) const {
  if (heuristicSampleFraction_ >= 0.0) {
    assert(heuristicSampleFraction_ <= 1.0);
    const auto segments = spaceInfo_->getStateSpace()->validSegmentCount(s1, s2);
    const auto numSamples =
        std::max(2u, static_cast<unsigned>(std::ceil(heuristicSampleFraction_ * (segments + 1u))));
    heuristicSampleFactors_.resize(numSamples);
    std::generate(heuristicSampleFactors_.begin(), heuristicSampleFactors_.end(),
                  [numSamples, i = 0u]() mutable {
                    return static_cast<double>(i++) / static_cast<double>(numSamples - 1u);
                  });
  } else {
    return ompl::base::Cost(0.0);
  }

  return motionCostHeuristic(s1, s2, heuristicSampleFactors_);
}

void ReciprocalClearanceOptimizationObjective::accept(const ObjectiveVisitor& visitor) const {
  visitor.visit(*this);
}

}  // namespace objectives

}  // namespace pdt
