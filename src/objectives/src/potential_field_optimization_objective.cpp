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

#include "pdt/objectives/potential_field_optimization_objective.h"

#include <memory>

#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace pdt {

namespace objectives {

PotentialFieldOptimizationObjective::PotentialFieldOptimizationObjective(
    const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
    const std::shared_ptr<const config::Configuration> config) :
    ompl::base::StateCostIntegralObjective(spaceInfo, true),
    config_(config),
    spaceInfo_(spaceInfo) {
  // Make sure this is the expexted state space type.
  if (spaceInfo_->getStateSpace()->getType() !=
      ompl::base::StateSpaceType::STATE_SPACE_REAL_VECTOR) {
    throw std::runtime_error(
        "Potential Field Optimization Objective only implemented for real vector state spaces.");
  }

  // Optimization objectives have descriptions. (...)
  description_ = "Potential Field";

  // Load the point sources as specified in the config.
  for (const auto& point : config_->get<std::vector<std::vector<double>>>(
           "objective/" +
           config_->get<std::string>("context/" + config_->get<std::string>("experiment/context") +
                                     "/objective") +
           "/sources")) {
    // allocate a state for each point source.
    pointSources_.push_back(spaceInfo_->allocState());

    // Ensure the point source has the correct dimension.
    if (point.size() != spaceInfo_->getStateDimension()) {
      std::cout << "point.size(): " << point.size()
                << ", dimensions: " << spaceInfo_->getStateDimension() << '\n';
      throw std::runtime_error("Source in potential field objective has wrong dimensionality.");
    }

    // Fill the state with the values from the config.
    for (std::size_t i = 0u; i < point.size(); ++i) {
      pointSources_.back()->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = point[i];
    }
  }

  // // There is no good cost-to-go heuristic for this objective.
  // setCostToGoHeuristic(ompl::base::goalRegionCostToGo);
}

PotentialFieldOptimizationObjective::~PotentialFieldOptimizationObjective() {
  // Free the point sources that were allocated in the constructor.
  for (auto source : pointSources_) {
    spaceInfo_->freeState(source);
  }
}

ompl::base::Cost PotentialFieldOptimizationObjective::stateCost(
    const ompl::base::State* state) const {
  // Sum up all cost of all sources, inversely proportional to the squared distance, capped at 1000.
  ompl::base::Cost totalCost = identityCost();
  for (const auto source : pointSources_) {
    totalCost = combineCosts(
        totalCost, ompl::base::Cost(
                       std::min(1.0 / std::pow(spaceInfo_->distance(source, state), 2.0), 1000.0)));
  }
  return totalCost;
}

ompl::base::Cost PotentialFieldOptimizationObjective::motionCostHeuristic(
    const ompl::base::State* /* state1 */, const ompl::base::State* /* state2 */) const {
  // double resolution = spaceInfo_->getStateValidityCheckingResolution();
  // return combineCosts(ompl::base::Cost(stateCost(state1).value() * resolution),
  //                     ompl::base::Cost(stateCost(state2).value() * resolution));
  return identityCost();
}

void PotentialFieldOptimizationObjective::accept(const ObjectiveVisitor& visitor) const {
  visitor.visit(*this);
}

}  // namespace objectives

}  // namespace pdt
