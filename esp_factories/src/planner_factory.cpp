/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Oxford
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
 *   * Neither the name of the University of Oxford nor the names of its
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

// Author: Marlin Strub

#include "esp_factories/planner_factory.h"

#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>

#include "nlohmann/json.hpp"

namespace esp {

namespace ompltools {

  PlannerFactory::PlannerFactory(const std::shared_ptr<Configuration> &config,
                                 const std::shared_ptr<BaseContext> &context) :
  config_(config),
  context_(context) {
  if (config_->contains("Planners") == 0) {
    throw std::runtime_error("Configuration does not contain planner data.");
  }
}

std::shared_ptr<ompl::base::Planner> PlannerFactory::create(const std::string &plannerType) const {
  // BIT*
  if (plannerType == std::string("BITstar")) {
    // Allocate and configure a BIT* planner.
    auto planner = std::make_shared<ompl::geometric::BITstar>(context_->getSpaceInformation());
    const auto &plannerConfig = config_->getPlannerConfig(plannerType);
    planner->setProblemDefinition(context_->newProblemDefinition());
    planner->setName(plannerConfig["name"]);
    planner->setUseKNearest(plannerConfig["useKNearest"]);
    planner->setRewireFactor(plannerConfig["rewireFactor"]);
    planner->setSamplesPerBatch(plannerConfig["samplesPerBatch"]);
    planner->setPruning(plannerConfig["enablePruning"]);
    planner->setPruneThresholdFraction(plannerConfig["pruningThreshold"]);
    planner->setDropSamplesOnPrune(plannerConfig["dropSamplesOnPrune"]);
    planner->setJustInTimeSampling(plannerConfig["useJustInTimeSampling"]);
    planner->setStopOnSolnImprovement(plannerConfig["stopOnSolutionImprovement"]);
    planner->setInitialInflationFactor(1.0);
    planner->setInflationFactorParameter(0.0);
    planner->setTruncationFactorParameter(0.0);
    return planner;
  } else if (plannerType == std::string("SBITstar")) {
    // Allocate and configure an SBIT* planner.
    auto planner = std::make_shared<ompl::geometric::BITstar>(context_->getSpaceInformation());
    const auto &plannerConfig = config_->getPlannerConfig(plannerType);
    planner->setProblemDefinition(context_->newProblemDefinition());
    planner->setName(plannerConfig["name"]);
    planner->setUseKNearest(plannerConfig["useKNearest"]);
    planner->setRewireFactor(plannerConfig["rewireFactor"]);
    planner->setSamplesPerBatch(plannerConfig["samplesPerBatch"]);
    planner->setPruning(plannerConfig["enablePruning"]);
    planner->setPruneThresholdFraction(plannerConfig["pruningThreshold"]);
    planner->setDropSamplesOnPrune(plannerConfig["dropSamplesOnPrune"]);
    planner->setJustInTimeSampling(plannerConfig["useJustInTimeSampling"]);
    planner->setStopOnSolnImprovement(plannerConfig["stopOnSolutionImprovement"]);
    planner->setInitialInflationFactor(plannerConfig["initialInflation"]);
    planner->setInflationFactorParameter(plannerConfig["inflationParameter"]);
    planner->setTruncationFactorParameter(plannerConfig["truncationParameter"]);
    return planner;
  } else if (plannerType == std::string("RRTConnect")) {
    // Allocate and configure an RRT-Connect planner.
    auto planner = std::make_shared<ompl::geometric::RRTConnect>(context_->getSpaceInformation());
    auto dimKey = std::to_string(context_->getDimensions()) + "d";
    const auto &plannerConfig = config_->getPlannerConfig(plannerType);
    planner->setProblemDefinition(context_->newProblemDefinition());
    planner->setName(plannerConfig["name"]);
    planner->setRange(plannerConfig["maxEdgeLength"][dimKey]);
    planner->setIntermediateStates(plannerConfig["addIntermediateStates"]);
    return planner;
  } else if (plannerType == std::string("RRTstar")) {
    // Allocate and configure an RRT-Connect planner.
    auto planner = std::make_shared<ompl::geometric::RRTstar>(context_->getSpaceInformation());
    auto dimKey = std::to_string(context_->getDimensions()) + "d";
    const auto &plannerConfig = config_->getPlannerConfig(plannerType);
    planner->setProblemDefinition(context_->newProblemDefinition());
    planner->setName(plannerConfig["name"]);
    planner->setKNearest(plannerConfig["useKNearest"]);
    planner->setRange(plannerConfig["maxEdgeLength"][dimKey]);
    planner->setGoalBias(plannerConfig["goalBias"]);
    planner->setDelayCC(plannerConfig["delayCollisionChecks"]);
    planner->setTreePruning(false);
    planner->setPruneThreshold(1.0);
    planner->setPrunedMeasure(false);
    planner->setSampleRejection(false);
    planner->setNewStateRejection(false);
    planner->setAdmissibleCostToCome(true);
    planner->setInformedSampling(false);
    return planner;
  } else if (plannerType == std::string("LBTRRT")) {
    // Allocate and configure an LBTRRT planner.
    auto planner = std::make_shared<ompl::geometric::LBTRRT>(context_->getSpaceInformation());
    auto dimKey = std::to_string(context_->getDimensions()) + "d";
    const auto &plannerConfig = config_->getPlannerConfig(plannerType);
    planner->setProblemDefinition(context_->newProblemDefinition());
    planner->setName(plannerConfig["name"]);
    planner->setRange(plannerConfig["maxEdgeLength"][dimKey]);
    planner->setGoalBias(plannerConfig["goalBias"]);
    planner->setApproximationFactor(plannerConfig["approximationFactor"]);
    return planner;
  } else {
    throw std::runtime_error("Requested to create unknown planner at factory.");
    return std::make_shared<ompl::geometric::BITstar>(context_->getSpaceInformation());
  }
}

}  // namespace ompltools

}  // namespace esp
