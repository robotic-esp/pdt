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
#include <ompl/geometric/planners/bitstar_regression/BITstarRegression.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/tbdstar/TBDstar.h>

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

std::pair<std::shared_ptr<ompl::base::Planner>, PLANNER_TYPE> PlannerFactory::create(
    const std::string &plannerName) const {
  const std::string parentKey{"Planners/" + plannerName};
  if (!config_->contains(parentKey)) {
    OMPL_ERROR("Configuration has no entry for requested planner '%s'.", parentKey.c_str());
  }
  const auto type = config_->get<PLANNER_TYPE>(parentKey + "/type");
  // BIT*
  switch (type) {
    case PLANNER_TYPE::BITSTAR: {
      // Allocate and configure a BIT* planner.
      auto planner = std::make_shared<ompl::geometric::BITstar>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->newProblemDefinition());
      planner->setName(plannerName);
      planner->setUseKNearest(config_->get<bool>(parentKey + "/useKNearest"));
      planner->setRewireFactor(config_->get<double>(parentKey + "/rewireFactor"));
      planner->setSamplesPerBatch(config_->get<std::size_t>(parentKey + "/samplesPerBatch"));
      planner->setPruning(config_->get<bool>(parentKey + "/enablePruning"));
      planner->setPruneThresholdFraction(config_->get<double>(parentKey + "/pruningThreshold"));
      planner->setDropSamplesOnPrune(config_->get<bool>(parentKey + "/dropSamplesOnPrune"));
      planner->setJustInTimeSampling(config_->get<bool>(parentKey + "/useJustInTimeSampling"));
      planner->setStopOnSolnImprovement(
          config_->get<bool>(parentKey + "/stopOnSolutionImprovement"));
      planner->setInitialInflationFactor(1.0);
      planner->setInflationFactorParameter(0.0);
      planner->setTruncationFactorParameter(0.0);
      return {planner, PLANNER_TYPE::BITSTAR};
    }
    case PLANNER_TYPE::BITSTARREGRESSION: {
      // Allocate and configure a BIT* Regression planner.
      auto planner =
          std::make_shared<ompl::geometric::BITstarRegression>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->newProblemDefinition());
      planner->setName(plannerName);
      planner->setUseKNearest(config_->get<bool>(parentKey + "/useKNearest"));
      planner->setRewireFactor(config_->get<double>(parentKey + "/rewireFactor"));
      planner->setSamplesPerBatch(config_->get<std::size_t>(parentKey + "/samplesPerBatch"));
      planner->setPruning(config_->get<bool>(parentKey + "/enablePruning"));
      planner->setPruneThresholdFraction(config_->get<double>(parentKey + "/pruningThreshold"));
      planner->setDropSamplesOnPrune(config_->get<bool>(parentKey + "/dropSamplesOnPrune"));
      planner->setJustInTimeSampling(config_->get<bool>(parentKey + "/useJustInTimeSampling"));
      planner->setStopOnSolnImprovement(
          config_->get<bool>(parentKey + "/stopOnSolutionImprovement"));
      return {planner, PLANNER_TYPE::BITSTARREGRESSION};
    }
    case PLANNER_TYPE::RRTCONNECT: {
      // Allocate and configure an RRT-Connect planner.
      auto planner = std::make_shared<ompl::geometric::RRTConnect>(context_->getSpaceInformation());
      auto dimKey = std::to_string(context_->getDimensions()) + "d";
      planner->setProblemDefinition(context_->newProblemDefinition());
      planner->setName(plannerName);
      planner->setRange(config_->get<double>(parentKey + "/maxEdgeLength/" + dimKey));
      planner->setIntermediateStates(config_->get<bool>(parentKey + "/addIntermediateStates"));
      return {planner, PLANNER_TYPE::RRTCONNECT};
    }
    case PLANNER_TYPE::RRTSHARP: {
      // Allocate and configure an RRTSharp planner.
      auto planner = std::make_shared<ompl::geometric::RRTsharp>(context_->getSpaceInformation());
      auto dimKey = std::to_string(context_->getDimensions()) + "d";
      planner->setProblemDefinition(context_->newProblemDefinition());
      planner->setName(plannerName);
      planner->setKNearest(config_->get<bool>(parentKey + "/useKNearest"));
      planner->setRange(config_->get<double>(parentKey + "/maxEdgeLength/" + dimKey));
      planner->setGoalBias(config_->get<double>(parentKey + "/goalBias"));
      planner->setSampleRejection(config_->get<bool>(parentKey + "/enableSampleRejection"));
      planner->setVariant(config_->get<std::size_t>(parentKey + "/variant"));
      planner->setInformedSampling(false);
      return {planner, PLANNER_TYPE::RRTSHARP};
    }
    case PLANNER_TYPE::RRTSTAR: {
      // Allocate and configure an RRTstar planner.
      auto planner = std::make_shared<ompl::geometric::RRTstar>(context_->getSpaceInformation());
      auto dimKey = std::to_string(context_->getDimensions()) + "d";
      planner->setProblemDefinition(context_->newProblemDefinition());
      planner->setName(plannerName);
      planner->setKNearest(config_->get<bool>(parentKey + "/useKNearest"));
      planner->setRange(config_->get<double>(parentKey + "/maxEdgeLength/" + dimKey));
      planner->setGoalBias(config_->get<double>(parentKey + "/goalBias"));
      planner->setDelayCC(config_->get<bool>(parentKey + "/delayCollisionChecks"));
      planner->setTreePruning(false);
      planner->setPruneThreshold(1.0);
      planner->setPrunedMeasure(false);
      planner->setSampleRejection(false);
      planner->setNewStateRejection(false);
      planner->setInformedSampling(false);
      return {planner, PLANNER_TYPE::RRTSTAR};
    }
    case PLANNER_TYPE::SBITSTAR: {
      // Allocate and configure an SBIT* planner.
      auto planner = std::make_shared<ompl::geometric::BITstar>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->newProblemDefinition());
      planner->setName(plannerName);
      planner->setUseKNearest(config_->get<bool>(parentKey + "/useKNearest"));
      planner->setRewireFactor(config_->get<double>(parentKey + "/rewireFactor"));
      planner->setSamplesPerBatch(config_->get<std::size_t>(parentKey + "/samplesPerBatch"));
      planner->setPruning(config_->get<bool>(parentKey + "/enablePruning"));
      planner->setPruneThresholdFraction(config_->get<double>(parentKey + "/pruningThreshold"));
      planner->setDropSamplesOnPrune(config_->get<bool>(parentKey + "/dropSamplesOnPrune"));
      planner->setJustInTimeSampling(config_->get<bool>(parentKey + "/useJustInTimeSampling"));
      planner->setStopOnSolnImprovement(
          config_->get<bool>(parentKey + "/stopOnSolutionImprovement"));
      planner->setInitialInflationFactor(config_->get<double>(parentKey + "/initialInflation"));
      planner->setInflationFactorParameter(config_->get<double>(parentKey + "/inflationParameter"));
      planner->setTruncationFactorParameter(
          config_->get<double>(parentKey + "/truncationParameter"));
      return {planner, PLANNER_TYPE::SBITSTAR};
    }
    case PLANNER_TYPE::TBDSTAR: {
      // Allocate and configure a TBD* planner.
      auto planner = std::make_shared<ompl::geometric::TBDstar>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->newProblemDefinition());
      planner->setName(plannerName);
      return {planner, PLANNER_TYPE::TBDSTAR};
    }
    default: {
      OMPL_ERROR("Planner factory recieved request to create planner of unknown type '%s'.",
                 config_->get<std::string>("Planner/" + plannerName + "/type"));
      throw std::runtime_error("Planner factory error.");
      return {std::make_shared<ompl::geometric::BITstar>(context_->getSpaceInformation()),
              PLANNER_TYPE::BITSTAR};
    }
  }
}

}  // namespace ompltools

}  // namespace esp
