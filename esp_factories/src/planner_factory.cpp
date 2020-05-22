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

#include <ompl/geometric/planners/aitstar/AITstar.h>
#include <ompl/geometric/planners/bitstar/ABITstar.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/eitstar/EITstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>

#include "nlohmann/json.hpp"

namespace esp {

namespace ompltools {

using namespace std::string_literals;

PlannerFactory::PlannerFactory(const std::shared_ptr<Configuration> &config,
                               const std::shared_ptr<BaseContext> &context) :
    config_(config),
    context_(context) {
  if (config_->contains("planner") == 0) {
    throw std::runtime_error("Configuration does not contain planner data.");
  }
}

std::pair<std::shared_ptr<ompl::base::Planner>, PLANNER_TYPE> PlannerFactory::create(
    const std::string &plannerName) const {
  const std::string parentKey{"planner/" + plannerName};
  if (!config_->contains(parentKey)) {
    throw std::invalid_argument("Requested unknown planner '"s + plannerName + "'."s);
  }
  const auto type = config_->get<PLANNER_TYPE>(parentKey + "/type");
  const auto optionsKey = parentKey + "/options"s;
  // BIT*
  switch (type) {
    case PLANNER_TYPE::ABITSTAR: {
      // Allocate and configure an ABIT* planner.
      auto planner = std::make_shared<ompl::geometric::ABITstar>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      planner->setName(plannerName);
      planner->setUseKNearest(config_->get<bool>(optionsKey + "/useKNearest"));
      planner->setRewireFactor(config_->get<double>(optionsKey + "/rewireFactor"));
      planner->setSamplesPerBatch(config_->get<std::size_t>(optionsKey + "/samplesPerBatch"));
      planner->setPruning(config_->get<bool>(optionsKey + "/enablePruning"));
      planner->setPruneThresholdFraction(config_->get<double>(optionsKey + "/pruningThreshold"));
      planner->setDropSamplesOnPrune(config_->get<bool>(optionsKey + "/dropSamplesOnPrune"));
      planner->setJustInTimeSampling(config_->get<bool>(optionsKey + "/useJustInTimeSampling"));
      planner->setStopOnSolnImprovement(
          config_->get<bool>(optionsKey + "/stopOnSolutionImprovement"));
      planner->setInitialInflationFactor(config_->get<double>(optionsKey + "/initialInflation"));
      planner->setInflationScalingParameter(
          config_->get<double>(optionsKey + "/inflationParameter"));
      planner->setTruncationScalingParameter(
          config_->get<double>(optionsKey + "/truncationParameter"));
      return {planner, PLANNER_TYPE::ABITSTAR};
    }
    case PLANNER_TYPE::EITSTAR: {
      // Allocate and configure an EIT* planner.
      auto planner = std::make_shared<ompl::geometric::EITstar>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      planner->setName(plannerName);
      planner->setNumSamplesPerBatch(config_->get<std::size_t>(optionsKey + "/samplesPerBatch"));
      planner->setInitialNumberOfSparseCollisionChecks(
          config_->get<std::size_t>(optionsKey + "/numInitialCollisionChecks"));
      planner->setRadiusFactor(config_->get<double>(optionsKey + "/radiusFactor"));
      planner->setRepairFactor(config_->get<double>(optionsKey + "/repairFactor"));
      planner->enableRepairingReverseTree(
          config_->get<bool>(optionsKey + "/repairReverseSearchTreeUponCollisionDetection"));
      planner->enableCollisionDetectionInReverseSearch(
          config_->get<bool>(optionsKey + "/collisionDetectionOnReverseSearch"));
      return {planner, PLANNER_TYPE::EITSTAR};
    }
    case PLANNER_TYPE::AITSTAR: {
      // Allocate and configure a TBD* planner.
      auto planner = std::make_shared<ompl::geometric::AITstar>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      planner->setName(plannerName);
      planner->setRepairReverseSearch(config_->get<bool>(optionsKey + "/repairBackwardSearch"));
      planner->setBatchSize(config_->get<std::size_t>(optionsKey + "/batchSize"));
      planner->setRewireFactor(config_->get<double>(optionsKey + "/rewireFactor"));
      return {planner, PLANNER_TYPE::AITSTAR};
    }
    case PLANNER_TYPE::BITSTAR: {
      // Allocate and configure a BIT* planner.
      auto planner = std::make_shared<ompl::geometric::BITstar>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      planner->setName(plannerName);
      planner->setUseKNearest(config_->get<bool>(optionsKey + "/useKNearest"));
      planner->setRewireFactor(config_->get<double>(optionsKey + "/rewireFactor"));
      planner->setSamplesPerBatch(config_->get<std::size_t>(optionsKey + "/samplesPerBatch"));
      planner->setPruning(config_->get<bool>(optionsKey + "/enablePruning"));
      planner->setPruneThresholdFraction(config_->get<double>(optionsKey + "/pruningThreshold"));
      planner->setDropSamplesOnPrune(config_->get<bool>(optionsKey + "/dropSamplesOnPrune"));
      planner->setJustInTimeSampling(config_->get<bool>(optionsKey + "/useJustInTimeSampling"));
      planner->setStopOnSolnImprovement(
          config_->get<bool>(optionsKey + "/stopOnSolutionImprovement"));
      return {planner, PLANNER_TYPE::BITSTAR};
    }
    case PLANNER_TYPE::FMTSTAR: {
      // Allocate and configure an FMT* planner.
      auto planner = std::make_shared<ompl::geometric::FMT>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      planner->setName(plannerName);
      planner->setNumSamples(config_->get<std::size_t>(optionsKey + "/numSamples"));
      planner->setNearestK(config_->get<bool>(optionsKey + "/useKNearest"));
      planner->setRadiusMultiplier(config_->get<double>(optionsKey + "/radiusFactor"));
      planner->setCacheCC(config_->get<bool>(optionsKey + "/useCollisionDetectionCache"));
      planner->setHeuristics(config_->get<bool>(optionsKey + "/useHeuristics"));
      planner->setExtendedFMT(config_->get<bool>(optionsKey + "/useMoreSamplesIfUnsuccessful"));
      return {planner, PLANNER_TYPE::FMTSTAR};
    }
    case PLANNER_TYPE::INFORMEDRRTSTAR: {
      auto planner =
          std::make_shared<ompl::geometric::InformedRRTstar>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      planner->setName(plannerName);
      planner->setRewireFactor(config_->get<double>(optionsKey + "/rewireFactor"));
      planner->setNumSamplingAttempts(
          config_->get<std::size_t>(optionsKey + "/numSamplingAttempts"));
      return {planner, PLANNER_TYPE::INFORMEDRRTSTAR};
    }
    case PLANNER_TYPE::RRT: {
      // Allocate and configure an RRT planner.
      auto planner = std::make_shared<ompl::geometric::RRT>(context_->getSpaceInformation());
      auto dimKey = std::to_string(context_->getSpaceInformation()->getStateDimension()) + "d";
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      planner->setName(plannerName);
      planner->setGoalBias(config_->get<double>(optionsKey + "/goalBias"));
      planner->setRange(config_->get<double>(optionsKey + "/maxEdgeLength/" + dimKey));
      planner->setIntermediateStates(config_->get<bool>(optionsKey + "/addIntermediateStates"));
      return {planner, PLANNER_TYPE::RRT};
    }
    case PLANNER_TYPE::RRTCONNECT: {
      // Allocate and configure an RRT-Connect planner.
      auto planner = std::make_shared<ompl::geometric::RRTConnect>(context_->getSpaceInformation());
      auto dimKey = std::to_string(context_->getSpaceInformation()->getStateDimension()) + "d";
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      planner->setName(plannerName);
      planner->setRange(config_->get<double>(optionsKey + "/maxEdgeLength/" + dimKey));
      planner->setIntermediateStates(config_->get<bool>(optionsKey + "/addIntermediateStates"));
      return {planner, PLANNER_TYPE::RRTCONNECT};
    }
    case PLANNER_TYPE::RRTSHARP: {
      // Allocate and configure an RRTSharp planner.
      auto planner = std::make_shared<ompl::geometric::RRTsharp>(context_->getSpaceInformation());
      auto dimKey = std::to_string(context_->getSpaceInformation()->getStateDimension()) + "d";
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      planner->setName(plannerName);
      planner->setKNearest(config_->get<bool>(optionsKey + "/useKNearest"));
      planner->setRange(config_->get<double>(optionsKey + "/maxEdgeLength/" + dimKey));
      planner->setGoalBias(config_->get<double>(optionsKey + "/goalBias"));
      planner->setRewireFactor(config_->get<double>(optionsKey + "/rewireFactor"));
      planner->setSampleRejection(config_->get<bool>(optionsKey + "/enableSampleRejection"));
      planner->setVariant(config_->get<std::size_t>(optionsKey + "/variant"));
      planner->setInformedSampling(false);
      return {planner, PLANNER_TYPE::RRTSHARP};
    }
    case PLANNER_TYPE::RRTSTAR: {
      // Allocate and configure an RRTstar planner.
      auto planner = std::make_shared<ompl::geometric::RRTstar>(context_->getSpaceInformation());
      auto dimKey = std::to_string(context_->getSpaceInformation()->getStateDimension()) + "d";
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      planner->setName(plannerName);
      planner->setKNearest(config_->get<bool>(optionsKey + "/useKNearest"));
      planner->setRange(config_->get<double>(optionsKey + "/maxEdgeLength/" + dimKey));
      planner->setRewireFactor(config_->get<double>(optionsKey + "/rewireFactor"));
      planner->setGoalBias(config_->get<double>(optionsKey + "/goalBias"));
      planner->setDelayCC(config_->get<bool>(optionsKey + "/delayCollisionChecks"));
      planner->setTreePruning(false);
      planner->setPruneThreshold(1.0);
      planner->setPrunedMeasure(false);
      planner->setSampleRejection(false);
      planner->setNewStateRejection(false);
      planner->setInformedSampling(false);
      return {planner, PLANNER_TYPE::RRTSTAR};
    }
    default: { throw std::runtime_error("Planner '"s + plannerName + "' is of unknown type."s); }
  }
}

}  // namespace ompltools

}  // namespace esp
