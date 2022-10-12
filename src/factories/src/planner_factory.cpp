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

// Author: Marlin Strub

#include "pdt/factories/planner_factory.h"

#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>

#ifdef PDT_EXTRA_EITSTAR_PR
#include <ompl/geometric/planners/informedtrees/EIRMstar.h>
#include <ompl/geometric/planners/informedtrees/EITstar.h>
#endif  // #ifdef PDT_EXTRA_EITSTAR_PR

#include "pdt/time/CumulativeTimer.h"

#include "nlohmann/json.hpp"

namespace pdt {

namespace factories {

using namespace std::string_literals;

PlannerFactory::PlannerFactory(const std::shared_ptr<config::Configuration> &config,
                               const std::shared_ptr<planning_contexts::BaseContext> &context) :
    config_(config),
    context_(context) {
  if (config_->contains("planner") == 0) {
    throw std::runtime_error("Configuration does not contain planner data.");
  }
}

std::tuple<std::shared_ptr<ompl::base::Planner>, common::PLANNER_TYPE, time::Duration>
PlannerFactory::create(const std::string &plannerName) const {
  const std::string parentKey{"planner/" + plannerName};
  if (!config_->contains(parentKey)) {
    throw std::invalid_argument("Requested unknown planner '"s + plannerName + "'."s);
  }
  const auto type = config_->get<common::PLANNER_TYPE>(parentKey + "/type");
  const auto optionsKey = parentKey + "/options"s;
  time::CumulativeTimer createTimer;

  switch (type) {
    case common::PLANNER_TYPE::ABITSTAR: {
      // Allocate and configure an ABIT* planner.
      createTimer.start();
      auto planner = std::make_shared<ompl::geometric::ABITstar>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      createTimer.stop();
      planner->setName(plannerName);
      planner->setUseKNearest(config_->get<bool>(optionsKey + "/useKNearest"));
      planner->setRewireFactor(config_->get<double>(optionsKey + "/rewireFactor"));
      planner->setSamplesPerBatch(config_->get<unsigned>(optionsKey + "/samplesPerBatch"));
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
      return {planner, common::PLANNER_TYPE::ABITSTAR, createTimer.duration()};
    }
    case common::PLANNER_TYPE::AITSTAR: {
      // Allocate and configure a AIT* planner.
      createTimer.start();
      auto planner = std::make_shared<ompl::geometric::AITstar>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      createTimer.stop();
      planner->setName(plannerName);
      planner->enablePruning(config_->get<bool>(optionsKey + "/enablePruning"));
      planner->setBatchSize(config_->get<std::size_t>(optionsKey + "/batchSize"));
      planner->setUseKNearest(config_->get<bool>(optionsKey + "/useKNearest"));
      planner->setRewireFactor(config_->get<double>(optionsKey + "/rewireFactor"));
      planner->trackApproximateSolutions(
          config_->get<bool>(optionsKey + "/trackApproximateSolutions"));
      return {planner, common::PLANNER_TYPE::AITSTAR, createTimer.duration()};
    }
    case common::PLANNER_TYPE::BITSTAR: {
      // Allocate and configure a BIT* planner.
      createTimer.start();
      auto planner = std::make_shared<ompl::geometric::BITstar>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      createTimer.stop();
      planner->setName(plannerName);
      planner->setUseKNearest(config_->get<bool>(optionsKey + "/useKNearest"));
      planner->setRewireFactor(config_->get<double>(optionsKey + "/rewireFactor"));
      planner->setSamplesPerBatch(config_->get<unsigned>(optionsKey + "/samplesPerBatch"));
      planner->setPruning(config_->get<bool>(optionsKey + "/enablePruning"));
      planner->setPruneThresholdFraction(config_->get<double>(optionsKey + "/pruningThreshold"));
      planner->setDropSamplesOnPrune(config_->get<bool>(optionsKey + "/dropSamplesOnPrune"));
      planner->setJustInTimeSampling(config_->get<bool>(optionsKey + "/useJustInTimeSampling"));
      planner->setStopOnSolnImprovement(
          config_->get<bool>(optionsKey + "/stopOnSolutionImprovement"));
      return {planner, common::PLANNER_TYPE::BITSTAR, createTimer.duration()};
    }
#ifdef PDT_EXTRA_EITSTAR_PR
    case common::PLANNER_TYPE::EIRMSTAR: {
      // Allocate and configure an EIRM* planner.
      createTimer.start();
      auto planner = std::make_shared<ompl::geometric::EIRMstar>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      createTimer.stop();
      planner->setName(plannerName);
      planner->setStartGoalPruningThreshold(
          config_->get<unsigned>(optionsKey + "/startGoalPruningThreshold"));
      planner->enablePruning(config_->get<bool>(optionsKey + "/enablePruning"));
      planner->setBatchSize(config_->get<unsigned>(optionsKey + "/batchSize"));
      planner->setInitialNumberOfSparseCollisionChecks(
          config_->get<std::size_t>(optionsKey + "/numInitialCollisionChecks"));
      planner->setRadiusFactor(config_->get<double>(optionsKey + "/radiusFactor"));
      planner->setUseKNearest(config_->get<bool>(optionsKey + "/useKNearest"));
      planner->trackApproximateSolutions(
          config_->get<bool>(optionsKey + "/trackApproximateSolutions"));
      return {planner, common::PLANNER_TYPE::EIRMSTAR, createTimer.duration()};
    }
    case common::PLANNER_TYPE::EITSTAR: {
      // Allocate and configure an EIT* planner.
      createTimer.start();
      auto planner = std::make_shared<ompl::geometric::EITstar>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      createTimer.stop();
      planner->setName(plannerName);
      planner->enablePruning(config_->get<bool>(optionsKey + "/enablePruning"));
      planner->setBatchSize(config_->get<unsigned>(optionsKey + "/batchSize"));
      planner->setInitialNumberOfSparseCollisionChecks(
          config_->get<std::size_t>(optionsKey + "/numInitialCollisionChecks"));
      planner->setRadiusFactor(config_->get<double>(optionsKey + "/radiusFactor"));
      planner->setUseKNearest(config_->get<bool>(optionsKey + "/useKNearest"));
      planner->trackApproximateSolutions(
          config_->get<bool>(optionsKey + "/trackApproximateSolutions"));
      return {planner, common::PLANNER_TYPE::EITSTAR, createTimer.duration()};
    }
#endif  // #ifdef PDT_EXTRA_EITSTAR_PR
    case common::PLANNER_TYPE::FMTSTAR: {
      // Allocate and configure an FMT* planner.
      createTimer.start();
      auto planner = std::make_shared<ompl::geometric::FMT>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      createTimer.stop();
      planner->setName(plannerName);
      planner->setNumSamples(config_->get<unsigned>(optionsKey + "/numSamples"));
      planner->setNearestK(config_->get<bool>(optionsKey + "/useKNearest"));
      planner->setRadiusMultiplier(config_->get<double>(optionsKey + "/radiusFactor"));
      planner->setCacheCC(config_->get<bool>(optionsKey + "/useCollisionDetectionCache"));
      planner->setHeuristics(config_->get<bool>(optionsKey + "/useHeuristics"));
      planner->setExtendedFMT(config_->get<bool>(optionsKey + "/useMoreSamplesIfUnsuccessful"));
      return {planner, common::PLANNER_TYPE::FMTSTAR, createTimer.duration()};
    }
    case common::PLANNER_TYPE::INFORMEDRRTSTAR: {
      // Allocate and configure an Informed RRT* planner.
      auto dimKey = std::to_string(context_->getSpaceInformation()->getStateDimension()) + "d";
      createTimer.start();
      auto planner =
          std::make_shared<ompl::geometric::InformedRRTstar>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      createTimer.stop();
      planner->setName(plannerName);
      planner->setKNearest(config_->get<bool>(optionsKey + "/useKNearest"));
      planner->setGoalBias(config_->get<double>(optionsKey + "/goalBias"));
      planner->setRange(config_->get<double>(optionsKey + "/maxEdgeLength/" + dimKey));
      planner->setRewireFactor(config_->get<double>(optionsKey + "/rewireFactor"));
      planner->setNumSamplingAttempts(config_->get<unsigned>(optionsKey + "/numSamplingAttempts"));
      return {planner, common::PLANNER_TYPE::INFORMEDRRTSTAR, createTimer.duration()};
    }
    case common::PLANNER_TYPE::LAZYPRMSTAR: {
      // Allocate and configure a Lazy PRM* planner.
      createTimer.start();
      auto planner =
          std::make_shared<ompl::geometric::LazyPRMstar>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      createTimer.stop();
      planner->setName(plannerName);
      return {planner, common::PLANNER_TYPE::LAZYPRMSTAR, createTimer.duration()};
    }
    case common::PLANNER_TYPE::LBTRRT: {
      // Allocate and configure an LBTRRT planner.
      auto dimKey = std::to_string(context_->getSpaceInformation()->getStateDimension()) + "d";
      createTimer.start();
      auto planner = std::make_shared<ompl::geometric::LBTRRT>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      createTimer.stop();
      planner->setName(plannerName);
      planner->setGoalBias(config_->get<double>(optionsKey + "/goalBias"));
      planner->setRange(config_->get<double>(optionsKey + "/maxEdgeLength/" + dimKey));
      planner->setApproximationFactor(config_->get<double>(optionsKey + "/approximationFactor"));
      return {planner, common::PLANNER_TYPE::LBTRRT, createTimer.duration()};
    }
    case common::PLANNER_TYPE::PRMSTAR: {
      // Allocate and configure a PRM* planner.
      createTimer.start();
      auto planner = std::make_shared<ompl::geometric::PRMstar>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      createTimer.stop();
      planner->setName(plannerName);
      return {planner, common::PLANNER_TYPE::PRMSTAR, createTimer.duration()};
    }
    case common::PLANNER_TYPE::RRT: {
      // Allocate and configure an RRT planner.
      auto dimKey = std::to_string(context_->getSpaceInformation()->getStateDimension()) + "d";
      createTimer.start();
      auto planner = std::make_shared<ompl::geometric::RRT>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      createTimer.stop();
      planner->setName(plannerName);
      planner->setGoalBias(config_->get<double>(optionsKey + "/goalBias"));
      planner->setRange(config_->get<double>(optionsKey + "/maxEdgeLength/" + dimKey));
      planner->setIntermediateStates(config_->get<bool>(optionsKey + "/addIntermediateStates"));
      return {planner, common::PLANNER_TYPE::RRT, createTimer.duration()};
    }
    case common::PLANNER_TYPE::RRTCONNECT: {
      // Allocate and configure an RRT-Connect planner.
      auto dimKey = std::to_string(context_->getSpaceInformation()->getStateDimension()) + "d";
      createTimer.start();
      auto planner = std::make_shared<ompl::geometric::RRTConnect>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      createTimer.stop();
      planner->setName(plannerName);
      planner->setRange(config_->get<double>(optionsKey + "/maxEdgeLength/" + dimKey));
      planner->setIntermediateStates(config_->get<bool>(optionsKey + "/addIntermediateStates"));
      return {planner, common::PLANNER_TYPE::RRTCONNECT, createTimer.duration()};
    }
    case common::PLANNER_TYPE::RRTSHARP: {
      // Allocate and configure an RRTSharp planner.
      auto dimKey = std::to_string(context_->getSpaceInformation()->getStateDimension()) + "d";
      createTimer.start();
      auto planner = std::make_shared<ompl::geometric::RRTsharp>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      createTimer.stop();
      planner->setName(plannerName);
      planner->setKNearest(config_->get<bool>(optionsKey + "/useKNearest"));
      planner->setRange(config_->get<double>(optionsKey + "/maxEdgeLength/" + dimKey));
      planner->setGoalBias(config_->get<double>(optionsKey + "/goalBias"));
      planner->setRewireFactor(config_->get<double>(optionsKey + "/rewireFactor"));
      planner->setSampleRejection(config_->get<bool>(optionsKey + "/enableSampleRejection"));
      planner->setVariant(config_->get<int>(optionsKey + "/variant"));
      planner->setInformedSampling(false);
      return {planner, common::PLANNER_TYPE::RRTSHARP, createTimer.duration()};
    }
    case common::PLANNER_TYPE::RRTSTAR: {
      // Allocate and configure an RRTstar planner.
      auto dimKey = std::to_string(context_->getSpaceInformation()->getStateDimension()) + "d";
      createTimer.start();
      auto planner = std::make_shared<ompl::geometric::RRTstar>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      createTimer.stop();
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
      return {planner, common::PLANNER_TYPE::RRTSTAR, createTimer.duration()};
    }
    case common::PLANNER_TYPE::SPARSTWO: {
      // Allocate and configure an SPARS2 planner.
      createTimer.start();
      auto planner = std::make_shared<ompl::geometric::SPARStwo>(context_->getSpaceInformation());
      planner->setProblemDefinition(context_->instantiateNewProblemDefinition());
      createTimer.stop();
      planner->setName(plannerName);
      planner->setStretchFactor(config_->get<double>(optionsKey + "/stretchFactor"));
      planner->setSparseDeltaFraction(config_->get<double>(optionsKey + "/sparseDeltaFraction"));
      planner->setDenseDeltaFraction(config_->get<double>(optionsKey + "/denseDeltaFraction"));
      planner->setMaxFailures(config_->get<unsigned>(optionsKey + "/maxFailures"));
      return {planner, common::PLANNER_TYPE::SPARSTWO, createTimer.duration()};
    }
    default: { throw std::runtime_error("Planner '"s + plannerName + "' is of unknown type."s); }
  }
}

}  // namespace factories

}  // namespace pdt
