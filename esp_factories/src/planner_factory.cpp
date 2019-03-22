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

namespace esp_ompl_tools {

// Convenience namespace.
namespace fs = std::experimental::filesystem;

PlannerFactory::PlannerFactory(fs::path plannerConfigFile) {
  // Check the config file exists.
  if (fs::exists(plannerConfigFile)) {
    // It does, load it into the parameters.
    std::ifstream file(plannerConfigFile.string());
    file >> parameters_;
  } else {
    // It does not, the factory is useless, throw.
    throw fs::filesystem_error("File does not exist.", plannerConfigFile, std::error_code());
  }
}

std::shared_ptr<ompl::base::Planner> PlannerFactory::create(
    const std::string& plannerType, const BaseContextPtr &experiment) const {
  // BIT*
  if (plannerType == std::string("BITstar")) {
    // Allocate and configure a BIT* planner.
    auto planner = std::make_shared<ompl::geometric::BITstar>(experiment->getSpaceInformation());
    const auto &plannerParams = parameters_[plannerType];
    planner->setProblemDefinition(experiment->newProblemDefinition());
    planner->setName(plannerParams["name"]);
    planner->setUseKNearest(plannerParams["useKNearest"]);
    planner->setRewireFactor(plannerParams["rewireFactor"]);
    planner->setSamplesPerBatch(plannerParams["samplesPerBatch"]);
    planner->setPruning(plannerParams["enablePruning"]);
    planner->setPruneThresholdFraction(plannerParams["pruningThreshold"]);
    planner->setDropSamplesOnPrune(plannerParams["dropSamplesOnPrune"]);
    planner->setJustInTimeSampling(plannerParams["useJustInTimeSampling"]);
    planner->setStopOnSolnImprovement(plannerParams["stopOnSolutionImprovement"]);
    planner->setInitialInflationFactor(1.0);
    planner->setInflationFactorParameter(0.0);
    planner->setTruncationFactorParameter(0.0);
    return planner;
  } else if (plannerType == std::string("SBITstar")) {
    // Allocate and configure an SBIT* planner.
    auto planner = std::make_shared<ompl::geometric::BITstar>(experiment->getSpaceInformation());
    const auto &plannerParams = parameters_[plannerType];
    planner->setProblemDefinition(experiment->newProblemDefinition());
    planner->setName(plannerParams["name"]);
    planner->setUseKNearest(plannerParams["useKNearest"]);
    planner->setRewireFactor(plannerParams["rewireFactor"]);
    planner->setSamplesPerBatch(plannerParams["samplesPerBatch"]);
    planner->setPruning(plannerParams["enablePruning"]);
    planner->setPruneThresholdFraction(plannerParams["pruningThreshold"]);
    planner->setDropSamplesOnPrune(plannerParams["dropSamplesOnPrune"]);
    planner->setJustInTimeSampling(plannerParams["useJustInTimeSampling"]);
    planner->setStopOnSolnImprovement(plannerParams["stopOnSolutionImprovement"]);
    planner->setInitialInflationFactor(plannerParams["initialInflation"]);
    planner->setInflationFactorParameter(plannerParams["inflationParameter"]);
    planner->setTruncationFactorParameter(plannerParams["truncationParameter"]);
    return planner;
  } else if (plannerType == std::string("RRTConnect")) {
    // Allocate and configure an RRT-Connect planner.
    auto planner = std::make_shared<ompl::geometric::RRTConnect>(experiment->getSpaceInformation());
    auto dimKey = std::to_string(experiment->getDimensions()) + "d";
    const auto &plannerParams = parameters_[plannerType];
    planner->setProblemDefinition(experiment->newProblemDefinition());
    planner->setName(plannerParams["name"]);
    planner->setRange(plannerParams["maxEdgeLength"][dimKey]);
    planner->setIntermediateStates(plannerParams["addIntermediateStates"]);
    return planner;
  } else if (plannerType == std::string("RRTstar")) {
    // Allocate and configure an RRT-Connect planner.
    auto planner = std::make_shared<ompl::geometric::RRTstar>(experiment->getSpaceInformation());
    auto dimKey = std::to_string(experiment->getDimensions()) + "d";
    const auto &plannerParams = parameters_[plannerType];
    planner->setProblemDefinition(experiment->newProblemDefinition());
    planner->setName(plannerParams["name"]);
    planner->setKNearest(plannerParams["useKNearest"]);
    planner->setRange(plannerParams["maxEdgeLength"][dimKey]);
    planner->setGoalBias(plannerParams["goalBias"]);
    planner->setDelayCC(plannerParams["delayCollisionChecks"]);
    planner->setTreePruning(false);
    planner->setPruneThreshold(1.0);
    planner->setPrunedMeasure(false);
    planner->setSampleRejection(false);
    planner->setNewStateRejection(false);
    planner->setAdmissibleCostToCome(true);
    planner->setInformedSampling(false);
    return planner;
  } else {
    throw std::runtime_error("Requested to create unknown planner at factory.");
    return std::make_shared<ompl::geometric::BITstar>(experiment->getSpaceInformation());
  }
}

void PlannerFactory::dumpParameters(std::ostream& out) const {
  out << "Planner Parameters:\n" << parameters_.dump(2) << '\n';
}

}  // namespace esp_ompl_tools
