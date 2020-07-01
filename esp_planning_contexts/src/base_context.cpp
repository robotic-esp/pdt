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

#include "esp_planning_contexts/base_context.h"

#include <ompl/base/goals/GoalSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include "esp_common/goal_type.h"
#include "esp_common/objective_type.h"
#include "esp_optimization_objectives/max_min_clearance_optimization_objective.h"
#include "esp_optimization_objectives/potential_field_optimization_objective.h"
#include "esp_optimization_objectives/reciprocal_clearance_optimization_objective.h"

using namespace std::string_literals;

namespace esp {

namespace ompltools {

BaseContext::BaseContext(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                         const std::shared_ptr<const Configuration>& config,
                         const std::string& name) :
    spaceInfo_(spaceInfo),
    dimensionality_(spaceInfo_->getStateDimension()),
    name_(name),
    maxSolveDuration_(time::seconds(config->get<double>("context/" + name + "/maxTime"))),
    config_(config) {
  // Get the optimization objective.
  switch (config_->get<OBJECTIVE_TYPE>(
      "objective/" + config_->get<std::string>("context/" + name_ + "/objective") + "/type")) {
    case OBJECTIVE_TYPE::COSTMAP: {
      throw std::runtime_error("CostMap objective is not yet implemented.");
      break;
    }
    case OBJECTIVE_TYPE::MAXMINCLEARANCE: {
      objective_ = std::make_shared<MaxMinClearanceOptimizationObjective>(spaceInfo_);
      objective_->setCostToGoHeuristic([this](const ompl::base::State*, const ompl::base::Goal*) {
        return objective_->identityCost();
      });
      break;
    }
    case OBJECTIVE_TYPE::RECIPROCALCLEARANCE: {
      objective_ = std::make_shared<ReciprocalClearanceOptimizationObjective>(spaceInfo_);
      objective_->setCostToGoHeuristic([this](const ompl::base::State*, const ompl::base::Goal*) {
        return objective_->identityCost();
      });
      break;
    }
    case OBJECTIVE_TYPE::PATHLENGTH: {
      objective_ = std::make_shared<ompl::base::PathLengthOptimizationObjective>(spaceInfo_);
      objective_->setCostToGoHeuristic(&ompl::base::goalRegionCostToGo);
      break;
    }
    case OBJECTIVE_TYPE::POTENTIALFIELD: {
      objective_ = std::make_shared<PotentialFieldOptimizationObjective>(spaceInfo_, config_);
      objective_->setCostToGoHeuristic([this](const ompl::base::State*, const ompl::base::Goal*) {
        return objective_->identityCost();
      });
      break;
    }
    case OBJECTIVE_TYPE::INVALID: {
      throw std::runtime_error("Invalid optimization objective.");
      break;
    }
    default:
      throw std::runtime_error("Unknown optimization objective.");
  }

  // Get the goal.
  auto goalType = config_->get<std::string>("context/" + name_ + "/goalType");
  if (goalType == "GoalState"s) {
    goalType_ = ompl::base::GoalType::GOAL_STATE;
  } else if (goalType == "GoalStates"s) {
    goalType_ = ompl::base::GoalType::GOAL_STATES;
  } else if (goalType == "GoalSpace"s) {
    goalType_ = ompl::base::GoalType::GOAL_SPACE;
  } else {
    throw std::runtime_error("Invalid goal type.");
  }

  // // Why doesn't this work?
  // goalType_ = config_->get<ompl::base::GoalType>("context/" + name_ + "/goalType");
}

std::string BaseContext::getName() const {
  return name_;
}

std::shared_ptr<ompl::base::SpaceInformation> BaseContext::getSpaceInformation() const {
  return spaceInfo_;
}

std::shared_ptr<ompl::base::StateSpace> BaseContext::getStateSpace() const {
  return spaceInfo_->getStateSpace();
}

std::size_t BaseContext::getDimension() const {
  return spaceInfo_->getStateDimension();
}

ompl::base::OptimizationObjectivePtr BaseContext::getObjective() const {
  return objective_;
}

time::Duration BaseContext::getMaxSolveDuration() const {
  return maxSolveDuration_;
}

}  // namespace ompltools

}  // namespace esp
