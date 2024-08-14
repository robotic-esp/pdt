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

#include "pdt/planning_contexts/base_context.h"

#include <ompl/base/goals/GoalSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "pdt/common/goal_type.h"
#include "pdt/common/objective_type.h"
#include "pdt/objectives/max_min_clearance_optimization_objective.h"
#include "pdt/objectives/potential_field_optimization_objective.h"
#include "pdt/objectives/reciprocal_clearance_optimization_objective.h"

using namespace std::string_literals;

namespace pdt {

namespace planning_contexts {

BaseContext::BaseContext(const std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo,
                         const std::shared_ptr<const config::Configuration> &config,
                         const std::string &name) :
    spaceInfo_(spaceInfo),
    dimensionality_(spaceInfo_->getStateDimension()),
    name_(name),
    maxSolveDuration_(time::seconds(config->get<double>("context/" + name + "/maxTime"))),
    config_(config) {
  // Construct the parent key.
  const auto parentKey =
      "objective/"s + config_->get<std::string>("context/" + name_ + "/objective");

  // Get the optimization objective.
  switch (config_->get<common::OBJECTIVE_TYPE>(parentKey + "/type")) {
    case common::OBJECTIVE_TYPE::COSTMAP: {
      throw std::runtime_error("CostMap objective is not yet implemented.");
      break;
    }
    case common::OBJECTIVE_TYPE::MAXMINCLEARANCE: {
      objective_ = std::make_shared<objectives::MaxMinClearanceOptimizationObjective>(spaceInfo_);
      objective_->setCostThreshold(
          ompl::base::Cost(config_->get<double>(parentKey + "/solvedCost")));
      objective_->setCostToGoHeuristic([this](const ompl::base::State *, const ompl::base::Goal *) {
        return objective_->identityCost();
      });
      break;
    }
    case common::OBJECTIVE_TYPE::RECIPROCALCLEARANCE: {
      if (config_->get<std::string>(parentKey + "/heuristicType") == "fraction"s) {
        const auto fraction = config_->get<double>(parentKey + "/heuristicFraction");
        objective_ = std::make_shared<objectives::ReciprocalClearanceOptimizationObjective>(
            spaceInfo_, fraction);
      } else if (config_->get<std::string>(parentKey + "/heuristicType") == "factors"s) {
        const auto factors = config_->get<std::vector<double>>(parentKey + "/heuristicFactors");
        objective_ = std::make_shared<objectives::ReciprocalClearanceOptimizationObjective>(
            spaceInfo_, factors);
      } else {
        throw std::runtime_error("Unknown heuristic type for reciprocal clearance objective.");
      }
      objective_->setCostThreshold(
          ompl::base::Cost(config_->get<double>(parentKey + "/solvedCost")));
      objective_->setCostToGoHeuristic([this](const ompl::base::State *, const ompl::base::Goal *) {
        return objective_->identityCost();
      });
      break;
    }
    case common::OBJECTIVE_TYPE::PATHLENGTH: {
      objective_ = std::make_shared<ompl::base::PathLengthOptimizationObjective>(spaceInfo_);
      objective_->setCostThreshold(
          ompl::base::Cost(config_->get<double>(parentKey + "/solvedCost")));
      objective_->setCostToGoHeuristic(&ompl::base::goalRegionCostToGo);
      break;
    }
    case common::OBJECTIVE_TYPE::POTENTIALFIELD: {
      objective_ =
          std::make_shared<objectives::PotentialFieldOptimizationObjective>(spaceInfo_, config_);
      objective_->setCostThreshold(
          ompl::base::Cost(config_->get<double>(parentKey + "/solvedCost")));
      objective_->setCostToGoHeuristic([this](const ompl::base::State *, const ompl::base::Goal *) {
        return objective_->identityCost();
      });
      break;
    }
    case common::OBJECTIVE_TYPE::INVALID: {
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
  }
#ifdef PDT_EXTRA_GOAL_SPACE
  else if (goalType == "GoalSpace"s) {
    goalType_ = ompl::base::GoalType::GOAL_SPACE;
  }
#endif  // #ifdef PDT_EXTRA_GOAL_SPACE
  else {
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
  return dimensionality_;
}

ompl::base::OptimizationObjectivePtr BaseContext::getObjective() const {
  return objective_;
}

time::Duration BaseContext::getMaxSolveDuration() const {
  return maxSolveDuration_;
}

StartGoalPair BaseContext::getNthStartGoalPair(std::size_t n) const {
  if (n >= getNumQueries()) {
    throw std::runtime_error("Query number out of bounds.");
  }
  return startGoalPairs_[n];
}

std::size_t BaseContext::getNumQueries() const {
  return startGoalPairs_.size();
}

ompl::base::ProblemDefinitionPtr BaseContext::instantiateNewProblemDefinition() const {
  return instantiateNthProblemDefinition(0u);
}

ompl::base::ProblemDefinitionPtr BaseContext::instantiateNthProblemDefinition(
    const std::size_t n) const {
  if (n >= getNumQueries()) {
    throw std::runtime_error("Query number out of bounds.");
  }

  // Instantiate a new problem definition.
  auto problemDefinition = std::make_shared<ompl::base::ProblemDefinition>(spaceInfo_);

  // Set the objective.
  problemDefinition->setOptimizationObjective(objective_);

  // Set the start state in the problem definition.
  for (auto s : startGoalPairs_[n].start) {
    problemDefinition->addStartState(s);
  }

  // Set the goal for the problem definition.
  problemDefinition->setGoal(startGoalPairs_[n].goal);

  // Return the new definition.
  return problemDefinition;
}

std::vector<ompl::base::ScopedState<>> BaseContext::parseSpecifiedStates(
    const std::string &key) const {
  if (!config_->contains(key)) {
    throw std::runtime_error("No states specified.");
  }
  std::vector<ompl::base::ScopedState<>> states;

  const auto positions = config_->get<std::vector<std::vector<double>>>(key);

  for (const auto &s : positions) {
    // ensure that the dimensionality works
    if (s.size() != dimensionality_) {
      OMPL_ERROR("%s: Dimensionality of problem and of state specification does not match.",
                 name_.c_str());
      throw std::runtime_error("Context error.");
    }

    // Allocate a state and fill the state's coordinates.
    ompl::base::ScopedState<> state(spaceInfo_);
    for (auto i = 0u; i < dimensionality_; ++i) {
      state[i] = s.at(i);
    }

    states.push_back(state);
  }

  return states;
}

std::vector<ompl::base::ScopedState<>> BaseContext::generateRandomStates(
    const std::string &key) const {
  if (!spaceInfo_->isSetup()) {
    throw std::runtime_error("GenerateRandomStates() called before spaceInfo is setup.");
  }

  const std::size_t numStates = config_->get<std::size_t>(key + "/numGenerated");
  std::vector<ompl::base::ScopedState<>> states;

  for (auto i = 0u; i < numStates; ++i) {
    ompl::base::ScopedState<> s(spaceInfo_);
    if (config_->get<std::string>(key + "/generativeModel") == "uniform") {
      do {
        s.random();
      } while (!spaceInfo_->isValid(s.get()));
    } else if (config_->get<std::string>(key + "/generativeModel") == "subregion") {
      if (spaceInfo_->getStateSpace()->getType() != ompl::base::STATE_SPACE_REAL_VECTOR) {
        OMPL_ERROR("%s: Subregion currently only supports RealVectorStateSpace.", name_.c_str());
        throw std::runtime_error("Context error.");
      }

      const auto low = config_->get<std::vector<double>>(key + "/generator/lowerBounds");
      const auto high = config_->get<std::vector<double>>(key + "/generator/upperBounds");

      ompl::base::RealVectorStateSpace samplingStateSpace(
          static_cast<unsigned int>(dimensionality_));
      ompl::base::RealVectorBounds bounds(static_cast<unsigned int>(dimensionality_));
      for (auto j = 0u; j < dimensionality_; ++j) {
        bounds.setLow(j, low[j]);
        bounds.setHigh(j, high[j]);
      }
      samplingStateSpace.setBounds(bounds);

      ompl::base::RealVectorStateSampler sampler(&samplingStateSpace);

      do {
        sampler.sampleUniform(s());
      } while (!spaceInfo_->isValid(s.get()));
    }

    states.push_back(s);
  }

  return states;
}

std::vector<StartGoalPair> BaseContext::parseMultiqueryStartGoalPairs() const {
  std::vector<StartGoalPair> pairs;

  const std::string baseKey = "context/" + name_;

  // Sanity checks
  if (!config_->contains(baseKey + "/starts")) {
    throw std::runtime_error("No starts specified.");
  }
  if (!config_->contains(baseKey + "/starts/type")) {
    throw std::runtime_error("No start type specified.");
  }

  if (!config_->contains(baseKey + "/goals")) {
    throw std::runtime_error("No goals specified.");
  }

  if (!config_->contains(baseKey + "/goals/type")) {
    throw std::runtime_error("No goals type specified.");
  }

  // read starts from config
  std::vector<ompl::base::ScopedState<>> startStates;
  if (config_->get<std::string>(baseKey + "/starts/type") == "specified") {
    startStates = parseSpecifiedStates(baseKey + "starts/states");
  } else if (config_->get<std::string>(baseKey + "/starts/type") == "generated") {
    startStates = generateRandomStates(baseKey + "/starts");
  } else {
    throw std::runtime_error(
        "Start type not supported. Must be either 'specified' or 'generated'.");
  }

  std::vector<std::vector<ompl::base::ScopedState<>>> starts;
  for (const auto &state : startStates) {
    std::vector<ompl::base::ScopedState<>> tmp{state};
    starts.push_back(tmp);
  }

  // read goals from config
  std::vector<std::shared_ptr<ompl::base::Goal>> goals;

  std::vector<ompl::base::ScopedState<>> goalStates;
  if (config_->get<std::string>(baseKey + "/goals/type") == "specified") {
    goalStates = parseSpecifiedStates(baseKey + "/goals/states");
  } else if (config_->get<std::string>(baseKey + "/goals/type") == "generated") {
    goalStates = generateRandomStates(baseKey + "/goals");
  } else if (config_->get<std::string>(baseKey + "/goals/type") == "followStarts") {
    // the goal of the first query is the start of the next query.
    const std::size_t numGoals = config_->get<std::size_t>(baseKey + "/goals/numGenerated");
    for (auto i = 0u; i < numGoals; ++i) {
      if (i + 1 < starts.size()) {
        for (const auto &tmp : starts[i + 1]) {
          goalStates.push_back(tmp);
        }
      } else {
        ompl::base::ScopedState<> g(spaceInfo_);
        do {
          g.random();
        } while (!spaceInfo_->isValid(g.get()));
        goalStates.push_back(g);
      }
    }
  } else {
    throw std::runtime_error(
        "Goal type not supported. Must be either 'specified', 'generated', or 'followStarts'.");
  }

  for (const auto &state : goalStates) {
    auto goal = std::make_shared<ompl::base::GoalState>(spaceInfo_);
    goal->as<ompl::base::GoalState>()->setState(state);
    goals.push_back(goal);
  }

  // merge starts and goals
  if (starts.size() != goals.size()) {
    throw std::runtime_error("Different number of starts and goals specified.");
  }

  for (auto i = 0u; i < starts.size(); ++i) {
    StartGoalPair pair;
    pair.start = starts[i];
    pair.goal = goals[i];

    pairs.push_back(pair);
  }

  return pairs;
}

std::vector<StartGoalPair> BaseContext::makeStartGoalPair() const {
  std::vector<StartGoalPair> pairs;

  if (config_->contains("context/" + name_ +
                        "/starts")) {  // if a 'starts' spec is given, read that
    pairs = parseMultiqueryStartGoalPairs();
  } else if (config_->contains("context/" + name_ +
                               "/start")) {  // else check if a single start state is given
    const auto startPosition = config_->get<std::vector<double>>("context/" + name_ + "/start");

    // ensure that the dimensionality works
    if (startPosition.size() != dimensionality_) {
      OMPL_ERROR("%s: Dimensionality of problem and of start specification does not match.",
                 name_.c_str());
      throw std::runtime_error("Context error.");
    }

    // Fill the start and goal states' coordinates.
    ompl::base::ScopedState<> startState(spaceInfo_);
    for (auto i = 0u; i < spaceInfo_->getStateDimension(); ++i) {
      startState[i] = startPosition.at(i);
    }

    StartGoalPair pair;
    pair.start = {startState};
    pair.goal = createGoal();

    pairs.push_back(pair);
  } else {
    OMPL_ERROR("%s: Neither 'start' nor 'starts' specified.", name_.c_str());
    throw std::runtime_error("Context error.");
  }

  return pairs;
}

void BaseContext::regenerateQueries() {
  startGoalPairs_ = makeStartGoalPair();
}

}  // namespace planning_contexts

}  // namespace pdt
