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

#pragma once

#include <memory>
#include <string>

#include <ompl/base/Goal.h>
#include <ompl/base/GoalTypes.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>

#include "pdt/config/configuration.h"
#include "pdt/obstacles/base_obstacle.h"
#include "pdt/planning_contexts/context_visitor.h"
#include "pdt/time/time.h"

namespace pdt {

namespace planning_contexts {

/** \brief Struct defining the start/end points of a planning problem. */
struct StartGoalPair {
  std::vector<ompl::base::ScopedState<>> start;
  std::shared_ptr<ompl::base::Goal> goal;
};

/** \brief The base class for an experiment */
class BaseContext {
 public:
  BaseContext(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
              const std::shared_ptr<const config::Configuration>& config, const std::string& name);

  /** \brief Returns the name of this context. */
  std::string getName() const;

  /** \brief Returns the space information. */
  std::shared_ptr<ompl::base::SpaceInformation> getSpaceInformation() const;

  /** \brief Returns the state space. */
  std::shared_ptr<ompl::base::StateSpace> getStateSpace() const;

  /** \brief Returns the dimension of the state space. */
  std::size_t getDimension() const;

  /** \brief Returns the number of start/goal queries for this context. */
  std::size_t getNumQueries() const;

  /** \brief Returns the optimization objective of this context. */
  ompl::base::OptimizationObjectivePtr getObjective() const;

  /** \brief Returns the maximum duration to solve this context. */
  time::Duration getMaxSolveDuration() const;

  /** \brief Returns the start/goal pair. */
  StartGoalPair getNthStartGoalPair(const std::size_t n) const;

  /** \brief Return a newly generated problem definition. Equivalent to
   * instantiateNthNewProblemDefinition(0u). */
  virtual std::shared_ptr<ompl::base::ProblemDefinition> instantiateNewProblemDefinition() const;

  /** \brief Return a newly generated problem definition for the n-th query. */
  virtual std::shared_ptr<ompl::base::ProblemDefinition> instantiateNthProblemDefinition(
      const std::size_t n) const;

  /** \brief Accepts a context visitor. */
  virtual void accept(const ContextVisitor& visitor) const = 0;

  /** \brief Get the obstacles. */
  virtual std::vector<std::shared_ptr<obstacles::BaseObstacle>> getObstacles() const = 0;

  /** \brief Get the antiobstacles */
  virtual std::vector<std::shared_ptr<obstacles::BaseAntiObstacle>> getAntiObstacles() const = 0;

  /** \brief Create a goal. */
  virtual std::shared_ptr<ompl::base::Goal> createGoal() const = 0;

  /** \brief Regenerates the queries: In case they are generated randomly, this gives a new set of
   * queries.*/
  void regenerateQueries();

 protected:
  /** \brief Loads the specified or randomly generates the start/goal pairs (depending on the config
   * file). */
  virtual std::vector<StartGoalPair> makeStartGoalPair() const;

  /** \brief Parse start and goal specification used for multiquery benchmarking. */
  std::vector<StartGoalPair> parseMultiqueryStartGoalPairs() const;

  /** \brief Parse explicitly specified starts/goals. */
  std::vector<ompl::base::ScopedState<>> parseSpecifiedStates(const std::string& key) const;

  /** \brief Generate randomly sampled starts/goals. */
  std::vector<ompl::base::ScopedState<>> generateRandomStates(const std::string& key) const;

  /** \brief The space information associated with this context. */
  ompl::base::SpaceInformationPtr spaceInfo_{};

  /** \brief The dimension of the state space. */
  std::size_t dimensionality_;

  /** \brief The name of the context. */
  std::string name_{};

  /** \brief The optimization objective. */
  ompl::base::OptimizationObjectivePtr objective_{};

  /** \brief The goal specification of the planning problem. */
  /** \todo Consider removing this variable and instead calling StartGoalPair.goal.getType() */
  ompl::base::GoalType goalType_{ompl::base::GoalType::GOAL_ANY};

  /** \brief The start/goal pair for the planning problem. */
  std::vector<StartGoalPair> startGoalPairs_{};

  /** \brief The maximum duration to solve this context. */
  time::Duration maxSolveDuration_{};

  /** \brief The configuration. */
  const std::shared_ptr<const config::Configuration> config_;
};

}  // namespace planning_contexts

}  // namespace pdt
