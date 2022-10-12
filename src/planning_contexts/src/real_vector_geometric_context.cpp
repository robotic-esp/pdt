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

/* Authors: Marlin Strub */

#include "pdt/planning_contexts/real_vector_geometric_context.h"

#include <ompl/base/goals/GoalSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace pdt {

namespace planning_contexts {

RealVectorGeometricContext::RealVectorGeometricContext(
    const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
    const std::shared_ptr<const config::Configuration>& config, const std::string& name) :
    BaseContext(spaceInfo, config, name),
    bounds_(static_cast<unsigned int>(dimensionality_)) {
  // Fill the state space bounds.
  auto sideLengths = config->get<std::vector<double>>("context/" + name + "/boundarySideLengths");
  assert(sideLengths.size() == dimensionality_);
  for (std::size_t dim = 0u; dim < dimensionality_; ++dim) {
    bounds_.low.at(dim) = -0.5 * sideLengths.at(dim);
    bounds_.high.at(dim) = 0.5 * sideLengths.at(dim);
  }
}

std::vector<std::shared_ptr<obstacles::BaseObstacle>> RealVectorGeometricContext::getObstacles()
    const {
  return obstacles_;
}

std::vector<std::shared_ptr<obstacles::BaseAntiObstacle>>
RealVectorGeometricContext::getAntiObstacles() const {
  return antiObstacles_;
}

const ompl::base::RealVectorBounds& RealVectorGeometricContext::getBoundaries() const {
  return bounds_;
}

void RealVectorGeometricContext::accept(const ContextVisitor& visitor) const {
  visitor.visit(*this);
}

std::shared_ptr<ompl::base::Goal> RealVectorGeometricContext::createGoal() const {
  // Instantiate the goal.
  switch (goalType_) {
    case ompl::base::GoalType::GOAL_STATE: {
      // Get the goal position.
      const auto goalPosition = config_->get<std::vector<double>>("context/" + name_ + "/goal");

      // Check dimensionality of the goal state position.
      if (goalPosition.size() != dimensionality_) {
        OMPL_ERROR("%s: Dimensionality of problem and of goal specification does not match.",
                   name_.c_str());
        throw std::runtime_error("Context error.");
      }

      // Allocate a goal state and set the position.
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goalState(spaceInfo_);

      // Fill the goal state's coordinates.
      for (auto i = 0u; i < dimensionality_; ++i) {
        goalState[i] = goalPosition.at(i);
      }

      // Register the goal state with the goal.
      auto goal = std::make_shared<ompl::base::GoalState>(spaceInfo_);
      goal->as<ompl::base::GoalState>()->setState(goalState);
      return goal;
    }
    case ompl::base::GoalType::GOAL_STATES: {
      const auto numGoals = config_->get<unsigned>("context/" + name_ + "/numGoals");
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goalState(spaceInfo_);
      auto goal = std::make_shared<ompl::base::GoalStates>(spaceInfo_);
      for (auto i = 0u; i < numGoals; ++i) {
        goalState.random();
        goal->as<ompl::base::GoalStates>()->addState(goalState);
      }
      return goal;
    }
#ifdef PDT_EXTRA_GOAL_SPACE
    case ompl::base::GoalType::GOAL_SPACE: {
      // Get the goal bounds.
      ompl::base::RealVectorBounds goalBounds(static_cast<unsigned>(dimensionality_));
      goalBounds.low = config_->get<std::vector<double>>("context/" + name_ + "/goalLowerBounds");
      goalBounds.high = config_->get<std::vector<double>>("context/" + name_ + "/goalUpperBounds");

      // Generate a goal space.
      auto goalSpace = std::make_shared<ompl::base::RealVectorStateSpace>(dimensionality_);

      // Set the goal bounds.
      goalSpace->setBounds(goalBounds);

      // Let the goal know about the goal space.
      auto goal = std::make_shared<ompl::base::GoalSpace>(spaceInfo_);
      goal->as<ompl::base::GoalSpace>()->setSpace(goalSpace);
      return goal;
    }
#endif  // #ifdef PDT_EXTRA_GOAL_SPACE
    default: { throw std::runtime_error("Goal type not implemented."); }
  }
}

}  // namespace planning_contexts

}  // namespace pdt
