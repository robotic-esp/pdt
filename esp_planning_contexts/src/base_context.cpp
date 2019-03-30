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

/* Authors: Jonathan Gammell */

#include "esp_planning_contexts/base_context.h"

namespace esp {

namespace ompltools {

BaseContext::BaseContext(const unsigned int dim,
                         const std::vector<std::pair<double, double>> limits,
                         const double runSeconds, std::string name) :
    name_(name),
    dim_(dim),
    limits_(limits),
    targetDuration_(time::seconds(runSeconds)) {
}

ompl::base::SpaceInformationPtr BaseContext::getSpaceInformation() const {
  return si_;
}

ompl::base::StateSpacePtr BaseContext::getStateSpace() const {
  return si_->getStateSpace();
}

ompl::base::ProblemDefinitionPtr BaseContext::newProblemDefinition() const {
  // Variables
  // The problem definition
  ompl::base::ProblemDefinitionPtr pdef;
  // Allocate
  pdef = std::make_shared<ompl::base::ProblemDefinition>(si_);

  // Store the optimization objective
  pdef->setOptimizationObjective(opt_);

  // Add the start state(s)
  for (unsigned int i = 0u; i < startStates_.size(); ++i) {
    pdef->addStartState(startStates_.at(i));
  }

  // Store
  pdef->setGoal(goalPtr_);

  // Return
  return pdef;
}

ompl::base::OptimizationObjectivePtr BaseContext::getOptimizationObjective() const {
  return opt_;
}

time::Duration BaseContext::getTargetDuration() const {
  return targetDuration_;
}

ompl::base::GoalPtr BaseContext::getGoalPtr() const {
  return goalPtr_;
}

std::vector<ompl::base::ScopedState<>> BaseContext::getStartStates() const {
  return startStates_;
}

std::vector<ompl::base::ScopedState<>> BaseContext::getGoalStates() const {
  return goalStates_;
}

std::string BaseContext::getName() const {
  return name_;
}

std::vector<std::pair<double, double>> BaseContext::getLimits() const {
  return limits_;
}

unsigned int BaseContext::getDimensions() const {
  return dim_;
}

ompl::base::Cost BaseContext::getMinimum() const {
  // Return the minimum of each start to the goal
  ompl::base::Cost minCost = opt_->infiniteCost();

  // Iterate over the list of starts:
  for (unsigned int i = 0u; i < startStates_.size(); ++i) {
    // Store the best cost to go from this start
    minCost = opt_->betterCost(
        minCost, ompl::base::goalRegionCostToGo(startStates_.at(i).get(), goalPtr_.get()));
  }

  return minCost;
}

void BaseContext::print(const bool verbose /* == false */) const {
  std::cout << name_ << " in R^" << dim_
            << ". runtime: " << time::seconds(targetDuration_)
            << ". target: " << opt_->getCostThreshold();
  if (this->knowsOptimum()) {
    std::cout << " (opt: " << this->getOptimum() << ")";
  }
  // No else
  std::cout << ". map: (" << limits_.at(0u).first << ", " << limits_.at(0u).second << "). "
            << this->lineInfo() << std::endl;

  if (verbose == true) {
    std::cout << "starts =" << std::endl;
    for (unsigned int i = 0u; i < startStates_.size(); ++i) {
      std::cout << "    [" << startStates_.at(i)[0u] << ", " << startStates_.at(i)[1u] << ", ..."
                << startStates_.at(i)[dim_ - 1u] << "]" << std::endl;
    }

    std::cout << "goals =" << std::endl;
    for (unsigned int i = 0u; i < goalStates_.size(); ++i) {
      std::cout << "    [" << goalStates_.at(i)[0u] << ", " << goalStates_.at(i)[1u] << ", ..."
                << goalStates_.at(i)[dim_ - 1u] << "]" << std::endl;
    }

    std::cout << this->paraInfo();
  }
}

}  // namespace ompltools

}  // namespace esp
