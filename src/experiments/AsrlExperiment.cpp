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

// Me!
#include "experiments/AsrlExperiment.h"

// STL
#include <cmath>
//For std::shared_ptr, etc.
#include <memory>
//For std::bind
#include <functional>

// OMPL
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/goals/GoalStates.h"

AsrlExperiment::AsrlExperiment(const bool plotCsv, const bool plotOverlay, const double runSeconds, const double checkResolution)
    :   BaseExperiment(2u, limits_t(2u, std::pair<double, double>(-1,1)), runSeconds, "ASRL"), //The limits will be overwritten in the constructor
        startPosX_(5.0),
        startPosY_(80.0),
        goalPosX_(385.0),
        goalPosY_(80.0)
{
    // Variable
    // The state space
    std::shared_ptr<ompl::base::RealVectorStateSpace> ss;
    // The problem bounds
    ompl::base::RealVectorBounds problemBounds(BaseExperiment::dim_);

    // Make the state space Rn:
    ss = std::make_shared<ompl::base::RealVectorStateSpace>(BaseExperiment::dim_);

    // Create the space information class:
    BaseExperiment::si_ = std::make_shared<ompl::base::SpaceInformation>(ss);

    //Update the problem limits
    BaseExperiment::limits_.at(0u).first = 0.0;
    BaseExperiment::limits_.at(0u).second = 391.0;
    BaseExperiment::limits_.at(1u).first = 0.0;
    BaseExperiment::limits_.at(1u).second = 163.0;

    //And store them into the bounds
    problemBounds.setLow(0u, BaseExperiment::limits_.at(0u).first);
    problemBounds.setHigh(0u, BaseExperiment::limits_.at(0u).second);
    problemBounds.setLow(1u, BaseExperiment::limits_.at(1u).first);
    problemBounds.setHigh(1u, BaseExperiment::limits_.at(1u).second);

    //Store the problem bounds in the statespace
    ss->setBounds(problemBounds);

    // Allocate the obstacle world
    if (plotOverlay == true)
    {
        asrl_ = std::make_shared<CsvObstacle>(BaseExperiment::si_, 0.5, "./maps/asrl_logo/asrl.csv", plotCsv, false, "./maps/asrl_logo/asrl.png");
    }
    else
    {
        asrl_ = std::make_shared<CsvObstacle>(BaseExperiment::si_, 0.5, "./maps/asrl_logo/asrl.csv", plotCsv, true);
    }
    BaseExperiment::obs_ = asrl_;

    // Set the validity checker and checking resolution
    BaseExperiment::si_->setStateValidityChecker(static_cast<ompl::base::StateValidityCheckerPtr>(asrl_));
    BaseExperiment::si_->setStateValidityCheckingResolution(checkResolution);

    // Call setup!
    BaseExperiment::si_->setup();

    // Allocate the optimization objective
    BaseExperiment::opt_ = std::make_shared<ompl::base::PathLengthOptimizationObjective>(BaseExperiment::si_);

    // Set the heuristic to the default:
    BaseExperiment::opt_->setCostToGoHeuristic(std::bind(&ompl::base::goalRegionCostToGo, std::placeholders::_1, std::placeholders::_2));

    // Create my start:
    // Create a start state on the vector:
    BaseExperiment::startStates_.push_back( ompl::base::ScopedState<>(ss) );

    //Assign to each component
    BaseExperiment::startStates_.back()[0u] = startPosX_;
    BaseExperiment::startStates_.back()[1u] = startPosY_;

    // Create my goal:
    // Create a goal state on the vector:
    BaseExperiment::goalStates_.push_back( ompl::base::ScopedState<>(ss) );

    //Assign to each component
    BaseExperiment::goalStates_.back()[0u] = goalPosX_;
    BaseExperiment::goalStates_.back()[1u] = goalPosY_;

    // Allocate the goal:
    BaseExperiment::goalPtr_ = std::make_shared<ompl::base::GoalState>(BaseExperiment::si_);

    // Add
    BaseExperiment::goalPtr_->as<ompl::base::GoalState>()->setState(BaseExperiment::goalStates_.back());

    //Finally specify the optimization target as the minimum:
    BaseExperiment::opt_->setCostThreshold(BaseExperiment::getMinimum());
}

bool AsrlExperiment::knowsOptimum() const
{
    return false;
}

ompl::base::Cost AsrlExperiment::getOptimum() const
{
    throw ompl::Exception("The optimum is unknown");

    // Combine and return:
    return BaseExperiment::opt_->identityCost();
}

void AsrlExperiment::setTarget(double targetSpecifier)
{
    BaseExperiment::opt_->setCostThreshold( ompl::base::Cost(targetSpecifier) );
}

std::string AsrlExperiment::lineInfo() const
{
    std::stringstream rval;

    rval << "ASRL!";

    return rval.str();
}

std::string AsrlExperiment::paraInfo() const
{
    return std::string();
}
