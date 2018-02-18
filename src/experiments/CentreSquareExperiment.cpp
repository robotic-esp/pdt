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
#include "experiments/CentreSquareExperiment.h"

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

CentreSquareExperiment::CentreSquareExperiment(const unsigned int dim, const double obsWidth, const double worldWidth, const double runSeconds, const double checkResolution)
    :   BaseExperiment(dim, limits_t(dim, std::pair<double, double>(-0.5*worldWidth, 0.5*worldWidth)), runSeconds, "CentreSquare"),
        obsWidth_(obsWidth),
        startPos_(-0.5),
        goalPos_(0.5)
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

    // Allocate the obstacle world
    rectObs_ = std::make_shared<HyperrectangleObstacles>(BaseExperiment::si_, false);
    BaseExperiment::obs_ = rectObs_;

    //Set the problem bounds:
    problemBounds.setLow(BaseExperiment::limits_.at(0u).first);
    problemBounds.setHigh(BaseExperiment::limits_.at(0u).second);

    //Store the problem bounds:
    ss->setBounds(problemBounds);

    // Set the validity checker and checking resolution
    BaseExperiment::si_->setStateValidityChecker(static_cast<ompl::base::StateValidityCheckerPtr>(rectObs_));
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
    for (unsigned int j = 0u; j < BaseExperiment::dim_; ++j)
    {
        if (j == 0u)
        {
            BaseExperiment::startStates_.back()[j] = startPos_;
        }
        else
        {
            BaseExperiment::startStates_.back()[j] = 0.0;
        }
    }

    // Create my goal:
    // Create a goal state on the vector:
    BaseExperiment::goalStates_.push_back( ompl::base::ScopedState<>(ss) );

    //Assign to each component
    for (unsigned int j = 0u; j < BaseExperiment::dim_; ++j)
    {
        if (j == 0u)
        {
            BaseExperiment::goalStates_.back()[j] = goalPos_;
        }
        else
        {
            BaseExperiment::goalStates_.back()[j] = 0.0;
        }
    }

    // Allocate the goal:
    BaseExperiment::goalPtr_ = std::make_shared<ompl::base::GoalState>(BaseExperiment::si_);

    // Add
    BaseExperiment::goalPtr_->as<ompl::base::GoalState>()->setState(BaseExperiment::goalStates_.back());

    //Set the obstacle's lower-left corner:
    sightLineObs_ = std::make_shared<ompl::base::ScopedState<> >(ss);
    for (unsigned int i = 0u; i < BaseExperiment::dim_; ++i)
    {
        (*sightLineObs_)[i] = (BaseExperiment::goalStates_.back()[i] + BaseExperiment::startStates_.back()[i])/2.0 - 0.5*obsWidth_;
    }

    //Add the obstacle:
    rectObs_->addObstacle(std::make_pair(sightLineObs_->get(), std::vector<double> (BaseExperiment::dim_, obsWidth_)));

    //Finally specify the optimization target:
    BaseExperiment::opt_->setCostThreshold(this->getOptimum());
}

bool CentreSquareExperiment::knowsOptimum() const
{
    return true;
}

ompl::base::Cost CentreSquareExperiment::getOptimum() const
{
    ompl::base::Cost startToCorner (std::sqrt(std::pow((*sightLineObs_)[0u] - BaseExperiment::startStates_.front()[0u], 2.0) + std::pow((*sightLineObs_)[1u] - BaseExperiment::startStates_.front()[1u], 2.0)));
    ompl::base::Cost obsEdge (obsWidth_);
    ompl::base::Cost otherCornerToGoal (std::sqrt(std::pow(BaseExperiment::goalStates_.front()[0u] - ((*sightLineObs_)[0u] + obsWidth_), 2.0) + std::pow(BaseExperiment::goalStates_.front()[1u] - (*sightLineObs_)[1u], 2.0)));

    // Combine and return:
    return BaseExperiment::opt_->combineCosts(BaseExperiment::opt_->combineCosts(startToCorner, obsEdge), otherCornerToGoal);
}

void CentreSquareExperiment::setTarget(double targetSpecifier)
{
    BaseExperiment::opt_->setCostThreshold( ompl::base::Cost(targetSpecifier*this->getOptimum().value()) );
}

std::string CentreSquareExperiment::lineInfo() const
{
    std::stringstream rval;

    rval << "Obstacle width: " << obsWidth_ << ".";

    return rval.str();
}

std::string CentreSquareExperiment::paraInfo() const
{
    return std::string();
}
