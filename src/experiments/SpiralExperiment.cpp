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
#include "experiments/SpiralExperiment.h"

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

SpiralExperiment::SpiralExperiment(const double distFraction, const double runSeconds, const double checkResolution)
    :   BaseExperiment(2u, limits_t(2u, std::pair<double, double>(-1.0, 1.0)), runSeconds, "SpiralExperiment"),
        obsThickness_ (0.05)
{
    // Variable
    // The state space
    std::shared_ptr<ompl::base::RealVectorStateSpace> ss;
    // The problem bounds
    ompl::base::RealVectorBounds problemBounds(BaseExperiment::dim_);
    // The "characteristic width" of the problem
    double goalDist;
    // The start and goal position
    double startPos;
    double goalPos;
    // Some parameters of the spiral
    double passThickness;
    double uDepth;

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

    //Calculate the characteristic width for the problem
    goalDist = distFraction*(BaseExperiment::limits_.at(0u).second - BaseExperiment::limits_.at(0u).first);

    //Calculate the start and goal position:
    startPos = -0.5*goalDist;
    goalPos = 0.5*goalDist;

    // Create my start:
    // Create a start state on the vector:
    BaseExperiment::startStates_.push_back( ompl::base::ScopedState<>(ss) );

    //Assign to each component
    for (unsigned int j = 0u; j < BaseExperiment::dim_; ++j)
    {
        if (j == 0u)
        {
            BaseExperiment::startStates_.back()[j] = startPos;
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
            BaseExperiment::goalStates_.back()[j] = goalPos;
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

    // Define the obstacles
    //The parameters for the spiral walls:
    passThickness = 2.0*obsThickness_;
    uDepth = 0.5*goalDist - 2.0*obsThickness_ - 2.0*passThickness;

    if ( (1.5*obsThickness_ + uDepth >= goalDist) || (goalDist < 4.0*obsThickness_) )
    {
        throw ompl::Exception("The U of the spiral must fit completely between the start and goal, with buffer.");
    }


    // Allocate the obstacles' lower-left corners and widths:
    //////OBSTACLE 0//////
    obsCorners_.push_back( ompl::base::ScopedState<>(ss) );
    obsCorners_.back()[0u] = 0.5*passThickness - 0.5*goalDist;
    obsCorners_.back()[1u] = 0.5*passThickness;
    obsWidths_.push_back( std::vector<double>(2u, 0.0) );
    obsWidths_.back().at(0u) = obsThickness_ + uDepth;;
    obsWidths_.back().at(1u) = obsThickness_;

    //////OBSTACLE 1//////
    obsCorners_.push_back( ompl::base::ScopedState<>(ss) );
    obsCorners_.back()[0u] = 0.5*passThickness + uDepth - 0.5*goalDist;
    obsCorners_.back()[1u] = -0.5*passThickness - obsThickness_;
    obsWidths_.push_back( std::vector<double>(2u, 0.0) );
    obsWidths_.back().at(0u) = obsThickness_;
    obsWidths_.back().at(1u) = passThickness + obsThickness_;

    //////OBSTACLE 2//////
    obsCorners_.push_back( ompl::base::ScopedState<>(ss) );
    obsCorners_.back()[0u] = -0.5*passThickness - obsThickness_- 0.5*goalDist;
    obsCorners_.back()[1u] = -0.5*passThickness - obsThickness_;
    obsWidths_.push_back( std::vector<double>(2u, 0.0) );
    obsWidths_.back().at(0u) = passThickness + obsThickness_ + uDepth;
    obsWidths_.back().at(1u) = obsThickness_;

    //////OBSTACLE 3//////
    obsCorners_.push_back( ompl::base::ScopedState<>(ss) );
    obsCorners_.back()[0u] = -0.5*passThickness - obsThickness_ - 0.5*goalDist;
    obsCorners_.back()[1u] =  -0.5*passThickness;
    obsWidths_.push_back( std::vector<double>(2u, 0.0) );
    obsWidths_.back().at(0u) = obsThickness_;
    obsWidths_.back().at(1u) = 2.0*passThickness + 2.0*obsThickness_;

    //////OBSTACLE 4//////
    obsCorners_.push_back( ompl::base::ScopedState<>(ss) );
    obsCorners_.back()[0u] = -0.5*passThickness - 0.5*goalDist;
    obsCorners_.back()[1u] = 1.5*passThickness + obsThickness_;
    obsWidths_.push_back( std::vector<double>(2u, 0.0) );
    obsWidths_.back().at(0u) = 2.0*passThickness + 2.0*obsThickness_ + uDepth;
    obsWidths_.back().at(1u) = obsThickness_;

    //////OBSTACLE 5//////
    obsCorners_.push_back( ompl::base::ScopedState<>(ss) );
    obsCorners_.back()[0u] = 1.5*passThickness + obsThickness_ + uDepth - 0.5*goalDist;
    obsCorners_.back()[1u] = -1.5*passThickness - 2.0*obsThickness_;
    obsWidths_.push_back( std::vector<double>(2u, 0.0) );
    obsWidths_.back().at(0u) = obsThickness_;
    obsWidths_.back().at(1u) = 3.0*passThickness + 3.0*obsThickness_;

    //////OBSTACLE 6//////
    obsCorners_.push_back( ompl::base::ScopedState<>(ss) );
    obsCorners_.back()[0u] = -1.5*passThickness - 2.0*obsThickness_ - 0.5*goalDist;
    obsCorners_.back()[1u] = -1.5*passThickness - 2.0*obsThickness_;
    obsWidths_.push_back( std::vector<double>(2u, 0.0) );
    obsWidths_.back().at(0u) = 3.0*passThickness + 3.0*obsThickness_ + uDepth;;
    obsWidths_.back().at(1u) = obsThickness_;

    //////OBSTACLE 7//////
    obsCorners_.push_back( ompl::base::ScopedState<>(ss) );
    obsCorners_.back()[0u] = -1.5*passThickness - 2.0*obsThickness_ - 0.5*goalDist;
    obsCorners_.back()[1u] = -1.5*passThickness - 1.0*obsThickness_;
    obsWidths_.push_back( std::vector<double>(2u, 0.0) );
    obsWidths_.back().at(0u) = obsThickness_;
    obsWidths_.back().at(1u) = 4.0*passThickness + 4.0*obsThickness_;

    //////OBSTACLE 8//////
    obsCorners_.push_back( ompl::base::ScopedState<>(ss) );
    obsCorners_.back()[0u] = -1.5*passThickness - 1.0*obsThickness_ - 0.5*goalDist;
    obsCorners_.back()[1u] = 2.5*passThickness + 2.0*obsThickness_;
    obsWidths_.push_back( std::vector<double>(2u, 0.0) );
    obsWidths_.back().at(0u) = 4.0*passThickness + 4.0*obsThickness_ + uDepth;
    obsWidths_.back().at(1u) = obsThickness_;

    //////OBSTACLE 9//////
    obsCorners_.push_back( ompl::base::ScopedState<>(ss) );
    obsCorners_.back()[0u] = 2.5*passThickness + 2.0*obsThickness_ + uDepth - 0.5*goalDist;
    obsCorners_.back()[1u] = -1.5*passThickness - 2.0*obsThickness_;
    obsWidths_.push_back( std::vector<double>(2u, 0.0) );
    obsWidths_.back().at(0u) = obsThickness_;
    obsWidths_.back().at(1u) = 4.0*passThickness + 4.0*obsThickness_;

    // Add the pairs
    for (unsigned int i = 0u; i < obsCorners_.size(); ++i)
    {
        rectObs_->addObstacle(std::make_pair(obsCorners_.at(i).get(), obsWidths_.at(i)));
    }

    //Finally specify the optimization target:
    BaseExperiment::opt_->setCostThreshold(BaseExperiment::getMinimum());
}

bool SpiralExperiment::knowsOptimum() const
{
    return false;
}

ompl::base::Cost SpiralExperiment::getOptimum() const
{
    throw ompl::Exception("The global optimum is unknown", BaseExperiment::name_);
}

void SpiralExperiment::setTarget(double targetSpecifier)
{
    BaseExperiment::opt_->setCostThreshold( ompl::base::Cost(targetSpecifier) );
}

std::string SpiralExperiment::lineInfo() const
{
    return std::string();
}

std::string SpiralExperiment::paraInfo() const
{
    return std::string();
}
