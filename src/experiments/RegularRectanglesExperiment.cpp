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
#include "experiments/RegularRectanglesExperiment.h"

// STL
#include <cmath>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

// OMPL
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/goals/GoalStates.h"

RegularRectanglesExperiment::RegularRectanglesExperiment(const unsigned int dim, const double worldWidth, unsigned int numObsBetween, const double runSeconds, const double checkResolution)
    :   BaseExperiment(dim, limits_t(dim, std::pair<double, double>(-0.5*worldWidth, 0.5*worldWidth)), runSeconds, "RegularRects"),
        startPos_(-0.5),
        goalPos_(0.5)
{
    // Variable
    // The state space
    boost::shared_ptr<ompl::base::RealVectorStateSpace> ss;
    // The problem bounds
    ompl::base::RealVectorBounds problemBounds(BaseExperiment::dim_);

    // Make the state space Rn:
    ss = boost::make_shared<ompl::base::RealVectorStateSpace>(BaseExperiment::dim_);

    // Create the space information class:
    BaseExperiment::si_ = boost::make_shared<ompl::base::SpaceInformation>(ss);

    // Allocate the obstacle world
    obsWidths_.reserve(BaseExperiment::dim_);
    blankWidths_.reserve(BaseExperiment::dim_);
    origin_.reserve(BaseExperiment::dim_);

    for (unsigned int i = 0u; i < BaseExperiment::dim_; ++i)
    {
        obsWidths_.push_back((goalPos_ - startPos_)/(3.0*numObsBetween));
        blankWidths_.push_back(2.0*obsWidths_.at(i));

        if (numObsBetween%2 == 0)
        {
            origin_.push_back(0.5*blankWidths_.at(0u));
        }
        else
        {
            origin_.push_back(-0.5*obsWidths_.at(0u));
        }
    }

    regObs_ = boost::make_shared<RepeatingHyperrectangleObstacles>(BaseExperiment::si_, obsWidths_, blankWidths_, origin_);
    BaseExperiment::obs_ = regObs_;

    //Set the problem bounds:
    problemBounds.setLow(BaseExperiment::limits_.at(0u).first);
    problemBounds.setHigh(BaseExperiment::limits_.at(0u).second);

    //Store the problem bounds:
    ss->setBounds(problemBounds);

    // Set the validity checker and checking resolution
    BaseExperiment::si_->setStateValidityChecker(static_cast<ompl::base::StateValidityCheckerPtr>(regObs_));
    BaseExperiment::si_->setStateValidityCheckingResolution(checkResolution);

    // Call setup!
    BaseExperiment::si_->setup();

    // Allocate the optimization objective
    BaseExperiment::opt_ = boost::make_shared<ompl::base::PathLengthOptimizationObjective>(BaseExperiment::si_);

    // Set the heuristic to the default:
    BaseExperiment::opt_->setCostToGoHeuristic(boost::bind(&ompl::base::goalRegionCostToGo, _1, _2));

    // Create my start:
    // Create a start state on the vector:
    BaseExperiment::startStates_.push_back( ompl::base::ScopedState<>(ss) );

    //Assign to each component
    BaseExperiment::startStates_.back()[0u] = startPos_;
    for (unsigned int j = 1u; j < BaseExperiment::dim_; ++j)
    {
        BaseExperiment::startStates_.back()[j] = 0.0;
    }

    // Create my goal:
    // Create a goal state on the vector:
    BaseExperiment::goalStates_.push_back( ompl::base::ScopedState<>(ss) );

    //Assign to each component
    BaseExperiment::goalStates_.back()[0u] = goalPos_;
    for (unsigned int j = 1u; j < BaseExperiment::dim_; ++j)
    {
        BaseExperiment::goalStates_.back()[j] = 0.0;
    }

    // Allocate the goal:
    BaseExperiment::goalPtr_ = boost::make_shared<ompl::base::GoalState>(BaseExperiment::si_);

    // Add
    BaseExperiment::goalPtr_->as<ompl::base::GoalState>()->setState(BaseExperiment::goalStates_.back());

    //Finally specify the optimization target:
    BaseExperiment::opt_->setCostThreshold(this->getMinimum());
}

bool RegularRectanglesExperiment::knowsOptimum() const
{
    return false;
}

ompl::base::Cost RegularRectanglesExperiment::getOptimum() const
{
    throw ompl::Exception("The global optimum is unknown", BaseExperiment::name_);
}

void RegularRectanglesExperiment::setTarget(double targetSpecifier)
{
    BaseExperiment::opt_->setCostThreshold( ompl::base::Cost(targetSpecifier) );
}

std::string RegularRectanglesExperiment::lineInfo() const
{
    std::stringstream rval;

    rval << "Obstacles starting at [";
    for (unsigned int i = 0u; i < BaseExperiment::dim_; ++i)
    {
        rval << origin_.at(i);
        if (i != BaseExperiment::dim_ - 1u)
        {
            rval << ", ";
        }
    }
    rval << "] with widths [";
    for (unsigned int i = 0u; i < BaseExperiment::dim_; ++i)
    {
        rval << obsWidths_.at(i);
        if (i != BaseExperiment::dim_ - 1u)
        {
            rval << ", ";
        }
    }
    rval << "] and gaps [";
    for (unsigned int i = 0u; i < BaseExperiment::dim_; ++i)
    {
        rval << blankWidths_.at(i);
        if (i != BaseExperiment::dim_ - 1u)
        {
            rval << ", ";
        }
    }
    rval << "]." << std::flush;


    return rval.str();
}

std::string RegularRectanglesExperiment::paraInfo() const
{
    return std::string();
}
