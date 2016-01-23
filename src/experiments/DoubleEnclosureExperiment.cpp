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
#include "experiments/DoubleEnclosureExperiment.h"

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

DoubleEnclosureExperiment::DoubleEnclosureExperiment(const unsigned int dim, const double gapWidth, const double runSeconds, const double checkResolution)
    :   BaseExperiment(dim, limits_t(dim, std::pair<double, double>(-1.0, 1.0)), runSeconds, "DblEncl"),
        obsThickness_(0.1),
        enclWidth_(0.5),
        gapWidth_(gapWidth),
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
    rectObs_ = boost::make_shared<HyperrectangleObstacles>(BaseExperiment::si_, false);
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
    BaseExperiment::opt_ = boost::make_shared<ompl::base::PathLengthOptimizationObjective>(BaseExperiment::si_);

    // Set the heuristic to the default:
    BaseExperiment::opt_->setCostToGoHeuristic(boost::bind(&ompl::base::goalRegionCostToGo, _1, _2));

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
    BaseExperiment::goalPtr_ = boost::make_shared<ompl::base::GoalState>(BaseExperiment::si_);

    // Add
    BaseExperiment::goalPtr_->as<ompl::base::GoalState>()->setState(BaseExperiment::goalStates_.back());

    // Create the start and goal enclosures
    // There are 5 obstacles for each.

    // Allocate the obstacles
    startEnclObs_.push_back( boost::make_shared<ompl::base::ScopedState<> >(ss) );
    startEnclObs_.push_back( boost::make_shared<ompl::base::ScopedState<> >(ss) );
    startEnclObs_.push_back( boost::make_shared<ompl::base::ScopedState<> >(ss) );
    startEnclObs_.push_back( boost::make_shared<ompl::base::ScopedState<> >(ss) );
    startEnclObs_.push_back( boost::make_shared<ompl::base::ScopedState<> >(ss) );

    goalEnclObs_.push_back( boost::make_shared<ompl::base::ScopedState<> >(ss) );
    goalEnclObs_.push_back( boost::make_shared<ompl::base::ScopedState<> >(ss) );
    goalEnclObs_.push_back( boost::make_shared<ompl::base::ScopedState<> >(ss) );
    goalEnclObs_.push_back( boost::make_shared<ompl::base::ScopedState<> >(ss) );
    goalEnclObs_.push_back( boost::make_shared<ompl::base::ScopedState<> >(ss) );

    //And their widths
    startEnclWidths_ = std::vector< std::vector<double> > (5u, std::vector<double>(BaseExperiment::dim_, 0.0) );
    goalEnclWidths_ = std::vector< std::vector<double> > (5u, std::vector<double>(BaseExperiment::dim_, 0.0) );

    // Specify the positions of the starts. Start from the obstacle below the gap and go around
    // 0th: the x and y are 1/2 encl width to the left/below of the start
    (*startEnclObs_.at(0u))[0u] = startPos_ - 0.5*enclWidth_;
    (*startEnclObs_.at(0u))[1u] = 0.0 - 0.5*enclWidth_;
    // The widths are obsThickness in x, 1/2 encl. width - 1/2 gap in y
    startEnclWidths_.at(0u).at(0u) = obsThickness_;
    startEnclWidths_.at(0u).at(1u) = 0.5*enclWidth_ - 0.5*gapWidth_;

    // 1st: the x is shifted over from the 0th by the obstacle thickness. The y is the same
    (*startEnclObs_.at(1u))[0u] = (*startEnclObs_.at(0u))[0u] + obsThickness_;
    (*startEnclObs_.at(1u))[1u] = (*startEnclObs_.at(0u))[1u];
    // The widths are obsThickness in y, encl. width - 2*obs in x
    startEnclWidths_.at(1u).at(0u) = enclWidth_ - 2.0*obsThickness_;
    startEnclWidths_.at(1u).at(1u) = obsThickness_;

    // 2nd: the x is an obstacle thickness less than being 1/2 encl. width to the right of the start. The y is the same
    (*startEnclObs_.at(2u))[0u] = startPos_ + 0.5*enclWidth_ - obsThickness_;
    (*startEnclObs_.at(2u))[1u] = (*startEnclObs_.at(0u))[1u];
    // The widths are obsThickness in x, encl. width in y
    startEnclWidths_.at(2u).at(0u) = obsThickness_;
    startEnclWidths_.at(2u).at(1u) = enclWidth_;

    // 3rd: The x is the same as the 1st. The y is an obstacle thickness less than being 1/2 encl. width above the start
    (*startEnclObs_.at(3u))[0u] = (*startEnclObs_.at(1u))[0u];
    (*startEnclObs_.at(3u))[1u] = 0.0 + 0.5*enclWidth_ - obsThickness_;
    // The widths are the same as the 1st
    startEnclWidths_.at(3u).at(0u) = startEnclWidths_.at(1u).at(0u);
    startEnclWidths_.at(3u).at(1u) = startEnclWidths_.at(1u).at(1u);

    // 4th: the x is the same as the 0th, the y is the gap-width above the centreline
    (*startEnclObs_.at(4u))[0u] = (*startEnclObs_.at(0u))[0u];
    (*startEnclObs_.at(4u))[1u] = 0.0 + 0.5*gapWidth_;
    // The widths are the same as the 0th
    startEnclWidths_.at(4u).at(0u) = startEnclWidths_.at(0u).at(0u);
    startEnclWidths_.at(4u).at(1u) = startEnclWidths_.at(0u).at(1u);

    // Specify the positions of the goals. Start from the obstacle below the gap and go around
    // 0th: the x is obsThickness in from being a full 1/2 encl width to the right of the goal. The y is 1/2 encl. width below the goal
    (*goalEnclObs_.at(0u))[0u] = goalPos_ + 0.5*enclWidth_ - obsThickness_;
    (*goalEnclObs_.at(0u))[1u] = 0.0 - 0.5*enclWidth_;
    // The widths are obs in x, 1/2 encl. width - 1/2 gap in y
    goalEnclWidths_.at(0u).at(0u) = obsThickness_;
    goalEnclWidths_.at(0u).at(1u) = 0.5*enclWidth_ - 0.5*gapWidth_;

    // 1st: the x is obsThickness in from being a full 1/2 encl width to the left of the goal. The y is that same
    (*goalEnclObs_.at(1u))[0u] = goalPos_ - 0.5*enclWidth_ + obsThickness_;
    (*goalEnclObs_.at(1u))[1u] = (*goalEnclObs_.at(0u))[1u];
    // The widths are obs in y, encl. width - 2*obs in x
    goalEnclWidths_.at(1u).at(0u) = enclWidth_ - 2.0*obsThickness_;
    goalEnclWidths_.at(1u).at(1u) = obsThickness_;

    // 2nd: the x and y are a full 1/2 encl width to the left/below the goal.
    (*goalEnclObs_.at(2u))[0u] = goalPos_ - 0.5*enclWidth_;
    (*goalEnclObs_.at(2u))[1u] = (*goalEnclObs_.at(0u))[1u];
    // The widths are obs in x, encl. width in y
    goalEnclWidths_.at(2u).at(0u) = obsThickness_;
    goalEnclWidths_.at(2u).at(1u) = enclWidth_;

    // 3rd: The y is obsThickness from being a full 1/2 encl. width above the goal .The x is the same as the 1st obstacle.
    (*goalEnclObs_.at(3u))[0u] = (*goalEnclObs_.at(1u))[0u];
    (*goalEnclObs_.at(3u))[1u] = 0.0 + 0.5*enclWidth_ - obsThickness_;
    // The widths are the same as the 1st
    goalEnclWidths_.at(3u).at(0u) = goalEnclWidths_.at(1u).at(0u);
    goalEnclWidths_.at(3u).at(1u) = goalEnclWidths_.at(1u).at(1u);

    // 4th: The x is the same as the 0th, the y is 1/2 gap width above the centreline
    (*goalEnclObs_.at(4u))[0u] = (*goalEnclObs_.at(0u))[0u];
    (*goalEnclObs_.at(4u))[1u] = 0.0 + 0.5*gapWidth_;
    // The widths are the same as the 0th
    goalEnclWidths_.at(4u).at(0u) = goalEnclWidths_.at(0u).at(0u);
    goalEnclWidths_.at(4u).at(1u) = goalEnclWidths_.at(0u).at(1u);

    // The rest of the corners are the lower limits of state space and the widths the range:
    for (unsigned int i = 0u; i < startEnclObs_.size(); ++i)
    {
        for (unsigned int j = 2u; j < BaseExperiment::dim_; ++j)
        {
            (*startEnclObs_.at(i))[j] = BaseExperiment::limits_.at(j).first; //z
            startEnclWidths_.at(i).at(j) = BaseExperiment::limits_.at(j).second - BaseExperiment::limits_.at(j).first;
        }
    }
    for (unsigned int i = 0u; i < goalEnclObs_.size(); ++i)
    {
        for (unsigned int j = 2u; j < BaseExperiment::dim_; ++j)
        {
            (*goalEnclObs_.at(i))[j] = BaseExperiment::limits_.at(j).first; //z
            goalEnclWidths_.at(i).at(j) = BaseExperiment::limits_.at(j).second - BaseExperiment::limits_.at(j).first;
        }
    }

    // Add the obstacles
    for (unsigned int i = 0u; i < startEnclObs_.size(); ++ i)
    {
        rectObs_->addObstacle(std::make_pair(startEnclObs_.at(i)->get(), startEnclWidths_.at(i)));
    }
    for (unsigned int i = 0u; i < goalEnclObs_.size(); ++ i)
    {
        rectObs_->addObstacle(std::make_pair(goalEnclObs_.at(i)->get(), goalEnclWidths_.at(i)));
    }

    //Finally specify the optimization target as the minimum:
    BaseExperiment::opt_->setCostThreshold(BaseExperiment::getMinimum());
}

bool DoubleEnclosureExperiment::knowsOptimum() const
{
    return false;
}

ompl::base::Cost DoubleEnclosureExperiment::getOptimum() const
{
    throw ompl::Exception("The optimum is not implemented");

    // Combine and return:
    return BaseExperiment::opt_->identityCost();
}

void DoubleEnclosureExperiment::setTarget(double targetSpecifier)
{
    BaseExperiment::opt_->setCostThreshold( ompl::base::Cost(targetSpecifier) );
}

std::string DoubleEnclosureExperiment::lineInfo() const
{
    return std::string();
}

std::string DoubleEnclosureExperiment::paraInfo() const
{
    std::stringstream rval;

    for (unsigned int i = 0u; i < startEnclObs_.size(); ++i)
    {
        rval << "Start enclosure(" << i << ") [";
        for (unsigned int j = 0u; j < BaseExperiment::dim_; ++j)
        {
            rval << (*startEnclObs_.at(i))[j];
            if (j != BaseExperiment::dim_-1u)
            {
                rval << ", ";
            }
        }
        rval << "], [";
        for (unsigned int j = 0u; j < BaseExperiment::dim_; ++j)
        {
            rval << (*startEnclObs_.at(i))[j] + startEnclWidths_.at(i).at(j);
            if (j != BaseExperiment::dim_-1u)
            {
                rval << ", ";
            }
        }
        rval << "]" << std::endl;
    }

    for (unsigned int i = 0u; i < goalEnclObs_.size(); ++i)
    {
        rval << "Goal enclosure(" << i << ") [";
        for (unsigned int j = 0u; j < BaseExperiment::dim_; ++j)
        {
            rval << (*goalEnclObs_.at(i))[j];
            if (j != BaseExperiment::dim_-1u)
            {
                rval << ", ";
            }
        }
        rval << "], [";
        for (unsigned int j = 0u; j < BaseExperiment::dim_; ++j)
        {
            rval << (*goalEnclObs_.at(i))[j] + goalEnclWidths_.at(i).at(j);
            if (j != BaseExperiment::dim_-1u)
            {
                rval << ", ";
            }
        }
        rval << "]" << std::endl;
    }

    return rval.str();
}
