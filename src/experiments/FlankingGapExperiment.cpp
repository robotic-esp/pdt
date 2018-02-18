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
#include "experiments/FlankingGapExperiment.h"

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

FlankingGapExperiment::FlankingGapExperiment(const bool onlyFindGap, const double gapWidth, const double runSeconds, const double checkResolution)
    :   BaseExperiment(2u, limits_t(2u, std::pair<double, double>(-1.0, 1.0)), runSeconds, "FlankingGap"),
        stopClassSwitch_(onlyFindGap),
        gapWidth_(gapWidth),
        obsThickness_(0.2),
        obsTotalHeight_(1.0),
        startPos_(-0.75),
        goalPos_(0.75)
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

    // Allocate the obstacles lower-left corners:
    lowerObs_ = std::make_shared<ompl::base::ScopedState<> >(ss);
    upperObs_ = std::make_shared<ompl::base::ScopedState<> >(ss);

    // Specify the lower obstacle
    // Position
    (*lowerObs_)[0u] = (goalPos_ + startPos_)/2.0 - 0.5*obsThickness_;
    (*lowerObs_)[1u] = -0.5*obsTotalHeight_;

    // Widths
    lowerWidths_ = std::vector<double> (BaseExperiment::dim_, obsThickness_);

    // Specify the upper obstacle
    (*upperObs_)[0u] = (*lowerObs_)[0]; // x
    (*upperObs_)[1u] = (*lowerObs_)[1] + lowerWidths_.at(1) + gapWidth_;

    // Widths, which are only different from the lower widths in the second dimension
    upperWidths_ = lowerWidths_;
    upperWidths_.at(1u) = 0.5*obsTotalHeight_ - (*upperObs_)[1u];

    // Add the obstacles
    rectObs_->addObstacle(std::make_pair(lowerObs_->get(), lowerWidths_));
    rectObs_->addObstacle(std::make_pair(upperObs_->get(), upperWidths_));

    //Finally specify the optimization target:
    if (stopClassSwitch_ == true)
    {
        //Stop if we find a cost better than the max non flanking cost
        BaseExperiment::opt_->setCostThreshold(this->minFlankingCost());
    }
    else
    {
        //The optimum:
        BaseExperiment::opt_->setCostThreshold(this->getOptimum());
    }

    //Make sure this is a sane problem:
    if (this->minFlankingCost().value() < this->maxGapCost().value())
    {
        throw ompl::Exception("For the given gap width, a path through the gap can be worse than a flanking path.");
    }
    //No else
}

bool FlankingGapExperiment::knowsOptimum() const
{
    return true;
}

ompl::base::Cost FlankingGapExperiment::getOptimum() const
{
    // Variables
    ompl::base::Cost startToCorner;
    ompl::base::Cost obsEdge;
    ompl::base::Cost otherCornerToGoal;

    //The optimum goes exactly under the (*upperObs_):
    startToCorner = ompl::base::Cost (std::sqrt(std::pow((*upperObs_)[0u] - BaseExperiment::startStates_.front()[0u], 2.0) + std::pow((*upperObs_)[1u] - BaseExperiment::startStates_.front()[1u], 2.0)));
    obsEdge = ompl::base::Cost (lowerWidths_.at(0u));
    otherCornerToGoal = ompl::base::Cost (std::sqrt(std::pow(BaseExperiment::goalStates_.front()[0u] - ((*upperObs_)[0u] + lowerWidths_.at(0u)), 2.0) + std::pow(BaseExperiment::goalStates_.front()[1u] - (*upperObs_)[1u], 2.0)));

    // Combine and return:
    return BaseExperiment::opt_->combineCosts(BaseExperiment::opt_->combineCosts(startToCorner, obsEdge), otherCornerToGoal);
}

ompl::base::Cost FlankingGapExperiment::minFlankingCost() const
{
    // Variables
    ompl::base::Cost startToCorner;
    ompl::base::Cost obsEdge;
    ompl::base::Cost otherCornerToGoal;

    //The minimum cost not through the gap goes exactly under the *lowerObs_ (or over the *upperObs_):
    startToCorner = ompl::base::Cost (std::sqrt(std::pow((*lowerObs_)[0u] - BaseExperiment::startStates_.front()[0u], 2.0) + std::pow((*lowerObs_)[1u] - BaseExperiment::startStates_.front()[1u], 2.0)));
    obsEdge = ompl::base::Cost (lowerWidths_.at(0u));
    otherCornerToGoal = ompl::base::Cost (std::sqrt(std::pow(BaseExperiment::goalStates_.front()[0u] - ((*lowerObs_)[0u] + lowerWidths_.at(0u)), 2.0) + std::pow(BaseExperiment::goalStates_.front()[1u] - (*lowerObs_)[1u], 2.0)));

    // Combine and return:
    return BaseExperiment::opt_->combineCosts(BaseExperiment::opt_->combineCosts(startToCorner, obsEdge), otherCornerToGoal);
}

ompl::base::Cost FlankingGapExperiment::maxGapCost() const
{

    // Variables
    ompl::base::Cost startToCorner;
    ompl::base::Cost obsEdge;
    ompl::base::Cost otherCornerToGoal;

    //The worst-case straight-line-path through the gap goes exactly over *lowerObs_:
    startToCorner = ompl::base::Cost(std::sqrt(std::pow((*lowerObs_)[0u] - BaseExperiment::startStates_.front()[0u], 2.0) + std::pow(((*lowerObs_)[1u] + lowerWidths_.at(1u)) - BaseExperiment::startStates_.front()[1u], 2.0)));
    obsEdge = ompl::base::Cost(lowerWidths_.at(0u));
    otherCornerToGoal = ompl::base::Cost(std::sqrt(std::pow(BaseExperiment::goalStates_.front()[0u] - ((*lowerObs_)[0u] + lowerWidths_.at(0u)), 2.0) + std::pow(BaseExperiment::goalStates_.front()[1u] - ((*lowerObs_)[1u] + lowerWidths_.at(1u)), 2.0)));

    // Combine and return:
    return BaseExperiment::opt_->combineCosts(BaseExperiment::opt_->combineCosts(startToCorner, obsEdge), otherCornerToGoal);
}

void FlankingGapExperiment::setTarget(double targetSpecifier)
{
    if (stopClassSwitch_ == true)
    {
        throw ompl::Exception("Cannot set target when searching for class switch");
    }
    //No else

    BaseExperiment::opt_->setCostThreshold( ompl::base::Cost(targetSpecifier*this->getOptimum().value()) );
}

/** \brief Derived class specific information. */
std::string FlankingGapExperiment::lineInfo() const
{
    std::stringstream rval;

    rval << "(min flanking: " << this->minFlankingCost() << ", max gap: " << this->maxGapCost() << ".";

    return rval.str();
}

std::string FlankingGapExperiment::paraInfo() const
{
    std::stringstream rval;

    rval << "bottom obstacle: [";
    for (unsigned int i = 0u; i < BaseExperiment::dim_; ++i)
    {
        rval << (*lowerObs_)[i];
        if (i != BaseExperiment::dim_-1u)
        {
            rval << ", ";
        }
    }
    rval << "], [";
    for (unsigned int i = 0u; i < BaseExperiment::dim_; ++i)
    {
        rval << (*lowerObs_)[i] + lowerWidths_.at(i);
        if (i != BaseExperiment::dim_-1u)
        {
            rval << ", ";
        }
    }
    rval << "]" << std::endl;
    rval << "upper obstacle: [";
    for (unsigned int i = 0u; i < BaseExperiment::dim_; ++i)
    {
        rval << (*upperObs_)[i];
        if (i != BaseExperiment::dim_-1u)
        {
            rval << ", ";
        }
    }
    rval << "], [";
    for (unsigned int i = 0u; i < BaseExperiment::dim_; ++i)
    {
        rval << (*upperObs_)[i] + upperWidths_.at(i);
        if (i != BaseExperiment::dim_-1u)
        {
            rval << ", ";
        }
    }
    rval << "]" << std::endl;

    return rval.str();
}
