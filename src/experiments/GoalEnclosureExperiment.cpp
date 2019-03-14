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
#include "experiments/GoalEnclosureExperiment.h"

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


#include "obstacles/HyperrectangleObstacles.h"

GoalEnclosureExperiment::GoalEnclosureExperiment(const unsigned int dim, const double worldHalfWidth, const double insideWidth, const double wallThickness, const double gapWidth, const double runSeconds, const double checkResolution)
    :   BaseExperiment(dim, limits_t(dim, std::pair<double, double>(-worldHalfWidth, worldHalfWidth)), runSeconds, "GoalEnc"),
        insideWidth_(insideWidth),
        wallThickness_(wallThickness),
        gapWidth_(gapWidth),
        startPos_(-0.5),
        goalPos_(0.5)
{
    // Variables
    // The state space
    std::shared_ptr<ompl::base::RealVectorStateSpace> ss;
    // Temporary variables for the obstacle and antiobstacle
    std::shared_ptr<HyperrectangleObstacles> obs;
    std::shared_ptr<HyperrectangleObstacles> anti;

    if (gapWidth_ <= 0 || insideWidth_ <= 0 || wallThickness_ <= 0)
    {
        throw ompl::Exception("Widths must be > 0.");
    }
    if (gapWidth_ > insideWidth_ &&  gapWidth_ != (insideWidth_ + 2.0*wallThickness_))
    {
        throw ompl::Exception("Gap can only be wider than the interior width if it is as wide as the entire obstacle.");
    }
    if (gapWidth_ > (insideWidth_ + 2.0*wallThickness_))
    {
        throw ompl::Exception("Gap is wider than the enclosure.");
    }

    // The problem bounds
    ompl::base::RealVectorBounds problemBounds(BaseExperiment::dim_);

    // Make the state space Rn:
    ss = std::make_shared<ompl::base::RealVectorStateSpace>(BaseExperiment::dim_);

    // Create the space information class:
    BaseExperiment::si_ = std::make_shared<ompl::base::SpaceInformation>(ss);

    // Allocate the obstacle world
    enclObs_ = std::make_shared<CutoutObstacles>(BaseExperiment::si_);
    BaseExperiment::obs_ = enclObs_;

    //Set the problem bounds:
    problemBounds.setLow(BaseExperiment::limits_.at(0u).first);
    problemBounds.setHigh(BaseExperiment::limits_.at(0u).second);

    //Store the problem bounds:
    ss->setBounds(problemBounds);

    // Set the validity checker and checking resolution
    BaseExperiment::si_->setStateValidityChecker(static_cast<ompl::base::StateValidityCheckerPtr>(enclObs_));
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

    // Allocate the temporary variable for the obstacles, antiobstacles, and the lower-left corners
    obs = std::make_shared<HyperrectangleObstacles>(BaseExperiment::si_, false);
    anti = std::make_shared<HyperrectangleObstacles>(BaseExperiment::si_, false);

    //*****Obstacle*****//
    goalEnclCorners_.push_back(std::make_shared<ompl::base::ScopedState<> >(ss));

    // All widths are the same
    goalEnclWidths_.push_back(std::vector<double>(BaseExperiment::dim_, insideWidth_ + 2*wallThickness_));

    // X-position is unique
    (*goalEnclCorners_.back())[0u] = goalPos_ - 0.5*goalEnclWidths_.back().at(0u);

    // All the other dimensions are the same
    for (unsigned int i = 1u; i < BaseExperiment::dim_; ++i)
    {
        (*goalEnclCorners_.back())[i] = 0.0 - 0.5*goalEnclWidths_.back().at(i);
    }

    // Add the obstacle.
    obs->addObstacle(std::make_pair(goalEnclCorners_.back()->get(), goalEnclWidths_.back()));

    //*****Internal Space*****//
    goalEnclCorners_.push_back(std::make_shared<ompl::base::ScopedState<> >(ss));

    // All widths are the same
    goalEnclWidths_.push_back(std::vector<double>(BaseExperiment::dim_, insideWidth_));

    // X-position is unique
    (*goalEnclCorners_.back())[0u] = goalPos_ - 0.5*goalEnclWidths_.back().at(0u);

    // All the other dimensions are the same
    for (unsigned int i = 1u; i < BaseExperiment::dim_; ++i)
    {
        (*goalEnclCorners_.back())[i] = 0.0 - 0.5*goalEnclWidths_.back().at(i);
    }

    // Add to the anti obstacle.
    anti->addObstacle(std::make_pair(goalEnclCorners_.back()->get(), goalEnclWidths_.back()));

    //*****Right opening*****//
    startEnclCorners_.push_back(std::make_shared<ompl::base::ScopedState<> >(ss));
    startEnclWidths_.push_back(std::vector<double>());

    // All the other dimensions are the same
    for (unsigned int i = 0u; i < BaseExperiment::dim_; ++i)
    {
        // X is unique (it's the outside of the wall on the left)
        if (i == 0u)
        {
            (*startEnclCorners_.back())[i] = goalPos_ + (0.5*insideWidth_);
            startEnclWidths_.back().push_back(wallThickness_);
        }
        else
        {
            (*startEnclCorners_.back())[i] = 0.0 - 0.5*gapWidth_;
            startEnclWidths_.back().push_back(gapWidth_);
        }
    }

    // Add to the anti obstacle.
    anti->addObstacle(std::make_pair(startEnclCorners_.back()->get(), startEnclWidths_.back()));

    // Add the obstacles and antiobstacles into the CutoutObstacles class
    enclObs_->addObstacle(obs);
    enclObs_->addAntiObstacle(anti);

    //Finally specify the optimization target as the minimum:
    BaseExperiment::opt_->setCostThreshold(BaseExperiment::getMinimum());
}

bool GoalEnclosureExperiment::knowsOptimum() const
{
    return true;
}

ompl::base::Cost GoalEnclosureExperiment::getOptimum() const
{
    /*
    The optimal solution has 5 components and is constrained to the xy plane:
      - diagonally from the start to the corner of the opening
      - vertically to the far obstacle corner
      - horizontal across to the other obstacle's far corner
      - vertically to the corner of the opening
      - diagonally from the corner of the opening to the goal
    but the 1 & 5 and 2 & 3 are the same (as we're symmetric)
    */

    // Variables
    ompl::base::Cost startToOpening;
    ompl::base::Cost openingToCorner;
    ompl::base::Cost cornerToCorner;

    // Calculate the distances They depend on whether the gap is wider than the interior or not
    if (gapWidth_ <= insideWidth_)
    {
        //It is not, we must go to the top-left corner of the passage
        startToOpening = ompl::base::Cost( std::sqrt(std::pow(0.5*insideWidth_ + wallThickness_, 2.0) + std::pow(0.5*gapWidth_, 2.0)) );

        //and then to the top-left corner of the obstacle
        openingToCorner = ompl::base::Cost( 0.5*(insideWidth_ + 2*wallThickness_ - gapWidth_) );

        //and then across to the other
        cornerToCorner = ompl::base::Cost( (goalPos_ - startPos_) + 2*(0.5*insideWidth_ + wallThickness_) );
    }
    else
    {
        //It is, it is therefore restricted to being the whole obstacle width, we go to the top-left corner of the interior
        startToOpening = ompl::base::Cost( std::sqrt(std::pow(0.5*insideWidth_, 2.0) + std::pow(0.5*insideWidth_, 2.0)) );

        //and then to the top of the passage
        openingToCorner = ompl::base::Cost(wallThickness_);

        //and then across to the other
        cornerToCorner = ompl::base::Cost( (goalPos_ - startPos_) + 2*(0.5*insideWidth_) );
    }

    // Combine and return:
    return BaseExperiment::opt_->combineCosts(startToOpening, BaseExperiment::opt_->combineCosts(openingToCorner, BaseExperiment::opt_->combineCosts(cornerToCorner, BaseExperiment::opt_->combineCosts(openingToCorner, startToOpening))));
}

void GoalEnclosureExperiment::setTarget(double targetSpecifier)
{
    BaseExperiment::opt_->setCostThreshold( ompl::base::Cost(targetSpecifier) );
}

std::string GoalEnclosureExperiment::lineInfo() const
{
    return std::string();
}

std::string GoalEnclosureExperiment::paraInfo() const
{
    std::stringstream rval;

    rval << "Start obstacle " << this->printRectangle(startEnclCorners_.at(0u), startEnclWidths_.at(0u));
    rval << "Start cavity " << this->printRectangle(startEnclCorners_.at(1u), startEnclWidths_.at(1u));
    rval << "Start passage " << this->printRectangle(startEnclCorners_.at(2u), startEnclWidths_.at(2u));

    rval << "Goal obstacle " << this->printRectangle(goalEnclCorners_.at(0u), goalEnclWidths_.at(0u));
    rval << "Goal cavity " << this->printRectangle(goalEnclCorners_.at(1u), goalEnclWidths_.at(1u));
    rval << "Goal passage " << this->printRectangle(goalEnclCorners_.at(2u), goalEnclWidths_.at(2u));

    return rval.str();
}

std::string GoalEnclosureExperiment::printRectangle(std::shared_ptr<ompl::base::ScopedState<> > llCorner, std::vector<double> widths) const
{
    std::stringstream rval;

    rval << "[";
    for (unsigned int i = 0u; i < BaseExperiment::dim_; ++i)
    {
        rval << (*llCorner)[i];
        if (i != BaseExperiment::dim_-1u)
        {
            rval << ", ";
        }
    }
    rval << "], [";
    for (unsigned int i = 0u; i < BaseExperiment::dim_; ++i)
    {
        rval << (*llCorner)[i] + widths.at(i);
        if (i != BaseExperiment::dim_-1u)
        {
            rval << ", ";
        }
    }
    rval << "]" << std::endl;

    return rval.str();
}