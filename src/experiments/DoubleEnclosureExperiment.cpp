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

DoubleEnclosureExperiment::DoubleEnclosureExperiment(const unsigned int dim, const double worldHalfWidth, const double insideWidth, const double wallThickness, const double gapWidth, const double runSeconds, const double checkResolution)
    :   BaseExperiment(dim, limits_t(dim, std::pair<double, double>(-worldHalfWidth, worldHalfWidth)), runSeconds, "DblEncl"),
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

    //*****The start first*****//
    //*****Obstacle*****//
    startEnclCorners_.push_back(std::make_shared<ompl::base::ScopedState<> >(ss));

    // All widths are the same
    startEnclWidths_.push_back(std::vector<double>(BaseExperiment::dim_, insideWidth_ + 2*wallThickness_));

    // X-position is unique
    (*startEnclCorners_.back())[0u] = startPos_ - 0.5*startEnclWidths_.back().at(0u);

    // All the other dimensions are the same
    for (unsigned int i = 1u; i < BaseExperiment::dim_; ++i)
    {
        (*startEnclCorners_.back())[i] = 0.0 - 0.5*startEnclWidths_.back().at(i);
    }

    // Add the obstacle.
    obs->addObstacle(std::make_pair(startEnclCorners_.back()->get(), startEnclWidths_.back()));

    //*****Internal Space*****//
    startEnclCorners_.push_back(std::make_shared<ompl::base::ScopedState<> >(ss));

    // All widths are the same
    startEnclWidths_.push_back(std::vector<double>(BaseExperiment::dim_, insideWidth_));

    // X-position is unique
    (*startEnclCorners_.back())[0u] = startPos_ - 0.5*startEnclWidths_.back().at(0u);

    // All the other dimensions are the same
    for (unsigned int i = 1u; i < BaseExperiment::dim_; ++i)
    {
        (*startEnclCorners_.back())[i] = 0.0 - 0.5*startEnclWidths_.back().at(i);
    }

    // Add to the anti obstacle.
    anti->addObstacle(std::make_pair(startEnclCorners_.back()->get(), startEnclWidths_.back()));

    //*****Passage*****//
    startEnclCorners_.push_back(std::make_shared<ompl::base::ScopedState<> >(ss));
    startEnclWidths_.push_back(std::vector<double>());

    // X is unique (it's the outside of the wall on the left)
    (*startEnclCorners_.back())[0u] = startPos_ - (0.5*insideWidth_ + wallThickness_);
    startEnclWidths_.back().push_back(wallThickness_);

    // All the other dimensions are the same
    for (unsigned int i = 1u; i < BaseExperiment::dim_; ++i)
    {
        (*startEnclCorners_.back())[i] = 0.0 - 0.5*gapWidth_;
        startEnclWidths_.back().push_back(gapWidth_);
    }

    // Add to the anti obstacle.
    anti->addObstacle(std::make_pair(startEnclCorners_.back()->get(), startEnclWidths_.back()));

    //*****Then the goal*****//
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

    //*****Passage*****//
    goalEnclCorners_.push_back(std::make_shared<ompl::base::ScopedState<> >(ss));
    goalEnclWidths_.push_back(std::vector<double>());

    // X is unique (it's the inside of the wall on the right)
    (*goalEnclCorners_.back())[0u] = goalPos_ + 0.5*insideWidth_;
    goalEnclWidths_.back().push_back(wallThickness_);

    // All the other dimensions are the same
    for (unsigned int i = 1u; i < BaseExperiment::dim_; ++i)
    {
        (*goalEnclCorners_.back())[i] = 0.0 - 0.5*gapWidth_;
        goalEnclWidths_.back().push_back(gapWidth_);
    }

    // Add to the anti obstacle.
    anti->addObstacle(std::make_pair(goalEnclCorners_.back()->get(), goalEnclWidths_.back()));

    // Add the obstacles and antiobstacles into the CutoutObstacles class
    enclObs_->addObstacle(obs);
    enclObs_->addAntiObstacle(anti);

    //Finally specify the optimization target as the minimum:
    BaseExperiment::opt_->setCostThreshold(BaseExperiment::getMinimum());
}

bool DoubleEnclosureExperiment::knowsOptimum() const
{
    return true;
}

ompl::base::Cost DoubleEnclosureExperiment::getOptimum() const
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

    rval << "Start obstacle " << this->printRectangle(startEnclCorners_.at(0u), startEnclWidths_.at(0u));
    rval << "Start cavity " << this->printRectangle(startEnclCorners_.at(1u), startEnclWidths_.at(1u));
    rval << "Start passage " << this->printRectangle(startEnclCorners_.at(2u), startEnclWidths_.at(2u));

    rval << "Goal obstacle " << this->printRectangle(goalEnclCorners_.at(0u), goalEnclWidths_.at(0u));
    rval << "Goal cavity " << this->printRectangle(goalEnclCorners_.at(1u), goalEnclWidths_.at(1u));
    rval << "Goal passage " << this->printRectangle(goalEnclCorners_.at(2u), goalEnclWidths_.at(2u));

    return rval.str();
}

std::string DoubleEnclosureExperiment::printRectangle(std::shared_ptr<ompl::base::ScopedState<> > llCorner, std::vector<double> widths) const
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


///This is the version of the code that made "channels" in higher dimensions:
//DoubleEnclosureExperiment::DoubleEnclosureExperiment(const unsigned int dim, const double gapWidth, const double runSeconds, const double checkResolution)
//    :   BaseExperiment(dim, limits_t(dim, std::pair<double, double>(-1.0, 1.0)), runSeconds, "DblEncl"),
//        obsThickness_(0.1),
//        enclWidth_(0.5),
//        gapWidth_(gapWidth),
//        startPos_(-0.5),
//        goalPos_(0.5)
//{
//    // Variable
//    // The state space
//    std::shared_ptr<ompl::base::RealVectorStateSpace> ss;
//    // The problem bounds
//    ompl::base::RealVectorBounds problemBounds(BaseExperiment::dim_);
//
//    // Make the state space Rn:
//    ss = std::make_shared<ompl::base::RealVectorStateSpace>(BaseExperiment::dim_);
//
//    // Create the space information class:
//    BaseExperiment::si_ = std::make_shared<ompl::base::SpaceInformation>(ss);
//
//    // Allocate the obstacle world
//    rectObs_ = std::make_shared<HyperrectangleObstacles>(BaseExperiment::si_, false);
//    BaseExperiment::obs_ = rectObs_;
//
//    //Set the problem bounds:
//    problemBounds.setLow(BaseExperiment::limits_.at(0u).first);
//    problemBounds.setHigh(BaseExperiment::limits_.at(0u).second);
//
//    //Store the problem bounds:
//    ss->setBounds(problemBounds);
//
//    // Set the validity checker and checking resolution
//    BaseExperiment::si_->setStateValidityChecker(static_cast<ompl::base::StateValidityCheckerPtr>(rectObs_));
//    BaseExperiment::si_->setStateValidityCheckingResolution(checkResolution);
//
//    // Call setup!
//    BaseExperiment::si_->setup();
//
//    // Allocate the optimization objective
//    BaseExperiment::opt_ = std::make_shared<ompl::base::PathLengthOptimizationObjective>(BaseExperiment::si_);
//
//    // Set the heuristic to the default:
//    BaseExperiment::opt_->setCostToGoHeuristic(std::bind(&ompl::base::goalRegionCostToGo, std::placeholders::_1, std::placeholders::_2));
//
//    // Create my start:
//    // Create a start state on the vector:
//    BaseExperiment::startStates_.push_back( ompl::base::ScopedState<>(ss) );
//
//    //Assign to each component
//    for (unsigned int j = 0u; j < BaseExperiment::dim_; ++j)
//    {
//        if (j == 0u)
//        {
//            BaseExperiment::startStates_.back()[j] = startPos_;
//        }
//        else
//        {
//            BaseExperiment::startStates_.back()[j] = 0.0;
//        }
//    }
//
//    // Create my goal:
//    // Create a goal state on the vector:
//    BaseExperiment::goalStates_.push_back( ompl::base::ScopedState<>(ss) );
//
//    //Assign to each component
//    for (unsigned int j = 0u; j < BaseExperiment::dim_; ++j)
//    {
//        if (j == 0u)
//        {
//            BaseExperiment::goalStates_.back()[j] = goalPos_;
//        }
//        else
//        {
//            BaseExperiment::goalStates_.back()[j] = 0.0;
//        }
//    }
//
//    // Allocate the goal:
//    BaseExperiment::goalPtr_ = std::make_shared<ompl::base::GoalState>(BaseExperiment::si_);
//
//    // Add
//    BaseExperiment::goalPtr_->as<ompl::base::GoalState>()->setState(BaseExperiment::goalStates_.back());
//
//    // Create the start and goal enclosures
//    // There are 5 obstacles for each.
//
//    // Allocate the obstacles
//    startEnclObs_.push_back( std::make_shared<ompl::base::ScopedState<> >(ss) );
//    startEnclObs_.push_back( std::make_shared<ompl::base::ScopedState<> >(ss) );
//    startEnclObs_.push_back( std::make_shared<ompl::base::ScopedState<> >(ss) );
//    startEnclObs_.push_back( std::make_shared<ompl::base::ScopedState<> >(ss) );
//    startEnclObs_.push_back( std::make_shared<ompl::base::ScopedState<> >(ss) );
//
//    goalEnclObs_.push_back( std::make_shared<ompl::base::ScopedState<> >(ss) );
//    goalEnclObs_.push_back( std::make_shared<ompl::base::ScopedState<> >(ss) );
//    goalEnclObs_.push_back( std::make_shared<ompl::base::ScopedState<> >(ss) );
//    goalEnclObs_.push_back( std::make_shared<ompl::base::ScopedState<> >(ss) );
//    goalEnclObs_.push_back( std::make_shared<ompl::base::ScopedState<> >(ss) );
//
//    //And their widths
//    startEnclWidths_ = std::vector< std::vector<double> > (5u, std::vector<double>(BaseExperiment::dim_, 0.0) );
//    goalEnclWidths_ = std::vector< std::vector<double> > (5u, std::vector<double>(BaseExperiment::dim_, 0.0) );
//
//    // Specify the positions of the starts. Start from the obstacle below the gap and go around
//    // 0th: the x and y are 1/2 encl width to the left/below of the start
//    (*startEnclObs_.at(0u))[0u] = startPos_ - 0.5*enclWidth_;
//    (*startEnclObs_.at(0u))[1u] = 0.0 - 0.5*enclWidth_;
//    // The widths are obsThickness in x, 1/2 encl. width - 1/2 gap in y
//    startEnclWidths_.at(0u).at(0u) = obsThickness_;
//    startEnclWidths_.at(0u).at(1u) = 0.5*enclWidth_ - 0.5*gapWidth_;
//
//    // 1st: the x is shifted over from the 0th by the obstacle thickness. The y is the same
//    (*startEnclObs_.at(1u))[0u] = (*startEnclObs_.at(0u))[0u] + obsThickness_;
//    (*startEnclObs_.at(1u))[1u] = (*startEnclObs_.at(0u))[1u];
//    // The widths are obsThickness in y, encl. width - 2*obs in x
//    startEnclWidths_.at(1u).at(0u) = enclWidth_ - 2.0*obsThickness_;
//    startEnclWidths_.at(1u).at(1u) = obsThickness_;
//
//    // 2nd: the x is an obstacle thickness less than being 1/2 encl. width to the right of the start. The y is the same
//    (*startEnclObs_.at(2u))[0u] = startPos_ + 0.5*enclWidth_ - obsThickness_;
//    (*startEnclObs_.at(2u))[1u] = (*startEnclObs_.at(0u))[1u];
//    // The widths are obsThickness in x, encl. width in y
//    startEnclWidths_.at(2u).at(0u) = obsThickness_;
//    startEnclWidths_.at(2u).at(1u) = enclWidth_;
//
//    // 3rd: The x is the same as the 1st. The y is an obstacle thickness less than being 1/2 encl. width above the start
//    (*startEnclObs_.at(3u))[0u] = (*startEnclObs_.at(1u))[0u];
//    (*startEnclObs_.at(3u))[1u] = 0.0 + 0.5*enclWidth_ - obsThickness_;
//    // The widths are the same as the 1st
//    startEnclWidths_.at(3u).at(0u) = startEnclWidths_.at(1u).at(0u);
//    startEnclWidths_.at(3u).at(1u) = startEnclWidths_.at(1u).at(1u);
//
//    // 4th: the x is the same as the 0th, the y is the gap-width above the centreline
//    (*startEnclObs_.at(4u))[0u] = (*startEnclObs_.at(0u))[0u];
//    (*startEnclObs_.at(4u))[1u] = 0.0 + 0.5*gapWidth_;
//    // The widths are the same as the 0th
//    startEnclWidths_.at(4u).at(0u) = startEnclWidths_.at(0u).at(0u);
//    startEnclWidths_.at(4u).at(1u) = startEnclWidths_.at(0u).at(1u);
//
//    // Specify the positions of the goals. Start from the obstacle below the gap and go around
//    // 0th: the x is obsThickness in from being a full 1/2 encl width to the right of the goal. The y is 1/2 encl. width below the goal
//    (*goalEnclObs_.at(0u))[0u] = goalPos_ + 0.5*enclWidth_ - obsThickness_;
//    (*goalEnclObs_.at(0u))[1u] = 0.0 - 0.5*enclWidth_;
//    // The widths are obs in x, 1/2 encl. width - 1/2 gap in y
//    goalEnclWidths_.at(0u).at(0u) = obsThickness_;
//    goalEnclWidths_.at(0u).at(1u) = 0.5*enclWidth_ - 0.5*gapWidth_;
//
//    // 1st: the x is obsThickness in from being a full 1/2 encl width to the left of the goal. The y is that same
//    (*goalEnclObs_.at(1u))[0u] = goalPos_ - 0.5*enclWidth_ + obsThickness_;
//    (*goalEnclObs_.at(1u))[1u] = (*goalEnclObs_.at(0u))[1u];
//    // The widths are obs in y, encl. width - 2*obs in x
//    goalEnclWidths_.at(1u).at(0u) = enclWidth_ - 2.0*obsThickness_;
//    goalEnclWidths_.at(1u).at(1u) = obsThickness_;
//
//    // 2nd: the x and y are a full 1/2 encl width to the left/below the goal.
//    (*goalEnclObs_.at(2u))[0u] = goalPos_ - 0.5*enclWidth_;
//    (*goalEnclObs_.at(2u))[1u] = (*goalEnclObs_.at(0u))[1u];
//    // The widths are obs in x, encl. width in y
//    goalEnclWidths_.at(2u).at(0u) = obsThickness_;
//    goalEnclWidths_.at(2u).at(1u) = enclWidth_;
//
//    // 3rd: The y is obsThickness from being a full 1/2 encl. width above the goal .The x is the same as the 1st obstacle.
//    (*goalEnclObs_.at(3u))[0u] = (*goalEnclObs_.at(1u))[0u];
//    (*goalEnclObs_.at(3u))[1u] = 0.0 + 0.5*enclWidth_ - obsThickness_;
//    // The widths are the same as the 1st
//    goalEnclWidths_.at(3u).at(0u) = goalEnclWidths_.at(1u).at(0u);
//    goalEnclWidths_.at(3u).at(1u) = goalEnclWidths_.at(1u).at(1u);
//
//    // 4th: The x is the same as the 0th, the y is 1/2 gap width above the centreline
//    (*goalEnclObs_.at(4u))[0u] = (*goalEnclObs_.at(0u))[0u];
//    (*goalEnclObs_.at(4u))[1u] = 0.0 + 0.5*gapWidth_;
//    // The widths are the same as the 0th
//    goalEnclWidths_.at(4u).at(0u) = goalEnclWidths_.at(0u).at(0u);
//    goalEnclWidths_.at(4u).at(1u) = goalEnclWidths_.at(0u).at(1u);
//
//    // The rest of the corners are the lower limits of state space and the widths the range:
//    for (unsigned int i = 0u; i < startEnclObs_.size(); ++i)
//    {
//        for (unsigned int j = 2u; j < BaseExperiment::dim_; ++j)
//        {
//            (*startEnclObs_.at(i))[j] = BaseExperiment::limits_.at(j).first; //z
//            startEnclWidths_.at(i).at(j) = BaseExperiment::limits_.at(j).second - BaseExperiment::limits_.at(j).first;
//        }
//    }
//    for (unsigned int i = 0u; i < goalEnclObs_.size(); ++i)
//    {
//        for (unsigned int j = 2u; j < BaseExperiment::dim_; ++j)
//        {
//            (*goalEnclObs_.at(i))[j] = BaseExperiment::limits_.at(j).first; //z
//            goalEnclWidths_.at(i).at(j) = BaseExperiment::limits_.at(j).second - BaseExperiment::limits_.at(j).first;
//        }
//    }
//
//    // Add the obstacles
//    for (unsigned int i = 0u; i < startEnclObs_.size(); ++ i)
//    {
//        rectObs_->addObstacle(std::make_pair(startEnclObs_.at(i)->get(), startEnclWidths_.at(i)));
//    }
//    for (unsigned int i = 0u; i < goalEnclObs_.size(); ++ i)
//    {
//        rectObs_->addObstacle(std::make_pair(goalEnclObs_.at(i)->get(), goalEnclWidths_.at(i)));
//    }
//
//    //Finally specify the optimization target as the minimum:
//    BaseExperiment::opt_->setCostThreshold(BaseExperiment::getMinimum());
//}
//std::string DoubleEnclosureExperiment::paraInfo() const
//{
//    std::stringstream rval;
//
//    for (unsigned int i = 0u; i < startEnclObs_.size(); ++i)
//    {
//        rval << "Start enclosure(" << i << ") [";
//        for (unsigned int j = 0u; j < BaseExperiment::dim_; ++j)
//        {
//            rval << (*startEnclObs_.at(i))[j];
//            if (j != BaseExperiment::dim_-1u)
//            {
//                rval << ", ";
//            }
//        }
//        rval << "], [";
//        for (unsigned int j = 0u; j < BaseExperiment::dim_; ++j)
//        {
//            rval << (*startEnclObs_.at(i))[j] + startEnclWidths_.at(i).at(j);
//            if (j != BaseExperiment::dim_-1u)
//            {
//                rval << ", ";
//            }
//        }
//        rval << "]" << std::endl;
//    }
//
//    for (unsigned int i = 0u; i < goalEnclObs_.size(); ++i)
//    {
//        rval << "Goal enclosure(" << i << ") [";
//        for (unsigned int j = 0u; j < BaseExperiment::dim_; ++j)
//        {
//            rval << (*goalEnclObs_.at(i))[j];
//            if (j != BaseExperiment::dim_-1u)
//            {
//                rval << ", ";
//            }
//        }
//        rval << "], [";
//        for (unsigned int j = 0u; j < BaseExperiment::dim_; ++j)
//        {
//            rval << (*goalEnclObs_.at(i))[j] + goalEnclWidths_.at(i).at(j);
//            if (j != BaseExperiment::dim_-1u)
//            {
//                rval << ", ";
//            }
//        }
//        rval << "]" << std::endl;
//    }
//
//    return rval.str();
//}
