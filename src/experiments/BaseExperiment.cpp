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
#include "experiments/BaseExperiment.h"

BaseExperiment::BaseExperiment(const unsigned int dim, const limits_t limits, const double runSeconds, std::string name)
        :   name_(name),
            dim_(dim),
            limits_(limits),
            targetTime_(ompl::time::seconds(runSeconds))
{
}

ompl::base::SpaceInformationPtr BaseExperiment::getSpaceInformation() const
{
    return si_;
}

ompl::base::ProblemDefinitionPtr BaseExperiment::newProblemDefinition() const
{
    // Variables
    // The problem definition
    ompl::base::ProblemDefinitionPtr pdef;
    // Allocate
    pdef = boost::make_shared<ompl::base::ProblemDefinition>(si_);

    // Store the optimization objective
    pdef->setOptimizationObjective(opt_);

    // Add the start state(s)
    for (unsigned int i = 0u; i < startStates_.size(); ++i)
    {
        pdef->addStartState(startStates_.at(i));
    }

    // Store
    pdef->setGoal(goalPtr_);

    // Return
    return pdef;
}

ompl::base::OptimizationObjectivePtr BaseExperiment::getOptimizationObjective() const
{
    return opt_;
}

ompl::time::duration BaseExperiment::getTargetTime() const
{
    return targetTime_;
}

ompl::base::GoalPtr BaseExperiment::getGoalPtr() const
{
    return goalPtr_;
}

std::vector<ompl::base::ScopedState<> > BaseExperiment::getStartStates() const
{
    return startStates_;
}

std::vector<ompl::base::ScopedState<> > BaseExperiment::getGoalStates() const
{
    return goalStates_;
}

std::string BaseExperiment::getName() const
{
    return name_;
}

BaseExperiment::limits_t BaseExperiment::getLimits() const
{
    return limits_;
}

ompl::base::Cost BaseExperiment::getMinimum() const
{
    // Return the minimum of each start to the goal
    ompl::base::Cost minCost = opt_->infiniteCost();

    // Iterate over the list of starts:
    for (unsigned int i = 0u; i < startStates_.size(); ++i)
    {
        // Store the best cost to go from this start
        minCost = opt_->betterCost(minCost, ompl::base::goalRegionCostToGo(startStates_.at(i).get(), goalPtr_.get()));
    }

    return minCost;
}

void BaseExperiment::print(const bool verbose /* == false */) const
{
    std::cout << name_ << " in R^" << dim_ << ". runtime: " << targetTime_.total_seconds() << ". target: " << opt_->getCostThreshold() << ". map: (" << limits_.at(0u).first << ", " << limits_.at(0u).second << "). " << this->lineInfo() << std::endl;

    if (verbose == true)
    {
        std::cout << "starts =" << std::endl;
        for (unsigned int i = 0u; i < startStates_.size(); ++i)
        {
            std::cout << "    [" << startStates_.at(i)[0u] << ", " << startStates_.at(i)[1u] << ", ..." << startStates_.at(i)[dim_-1u] << "]" << std::endl;
        }

        std::cout << "goals =" << std::endl;
        for (unsigned int i = 0u; i < goalStates_.size(); ++i)
        {
            std::cout << "    [" << goalStates_.at(i)[0u] << ", " << goalStates_.at(i)[1u] << ", ..." << goalStates_.at(i)[dim_-1u] << "]" << std::endl;
        }

        std::cout << this->paraInfo();
    }
}

std::string BaseExperiment::mfileHeader(bool monochrome, double whiteShift /*= 0.0*/) const
{
    std::stringstream rval;

    rval << "%%%%%% Pre config %%%%%%" << std::endl;
    rval << "cla;" << std::endl;
    rval << "hold on;" << std::endl;
    rval << "desat = 0.9;" << std::endl;
    rval << "whiteShift = " << whiteShift << ";" << std::endl;
    rval << "y = desat*[1 1 0]; %[127 127 0]/255; %[1 1 0];" << std::endl;
    rval << "m = desat*[1 0 1]; %[127 0 127]/255; %[1 0 1];" << std::endl;
    rval << "c = desat*[0 1 1]; %[0 127 127]/255; %[0 1 1];" << std::endl;
    rval << "r = desat*[1 0 0]; %[127 0 0]/255; %[1 0 0];" << std::endl;
    rval << "g = desat*[0 1 0]; %[0 127 0]/255; %[0 1 0];" << std::endl;
    rval << "b = desat*[0 0 1]; %[0 0 127]/255; %[0 0 1];" << std::endl;
    rval << "w = [1 1 1];" << std::endl;
    rval << "k = [0 0 0];" << std::endl;
    if (monochrome == false)
    {
        rval << "startColour = g;" << std::endl;
        rval << "goalColour = r;" << std::endl;
        rval << "vertexColour = [0.0; 139/255; 139/255] + 0.5*whiteShift;" << std::endl;
        rval << "vertexColour(vertexColour > 1) = 1;" << std::endl;
        rval << "startEdgeColour = b + whiteShift;" << std::endl;
        rval << "goalEdgeColour = goalColour + whiteShift;" << std::endl;
        rval << "startEdgeColour(startEdgeColour > 1) = 1;" << std::endl;
        rval << "goalEdgeColour(goalEdgeColour > 1) = 1;" << std::endl;
        rval << "solnColour = m;" << std::endl;
        rval << "nextEdgeColour = g;" << std::endl;
        rval << "queueColour = c + 0.5*whiteShift;" << std::endl;
        rval << "queueColour(queueColour > 1) = 1;" << std::endl;

        rval << "vertexSize = 4;" << std::endl;
        rval << "edgeWeight = 1;" << std::endl;
        rval << "solnWeight = 2*edgeWeight;" << std::endl;
    }
    else
    {
        rval << "gray = [0.7 0.7 0.7];" << std::endl;
        rval << "startColour = k;" << std::endl;
        rval << "goalColour = k;" << std::endl;
        rval << "vertexColour = gray;" << std::endl;
        rval << "startEdgeColour = gray;" << std::endl;
        rval << "goalEdgeColour = gray;" << std::endl;
        rval << "solnColour = k;" << std::endl;
        rval << "nextEdgeColour = gray;" << std::endl;
        rval << "queueColour = gray;" << std::endl;

        rval << "vertexSize = 4;" << std::endl;
        rval << "edgeWeight = 1;" << std::endl;
        rval << "solnWeight = 3*edgeWeight;" << std::endl;
    }
    rval << "edgeStyle = '-';" << std::endl;
    rval << "solnStyle = '-';" << std::endl;
    rval << "nextEdgeWeight = 2*edgeWeight;" << std::endl;
    rval << "nextEdgeStyle = '-';" << std::endl;
    rval << "heuristicEdgeStyle = ':';" << std::endl;
    rval << "queueWeight = edgeWeight;" << std::endl;
    rval << "queueStyle = '-.';" << std::endl;

    //The problem limits:
    rval << "%%%%%% Problem info %%%%%%" << std::endl;
    rval << "minX = " << limits_.at(0u).first << ";" << std::endl;
    rval << "maxX = " << limits_.at(0u).second << ";" << std::endl;
    rval << "minY = " << limits_.at(1u).first << ";" << std::endl;
    rval << "maxY = " << limits_.at(1u).second << ";" << std::endl;

    //Write the starts to mfile:
    rval << "%%%%%% Starts and Goals %%%%%%" << std::endl;
    rval << "xstarts = [" ;
    for (unsigned int i = 0u; i < startStates_.size(); ++i)
    {
        rval << " [" << startStates_.at(i)[0u] << "; " << startStates_.at(i)[1u] << "] ";
    }
    rval << "];" << std::endl;

    //And the goals to mfile:
    rval << "xgoals = [" ;
    for (unsigned int i = 0u; i < goalStates_.size(); ++i)
    {
        rval << " [" << goalStates_.at(i)[0u] << "; " << goalStates_.at(i)[1u] << "] ";
    }
    rval << "];" << std::endl;

    //Write the obstacles
    rval << "%%%%%% Obstacles %%%%%%" << std::endl;
    rval << obs_->mfile();

    //Return
    return rval.str();
}

std::string BaseExperiment::mfileFooter() const
{
    std::stringstream rval;

    //Plot the start and goal vertices:
    rval << "for i = 1:size(xstarts,2)" << std::endl;
    rval << "    plot(xstarts(1,i), xstarts(2,i), '.', 'Color', startColour, 'MarkerSize', 5*vertexSize);" << std::endl;
    rval << "end" << std::endl;
    rval << "for i = 1:size(xgoals,2)" << std::endl;
    rval << "    plot(xgoals(1,i), xgoals(2,i), '.', 'Color', goalColour, 'MarkerSize', 5*vertexSize);" << std::endl;
    rval << "end" << std::endl;

    //Write some post config:
    rval << "%%%%%% Post config %%%%%%" << std::endl;
    rval << "axis equal;" << std::endl;
    rval << "xlim([minX maxX]);" << std::endl;
    rval << "ylim([minY maxY]);" << std::endl;
    rval << "box on;" << std::endl;
    rval << "grid off;" << std::endl;
    rval << "set(gca, 'XTickLabel', []);" << std::endl;
    rval << "set(gca, 'YTickLabel', []);" << std::endl;
    rval << "set(gca, 'XTick', get(gca, 'XTick'));" << std::endl;
    rval << "set(gca, 'YTick', get(gca, 'XTick'));" << std::endl;

    return rval.str();
}
