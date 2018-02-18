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

//For std::cout
#include <iostream>
//For std::ifstream and std::ofstream
#include <fstream>
//For std::setprecision
#include <iomanip>
//For std::stringstream
#include <sstream>
//For std::make_shared
#include <memory>
//For boost program options
#include <boost/program_options.hpp>
//For boost lexical cast
#include <boost/lexical_cast.hpp>

//OMPL:
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/goals/GoalStates.h"
#include <ompl/base/ScopedState.h>
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/samplers/informed/PathLengthDirectInfSampler.h"
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include "ompl/util/GeometricEquations.h"

#include "ompl/util/Console.h" //For OMPL_INFORM et al.
#include "ompl/tools/config/MagicConstants.h" //For BETTER_PATH_COST_MARGIN
#include <ompl/util/Exception.h>

#include "ExperimentDefinitions.h"

//The general helper functions
#include "tools/general_tools.h"

#define ASRL_DBL_INFINITY std::numeric_limits<double>::infinity()


bool argParse(int argc, char** argv, unsigned int* numSamplesPtr, double* minCostPtr, double* maxCostPtr, bool* rejPtr, bool* tightPtr, double* fociDistPtr, double* transDiaPtr, unsigned int* numStartsPtr, unsigned int* numGoalsPtr, bool* plotPtr)
{
    // Declare the supported options.
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("number-samples,n", boost::program_options::value<unsigned int>(), "The number of samples to draw.")
        ("min-cost,a", boost::program_options::value<double>()->default_value(0.0), "The cost which all samples must be above, i.e., f(x) ~ [a, b]. If not given, samples are drawn using the upper bound model, i.e., f(x) < b.")
        ("max-cost,b", boost::program_options::value<double>(), "The cost which all samples must be below, i.e., f(x) ~ [a, b].")
        ("tightly-bound,t", boost::program_options::value<bool>()->zero_tokens(), "Tightly bound the PHS with the problem bounds.")
        ("rejection,r", boost::program_options::value<bool>()->zero_tokens(), "Perform rejection sampling in parallel.")
        ("plot,p", boost::program_options::value<bool>()->zero_tokens(), "Create 2D and 3D plots.")
        ("foci-distance,f", boost::program_options::value<double>(), "The distance between the start and goal. Only valid if testing tightly-bounded sampling, i.e., if -t is given.")
        ("transverse-diameter,d", boost::program_options::value<double>(), "The transverse diameter. Only valid if testing tightly-bounded sampling, i.e., if -t is given.")
        ("num-starts,s", boost::program_options::value<unsigned int>()->default_value(1u), "The number of starts, defaults to 1. Only valid if not performing tightly-bounded sampling, i.e., if -t is not given.")
        ("num-goals,g", boost::program_options::value<unsigned int>()->default_value(1u), "The number of goals, defaults to 1. Only valid if not performing tightly-bounded sampling, i.e., if -t is not given.")
        ("log-level,l", boost::program_options::value<unsigned int>()->default_value(0u), "Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG. Defaults to 0 if not set.");
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return false;
    }

    if (vm.count("rejection"))
    {
        *rejPtr = true;
    }
    else
    {
        *rejPtr = false;
    }

    if (vm.count("plot"))
    {
        *plotPtr = true;
    }
    else
    {
        *plotPtr = false;
    }

    if (vm.count("tightly-bound"))
    {
        *tightPtr = true;
    }
    else
    {
        *tightPtr = false;
    }

    if (vm.count("number-samples"))
    {
        *numSamplesPtr = vm["number-samples"].as<unsigned int>();
        if ( (*numSamplesPtr < 1) )
        {
            std::cout << "number-samples must be >= 1" << std::endl << std::endl << desc << std::endl;
            return false;
        }
    }
    else
    {
        std::cout << "number-samples not set" << std::endl << std::endl << desc << std::endl;
        return false;
    }

    if (vm.count("foci-distance"))
    {
        *fociDistPtr = vm["foci-distance"].as<double>();

        if (*tightPtr == false)
        {
            std::cout << "Foci distance cannot be set if not performing tightly-bounded sampling." << std::endl << std::endl << desc << std::endl;
            return false;
        }
    }
    else if (*tightPtr == true)
    {
        std::cout << "Foci distance must be set for tightly-bounded sampling." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    if (vm.count("transverse-diameter"))
    {
        *transDiaPtr = vm["transverse-diameter"].as<double>();

        if (*tightPtr == false)
        {
            std::cout << "Transverse diameter cannot be set if not performing tightly-bounded sampling." << std::endl << std::endl << desc << std::endl;
            return false;
        }
    }
    else if (*tightPtr == true)
    {
        std::cout << "Transverse diameter must be set for tightly-bounded sampling." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    *minCostPtr = vm["min-cost"].as<double>();

    if (vm.count("max-cost"))
    {
        *maxCostPtr = vm["max-cost"].as<double>();
    }
    else if (*tightPtr == true)
    {
        *maxCostPtr = *transDiaPtr;
    }
    else
    {
        std::cout << "max-cost not set" << std::endl << std::endl << desc << std::endl;
        return false;
    }

    *numStartsPtr = vm["num-starts"].as<unsigned int>();
    *numGoalsPtr = vm["num-goals"].as<unsigned int>();

    if (*tightPtr == true && (*numStartsPtr != 1u || *numGoalsPtr != 1u))
    {
        std::cout << "Only 1 start and goal is supported when performing tight-bounded sampling." << std::endl << std::endl << desc << std::endl;
    }

    if (*numStartsPtr != 1u && *numGoalsPtr != 1u)
    {
        std::cout << "For this test function, either the start or goal must be a single state." << std::endl << std::endl << desc << std::endl;
    }


    if (vm.count("log-level"))
    {
        unsigned int logLevel;

        logLevel = vm["log-level"].as<unsigned int>();
        if (logLevel == 0u)
        {
            ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
        }
        else if (logLevel == 1u)
        {
            ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
        }
        else if (logLevel == 2u)
        {
            ompl::msg::setLogLevel(ompl::msg::LOG_DEBUG);
        }
        else
        {
            std::cout << "invalid log-level" << std::endl << std::endl << desc << std::endl;
            return false;
        }
    }
    else
    {
        ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
    }

    return true;
}

std::vector<const ompl::base::State*> scopedToState(std::vector<ompl::base::ScopedState<> > scopedStates)
{
    //Return value
    std::vector<const ompl::base::State*> states;

    for (unsigned int i = 0u; i < scopedStates.size(); ++i)
    {
        states.push_back(scopedStates.at(i).get());
    }

    return states;
}

double costValue(const std::vector<double>& state1, const std::vector<double>& state2)
{
    //Variable
    //Start with no cost:
    double costValue = 0.0;

    //Sum the squares
    for (unsigned int i = 0u; i < state1.size(); ++i)
    {
        costValue = costValue + std::pow(state1.at(i) - state2.at(i), 2.0);
    }

    //Return the square root
    return std::sqrt(costValue);
}

ompl::base::Cost costToGo(const ompl::base::SpaceInformationPtr& si, unsigned int numGoals, const ompl::base::State* state, const ompl::base::Goal* goal)
{
    // The state as std::vector
    std::vector<double> stateVector;

    //Copy the state:
    si->getStateSpace()->copyToReals(stateVector, state);

    //One goal is easy:
    if (numGoals == 1u)
    {
        //Variable
        //The goal as std::vector
        std::vector<double> goalVector;

        //Copy the goal
        si->getStateSpace()->copyToReals(goalVector, goal->as<ompl::base::GoalState>()->getState());

        //Return the cost value:
        return ompl::base::Cost(costValue(stateVector, goalVector));
    }
    else
    {
        //Find the min:

        //Variable
        //The best cost to date:
        double minValue = ASRL_DBL_INFINITY;

        //Iterate over the goals:
        for (unsigned int k = 0u; k < numGoals; ++k)
        {
            //Variable
            //The goal as std::vector
            std::vector<double> goalVector;

            //Copy the goal
            si->getStateSpace()->copyToReals(goalVector, goal->as<ompl::base::GoalStates>()->getState(k));


            //Get the best cost:
            minValue = std::min(minValue, costValue(stateVector, goalVector));
        }

        //Return the min:
        return ompl::base::Cost(minValue);
    }
}

void plotSample(BaseExperimentPtr expPtr, unsigned int n, std::ofstream* mfilePtr, ompl::base::State* statePtr, std::string styleString)
{
    if (n == 2u || n == 3u)
    {
        std::vector<double> stateVector;
        expPtr->getSpaceInformation()->getStateSpace()->copyToReals(stateVector, statePtr);

        //Plot
        if (n == 2u)
        {
            *mfilePtr << "plot(" << stateVector.at(0u) << ", " << stateVector.at(1u) << ", '" << styleString << "');" << std::endl;
        }
        else
        {
            *mfilePtr << "plot3(" << stateVector.at(0u) << ", " << stateVector.at(1u) << ", " << stateVector.at(2u) << ", '" << styleString << "');" << std::endl;
        }
    }
    //No else, this isn't a plottable dimension
}


int main(int argc, char **argv)
{
    //Variables
    //ompl::RNG::setSeed(4059103542);   std::cout << std::endl << "                   ---------> Seed set! <---------                   " << std::endl << std::endl;//Master seed:
    std::uint_fast32_t masterSeed = ompl::RNG::getSeed();

    //The number of samples to create
    unsigned int targetSamples;
    //THe minimum cost to sample
    double minCostValue;
    //The maximum cost to sample
    double maxCostValue;
    //Whether to perform tight bounding or not
    bool tightBounding;
    //The foci and transverse distances
    double dFoci;
    double dTrans;
    //The number of starts
    unsigned int maxNumStarts;
    unsigned int maxNumGoals;
    //Whether to perform rejection sampling
    bool rejSample;
    //Whether to plot results
    bool plotResults;
    //The max dimension
    unsigned int maxN = 16u;
    //The number of attempts per sample:
    unsigned int numAttempts = 1u;
    //The filestream for R2
    std::ofstream mfile;
    //The string stream of the filename for R2
    std::stringstream fileName;


    //Get the command line arguments
    if (argParse(argc, argv, &targetSamples, &minCostValue, &maxCostValue, &rejSample, &tightBounding, &dFoci, &dTrans, &maxNumStarts, &maxNumGoals, &plotResults) == false)
    {
        return 1;
    }

    std::cout << "Seed: " << masterSeed << " Samples: " << targetSamples << std::endl;

    for (unsigned int i = 2u; i <= maxN; ++i)
    {
        //The experiment definition
        BaseExperimentPtr expPtr;

        if (tightBounding == false)
        {
            expPtr = std::make_shared<ObstacleFreeExperiment> (i, maxNumStarts, maxNumGoals, 1.0); //1.0 is gibberish runtime, not used.
        }
        else
        {
            expPtr = std::make_shared<TightlyBoundingRectangle> (i, dFoci, dTrans, 1.0); //1.0 is gibberish runtime, not used.
        }
        //The problem definition
        ompl::base::ProblemDefinitionPtr pdef = expPtr->newProblemDefinition();
        //The samplers:
        ompl::base::StateSamplerPtr rawSampler;
        ompl::base::InformedSamplerPtr directInfSampler;
        ompl::base::InformedSamplerPtr rejInfSampler;
        //The time spent sampling
        asrl::time::duration rawTime(0);
        asrl::time::duration directTime(0);
        asrl::time::duration rejectTime(0);
        //The number of samples drawn
        unsigned int numSampled = 0u;
        unsigned int numFailed = 0u;

        //Let people know what's going on:

        //Create the header:
        if (i == 2u)
        {
            //The first column is 40 wide:
            std::cout << std::setw(40) << std::setfill(' ') << " ";
            //Then output the type of sampling. Time is atleast 15 wide:
            std::cout << std::setw(20) << std::setfill(' ') << "Entire space";
            std::cout << std::setw(20) << std::setfill(' ') << "Direct";
            if (tightBounding == true)
            {
                std::cout << std::setw(20) << std::setfill(' ') << "(Est. Reject)";
            }
            if (rejSample == true)
            {
                std::cout << std::setw(20) << std::setfill(' ') << "Rejection";
            }
            std::cout << std::endl;
        }

        //Create the first column:
        std::stringstream column1;

        column1 << "R^" << i << ": f(x) = [" << minCostValue << ", " << maxCostValue << ").";
        if (tightBounding == true)
        {
            column1 << " PHS/X: " << std::setprecision(2) << ompl::prolateHyperspheroidMeasure(i, dFoci, dTrans)/expPtr->getSpaceInformation()->getSpaceMeasure();
        }
        //No else
        std::cout << std::setw(40) << std::setfill(' ') << column1.str() << std::flush;

        //Allocate the samplers:
        rawSampler = expPtr->getSpaceInformation()->allocStateSampler();
        directInfSampler = expPtr->getOptimizationObjective()->allocInformedStateSampler(pdef, numAttempts);
        rejInfSampler = std::make_shared<ompl::base::RejectionInfSampler>(pdef, numAttempts);

        //Make an mfile
        if (plotResults == true && (i == 2u || i == 3u))
        {
            fileName.str("");
            fileName.clear();

            fileName << "SamplingR" << i << "S" << masterSeed << ".m";

            mfile.open(fileName.str().c_str());
            mfile << "figure;" << std::endl;
            mfile << "hold on;" << std::endl;
        }

        //Perform raw sampling:
        numSampled = 0u;
        while (numSampled < targetSamples)
        {
            //Variable
            //The state:
            ompl::base::State* statePtr = NULL;
            //The start time
            asrl::time::point startTime;

            //Allocate
            statePtr = expPtr->getSpaceInformation()->allocState();

            //Calculate the time for a sample from the general space
            startTime = asrl::time::now();
            rawSampler->sampleUniform(statePtr);
            rawTime = (asrl::time::now() - startTime) + rawTime;

            //Increment the number of samples
            ++numSampled;

            //Free the state
            expPtr->getSpaceInformation()->freeState(statePtr);
        }

        //Output the time:
        //Column width is 20 and time is 15 chars long but does not respond to setfill
        std::cout << std::setw(5) << std::setfill(' ') << " ";
        std::cout << rawTime << std::flush;

        //Perform direct sampling:
        numSampled = 0u;
        while (numSampled < targetSamples)
        {
            //Variable
            //The state:
            ompl::base::State* statePtr = NULL;
            //The start time
            asrl::time::point startTime;
            //The return value
            bool successRval = false;
            //The costs:
            ompl::base::Cost minCost (minCostValue);
            ompl::base::Cost maxCost (maxCostValue);

            //Allocate
            statePtr = expPtr->getSpaceInformation()->allocState();

            //Generate a sample:
            if (minCostValue == 0.0)
            {
                startTime = asrl::time::now();
                successRval = directInfSampler->sampleUniform(statePtr, maxCost);
                directTime = (asrl::time::now() - startTime) + directTime;
            }
            else
            {
                startTime = asrl::time::now();
                successRval = directInfSampler->sampleUniform(statePtr, minCost, maxCost);
                directTime = (asrl::time::now() - startTime) + directTime;
            }

            //If successful, increment the number
            if (successRval == true)
            {
                //Increment the number of successful samples
                ++numSampled;

                //Plot if appropriate
                if (plotResults == true)
                {
                    plotSample(expPtr, i, &mfile, statePtr, "c.");
                }
            }
            else
            {
                ++numFailed;
            }

            //Free the state
            expPtr->getSpaceInformation()->freeState(statePtr);
        }
        //Output results:
        std::cout << std::setw(5) << std::setfill(' ') << " ";
        std::cout << directTime << std::flush; //<< "(" << static_cast<double>(numSampled)/static_cast<double>(numFailed+numSampled) << ")";

        //Predicted rejection sampling:
        if (tightBounding == true)
        {
            //This was all broken out as a result of the boost->std time change. Kind of untested
            double timeFraction;
            double predictedTimeSecs;

            timeFraction = 1/(ompl::prolateHyperspheroidMeasure(i, dFoci, dTrans)/expPtr->getSpaceInformation()->getSpaceMeasure());
            predictedTimeSecs = timeFraction*asrl::time::seconds(rawTime);

            std::cout << std::setw(5) << std::setfill(' ') << " ";
            std::cout << asrl::time::seconds(predictedTimeSecs) << std::flush;
        }

        //Rejection sample, if asked for
        if (rejSample == true)
        {
            numSampled = 0u;
            numFailed = 0u;
            while (numSampled < targetSamples)
            {
                //Variable
                //The state:
                ompl::base::State* statePtr = NULL;
                //The start time
                asrl::time::point startTime;
                //The return value
                bool successRval = false;
                //The costs:
                ompl::base::Cost minCost (minCostValue);
                ompl::base::Cost maxCost (maxCostValue);

                //Allocate
                statePtr = expPtr->getSpaceInformation()->allocState();

                //Generate a sample:
                if (minCostValue == 0.0)
                {
                    startTime = asrl::time::now();
                    successRval = rejInfSampler->sampleUniform(statePtr, maxCost);
                    rejectTime = (asrl::time::now() - startTime) + rejectTime;
                }
                else
                {
                    startTime = asrl::time::now();
                    successRval = rejInfSampler->sampleUniform(statePtr, minCost, maxCost);
                    rejectTime = (asrl::time::now() - startTime) + rejectTime;
                }

                //If successful, increment the number
                if (successRval == true)
                {
                    //Increment the number of successful samples
                    ++numSampled;

                    //Plot if appropriate
                    if (plotResults == true)
                    {
                        plotSample(expPtr, i, &mfile, statePtr, "m.");
                    }
                }
                else
                {
                    ++numFailed;
                }

                //Free the state
                expPtr->getSpaceInformation()->freeState(statePtr);
            }

            std::cout << std::setw(5) << std::setfill(' ') << " ";
            std::cout << rejectTime << std::flush; //<< "(" << static_cast<double>(numSampled)/static_cast<double>(numFailed+numSampled) << ")";
        }
        std::cout << std::endl;

        // Finish the plot file.
        if (plotResults == true && (i == 2u || i == 3u))
        {
            //Plot the start and goal
            mfile << "xstarts = [" ;
            for (unsigned int k = 0u; k < expPtr->getStartStates().size(); ++k)
            {
                mfile << " [";
                for (unsigned int j = 0u; j < i; ++j)
                {
                    mfile << expPtr->getStartStates().at(k)[j];
                    if (j != i - 1u)
                    {
                        mfile << "; ";
                    }
                }
                mfile << "] ";
            }
            mfile << "];" << std::endl;

            mfile << "xgoals = [";
            for (unsigned int k = 0u; k < expPtr->getGoalStates().size(); ++k)
            {
                mfile << " [";
                for (unsigned int j = 0u; j < i; ++j)
                {
                    mfile << expPtr->getGoalStates().at(k)[j];
                    if (j != i - 1u)
                    {
                        mfile << "; ";
                    }
                }
                mfile << "] ";
            }
            mfile << "];" << std::endl;

            if (i == 2u)
            {
                mfile << "for i = 1:size(xstarts,2)" << std::endl;
                mfile << "    plot(xstarts(1,i), xstarts(2,i), 'g.');" << std::endl;
                mfile << "end" << std::endl;
                mfile << "for i = 1:size(xgoals,2)" << std::endl;
                mfile << "    plot(xgoals(1,i), xgoals(2,i), 'r.');" << std::endl;
                mfile << "end" << std::endl;
            }
            else
            {
                mfile << "for i = 1:size(xstarts,2)" << std::endl;
                mfile << "    plot3(xstarts(1,i), xstarts(2,i), xstarts(3,i), 'g.');" << std::endl;
                mfile << "end" << std::endl;
                mfile << "for i = 1:size(xgoals,2)" << std::endl;
                mfile << "    plot3(xgoals(1,i), xgoals(2,i), xgoals(3,i), 'r.');" << std::endl;
                mfile << "end" << std::endl;
            }

            //And the ellipses:
            if (i == 2u)
            {
                if (minCostValue != 0.0)
                {
                    mfile << "for  i = 1:size(xstarts,2)" << std::endl;
                    mfile << "    for  j = 1:size(xgoals,2)" << std::endl;
                    mfile << "        Smin = ellipseMatrix(xstarts(:,i), xgoals(:,j)," << minCostValue << ");" << std::endl;
                    mfile << "        plotEllipseMatrix(Smin, 0.5.*(xstarts(:,i) + xgoals(:,j)), 100);" << std::endl;
                    mfile << "    end" << std::endl;
                    mfile << "end" << std::endl;
                }

                mfile << "for  i = 1:size(xstarts,2)" << std::endl;
                mfile << "    for  j = 1:size(xgoals,2)" << std::endl;
                mfile << "        Smax = ellipseMatrix(xstarts(:,i), xgoals(:,j)," << maxCostValue << ");" << std::endl;
                mfile << "        plotEllipseMatrix(Smax, 0.5.*(xstarts(:,i) + xgoals(:,j)), 100);" << std::endl;
                mfile << "    end" << std::endl;
                mfile << "end" << std::endl;
            }

            mfile << "axis equal;" << std::endl;
            mfile << "xlim([" << expPtr->getLimits().at(0u).first << ", " << expPtr->getLimits().at(0u).second << "]);" << std::endl;
            mfile << "ylim([" << expPtr->getLimits().at(1u).first << ", " << expPtr->getLimits().at(1u).second << "]);" << std::endl;
            if (i == 3u)
            {
                mfile << "zlim([" << expPtr->getLimits().at(2u).first << ", " << expPtr->getLimits().at(2u).second << "]);" << std::endl;
            }
            mfile.close();
        }
    }
    return 0;
}
