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

/*********************************************************************
THIS CODE ONLY COMPILES ON THE set_rrtstar_seeds BRANCH!!!!
*********************************************************************/

//For std::cout
#include <iostream>
//For std::ifstream and std::ofstream
#include <fstream>
//For std::setprecision
#include <iomanip>
//For std::stringstream
#include <sstream>
//For boost::make_shared
#include <boost/make_shared.hpp>
//For boost program options
#include <boost/program_options.hpp>
//For boost lexical cast
#include <boost/lexical_cast.hpp>
//For boost time
#include <boost/date_time/posix_time/posix_time.hpp>
//For boost thread:
#include <boost/thread/thread.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>

//OMPL:
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
//#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
//#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/util/Time.h>
#include "ompl/util/Console.h" //For OMPL_INFORM et al.
#include "ompl/tools/config/MagicConstants.h" //For BETTER_PATH_COST_MARGIN
#include <ompl/util/Exception.h>

//Our Experiments
#include "ExperimentDefinitions.h"

//The helper functions for plotting:
#include "tools/plotting_tools.h"

#define ASRL_DBL_INFINITY std::numeric_limits<double>::infinity()



//World:
const double CHECK_RESOLUTION = 0.001;
const unsigned int MEAN_NUM_OBS = 75u;
const double MEAN_OBS_RATIO = 0.33;
const double MILLISEC_SLEEP = 1.0; //Period for logging data

//Common:
const double PRUNE_FRACTION = 0.05;
const bool K_NEAREST = false;
const double REWIRE_SCALE = 2.0; //The factor scaling the RGG term

//BITstar
const unsigned int BITSTAR_BATCH_SIZE = 100u;
const bool BITSTAR_STRICT_QUEUE = true;
const bool BITSTAR_DELAY_REWIRE = false;
const bool BITSTAR_JIT = false;
const bool BITSTAR_DROP_BATCHES = false;

//Others:
const double RRT_GOAL_BIAS = 0.05; //8D: 0.05; //2D: 0.05
const bool FMT_CACHE_CC = false;
const bool FMT_USE_HEURISTICS = false;

//Plotting:
const bool PLOT_WORLD_ELLIPSE = true;
const bool PLOT_BITSTAR_ELLIPSE = true;
const bool PLOT_BITSTAR_EDGE = true;
const bool PLOT_BITSTAR_QUEUE = false;


bool argParse(int argc, char** argv, unsigned int* dimensionPtr, double* steerPtr, unsigned int* numExperimentsPtr, double* runTimePtr, bool* animatePtr)
{
    // Declare the supported options.
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("state,r", boost::program_options::value<unsigned int>(), "The state dimension.")
        ("steer-eta,s", boost::program_options::value<double>()->default_value(0.0), "The steer eta, or maximum edge length, to use. Defaults to 0.0 which uses the OMPL auto calculation.")
        ("experiments,e", boost::program_options::value<unsigned int>(), "The number of unique experiments to run on the random world.")
        ("runtime,t", boost::program_options::value<double>(), "The CPU time in seconds for which to run the planners, (0,infty)")
        ("animate,a", boost::program_options::value<bool>()->zero_tokens(), "Create frame-by-frame animations.")
        ("log-level,l", boost::program_options::value<unsigned int>(), "Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG. Defaults to 0 if not set.");
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return false;
    }

    if (vm.count("state"))
    {
        *dimensionPtr = vm["state"].as<unsigned int>();
        if (*dimensionPtr  < 2)
        {
            std::cout << "State dimension must be greater or equal to 2. " << std::endl << std::endl << desc << std::endl;
            return false;
        }
    }
    else
    {
        std::cout << "state dimension not set" << std::endl << std::endl << desc << std::endl;
        return false;
    }

    *steerPtr = vm["steer-eta"].as<double>();

    if (vm.count("experiments"))
    {
        *numExperimentsPtr = vm["experiments"].as<unsigned int>();
    }
    else
    {
        std::cout << "Number of experiments not set" << std::endl << std::endl << desc << std::endl;
        return false;
    }

    if (vm.count("runtime"))
    {
        *runTimePtr = vm["runtime"].as<double>();
        if ( (*runTimePtr < 0) )
        {
            std::cout << "runtime must be greater than, or equal to, 0" << std::endl << std::endl << desc << std::endl;
            return false;
        }

        if (*runTimePtr == 0.0)
        {
            *runTimePtr = ASRL_DBL_INFINITY;
        }
    }
    else
    {
        std::cout << "runtime not set" << std::endl << std::endl << desc << std::endl;
        return false;
    }

    if (vm.count("animate"))
    {
        *animatePtr = true;
    }
    else
    {
        *animatePtr = false;
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
};

ompl::base::PlannerPtr allocatePlanner(const PlannerType plnrType, const BaseExperimentPtr& expDefn, const double steerEta, const unsigned int numSamples)
{
    switch(plnrType)
    {
        case PLANNER_RRT:
        {
            return allocateRrt(expDefn->getSpaceInformation(), steerEta, RRT_GOAL_BIAS);
            break;
        }
        case PLANNER_RRTCONNECT:
        {
            return allocateRrtConnect(expDefn->getSpaceInformation(), steerEta);
            break;
        }
        case PLANNER_RRTSTAR:
        {
            return allocateRrtStar(expDefn->getSpaceInformation(), steerEta, RRT_GOAL_BIAS, K_NEAREST, REWIRE_SCALE);
            break;
        }
        case PLANNER_RRTSTAR_INFORMED:
        {
            return allocateInformedRrtStar(expDefn->getSpaceInformation(), steerEta, RRT_GOAL_BIAS, K_NEAREST, REWIRE_SCALE, PRUNE_FRACTION);
            break;
        }
        case PLANNER_FMTSTAR:
        {
            return allocateFmtStar(expDefn->getSpaceInformation(), K_NEAREST, REWIRE_SCALE, numSamples, FMT_CACHE_CC, FMT_USE_HEURISTICS);
            break;
        }
        case PLANNER_BITSTAR:
        {
            return allocateBitStar(expDefn->getSpaceInformation(), K_NEAREST, REWIRE_SCALE, numSamples, PRUNE_FRACTION, BITSTAR_STRICT_QUEUE, BITSTAR_DELAY_REWIRE, BITSTAR_JIT, BITSTAR_DROP_BATCHES);
            break;
        }
        default:
        {
            throw ompl::Exception("Unrecognized planner type in allocatePlanner()");
        }
    }
};

void callSolve(const ompl::base::PlannerPtr& planner, const ompl::time::duration& solveDuration)
{
    //planner->solve( solveDuration );
    planner->solve( ompl::time::seconds(solveDuration) );
}

double currentSolution(const PlannerType& plnrType, const ompl::base::PlannerPtr& plnr)
{
    if (isRrtStar(plnrType) == true)
    {
        return plnr->as<ompl::geometric::RRTstar>()->bestCost().value();
    }
    else if (isBitStar(plnrType) == true)
    {
        return plnr->as<ompl::geometric::BITstar>()->bestCost().value();
    }
    else
    {
        throw ompl::Exception("Unrecognized planner type in currentSolution");
    }
};

int main(int argc, char **argv)
{
    //Argument Variables
    //The dimension size:
    unsigned int N;
    //The number of experiments
    unsigned int numExperiments;
    //The time for which to run the planners
    double maxTime;
    //The steer / range of RRT
    double steerEta;
    //Whether to make frame-by-frame animations
    bool createAnimationFrames;

    //Get the command line arguments
    if (argParse(argc, argv, &N, &steerEta, &numExperiments, &maxTime, &createAnimationFrames) == false)
    {
        return 1;
    }

    //Variables
//    ompl::RNG::setSeed(3330966526);    std::cout << std::endl << "                   ---------> Seed set! <---------                   " << std::endl << std::endl;
    //Master seed:
    boost::uint32_t masterSeed = ompl::RNG::getSeed();
    //The filename for progress
    std::stringstream fileName;
    //The world name
    std::stringstream worldName;
    //The vector of planner types:
    std::vector<std::pair<PlannerType, unsigned int> > plannersToTest;
    //The experiment
    RandomRectanglesExperimentPtr experiment;

    //Specify the planners:
    plannersToTest.push_back(std::make_pair(PLANNER_RRTCONNECT, 0u));
    plannersToTest.push_back(std::make_pair(PLANNER_RRT, 0u));
    plannersToTest.push_back(std::make_pair(PLANNER_RRTSTAR, 0u));
    plannersToTest.push_back(std::make_pair(PLANNER_RRTSTAR_INFORMED, 0u));
    plannersToTest.push_back(std::make_pair(PLANNER_FMTSTAR, 100u));
    plannersToTest.push_back(std::make_pair(PLANNER_FMTSTAR, 1000u));
    plannersToTest.push_back(std::make_pair(PLANNER_FMTSTAR, 10000u));
    if (N == 2u)
    {
        plannersToTest.push_back(std::make_pair(PLANNER_FMTSTAR, 100000u));
        plannersToTest.push_back(std::make_pair(PLANNER_FMTSTAR, 250000u));
    }
    else if (N == 8u)
    {
        plannersToTest.push_back(std::make_pair(PLANNER_FMTSTAR, 25000u));
    }
    else if (N == 16u)
    {
        plannersToTest.push_back(std::make_pair(PLANNER_FMTSTAR, 25000u));
    }
    plannersToTest.push_back(std::make_pair(PLANNER_BITSTAR, BITSTAR_BATCH_SIZE));

    //Create one experiment for all runs:
    experiment = boost::make_shared<RandomRectanglesExperiment>(N, MEAN_NUM_OBS, MEAN_OBS_RATIO, maxTime, CHECK_RESOLUTION);

    //The results output file:
    fileName << "R" << N << "S" << masterSeed << experiment->getName() << ".csv";

    //Let people know what's going on:
    std::cout << "Seed: " << masterSeed << std::endl;
    std::cout << "Output: " << fileName.str() << std::endl;

    ResultsFile<TimeCostHistory> progressHistory(fileName.str());

    //Perform numRuns
    for (unsigned int q = 0u; q < numExperiments; ++q)
    {
        //A RNG, we will use it's seed for all the planners, but not actually use the RNG...
        ompl::RNG seedRNG;
        bool badExperiment = false;

        //Iterate over the planners:
        for (unsigned int p = 0u; p < plannersToTest.size() && badExperiment == false; ++p)
        {
            //Variables
            //The start time for a call
            ompl::time::point startTime;
            //The run time
            ompl::time::duration runTime(0,0,0,0);
            //The current planner:
            ompl::base::PlannerPtr plnr;
            //The problem defintion used by this planner
            ompl::base::ProblemDefinitionPtr pdef;
            //The results from this planners run across all the variates:
            TimeCostHistory runResults(experiment->getTargetTime(), MILLISEC_SLEEP);
            //The final cost of this planner:
            ompl::base::Cost finalCost;

            //Allocate a planner
            plnr = allocatePlanner(plannersToTest.at(p).first, experiment, steerEta, plannersToTest.at(p).second);

            //Get the problem definition
            pdef = experiment->newProblemDefinition();

            //Give to the planner
            plnr->setProblemDefinition(pdef);

            //Setup
            startTime = ompl::time::now();
            plnr->setup();
            runTime = ompl::time::now() - startTime;

            //Set the planner seed:
            if (isRrtStar(plannersToTest.at(p).first) == true)
            {
                plnr->as<ompl::geometric::RRTstar>()->setLocalSeed(seedRNG.getLocalSeed());
            }

            //This must come after setup to get the steer info!
            //If this is the first planner of a trial, output at least the trial number:
            if (p == 0u)
            {
                //If this is also the first run ever, output start info:
                if (q == 0u)
                {
                    std::cout << "First experiment: " << std::endl;
                    experiment->print(false);
                    //The first column is 7 wide for a 6-digit trial number and a ":"
                    std::cout << std::setw(7) << std::setfill(' ') << " ";

                    //The subsequent columns are 30 wide for planner names, skip the first planner which is the reference solution:
                    for (unsigned int i = 0u; i < plannersToTest.size(); ++i)
                    {
                        std::cout << std::setw(30) << std::setfill(' ') << plannerName(plannersToTest.at(i).first) << std::flush;
                    }
                    std::cout << std::endl;
                }
                //No else

                //The trial number is 6-digits with 3 of padding
                std::cout << std::setw(6) << std::setfill(' ') << q << ":" << std::flush;
            }
            //No else

            //Run the planner:
            if (createAnimationFrames == true)
            {
                runTime = runTime + createAnimation(experiment, plannersToTest.at(p).first, plnr, masterSeed, experiment->getTargetTime() - runTime, PLOT_WORLD_ELLIPSE, PLOT_BITSTAR_ELLIPSE, PLOT_BITSTAR_EDGE, PLOT_BITSTAR_QUEUE);
                runResults.push_back( std::make_pair(runTime, currentSolution(plannersToTest.at(p).first, plnr)) );
            }
            else
            {
                //Store the starting time:
                startTime = ompl::time::now();

                //Start solving in another thread. We use an intermediate function for this as the class function is overloaded.
                boost::thread solveThread(callSolve, plnr, experiment->getTargetTime() - runTime);

                //Log data
                do
                {
                    if (isRrt(plannersToTest.at(p).first) == true || plannersToTest.at(p).first == PLANNER_FMTSTAR)
                    {
                        //Do nothing, these planners have no intermediate data to store
                    }
                    else
                    {
                        //Store the runtime and the current cost
                        runResults.push_back( std::make_pair(ompl::time::now() - startTime + runTime, currentSolution(plannersToTest.at(p).first, plnr)) );
                    }
                }
                while ( solveThread.timed_join(boost::posix_time::milliseconds(MILLISEC_SLEEP)) == false);

                //Store the final run time and cost
                runTime = runTime + (ompl::time::now() - startTime);

                //Find the final cost
                if (pdef->hasExactSolution() == false)
                {
                    //The final cost is infinite
                    finalCost = ompl::base::Cost(ASRL_DBL_INFINITY);

                    //If we didn't find a solution on the first attempt, mark as a bad experiment
                    if (p == 0u)
                    {
                        badExperiment = true;
                    }
                }
                else
                {
                    //The final cost is finite
                    finalCost = pdef->getSolutionPath()->cost(experiment->getOptimizationObjective());
                }

                //Is this a good experiment?
                if (badExperiment == true)
                {
                    //It's not a good experiment, so try a different one
                    --q;
                }
                else
                {
                    //It is, store the results
                    runResults.push_back( std::make_pair(runTime, finalCost.value()) );
                    progressHistory.addResult(plnr->getName(), runResults);
                }
            }

            //Save the map:
            if (plannersToTest.at(p).second <= 5000u)
            {
                writeMatlabMap(experiment, plannersToTest.at(p).first, plnr, masterSeed, PLOT_WORLD_ELLIPSE, PLOT_BITSTAR_ELLIPSE, PLOT_BITSTAR_EDGE, PLOT_BITSTAR_QUEUE, "plots/");
            }

            if (badExperiment == false)
            {
                //Output 3 characters of white space and then the time, which is 15 chars wide:
                std::cout << std::setw(5) << std::setfill(' ') << " " << std::flush;
                std::cout << runTime << std::flush;
                //Then a comma and 1 space,
                std::cout << " " << std::flush;
                //And the cost using the remaining 30 - 5 - 15 - 1 = 9 characters:
                std::cout << std::setprecision(6) << std::setw(9) << std::setfill(' ') << finalCost.value() << std::flush;
            }
        }
        //Get the next planner
        std::cout << std::endl;
    }
    //Perform the next experiment

    return 0;
}
