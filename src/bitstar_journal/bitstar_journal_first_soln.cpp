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
//For std::make_shared
#include <memory>
//For boost program options
#include <boost/program_options.hpp>
//For boost lexical cast
#include <boost/lexical_cast.hpp>
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
#include "ompl/util/Console.h" //For OMPL_INFORM et al.
#include "ompl/tools/config/MagicConstants.h" //For BETTER_PATH_COST_MARGIN
#include <ompl/util/Exception.h>

//Our Experiments
#include "ExperimentDefinitions.h"

//The helper functions for general and plotting:
#include "tools/general_tools.h"
#include "tools/plotting_tools.h"

#define ASRL_DBL_INFINITY std::numeric_limits<double>::infinity()



//World:
const double CHECK_RESOLUTION = 0.001;

//Common:
const double PRUNE_FRACTION = 0.01;
const bool K_NEAREST = false;
const double REWIRE_SCALE = 2.0;

//BITstar
const double BITSTAR_REWIRE_SCALE = REWIRE_SCALE;
const unsigned int BITSTAR_BATCH_SIZE = 100u;
const bool BITSTAR_STRICT_QUEUE = true;
const bool BITSTAR_DELAY_REWIRE = false;
const bool BITSTAR_JIT = false;
const bool BITSTAR_DROP_BATCHES = false;

//Others:
const unsigned int SORRTSTAR_BATCH_SIZE = 100u;
const double RRT_REWIRE_SCALE = REWIRE_SCALE;
const double RRT_GOAL_BIAS = 0.0;
const double FMT_REWIRE_SCALE = REWIRE_SCALE;
const bool FMT_CACHE_CC = false;
const bool FMT_USE_HEURISTICS = false;

//Plotting:
const bool PLOT_VERTICES = true;
const bool PLOT_WORLD_ELLIPSE = true;
const bool PLOT_BITSTAR_ELLIPSE = true;
const bool PLOT_BITSTAR_EDGE = true;
const bool PLOT_BITSTAR_QUEUE = false;

enum ProblemType
{
    TIME_V_GAPSIZE,
    TIME_V_MAPSIZE,
    TIME_V_TARGET
};


bool argParse(int argc, char** argv, unsigned int* dimensionPtr, double* steerPtr, unsigned int* numTrialsPtr, double* runTimePtr, bool* animatePtr)
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
        *numTrialsPtr = vm["experiments"].as<unsigned int>();
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
            return allocateRrtStar(expDefn->getSpaceInformation(), steerEta, RRT_GOAL_BIAS, K_NEAREST, RRT_REWIRE_SCALE);
            break;
        }
        case PLANNER_RRTSTAR_INFORMED:
        {
            return allocateInformedRrtStar(expDefn->getSpaceInformation(), steerEta, RRT_GOAL_BIAS, K_NEAREST, RRT_REWIRE_SCALE, PRUNE_FRACTION);
            break;
        }
        case PLANNER_SORRTSTAR:
        {
            return allocateSorrtStar(expDefn->getSpaceInformation(), steerEta, RRT_GOAL_BIAS, K_NEAREST, RRT_REWIRE_SCALE, PRUNE_FRACTION, numSamples);
            break;
        }
        case PLANNER_FMTSTAR:
        {
            return allocateFmtStar(expDefn->getSpaceInformation(), K_NEAREST, FMT_REWIRE_SCALE, numSamples, FMT_CACHE_CC, FMT_USE_HEURISTICS);
            break;
        }
        case PLANNER_BITSTAR:
        {
            return allocateBitStar(expDefn->getSpaceInformation(), K_NEAREST, BITSTAR_REWIRE_SCALE, numSamples, PRUNE_FRACTION, BITSTAR_STRICT_QUEUE, BITSTAR_DELAY_REWIRE, BITSTAR_JIT, BITSTAR_DROP_BATCHES);
            break;
        }
        default:
        {
            throw ompl::Exception("Unrecognized planner type in allocatePlanner()");
        }
    }
};

int main(int argc, char **argv)
{
    //Argument Variables
    //The dimension size:
    unsigned int N;
    //The number of experiments
    unsigned int numRuns;
    //The time for which to run the planners
    double maxTime;
    //The steer / range of RRT
    double steerEta;
    //Whether to make frame-by-frame animations
    bool createAnimationFrames;

    //Get the command line arguments
    if (argParse(argc, argv, &N, &steerEta, &numRuns, &maxTime, &createAnimationFrames) == false)
    {
        return 1;
    }

    //Variables
//    ompl::RNG::setSeed(2971235666);    std::cout << std::endl << "                   ---------> Seed set! <---------                   " << std::endl << std::endl;
    //Master seed:
    std::uint_fast32_t masterSeed = ompl::RNG::getSeed();
    //The filename for progress
    std::stringstream fileName;
    //The world name
    std::stringstream worldName;
    //The vector of planner types:
    std::vector<std::pair<PlannerType, unsigned int> > plannersToTest;
    //The experiment definitions for this run
    BaseExperimentPtr expDefn = std::make_shared<ObstacleFreeExperiment>(N, 1u, 1u, maxTime);;

    //Specify the planners:
//    plannersToTest.push_back(std::make_pair(PLANNER_RRTCONNECT, 0u));
    plannersToTest.push_back(std::make_pair(PLANNER_RRT, 0u));
//    plannersToTest.push_back(std::make_pair(PLANNER_RRTSTAR, 0u));
//    plannersToTest.push_back(std::make_pair(PLANNER_FMTSTAR, 100u));
//    plannersToTest.push_back(std::make_pair(PLANNER_FMTSTAR, 1000u));
//    plannersToTest.push_back(std::make_pair(PLANNER_FMTSTAR, 10000u));
//    plannersToTest.push_back(std::make_pair(PLANNER_RRTSTAR_INFORMED, 0u));
//    plannersToTest.push_back(std::make_pair(PLANNER_SORRTSTAR, SORRTSTAR_BATCH_SIZE));
//    plannersToTest.push_back(std::make_pair(PLANNER_BITSTAR, BITSTAR_BATCH_SIZE));

    //The results output file:
    fileName << "R" << N << "S" << masterSeed << "SolnTime.csv";

    //Let people know what's going on:
    std::cout << "Seed: " << masterSeed << std::endl;
    std::cout << "Output: " << fileName.str() << std::endl;

    ResultsFile<TargetTimeResults> timePerSoln(fileName.str());

    //Make the goal a goal region of radius 0.1 (?)
    expDefn->getGoalPtr()->as<ompl::base::GoalRegion>()->setThreshold(0.1);

    //Perform numRuns
    for (unsigned int q = 0u; q < numRuns; ++q)
    {
        //Variables:
        //A vector of RNGs, we will use their seeds for all the planners, but not actually use the RNG...
        std::shared_ptr<ompl::RNG> seedRNG;

        //Iterate over the planners:
        for (unsigned int p = 0u; p < plannersToTest.size(); ++p)
        {
            //Variables
            //The current planner:
            ompl::base::PlannerPtr plnr;
            //The cumulative runtime
            asrl::time::duration runTime(0);
            //The results from this planners for this one trial
            TargetTimeResults runResults(1u);
            //The start time for a call
            asrl::time::point startTime;
            //The problem defintion used by this planner
            ompl::base::ProblemDefinitionPtr pdef;

            //If this is the first planner of this experiment, create the seed for this target variable
            if (p == 0u)
            {
                seedRNG = std::make_shared<ompl::RNG>();
            }

            //allocate,
            plnr = allocatePlanner(plannersToTest.at(p).first, expDefn, steerEta, plannersToTest.at(p).second);

            //Get the problem definition
            pdef = expDefn->newProblemDefinition();

            //Give to the planner
            plnr->setProblemDefinition(pdef);

            //Setup
            startTime = asrl::time::now();
            plnr->setup();
            runTime = asrl::time::now() - startTime;

            //Set the planner seed:
            if (isRrtStar(plannersToTest.at(p).first) == true)
            {
                plnr->as<ompl::geometric::RRTstar>()->setLocalSeed(seedRNG->getLocalSeed());
            }
            if (isBitStar(plannersToTest.at(p).first) == true)
            {
                plnr->as<ompl::geometric::BITstar>()->setLocalSeed(seedRNG->getLocalSeed());
            }

            //This must come after setup to get the steer info!
            //If this is the first run of the first planner, we have info to output
            if (p == 0u)
            {
                //First, if this is also the first run ever, output start info:
                if (q == 0u)
                {
                    //The first column is 25 wide for planner names
                    std::cout << std::setw(25) << std::setfill(' ') << " ";
                    //The subsequent columns are 20 wide for 15 chars of time and 5 of padding
                    std::cout << std::setw(20) << std::setfill(' ') << ASRL_DBL_INFINITY << std::endl;
                }
                //No else

                //Then, output the trial number
                std::cout << std::setw(3) << std::setfill('0') << q << std::flush;// << std::endl;
            }
            //No else

            //Output the planner name:
            std::cout << std::setw(25) << std::setfill(' ') << plnr->getName() << std::flush;

            //Run the planner:
            if (createAnimationFrames == true)
            {
                runTime = runTime + createAnimation(expDefn, plannersToTest.at(p).first, plnr, masterSeed, expDefn->getTargetTime() - runTime, PLOT_VERTICES, PLOT_WORLD_ELLIPSE, PLOT_BITSTAR_ELLIPSE, PLOT_BITSTAR_EDGE, PLOT_BITSTAR_QUEUE);
            }
            else
            {
                //Get the end result:
                startTime = asrl::time::now();
                plnr->solve( asrl::time::seconds(expDefn->getTargetTime() - runTime) );
                runTime = runTime + (asrl::time::now() - startTime);
            }

            //Store the result:
            if (pdef->hasExactSolution() == true)
            {
                runResults.push_back(std::make_pair(ASRL_DBL_INFINITY, runTime));
            }
            else
            {
                runResults.push_back(std::make_pair(ASRL_DBL_INFINITY, ASRL_DURATION_INFINITY));
            }

//            if (plannersToTest.at(p).second <=  5000u)
//            {
//                //Save the map:
//                std::stringstream postFix;
//                postFix << problemPostfix(problemType) << indepVariables.at(v) << "E" << q;
//                writeMatlabMap(expDefn, plannersToTest.at(p).first, plnr, masterSeed, PLOT_VERTICES, PLOT_WORLD_ELLIPSE, PLOT_BITSTAR_ELLIPSE, PLOT_BITSTAR_EDGE, PLOT_BITSTAR_QUEUE, "plots/", postFix.str());
//            }

            //Output info to the terminal:
            //If the result is infinite, pad with an extra 6 white spaces, as the word "+infinity" is 9 chars long.:
            if (asrl::time::isfinite(runResults.back().second) == false)
            {
                std::cout << std::setw(6) << std::setfill(' ') << " ";
            }

            //Finally, output the time padded with 5 whitespaces in the front (manually, as it doesn't work on time), as the number is (nominally) of the form 00:00:00.123456
            std::cout << std::setw(5) << std::setfill(' ') << " " << runResults.back().second << std::flush;

            std::cout << std::endl;

            //Store this information in the time file.
            timePerSoln.addResult(plnr->getName(), runResults);
        }
        //Get the next planner
    }
    //Perform the next run

    return 0;
}
