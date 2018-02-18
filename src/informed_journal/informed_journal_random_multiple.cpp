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
//For boost time
#include <boost/chrono/chrono.hpp>
//For boost thread:
#include <boost/thread/thread.hpp>

//OMPL:
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
//#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
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
const unsigned int MICROSEC_SLEEP = 1000u; //Period for logging data, 1000us = 1ms


//Others:
const double PRUNE_FRACTION = 0.05;
const double GOAL_BIAS = 0.05;
const bool KNEAREST_RRT = false;
const double REWIRE_SCALE_RRT = 2.0; //The factor scaling the RGG term

//Plotting:
const bool PLOT_VERTICES = true;

bool argParse(int argc, char** argv, unsigned int* numObsPtr, double* obsRatioPtr, unsigned int* dimensionPtr, double* steerPtr, unsigned int* numExperimentsPtr, double* runTimePtr, bool* animatePtr)
{
    // Declare the supported options.
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("state,r", boost::program_options::value<unsigned int>(), "The state dimension.")
        ("steer-eta,s", boost::program_options::value<double>()->default_value(0.0), "The steer eta, or maximum edge length, to use. Defaults to 0.0 which uses the OMPL auto calculation.")
        ("experiments,e", boost::program_options::value<unsigned int>(), "The number of different random worlds to run.")
        ("number-obstacles,n", boost::program_options::value<unsigned int>(), "The number of obstacles used to calculate the max obstacle radius to give the desired obstacle coverage, [1, 2, ...). Note: radius is selected from 0.5x - 1x max radius, so the actual number of obstacles will vary.")
        ("obstacle-coverage,o", boost::program_options::value<double>(), "The mean percentage of volume taken up by obstacles in the random world, [0,1).")
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

    if (vm.count("number-obstacles"))
    {
        *numObsPtr = vm["number-obstacles"].as<unsigned int>();
        if ( (*numObsPtr  < 1) )
        {
            std::cout << "number-obstacles must be >= 1" << std::endl << std::endl << desc << std::endl;
            return false;
        }
    }
    else
    {
        std::cout << "number-obstacles not set" << std::endl << std::endl << desc << std::endl;
        return false;
    }

    if (vm.count("obstacle-coverage"))
    {
        *obsRatioPtr = vm["obstacle-coverage"].as<double>();
        if ( (*obsRatioPtr < 0) || (*obsRatioPtr >= 1) )
        {
            std::cout << "obstacle-coverage must be [0, 1)" << std::endl << std::endl << desc << std::endl;
            return false;
        }
    }
    else
    {
        std::cout << "obstacle-coverage not set" << std::endl << std::endl << desc << std::endl;
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


//Allocate and configure RRT*
std::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar(const ompl::base::SpaceInformationPtr &si, double steerEta)
{
    //Create a RRT* planner
    std::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = std::make_shared<ompl::geometric::RRTstar>(si);

    //Configure it
    plnr->setGoalBias(GOAL_BIAS);
    plnr->setKNearest(KNEAREST_RRT);
    plnr->setRewireFactor(REWIRE_SCALE_RRT);
    plnr->setDelayCC(true);
    plnr->setPruneThreshold(1.0);
    plnr->setTreePruning(false);
    plnr->setSampleRejection(false);
    plnr->setNewStateRejection(false);
    plnr->setAdmissibleCostToCome(true);
    plnr->setInformedRrtStar(false);
    plnr->setName("RRTstar");

    if (steerEta > 0.0)
    {
        plnr->setRange(steerEta);
    }
    // No else, use default

    //Return
    return plnr;
};

//Allocate and configure RRT* with sample rejection
std::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar_SampRej(const ompl::base::SpaceInformationPtr &si, double steerEta)
{
    //Create an RRT* planner
    std::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta);

    //Configure it to be Informed
    plnr->setSampleRejection(true);
    plnr->setName("RRTstar_SampleRejection");

    //Return
    return plnr;
};

//Allocate and configure RRT* with new-state rejection
std::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar_NewRej(const ompl::base::SpaceInformationPtr &si, double steerEta)
{
    //Create an RRT* planner
    std::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta);

    //Configure it to be Informed
    plnr->setNewStateRejection(true);
    plnr->setName("RRTstar_NewStateRejection");

    //Return
    return plnr;
};

//Allocate and configure RRT* with pruning
std::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar_Prune(const ompl::base::SpaceInformationPtr &si, double steerEta)
{
    //Create an RRT* planner
    std::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta);

    //Configure it to be Informed
    plnr->setPruneThreshold(PRUNE_FRACTION);
    plnr->setTreePruning(true);
    plnr->setName("RRTstar_Prune");

    //Return
    return plnr;
};

//Allocate and configure RRT* with pruning
std::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar_Trio(const ompl::base::SpaceInformationPtr &si, double steerEta)
{
    //Create an RRT* planner
    std::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta);

    //Configure it to be Informed
    plnr->setSampleRejection(true);
    plnr->setNewStateRejection(true);
    plnr->setPruneThreshold(PRUNE_FRACTION);
    plnr->setTreePruning(true);
    plnr->setName("RRTstar_Trio");

    //Return
    return plnr;
};

//Allocate and configure Informed RRT*
std::shared_ptr<ompl::geometric::RRTstar> allocateInformedRrtStar(const ompl::base::SpaceInformationPtr &si, double steerEta)
{
    //Create an RRT* planner
    std::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta);

    //Configure it to be Informed
    plnr->setPruneThreshold(PRUNE_FRACTION);
    plnr->setInformedRrtStar(true);
    plnr->setName("Informed_RRTstar");

    //Return
    return plnr;
};

std::shared_ptr<ompl::geometric::RRTstar> allocatePlanner(const PlannerType plnrType, const BaseExperimentPtr& expDefn, const double steerEta)
{
    switch(plnrType)
    {
        case PLANNER_RRTSTAR:
        {
            return allocateRrtStar(expDefn->getSpaceInformation(), steerEta);
            break;
        }
        case PLANNER_RRTSTAR_PRUNE:
        {
            return allocateRrtStar_Prune(expDefn->getSpaceInformation(), steerEta);
            break;
        }
        case PLANNER_RRTSTAR_NEW_REJECT:
        {
            return allocateRrtStar_NewRej(expDefn->getSpaceInformation(), steerEta);
            break;
        }
        case PLANNER_RRTSTAR_SAMPLE_REJECT:
        {
            return allocateRrtStar_SampRej(expDefn->getSpaceInformation(), steerEta);
            break;
        }
        case PLANNER_RRTSTAR_TRIO:
        {
            return allocateRrtStar_Trio(expDefn->getSpaceInformation(), steerEta);
            break;
        }
        case PLANNER_RRTSTAR_INFORMED:
        {
            return allocateInformedRrtStar(expDefn->getSpaceInformation(), steerEta);
            break;
        }
        default:
        {
            throw ompl::Exception("Unrecognized planner type in allocatePlanner()");
        }
    }
};

void callSolve(asrl::time::point* startTime, const ompl::base::PlannerPtr& planner, const asrl::time::duration& solveDuration)
{
    *startTime = asrl::time::now();
    planner->solve( asrl::time::seconds(solveDuration) );
}

int main(int argc, char **argv)
{
    //Argument Variables
    //The dimension size:
    unsigned int N;
    //The number of experiments
    unsigned int numExperiments;
    //The mean number of obstacles
    unsigned int numObs;
    //The mean obstacle coverage
    double obsRatio;
    //The time for which to run the planners
    double maxTime;
    //The steer / range of RRT
    double steerEta;
    //Whether to make frame-by-frame animations
    bool createAnimationFrames;

    //Get the command line arguments
    if (argParse(argc, argv, &numObs, &obsRatio, &N, &steerEta, &numExperiments, &maxTime, &createAnimationFrames) == false)
    {
        return 1;
    }

    //Variables
    //ompl::RNG::setSeed(1186638665);    std::cout << std::endl << "                   ---------> Seed set! <---------                   " << std::endl << std::endl;
    //Master seed:
    std::uint_fast32_t masterSeed = ompl::RNG::getSeed();
    //The filename for progress
    std::stringstream fileName;
    //The world name
    std::stringstream worldName;
    //The vector of planner types:
    std::vector<PlannerType> plannersToTest;
    //The experiment
    RandomRectanglesExperimentPtr experiment;

    //Specify the planners:
    plannersToTest.push_back(PLANNER_RRTSTAR);
//    plannersToTest.push_back(PLANNER_RRTSTAR);
//    plannersToTest.push_back(PLANNER_RRTSTAR_PRUNE);
//    plannersToTest.push_back(PLANNER_RRTSTAR_NEW_REJECT);
//    plannersToTest.push_back(PLANNER_RRTSTAR_SAMPLE_REJECT);
//    plannersToTest.push_back(PLANNER_RRTSTAR_TRIO);
    plannersToTest.push_back(PLANNER_RRTSTAR_INFORMED);

    //The results output file:
    fileName << "R" << N << "S" << masterSeed << "Random.csv";

    //Let people know what's going on:
    std::cout << "Seed: " << masterSeed << std::endl;
    std::cout << "Output: " << fileName.str() << std::endl;

    ResultsFile<TimeCostHistory> progressHistory(fileName.str());

    //Perform numRuns
    for (unsigned int q = 0u; q < numExperiments; ++q)
    {
        //Create experiments for this run:
        experiment = std::make_shared<RandomRectanglesExperiment>(N, numObs, obsRatio, maxTime, CHECK_RESOLUTION);

        //Reference cost:
        double refValue = 0.0;
        //A RNG, we will use it's seed for all the planners, but not actually use the RNG...
        ompl::RNG seedRNG;
        //The first-soln runtime
        asrl::time::duration firstSolnTime(0);
        //Whether this is a bad experiment
        bool badExperiment = false;

        //Iterate over the planners:
        for (unsigned int p = 0u; p < plannersToTest.size() && badExperiment == false; ++p)
        {
            //Variables
            //The start time for a call
            asrl::time::point startTime;
            //The run time
            asrl::time::duration runTime(0);
            //The current planner:
            std::shared_ptr<ompl::geometric::RRTstar> plnr;
            //The problem defintion used by this planner
            ompl::base::ProblemDefinitionPtr pdef;
            //The results from this planners run across all the variates:
            TimeCostHistory runResults(experiment->getTargetTime(), MICROSEC_SLEEP);

            //Allocate a planner
            plnr = allocatePlanner(plannersToTest.at(p), experiment, steerEta);

            //Get the problem definition
            pdef = experiment->newProblemDefinition();

            //Give to the planner
            plnr->setProblemDefinition(pdef);

            //Setup
            startTime = asrl::time::now();
            plnr->setup();
            runTime = asrl::time::now() - startTime;

            //Set the planner seed:
            plnr->setLocalSeed(seedRNG.getLocalSeed());

            //This must come after setup to get the steer info!
            //If this is the first planner of a trial, output at least the trial number:
            if (p == 0u)
            {
                //If this is also the first run ever, output start info:
                if (q == 0u)
                {
                    std::cout << "First experiment: " << std::endl;
                    experiment->print(false);
                    std::cout << "Very first steer: " << plnr->getRange() << std::endl;
                    //The first column is 7 wide for a 6-digit trial number and a ":"
                    std::cout << std::setw(7) << std::setfill(' ') << " ";

                    //The subsequent columns are 30 wide for planner names, skip the first planner which is the reference solution:
                    std::cout << std::setw(30) << std::setfill(' ') << "Init RRTstar Sol'n" << std::flush;
                    for (unsigned int i = 1u; i < plannersToTest.size(); ++i)
                    {
                        std::cout << std::setw(30) << std::setfill(' ') << plannerName(plannersToTest.at(i)) << std::flush;
                    }
                    std::cout << std::endl;
                }
                //No else

                //The trial number is 6-digits with 3 of padding
                std::cout << std::setw(6) << std::setfill(' ') << q << ":" << std::flush;
            }
            //No else

            //If this is the first planner, we stop on the first solution:
            if (p == 0u)
            {
                experiment->getOptimizationObjective()->setCostThreshold(experiment->getOptimizationObjective()->infiniteCost());
            }
            else
            {
                experiment->getOptimizationObjective()->setCostThreshold(experiment->getMinimum());
            }

            //Run the planner:
            if (createAnimationFrames == true)
            {
                runTime = runTime + createAnimation(experiment, plannersToTest.at(p), plnr, masterSeed+q, firstSolnTime + experiment->getTargetTime() - runTime, PLOT_VERTICES, true, false, false, false);
                runResults.push_back( std::make_pair(runTime, plnr->bestCost().value()) );
            }
            else
            {
                //Start solving in another thread. It will record startTime, but so we notice if it doesn't we will clear startTime first.
                startTime = asrl::time::point();
                boost::thread solveThread(callSolve, &startTime, plnr, firstSolnTime + experiment->getTargetTime() - runTime);

                //If the clock is not steady, sleep for 1us so that the upcoming first entry is not backwards in time.
                if (asrl::time::clock::is_steady == false)
                {
                    usleep(1u);
                }

                //Log data
                do
                {
                    //Store the runtime and the current cost
                    runResults.push_back( std::make_pair(runTime + (asrl::time::now() - startTime), plnr->bestCost().value()) );
                }
                while (solveThread.try_join_for(boost::chrono::microseconds(MICROSEC_SLEEP)) == false);

                //Store the final run time and cost
                runTime = runTime + (asrl::time::now() - startTime);
                runResults.push_back( std::make_pair(runTime, plnr->bestCost().value()) );
            }

            //Save the map:
            writeMatlabMap(experiment, plannersToTest.at(p), plnr, masterSeed+q, PLOT_VERTICES, true, false, false, false, "plots/");

            if (p == 0u && pdef->hasExactSolution() == false)
            {
                //If we didn't find a solution on the first attempt, leave the planner loop and retry this experiment
                --q;
                badExperiment = true;
            }
            else if (p == 0u)
            {
                //If this is the reference-time planner store the solution time:
                firstSolnTime = runTime;
                badExperiment = false;
            }
            else
            {
                //If not, the data
                progressHistory.addResult(plnr->getName(), runResults);
            }

            if (badExperiment == false)
            {
                //Output info to the terminal:
                if (p == 1u)
                {
                    refValue = plnr->bestCost().value();
                }

                //Output 3 characters of white space and then the time, which is 15 chars wide:
                std::cout << std::setw(5) << std::setfill(' ') << " " << std::flush;
                std::cout << runTime << std::flush;
                //Then a comma and 1 space,
                std::cout << " " << std::flush;
                //And the cost using the remaining 30 - 5 - 15 - 1 = 9 characters:
                if (p == 0u || p == 1u)
                {
                    std::cout << std::setprecision(6) << std::setw(9) << std::setfill(' ') << plnr->bestCost().value() << std::flush;
                }
                else
                {
                    std::cout << std::setprecision(6) << std::setw(8) << std::setfill(' ') << plnr->bestCost().value()/refValue << "x" << std::flush;
                }
            }
        }
        //Get the next planner
        std::cout << std::endl;
    }
    //Perform the next experiment

    return 0;
}
