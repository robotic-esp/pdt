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
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
//#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/util/Time.h>
#include "ompl/util/Console.h" //For OMPL_INFORM et al.
#include "ompl/tools/config/MagicConstants.h" //For BETTER_PATH_COST_MARGIN
#include <ompl/util/Exception.h>

//Our Experiments
#include "ExperimentDefinitions.h"

//The helper functions for plotting:
#include <plotting_tools.h>

#define ASRL_DBL_INFINITY std::numeric_limits<double>::infinity()



//World:
const double CHECK_RESOLUTION = 0.001;
const double MILLISEC_SLEEP = 1.0; //Period for logging data


//RRTStar:
const double RRT_PRUNE_FRACTION = 0.05;
const double RRT_GOAL_BIAS = 0.05; //8D: 0.05; //2D: 0.05
const bool RRT_KNEAREST = false;

//BITStar:
const unsigned int BITSTAR_SAMPLES = 100u;
const bool BITSTAR_TRACK_FAILURES = false;
const bool BITSTAR_STRICT_QUEUE = false;
const bool BITSTAR_DELAY_REWIRE = false;
const bool BITSTAR_PRUNE = true;
const double BITSTAR_PRUNE_FRACTION = 0.01;
const bool BITSTAR_KNEAREST = false;

//The factor scaling the RGG term
const double RRT_REWIRE_SCALE = 2.0;
const double FMT_REWIRE_SCALE = RRT_REWIRE_SCALE;
const double BITSTAR_REWIRE_SCALE = RRT_REWIRE_SCALE;


bool argParse(int argc, char** argv, double* steerPtr, double* runTimePtr, bool* animatePtr)
{
    // Declare the supported options.
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("steer-eta,s", boost::program_options::value<double>()->default_value(0.0), "The steer eta, or maximum edge length, to use. Defaults to 0.0 which uses the OMPL auto calculation.")
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

    *steerPtr = vm["steer-eta"].as<double>();

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

/*
//Allocate and configure RRT*
boost::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar(const ompl::base::SpaceInformationPtr &si, double steerEta)
{
    //Create a RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = boost::make_shared<ompl::geometric::RRTstar>(si);

    //Configure it
    plnr->setGoalBias(RRT_GOAL_BIAS);
    plnr->setKNearest(RRT_KNEAREST);
    plnr->setRewireFactor(RRT_REWIRE_SCALE);
    plnr->setDelayCC(true);

    plnr->setPruneThreshold(1.0);
    plnr->setTreePruning(false);

    plnr->setNumSamplingAttempts(1000u);
    plnr->setSampleRejection(false);
    plnr->setNewStateRejection(false);
    plnr->setAdmissibleCostToCome(true);

    plnr->setPrunedMeasure(false);
    plnr->setInformedSampling(false);

    if (steerEta > 0.0)
    {
        plnr->setRange(steerEta);
    }
    plnr->setName("RRTstar");
    // No else, use default

    //Return
    return plnr;
};

//Allocate and configure Informed RRT*
boost::shared_ptr<ompl::geometric::RRTstar> allocateInformedRrtStar(const ompl::base::SpaceInformationPtr &si, double steerEta)
{
    //Create an RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta);

    //Configure it to be Informed
    plnr->setPruneThreshold(RRT_PRUNE_FRACTION);
    plnr->setInformedSampling(true);
    plnr->setTreePruning(true);
    plnr->setPrunedMeasure(true);
    plnr->setName("Informed_RRTstar");

    //Return
    return plnr;
};
*/

//Allocate and configure BiT*
boost::shared_ptr<ompl::geometric::BITstar> allocateBitStar(const ompl::base::SpaceInformationPtr &si, unsigned int numSamples)
{
    std::stringstream plannerName;

    plannerName << "BITstar" << numSamples;

    //Create a BIT* planner
    boost::shared_ptr<ompl::geometric::BITstar> plnr;
    plnr = boost::make_shared<ompl::geometric::BITstar>(si);

    //Configure it
    plnr->setRewireFactor(BITSTAR_REWIRE_SCALE);
    plnr->setSamplesPerBatch(numSamples);
    plnr->setUseFailureTracking(BITSTAR_TRACK_FAILURES);
    plnr->setKNearest(BITSTAR_KNEAREST);
    plnr->setStrictQueueOrdering(BITSTAR_STRICT_QUEUE);
    plnr->setPruning(BITSTAR_PRUNE);
    plnr->setPruneThresholdFraction(BITSTAR_PRUNE_FRACTION);
    plnr->setDelayRewiringUntilInitialSolution(BITSTAR_DELAY_REWIRE);
    plnr->setName(plannerName.str());

    //Return
    return plnr;
}

ompl::base::PlannerPtr allocatePlanner(const PlannerType plnrType, const BaseExperimentPtr& expDefn, const double steerEta, unsigned int numSamples)
{
    switch(plnrType)
    {
        /*
        case PLANNER_RRTSTAR:
        {
            return allocateRrtStar(expDefn->getSpaceInformation(), steerEta);
            break;
        }
        case PLANNER_RRTSTAR_INFORMED:
        {
            return allocateInformedRrtStar(expDefn->getSpaceInformation(), steerEta);
            break;
        }
        */
        case PLANNER_BITSTAR:
        {
            return allocateBitStar(expDefn->getSpaceInformation(), numSamples);
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
    planner->solve( ompl::time::seconds(solveDuration) );
}

int main(int argc, char **argv)
{
    //Argument Variables
    //The time for which to run the planners
    double maxTime;
    //The steer / range of RRT
    double steerEta;
    //Whether to make frame-by-frame animations
    bool createAnimationFrames;

    //Get the command line arguments
    if (argParse(argc, argv, &steerEta, &maxTime, &createAnimationFrames) == false)
    {
        return 1;
    }

    //Variables
    ompl::RNG::setSeed(240822352);    std::cout << std::endl << "                   ---------> Seed set! <---------                   " << std::endl << std::endl;
    //Master seed:
    boost::uint32_t masterSeed = ompl::RNG::getSeed();
    //The filename for progress
    std::stringstream fileName;
    //The world name
    std::stringstream worldName;
    //The vector of planner types:
    std::vector<PlannerType> plannersToTest;
    //The vector of number of samples to use. Only meaningful for FMT* and BIT*
    std::vector<unsigned int> numSamples;
    //The experiment
    BaseExperimentPtr experiment;

    //Specify the planners:
    //plannersToTest.push_back(PLANNER_RRTSTAR);              numSamples.push_back(0u);
    //plannersToTest.push_back(PLANNER_RRTSTAR_INFORMED);     numSamples.push_back(0u);
    plannersToTest.push_back(PLANNER_BITSTAR);              numSamples.push_back(BITSTAR_SAMPLES);

    //Create one experiment for all runs:
    experiment = boost::make_shared<AsrlExperiment>(false, !createAnimationFrames, maxTime, CHECK_RESOLUTION);

    //The results output file:
    fileName << experiment->getName() << "S" << masterSeed << ".csv";

    //Let people know what's going on:
    std::cout << "Seed: " << masterSeed << std::endl;
    std::cout << "Output: " << fileName.str() << std::endl;

    ResultsFile<TimeCostHistory> progressHistory(fileName.str());

    //Iterate over the planners:
    for (unsigned int p = 0u; p < plannersToTest.size(); ++p)
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

        //Allocate a planner
        plnr = allocatePlanner(plannersToTest.at(p), experiment, steerEta, numSamples.at(p));

        //Get the problem definition
        pdef = experiment->newProblemDefinition();

        //Give to the planner
        plnr->setProblemDefinition(pdef);

        //Setup
        startTime = ompl::time::now();
        plnr->setup();
        runTime = ompl::time::now() - startTime;

        //This must come after setup to get the steer info!
        //If this is the first planner of a trial, output at least the trial number:
        if (p == 0u)
        {
            std::cout << "Experiment: " << std::endl;
            experiment->print(false);
            //The first column is 7 wide for a 6-digit trial number and a ":"
            std::cout << std::setw(7) << std::setfill(' ') << " ";

            //The subsequent columns are 30 wide for planner names
            for (unsigned int i = 0u; i < plannersToTest.size(); ++i)
            {
                std::cout << std::setw(30) << std::setfill(' ') << plannerName(plannersToTest.at(i)) << std::flush;
            }
            std::cout << std::endl;

            //The trial number is 6-digits with 3 of padding
            std::cout << std::setw(6) << std::setfill(' ') << 000 << ":" << std::flush;
        }
        //No else

        //Run the planner:
        if (createAnimationFrames == true)
        {
            if (plannersToTest.at(p) == PLANNER_RRT || plannersToTest.at(p) == PLANNER_RRTCONNECT || plannersToTest.at(p) == PLANNER_FMT)
            {
                //No video to make
            }
            else if (isRrtStar(plannersToTest.at(p)) == true)
            {
                runTime = runTime + createAnimation(experiment, plannersToTest.at(p), plnr, masterSeed, experiment->getTargetTime() - runTime, false, false, false, false, 0u, true);
                runResults.push_back( std::make_pair(runTime, plnr->as<ompl::geometric::RRTstar>()->bestCost().value()) );
            }
            else if (isBitStar(plannersToTest.at(p)) == true)
            {
                runTime = runTime + createAnimation(experiment, plannersToTest.at(p), plnr, masterSeed, experiment->getTargetTime() - runTime, false, false, false, false, 0u, true);
                runResults.push_back( std::make_pair(runTime, plnr->as<ompl::geometric::BITstar>()->bestCost().value()) );
            }
        }
        else
        {
            //Store the starting time:
            startTime = ompl::time::now();

            //Start solving in another thread. We use an intermediate function for this as the class function is overloaded.
            boost::thread solveThread(callSolve, plnr, experiment->getTargetTime() - runTime);

            //If we can, log the intermediate data:
            if (isRrtStar(plannersToTest.at(p)) == true)
            {
                ompl::geometric::RRTstar* rrtstar = plnr->as<ompl::geometric::RRTstar>();

                //Log data
                do
                {
                    //Store the runtime and the current cost
                    runResults.push_back( std::make_pair(ompl::time::now() - startTime + runTime, rrtstar->bestCost().value()) );
                }
                while ( solveThread.timed_join(boost::posix_time::milliseconds(MILLISEC_SLEEP)) == false);
            }
            else if (isBitStar(plannersToTest.at(p)) == true)
            {
                ompl::geometric::BITstar* bitstar = plnr->as<ompl::geometric::BITstar>();

                //Log data
                do
                {
                    //Store the runtime and the current cost
                    runResults.push_back( std::make_pair(ompl::time::now() - startTime + runTime, bitstar->bestCost().value()) );
                }
                while ( solveThread.timed_join(boost::posix_time::milliseconds(MILLISEC_SLEEP)) == false);
            }
            else
            {
                //We can't log intermediate data for these planners, so just wait. Sanjiban, I think there's a blocking way to do this.
                do
                {
                }
                while ( solveThread.timed_join(boost::posix_time::milliseconds(MILLISEC_SLEEP)) == false);
            }

            //Store the final run time and cost
            runTime = runTime + (ompl::time::now() - startTime);

            if (pdef->hasExactSolution() == true)
            {
                runResults.push_back(std::make_pair(runTime, pdef->getSolutionPath()->cost(experiment->getOptimizationObjective()).value()));
            }
            else
            {
                runResults.push_back(std::make_pair(runTime, ASRL_DBL_INFINITY));
            }

            //Store the data
            progressHistory.addResult(plnr->getName(), runResults);
        }

        //Save the map:
        writeMatlabMap(experiment, plannersToTest.at(p), plnr, masterSeed, false, false, true, false, "plots/");

        //Output 3 characters of white space and then the time, which is 15 chars wide:
        std::cout << std::setw(5) << std::setfill(' ') << " " << std::flush;
        std::cout << runTime << std::flush;
        //Then a comma and 1 space,
        std::cout << " " << std::flush;
        //And the cost using the remaining 30 - 5 - 15 - 1 = 9 characters:
        if (runResults.empty() == false)
        {
            std::cout << std::setprecision(6) << std::setw(9) << std::setfill(' ') << runResults.back().second << std::flush;
        }
        else
        {
            std::cout << std::setprecision(6) << std::setw(9) << std::setfill(' ') << "---" << std::flush;
        }
    }
    //Get the next planner
    std::cout << std::endl;

    return 0;
}
