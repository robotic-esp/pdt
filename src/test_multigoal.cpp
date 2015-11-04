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

//OMPL:
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
//#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
//#include <ompl/geometric/planners/bitstar/HybridBITstar.h>
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
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
const double lowerLimit = -1.0;
const double upperLimit = 1.0;
const bool separateObstacles = false;
const double checkResolution = 0.001;

//BIT*
const unsigned int bitStarSamples = 100u;
const bool bitStarPrune = true;
const double pruneFraction = 0.01;
const bool kNearestBIT = false;
const double rewireFactorBIT = 1.1; //The factor scaling the RGG term
const bool strictQueue = false;
const bool useFailureTracking = false;
const bool delayRewire = false;

//Hybrid BIT*
const unsigned int numShortcuts = 25u;
const unsigned int numShortcutFailures = 5u;
const double shortcutWidth = 0.5;
const double shortcutTol = 0.001;
const double interpTol = 0.1;

//ICRA16:
const bool shareInfo = true;
const bool parallelQueues = true;

//Others:
const double goalBias = 0.05; //8D: 0.05; //2D: 0.05
const double steerEta2D = 0.2;
const double steerEta8D = 1.25;
const bool kNearestRRT = kNearestBIT;
const double rewireFactorRRT = rewireFactorBIT; //The factor scaling the RGG term
const double rewireFactorFMT = rewireFactorBIT; //The factor scaling the RGG term

//Experiment:
const bool logProgress = true;
const bool logIterationsAndCost = false;
const double millisecSleep = 1.0; //Period for logging data

//Plotting:
const bool informedEllipse = true;
const bool bitStarEllipse = false;
const bool bitStarEdge = true;
const bool bitStarQueue = false;


bool argParse(int argc, char** argv, unsigned int* numObsPtr, double* obsRatioPtr, unsigned int* dimensionPtr, unsigned int* numTrialsPtr, double* runTimePtr, bool* animatePtr)
{
    // Declare the supported options.
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("state,r", boost::program_options::value<unsigned int>(), "The state dimension.")
        ("experiments,e", boost::program_options::value<unsigned int>(), "The number of unique experiments to run on the random world.")
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
        if ( (*dimensionPtr  != 2) && (*dimensionPtr  != 8) )
        {
            std::cout << "Due to hard-coded RRT* paramenters, state dimension must be 2 or 8 for now " << std::endl << std::endl << desc << std::endl;
            return false;
        }
    }
    else
    {
        std::cout << "state dimension not set" << std::endl << std::endl << desc << std::endl;
        return false;
    }


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
}

//Allocate and configure RRT*
ompl::base::PlannerPtr allocateRrt(const ompl::base::SpaceInformationPtr &si, double steerEta)
{
    //Create a RRT* planner
    ompl::base::PlannerPtr base = boost::make_shared<ompl::geometric::RRT>(si);

    //Configure it
    base->as<ompl::geometric::RRT>()->setGoalBias(goalBias);
    base->as<ompl::geometric::RRT>()->setRange(steerEta);
    base->setName("RRT");

    //Return
    return base;
}

//Allocate and configure RRT*
ompl::base::PlannerPtr allocateRrtConnect(const ompl::base::SpaceInformationPtr &si, double steerEta)
{
    //Create a RRT* planner
    ompl::base::PlannerPtr base = boost::make_shared<ompl::geometric::RRTConnect>(si);

    //Configure it
    base->as<ompl::geometric::RRTConnect>()->setRange(steerEta);
    base->setName("RRTConnect");

    //Return
    return base;
}


//Allocate and configure RRT*
ompl::base::PlannerPtr allocateRrtStar(const ompl::base::SpaceInformationPtr &si, double steerEta)
{
    //Create a RRT* planner
    ompl::base::PlannerPtr base = boost::make_shared<ompl::geometric::RRTstar>(si);

    //Configure it
    base->as<ompl::geometric::RRTstar>()->setGoalBias(goalBias);
    base->as<ompl::geometric::RRTstar>()->setRange(steerEta);
//    base->as<ompl::geometric::RRTstar>()->setKNearest(kNearestRRT);
//    base->as<ompl::geometric::RRTstar>()->setRewireFactor(rewireFactorRRT);
    base->as<ompl::geometric::RRTstar>()->setDelayCC(true);
    /* // Was:
    base->as<ompl::geometric::RRTstar>()->setPruneStatesImprovementThreshold(0.0);
    base->as<ompl::geometric::RRTstar>()->setPrune(false);
    */ // Proposed:
    base->as<ompl::geometric::RRTstar>()->setTreePruning(false);
    base->as<ompl::geometric::RRTstar>()->setSampleRejection(false);
    base->as<ompl::geometric::RRTstar>()->setNewStateRejection(false);
//    base->as<ompl::geometric::RRTstar>()->setInformedSampling(false);
    base->setName("RRTstar");

    //Return
    return base;
}


//Allocate and configure FMT*
ompl::base::PlannerPtr allocateFmtStar(const ompl::base::SpaceInformationPtr &si, unsigned int numSamples)
{
    std::stringstream plannerName;

    plannerName << "FMTstar" << numSamples;

    //Create a RRT* planner
    ompl::base::PlannerPtr base = boost::make_shared<ompl::geometric::FMT>(si);

    //Configure it
    base->as<ompl::geometric::FMT>()->setNumSamples(numSamples);
    base->as<ompl::geometric::FMT>()->setRadiusMultiplier(rewireFactorFMT);
    base->as<ompl::geometric::FMT>()->setFreeSpaceVolume(si->getSpaceMeasure());
    base->setName(plannerName.str());

    //Return
    return base;
}

//Allocate and configure Informed RRT*
ompl::base::PlannerPtr allocateInformedRrtStar(const ompl::base::SpaceInformationPtr &si, double steerEta)
{
    //Create an RRT* planner
    ompl::base::PlannerPtr base = allocateRrtStar(si, steerEta);
    OMPL_ERROR("This branch does not have Informed RRT* yet.");

    //Configure it to be Informed
//    base->as<ompl::geometric::RRTstar>()->setInformedSampling(true);
    base->setName("Informed_RRTstar");

    //Return
    return base;
}

//Allocate and configure BiT*
ompl::base::PlannerPtr allocateBitStar(const ompl::base::SpaceInformationPtr &si, unsigned int numSamples)
{
    std::stringstream plannerName;

    plannerName << "BITstar" << numSamples;

    //Create a BIT* planner
    ompl::base::PlannerPtr base = boost::make_shared<ompl::geometric::BITstar>(si);

    //Configure it
    base->as<ompl::geometric::BITstar>()->setRewireFactor(rewireFactorBIT);
    base->as<ompl::geometric::BITstar>()->setSamplesPerBatch(numSamples);
    base->as<ompl::geometric::BITstar>()->setUseFailureTracking(useFailureTracking);
    base->as<ompl::geometric::BITstar>()->setKNearest(kNearestBIT);
    base->as<ompl::geometric::BITstar>()->setStrictQueueOrdering(strictQueue);
    base->as<ompl::geometric::BITstar>()->setPruning(bitStarPrune);
    base->as<ompl::geometric::BITstar>()->setPruneThresholdFraction(pruneFraction);
    base->as<ompl::geometric::BITstar>()->setDelayRewiringUntilInitialSolution(delayRewire);
    base->setName(plannerName.str());

    //Return
    return base;
}

//Allocate Hybrid BIT*
ompl::base::PlannerPtr allocateHybridBitStar(const ompl::base::SpaceInformationPtr& /*si*/, unsigned int /*numSamples*/)
{
    OMPL_ERROR("Hybrid BIT* is not implemented in this branch."); return ompl::base::PlannerPtr();
//    std::stringstream plannerName;
//
//    plannerName << "HybridBITstar" << numSamples;
//
//    //Create a BIT* planner
//    ompl::base::PlannerPtr base = boost::make_shared<ompl::geometric::HybridBITstar>(si);
//
//    //Configure it
//    //BIT* settings:
//    base->as<ompl::geometric::HybridBITstar>()->setRewireFactor(rewireFactorBIT);
//    base->as<ompl::geometric::HybridBITstar>()->setSamplesPerBatch(numSamples);
//    base->as<ompl::geometric::HybridBITstar>()->setUseFailureTracking(useFailureTracking);
//    base->as<ompl::geometric::HybridBITstar>()->setKNearest(kNearestBIT);
//    base->as<ompl::geometric::HybridBITstar>()->setStrictQueueOrdering(strictQueue);
//    base->as<ompl::geometric::HybridBITstar>()->setPruning(bitStarPrune);
//    base->as<ompl::geometric::HybridBITstar>()->setPruneThresholdFraction(pruneFraction);
//    //Hybrid BIT* settings:
//    base->as<ompl::geometric::HybridBITstar>()->setMaxNumberOfLocalIterations(numShortcuts);
//    base->as<ompl::geometric::HybridBITstar>()->setMaxNumberofLocalFailures(numShortcutFailures);
//    base->as<ompl::geometric::HybridBITstar>()->setShortcutWindowFraction(shortcutWidth);
//    base->as<ompl::geometric::HybridBITstar>()->setShortcutTolerance(shortcutTol);
//    base->as<ompl::geometric::HybridBITstar>()->setInterpolationTolerance(interpTol);
//    base->setName(plannerName.str());
//
//    //Return
//    return base;
}

void callSolve(const ompl::base::PlannerPtr& planner, const ompl::time::duration& solveDuration)
{
    //planner->solve( solveDuration );
    planner->solve( ompl::time::seconds(solveDuration) );
}



int main(int argc, char **argv)
{
    //Argument Variables
    //The dimension size:
    unsigned int N;
    //The number of experiments
    unsigned int numExperiments;
    //The (maximum) percent of space covered by obstacles
    double obsRatio;
    //The number of obstacles to create
    unsigned int numObs;
    //The time for which to run the planners
    double targetTime;
    //Whether to make frame-by-frame animations
    bool createAnimationFrames;

    //Get the command line arguments
    if (argParse(argc, argv, &numObs, &obsRatio, &N, &numExperiments, &targetTime, &createAnimationFrames) == false)
    {
        return 1;
    }


    //Variables
//    ompl::RNG::setSeed(3210443914);     std::cout << std::endl << "                   ---------> Seed set! <---------                   " << std::endl << std::endl;
    //Master seed:
    boost::uint32_t masterSeed = ompl::RNG::getSeed();
    //Convenience typedefs:
    typedef std::pair<ompl::time::duration, ompl::base::Cost> planner_result_t;
    typedef std::vector<planner_result_t> experiment_result_t;
    //The steer eta used
    double steerEta;
    //The filename for progress
    std::stringstream fileName;
    //The world name
    std::stringstream worldName;
    //The experiment
    BaseExperimentPtr expDefn = boost::make_shared<MultiStartGoalExperiment>(N, numObs, obsRatio, targetTime, checkResolution);

    if (N == 2u)
    {
        steerEta = steerEta2D;
    }
    else if (N == 8u)
    {
        steerEta = steerEta8D;
    }
    else
    {
        throw ompl::Exception("No recorded steer eta for this dimension.");
    }

    //Let people know what's going on:
    std::cout <<  expDefn->getName() << " in R^" << N << " with an RRT edge length of " << steerEta << "." << std::endl;
    std::cout << "Seed: " << masterSeed << std::endl;
    expDefn->print();

    //The results output file:
    fileName << "R" << N << "S" << masterSeed << expDefn->getName() << ".csv";
    ProgressFile plannerProgress(fileName.str(), logIterationsAndCost);


    for (unsigned int q = 0u; q < numExperiments; ++q)
    {
        //Variables:
        //The vector of planners:
        std::vector<std::pair<PlannerType, ompl::base::PlannerPtr> > plannersToTest;

        //Add the planners to test:
//        plannersToTest.push_back( std::make_pair(PLANNER_RRT, allocateRrt(expDefn->getSpaceInformation(), steerEta) ) );
//        plannersToTest.push_back( std::make_pair(PLANNER_RRT, allocateRrtConnect(expDefn->getSpaceInformation(), steerEta) ) );
//        plannersToTest.push_back( std::make_pair(PLANNER_RRTSTAR, allocateRrtStar(expDefn->getSpaceInformation(), steerEta) ) );
//        plannersToTest.push_back( std::make_pair(PLANNER_RRTSTAR_INFORMED, allocateInformedRrtStar(expDefn->getSpaceInformation(), steerEta) ) );
        plannersToTest.push_back( std::make_pair(PLANNER_BITSTAR, allocateBitStar(expDefn->getSpaceInformation(), bitStarSamples) ) );
//        plannersToTest.push_back( std::make_pair(PLANNER_HYBRID_BITSTAR, allocateHybridBitStar(expDefn->getSpaceInformation(), bitStarSamples) ) );

        if (q == 0u)
        {
            std::cout << "   ";
            for (unsigned int i = 0u; i < plannersToTest.size(); ++i)
            {
                std::cout << std::setw(26) << std::setfill(' ') << plannersToTest.at(i).second->getName();
                if (i != plannersToTest.size() - 1u)
                {
                    std::cout << "    ";
                }
            }
            std::cout << std::endl;
        }
        std::cout << q << ": " << std::flush;

        //Iterate over the planners, using the seed and solution from the first planner:
        for (unsigned int i = 0u; i < plannersToTest.size(); ++i)
        {
            //Variables
            //The start time for a call, and the running time
            ompl::time::point startTime;
            ompl::time::duration runTime(0,0,0,0);
            //The problem defintion used by this planner
            ompl::base::ProblemDefinitionPtr pdef = expDefn->newProblemDefinition();

            //Set the problem definition
            plannersToTest.at(i).second->setProblemDefinition(pdef);

            //Set up the planner
            startTime = ompl::time::now();
            plannersToTest.at(i).second->setup();
            runTime = ompl::time::now() - startTime;

            //Run the planner
            if (createAnimationFrames == true)
            {
                if (plannersToTest.at(i).first == PLANNER_RRT)
                {
                        startTime = ompl::time::now();
                        //plannersToTest.at(i).second->solve( expDefn->getTargetTime() - runTime );
                        plannersToTest.at(i).second->solve( ompl::time::seconds(expDefn->getTargetTime() - runTime) );
                        runTime = runTime + (ompl::time::now() - startTime);
                    //runTime = runTime + createAnimation(expDefn plannersToTest.at(i), masterSeed, expDefn->getTargetTime());
                }
                else
                {
                    runTime = runTime + createAnimation(expDefn, plannersToTest.at(i).first, plannersToTest.at(i).second, masterSeed, expDefn->getTargetTime() - runTime, informedEllipse, bitStarEllipse, bitStarEdge, bitStarQueue);
                }
            }
            else if (logProgress == true)
            {
                //A vector of the planner progress
                progress_pair_vector_t progressPair;
                progress_tuple_vector_t progressTuple;

                //Store the starting time:
                startTime = ompl::time::now();

                //Start solving in another thread. We use an intermediate function for this as the class function is overloaded.
                boost::thread solveThread(callSolve, plannersToTest.at(i).second, expDefn->getTargetTime() - runTime);

                //Log data
                do
                {
                    if  (plannersToTest.at(i).first == PLANNER_RRT)
                    {
                        //Do nothing, these do not have intermediate data
                    }
                    else if (plannersToTest.at(i).first == PLANNER_FMT)
                    {
                        if (logIterationsAndCost == true)
                        {
                            //If FMT* is modified, 1 of 2:
                            //progressTuple.push_back( boost::make_tuple(ompl::time::now() - startTime + runTime, plannersToTest.at(i).second->as<ompl::geometric::FMT>()->iterationProgressProperty(), boost::lexical_cast<std::string>(ASRL_DBL_INFINITY)) );
                        }
                        else
                        {
                        }
                    }
                    else if (isRrtStar(plannersToTest.at(i).first))
                    {
                        //Store the runtime and the current cost
                        if (logIterationsAndCost == true)
                        {
                            progressTuple.push_back( boost::make_tuple(ompl::time::now() - startTime + runTime, plannersToTest.at(i).second->as<ompl::geometric::RRTstar>()->numIterations(), plannersToTest.at(i).second->as<ompl::geometric::RRTstar>()->bestCost().value()) );
                        }
                        else
                        {
                            progressPair.push_back( std::make_pair(ompl::time::now() - startTime + runTime, plannersToTest.at(i).second->as<ompl::geometric::RRTstar>()->bestCost().value()) );
                        }
                    }
                    else if (plannersToTest.at(i).first == PLANNER_BITSTAR || plannersToTest.at(i).first == PLANNER_BITSTAR_SEED)
                    {
                        if (logIterationsAndCost == true)
                        {
                            progressTuple.push_back( boost::make_tuple(ompl::time::now() - startTime + runTime, plannersToTest.at(i).second->as<ompl::geometric::BITstar>()->numIterations(), plannersToTest.at(i).second->as<ompl::geometric::BITstar>()->bestCost().value()) );
                        }
                        else
                        {
                            progressPair.push_back( std::make_pair(ompl::time::now() - startTime + runTime, plannersToTest.at(i).second->as<ompl::geometric::BITstar>()->bestCost().value()) );
                        }
                    }
//                    else if (plannersToTest.at(i).first == PLANNER_HYBRID_BITSTAR)
//                    {
//                        if (logIterationsAndCost == true)
//                        {
//                            progressTuple.push_back( boost::make_tuple(ompl::time::now() - startTime + runTime, plannersToTest.at(i).second->as<ompl::geometric::HybridBITstar>()->numIterations(), plannersToTest.at(i).second->as<ompl::geometric::HybridBITstar>()->bestCost().value()) );
//                        }
//                        else
//                        {
//                            progressPair.push_back( std::make_pair(ompl::time::now() - startTime + runTime, plannersToTest.at(i).second->as<ompl::geometric::HybridBITstar>()->bestCost().value()) );
//                        }
//                    }
                    else
                    {
                        throw ompl::Exception("Planner not recognized for progress logging.");
                    }
                }
                while ( solveThread.timed_join(boost::posix_time::milliseconds(millisecSleep)) == false);

                //Store the final run time
                runTime = runTime + (ompl::time::now() - startTime);

                if (logIterationsAndCost == true)
                {
                    double finalCost;

                    //Get the final solution
                    if (pdef->hasExactSolution() == true)
                    {
                        finalCost = pdef->getSolutionPath()->cost(expDefn->getOptimizationObjective()).value();
                    }
                    else
                    {
                        finalCost = ASRL_DBL_INFINITY;
                    }

                    if  (plannersToTest.at(i).first == PLANNER_RRT)
                    {
                        //Do nothing, these do not have intermediate data
                    }
                    else if (plannersToTest.at(i).first == PLANNER_FMT)
                    {
                        //If FMT* is modified, 2 of 2:
                        //progressTuple.push_back( boost::make_tuple(runTime, plannersToTest.at(i).second->as<ompl::geometric::FMT>()->iterationProgressProperty(), finalCost) );
                    }
                    else if (isRrtStar(plannersToTest.at(i).first))
                    {
                        progressTuple.push_back( boost::make_tuple(runTime, plannersToTest.at(i).second->as<ompl::geometric::RRTstar>()->numIterations(), finalCost) );
                    }
                    else if (plannersToTest.at(i).first == PLANNER_BITSTAR || plannersToTest.at(i).first == PLANNER_BITSTAR_SEED)
                    {
                        progressTuple.push_back( boost::make_tuple(runTime, plannersToTest.at(i).second->as<ompl::geometric::BITstar>()->numIterations(), finalCost) );
                    }
//                    else if (plannersToTest.at(i).first == PLANNER_HYBRID_BITSTAR)
//                    {
//                        progressTuple.push_back( boost::make_tuple(runTime, plannersToTest.at(i).second->as<ompl::geometric::HybridBITstar>()->numIterations(), finalCost) );
//                    }
                    else
                    {
                        throw ompl::Exception("Planner not recognized for progress logging.");
                    }

                    //Save the progress:
                    plannerProgress.addResult(plannersToTest.at(i).second->getName(), progressTuple);
                }
                else
                {
                    //Push back the final solution:
                    if (pdef->hasExactSolution() == true)
                    {
                        progressPair.push_back(std::make_pair(runTime, pdef->getSolutionPath()->cost(expDefn->getOptimizationObjective()).value()));
                    }
                    else
                    {
                        progressPair.push_back(std::make_pair(runTime, ASRL_DBL_INFINITY));
                    }

                    //Save the progress:
                    plannerProgress.addResult(plannersToTest.at(i).second->getName(), progressPair);
                }
            }
            else
            {
                startTime = ompl::time::now();
                //plannersToTest.at(i).second->solve( expDefn->getTargetTime() - runTime );
                plannersToTest.at(i).second->solve( ompl::time::seconds(expDefn->getTargetTime() - runTime) );
                runTime = runTime + (ompl::time::now() - startTime);
            }


            //Store the information:
            planner_result_t myResult;
            if (pdef->hasExactSolution() == true)
            {
                myResult = std::make_pair(runTime, pdef->getSolutionPath()->cost(expDefn->getOptimizationObjective()));
            }
            else
            {
                myResult = std::make_pair(runTime, ompl::base::Cost(ASRL_DBL_INFINITY) );
            }

            //Save the map?
            writeMatlabMap(expDefn, plannersToTest.at(i).first, plannersToTest.at(i).second, masterSeed, informedEllipse, bitStarEllipse, bitStarEdge, bitStarQueue);

            std::cout << myResult.first << ", " << std::setw(4) << myResult.second;
            if (i != plannersToTest.size() - 1u)
            {
                std::cout << ";    " << std::flush;
            }
        }
        std::cout << std::endl;
    }

    return 0;
}
