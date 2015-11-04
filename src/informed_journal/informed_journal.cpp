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
const double checkResolution = 0.001;

//Others:
const double pruneFraction = 0.05;
const double goalBias = 0.05; //8D: 0.05; //2D: 0.05
const bool kNearestRRT = false;
const double rewireFactorRRT = 1.1; //The factor scaling the RGG term

//Experiment:
const bool logProgress = true;
const double millisecSleep = 1.0; //Period for logging data

enum ProblemType
{
    COST_V_TIME,
    TIME_V_GAPSIZE,
    TIME_V_MAPSIZE,
    TIME_V_RN,
    TIME_V_TARGET
};



std::string ProblemName(ProblemType problemType)
{
    switch (problemType)
    {
        case COST_V_TIME:
            return "CostVTime";
            break;
        case TIME_V_GAPSIZE:
            return "TimeVGap";
            break;
        case TIME_V_MAPSIZE:
            return "TimeVMap";
            break;
        case TIME_V_RN:
            return "TimeVRn";
            break;
        case TIME_V_TARGET:
            return "TimeVTarget";
            break;
        default:
            throw ompl::Exception("Not implemented in problemName(PlannerType)");
            break;
    }
};


bool argParse(int argc, char** argv, unsigned int* dimensionPtr, ProblemType* problemPtr, unsigned int* numTrialsPtr, double* runTimePtr, double* widthPtr, bool* animatePtr)
{
    // Declare the supported options.
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("state,r", boost::program_options::value<unsigned int>(), "The state dimension.")
        ("problem,p", boost::program_options::value<std::string>(), "The experiment to use. Valid options are CostVTime, TimeVGap, TimeVMapSize, TimeVRn, TimeVTarget.") //Alphabetical order
        ("experiments,e", boost::program_options::value<unsigned int>(), "The number of unique experiments to run on the random world.")
        ("runtime,t", boost::program_options::value<double>(), "The CPU time in seconds for which to run the planners, (0,infty)")
        ("width,w", boost::program_options::value<double>(), "The width of world. Only valid when running the TimeVMapSize problem.")
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

    if (vm.count("problem"))
    {
        // Get the specified problem as a string
        std::string problemStr = vm["problem"].as<std::string>();

        if (boost::iequals("CostVTime", problemStr))
        {
            *problemPtr = COST_V_TIME;
            // No state dimension constraints
        }
        else if (boost::iequals("TimeVGap", problemStr))
        {
            *problemPtr = TIME_V_GAPSIZE;
            // No state dimension constraints
        }
        else if (boost::iequals("TimeVMapSize", problemStr))
        {
            *problemPtr = TIME_V_MAPSIZE;
            // No state dimension constraints
        }
        else if (boost::iequals("TimeVRn", problemStr))
        {
            *problemPtr = TIME_V_RN;
            // No state dimension constraints
        }
        else if (boost::iequals("TimeVTarget", problemStr))
        {
            *problemPtr = TIME_V_TARGET;
            // No state dimension constraints
        }
        else
        {
            std::cout << "Unrecognized problem name." << std::endl << std::endl << desc << std::endl;
        }
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

    if (vm.count("width"))
    {
        *widthPtr = vm["width"].as<double>();
    }
    else if (*problemPtr == TIME_V_MAPSIZE)
    {
        std::cout << "Width of the world not set for the Time v. Map problem." << std::endl << std::endl << desc << std::endl;
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
boost::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar(const ompl::base::SpaceInformationPtr &si, double steerEta = 0.0)
{
    //Create a RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = boost::make_shared<ompl::geometric::RRTstar>(si);

    //Configure it
    plnr->setGoalBias(goalBias);
    plnr->setKNearest(kNearestRRT);
    plnr->setRewireFactor(rewireFactorRRT);
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
}

//Allocate and configure RRT* with sample rejection
boost::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar_SampRej(const ompl::base::SpaceInformationPtr &si, double steerEta = 0.0)
{
    //Create an RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta);

    //Configure it to be Informed
    plnr->setSampleRejection(true);
    plnr->setName("RRTstar_SampleRejection");

    //Return
    return plnr;
}

//Allocate and configure RRT* with new-state rejection
boost::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar_NewRej(const ompl::base::SpaceInformationPtr &si, double steerEta = 0.0)
{
    //Create an RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta);

    //Configure it to be Informed
    plnr->setNewStateRejection(true);
    plnr->setName("RRTstar_NewStateRejection");

    //Return
    return plnr;
}

//Allocate and configure RRT* with pruning
boost::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar_Prune(const ompl::base::SpaceInformationPtr &si, double steerEta = 0.0)
{
    //Create an RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta);

    //Configure it to be Informed
    plnr->setPruneThreshold(pruneFraction);
    plnr->setTreePruning(true);
    plnr->setName("RRTstar_Prune");

    //Return
    return plnr;
}

//Allocate and configure RRT* with pruning
boost::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar_Trio(const ompl::base::SpaceInformationPtr &si, double steerEta = 0.0)
{
    //Create an RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta);

    //Configure it to be Informed
    plnr->setSampleRejection(true);
    plnr->setNewStateRejection(true);
    plnr->setPruneThreshold(pruneFraction);
    plnr->setTreePruning(true);
    plnr->setName("RRTstar_Trio");

    //Return
    return plnr;
}

//Allocate and configure Informed RRT*
boost::shared_ptr<ompl::geometric::RRTstar> allocateInformedRrtStar(const ompl::base::SpaceInformationPtr &si, double steerEta = 0.0)
{
    //Create an RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta);

    //Configure it to be Informed
    plnr->setPruneThreshold(pruneFraction);
    plnr->setInformedRrtStar(true);
    plnr->setName("Informed_RRTstar");

    //Return
    return plnr;
}

BaseExperimentPtr allocateExperiment(const ProblemType pType, const unsigned int dim, const double runTime, const double mapWidth)
{
    // A random number generator
    ompl::RNG rng;

    switch (pType)
    {
        case COST_V_TIME:
        {
            return boost::make_shared<RandomRectanglesExperiment> (dim, 75u, 0.33, runTime, checkResolution);
            break;
        }
        case TIME_V_GAPSIZE:
        {
            return boost::make_shared<WallGapExperiment> (dim, true, 0.02, runTime, checkResolution);
            break;
        }
        case TIME_V_MAPSIZE:
        {
            //Only use the default resolution for a map width of 2
            return boost::make_shared<CentreSquareExperiment> (dim, rng.uniformReal(0.25,0.5), mapWidth, runTime, checkResolution/(0.5*mapWidth));
            break;
        }
        case TIME_V_RN:
        {
            return boost::make_shared<CentreSquareExperiment> (dim, rng.uniformReal(0.25,0.5), 2.0, runTime, checkResolution);
            break;
        }
        case TIME_V_TARGET:
        {
            return boost::make_shared<CentreSquareExperiment> (dim, rng.uniformReal(0.25,0.5), 2.0, runTime, checkResolution);
            break;
        }
        default:
        {
            throw ompl::Exception("ProblemType not implemented in allocateExperiment()");
            break;
        }
    }
    throw ompl::Exception("Write me");
}

void callSolve(const ompl::base::PlannerPtr& planner, const ompl::time::duration& solveDuration)
{
    planner->solve( solveDuration );
}

int main(int argc, char **argv)
{
    std::cout << "TODO: Store target/optimum, and write a new log-processing script that uses it" << std::endl << std::endl;
    //Argument Variables
    //The dimension size:
    unsigned int N;
    //The problem
    ProblemType problemType;
    //The number of experiments
    unsigned int numExperiments;
    //The time for which to run the planners
    double targetTime;
    //The map width of the world (if valid to specify)
    double mapWidth;
    //Whether to make frame-by-frame animations
    bool createAnimationFrames;

    //Get the command line arguments
    if (argParse(argc, argv, &N, &problemType, &numExperiments, &targetTime, &mapWidth, &createAnimationFrames) == false)
    {
        return 1;
    }

    //Variables
//    ompl::RNG::setSeed(2971235666);    std::cout << std::endl << "                   ---------> Seed set! <---------                   " << std::endl << std::endl;
    //Master seed:
    boost::uint32_t masterSeed = ompl::RNG::getSeed();
    //The filename for progress
    std::stringstream fileName;
    //The world name
    std::stringstream worldName;
    //The experiment definitions
    BaseExperimentPtr expDefn;

    //Let people know what's going on:
    std::cout << "Logging: " << logProgress << std::endl;
    std::cout << "Seed: " << masterSeed << std::endl;

    //The results output file:
    fileName << "R" << N << "S" << masterSeed << ProblemName(problemType);
    if (problemType == TIME_V_MAPSIZE)
    {
        fileName << "W" << mapWidth;
    }
    //No else
    fileName << ".csv";

    ProgressFile plannerProgress(fileName.str(), false);


    for (unsigned int q = 0u; q < numExperiments; ++q)
    {
        //The experiment
        expDefn = allocateExperiment(problemType, N, targetTime, mapWidth);
        //The cost-factor
        double costFactor;

        //either 1/min or 1.
        if (expDefn->knowsOptimum() == true)
        {
            costFactor = 1.0/expDefn->getOptimizationObjective()->getCostThreshold().value();
        }
        else
        {
            costFactor = 1.0;
        }

        if (q==0u)
        {
            std::cout << "First experiment: " << std::flush;
            expDefn->print();
        }
        //No else

        //Variables:
        //The vector of planners:
        std::vector<std::pair<PlannerType, boost::shared_ptr<ompl::geometric::RRTstar> > > plannersToTest;

        //Add the planners to test, steer values <= 0 use the SelfConfig default:
        double steerEta = 0.0;
        plannersToTest.push_back( std::make_pair(PLANNER_RRTSTAR, allocateRrtStar(expDefn->getSpaceInformation(), steerEta) ) );
        plannersToTest.push_back( std::make_pair(PLANNER_RRTSTAR, allocateRrtStar_Prune(expDefn->getSpaceInformation(), steerEta) ) );
        plannersToTest.push_back( std::make_pair(PLANNER_RRTSTAR, allocateRrtStar_NewRej(expDefn->getSpaceInformation(), steerEta) ) );
        plannersToTest.push_back( std::make_pair(PLANNER_RRTSTAR, allocateRrtStar_SampRej(expDefn->getSpaceInformation(), steerEta) ) );
        plannersToTest.push_back( std::make_pair(PLANNER_RRTSTAR, allocateRrtStar_Trio(expDefn->getSpaceInformation(), steerEta) ) );
        plannersToTest.push_back( std::make_pair(PLANNER_RRTSTAR_INFORMED, allocateInformedRrtStar(expDefn->getSpaceInformation(), steerEta) ) );

        if (q == 0u)
        {
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

        //Iterate over the planners
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

            //This is here so I can output steer eta before it
            if (i == 0u)
            {
                if (q == 0u)
                {
                    std::cout << "steerEta: " << plannersToTest.front().second->getRange() << std::endl;
                }
                std::cout << q << ": " << std::flush;
            }

            //Run the planner
            if (createAnimationFrames == true)
            {
                runTime = runTime + createAnimation(expDefn, plannersToTest.at(i).first, plannersToTest.at(i).second, masterSeed, expDefn->getTargetTime() - runTime, true, false, false, false);
            }
            else if (logProgress == true)
            {
                // Create run histories
                //A vector of the planner progress
                progress_pair_vector_t progressPair;

                //Store the starting time:
                startTime = ompl::time::now();

                //Start solving in another thread. We use an intermediate function for this as the class function is overloaded.
                boost::thread solveThread(callSolve, plannersToTest.at(i).second, expDefn->getTargetTime() - runTime);

                //Log data
                do
                {
                    switch (plannersToTest.at(i).first)
                    {
                    case PLANNER_RRTSTAR:
                    case PLANNER_RRTSTAR_SEED:
                    case PLANNER_RRTSTAR_INFORMED:
                        progressPair.push_back( std::make_pair(ompl::time::now() - startTime + runTime, costFactor*plannersToTest.at(i).second->bestCost().value()) );
                        break;
                    default:
                        throw ompl::Exception("Planner not recognized for progress logging.");
                        break;
                    }
                }
                while ( solveThread.timed_join(boost::posix_time::milliseconds(millisecSleep)) == false);

                //Store the final run time
                runTime = runTime + (ompl::time::now() - startTime);

                //Push back the final solution:
                if (pdef->hasExactSolution() == true)
                {
                    progressPair.push_back(std::make_pair(runTime, costFactor*pdef->getSolutionPath()->cost(expDefn->getOptimizationObjective()).value()));
                }
                else
                {
                    progressPair.push_back(std::make_pair(runTime, ASRL_DBL_INFINITY));
                }

                //Save the progress:
                plannerProgress.addResult(plannersToTest.at(i).second->getName(), progressPair);
            }
            else
            {
                //  Just running to get the end result:
                startTime = ompl::time::now();
                static_cast<ompl::base::PlannerPtr>(plannersToTest.at(i).second)->solve( expDefn->getTargetTime() - runTime );
                runTime = runTime + (ompl::time::now() - startTime);
            }

            //Save the map?
            writeMatlabMap(expDefn, plannersToTest.at(i).first, plannersToTest.at(i).second, masterSeed, true, false, false, false);

            double finalCost;

            //Get the final cost:
            if (pdef->hasExactSolution() == true)
            {
                finalCost = costFactor*pdef->getSolutionPath()->cost(expDefn->getOptimizationObjective()).value();
            }
            else
            {
                finalCost = ASRL_DBL_INFINITY;
            }

            std::cout << runTime << ", " << std::setw(4) << finalCost << std::flush;
            if (expDefn->knowsOptimum() == true && std::isfinite(finalCost) == true)
            {
                std::cout << "x" << std::flush;
            }
            //No else

            if (i != plannersToTest.size() - 1u)
            {
                std::cout << ";    " << std::flush;
            }
        }
        std::cout << std::endl;
    }

    return 0;
}
