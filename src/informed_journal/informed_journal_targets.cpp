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
const double CHECK_RESOLUTION = 0.001;

//Others:
const double PRUNE_FRACTION = 0.05;
const double GOAL_BIAS = 0.05; //8D: 0.05; //2D: 0.05
const bool KNEAREST_RRT = false;
const double REWIRE_SCALE_RRT = 2.0; //The factor scaling the RGG term

enum ProblemType
{
    TIME_V_GAPSIZE,
    TIME_V_MAPSIZE,
    TIME_V_TARGET
};

std::string ProblemName(ProblemType problemType)
{
    switch (problemType)
    {
        case TIME_V_GAPSIZE:
            return "TimeVGap";
            break;
        case TIME_V_MAPSIZE:
            return "TimeVMap";
            break;
        case TIME_V_TARGET:
            return "TimeVTarget";
            break;
        default:
            throw ompl::Exception("Not implemented in problemName(PlannerType)");
            break;
    }
};

std::string problemPostfix(ProblemType probType)
{
    switch (probType)
    {
        case TIME_V_GAPSIZE:
            return "G";
            break;
        case TIME_V_MAPSIZE:
            return "W";
            break;
        case TIME_V_TARGET:
            return "F";
            break;
        default:
            throw ompl::Exception("Not implemented in problemPostfix(PlannerType)");
            break;
    }
};


bool argParse(int argc, char** argv, unsigned int* dimensionPtr, ProblemType* problemPtr, std::vector<double>* indepVariablePtrs, double* steerPtr, double* goalTolPtr, unsigned int* numTrialsPtr, double* runTimePtr, bool* animatePtr)
{
    // Declare the supported options.
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("state,r", boost::program_options::value<unsigned int>(), "The state dimension.")
        ("problem,p", boost::program_options::value<std::string>(), "The experiment to use. Valid options are TimeVGap, TimeVMapSize, TimeVTarget.") //Alphabetical order
        ("variable,v", boost::program_options::value<std::vector<double> >()->multitoken(), "The permuted variable for the specified problem. For TimeVGap this is the gap width, for TimeVMapSize the map width, and for TimeVTarget the percent optimum.")
        ("goal-tol,g", boost::program_options::value<double>(), "The fraction of the optimum that defines a successful run. Only valid for TimeVMapSize.")
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

    if (vm.count("problem"))
    {
        // Get the specified problem as a string
        std::string problemStr = vm["problem"].as<std::string>();

        if (boost::iequals("TimeVGap", problemStr))
        {
            *problemPtr = TIME_V_GAPSIZE;
            // No state dimension constraints
        }
        else if (boost::iequals("TimeVMapSize", problemStr))
        {
            *problemPtr = TIME_V_MAPSIZE;
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
            return false;
        }
    }

    if (vm.count("variable"))
    {
        bool correct = true;

        *indepVariablePtrs = vm["variable"].as<std::vector<double> >();

        if (*problemPtr == TIME_V_MAPSIZE)
        {
            for (unsigned int i = 0u; correct == true && i < indepVariablePtrs->size() - 1u; ++i)
            {
                correct = indepVariablePtrs->at(i) < indepVariablePtrs->at(i + 1u);
            }
        }
        else if (*problemPtr == TIME_V_GAPSIZE || *problemPtr == TIME_V_TARGET)
        {
            for (unsigned int i = 0u; correct == true && i < indepVariablePtrs->size() - 1u; ++i)
            {
                correct = indepVariablePtrs->at(i) > indepVariablePtrs->at(i + 1u);
            }
        }
        else
        {
            throw ompl::Exception("Unrecognized problem type in argParse()");
        }

        if (correct == false)
        {
            std::cout << "Variables must be listed in order of increasing difficulty." << std::endl << std::endl << desc << std::endl;
            return false;
        }
    }
    else
    {
        std::cout << "Variables not set." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    *steerPtr = vm["steer-eta"].as<double>();

    if (*problemPtr == TIME_V_MAPSIZE)
    {
        if (vm.count("goal-tol"))
        {
            *goalTolPtr = vm["goal-tol"].as<double>();
        }
        else
        {
            std::cout << "Goal tolerance not set for TimeVMapSize" << std::endl << std::endl << desc << std::endl;
            return false;
        }
    }
    else if (vm.count("goal-tol"))
    {
        std::cout << "Goal tolerance only valid for TimeVMapSize" << std::endl << std::endl << desc << std::endl;
        return false;
    }
    //No else

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


//Allocate and configure RRT*
boost::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar(const ompl::base::SpaceInformationPtr &si, double steerEta)
{
    //Create a RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = boost::make_shared<ompl::geometric::RRTstar>(si);

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
boost::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar_SampRej(const ompl::base::SpaceInformationPtr &si, double steerEta)
{
    //Create an RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta);

    //Configure it to be Informed
    plnr->setSampleRejection(true);
    plnr->setName("RRTstar_SampleRejection");

    //Return
    return plnr;
};

//Allocate and configure RRT* with new-state rejection
boost::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar_NewRej(const ompl::base::SpaceInformationPtr &si, double steerEta)
{
    //Create an RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta);

    //Configure it to be Informed
    plnr->setNewStateRejection(true);
    plnr->setName("RRTstar_NewStateRejection");

    //Return
    return plnr;
};

//Allocate and configure RRT* with pruning
boost::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar_Prune(const ompl::base::SpaceInformationPtr &si, double steerEta)
{
    //Create an RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta);

    //Configure it to be Informed
    plnr->setPruneThreshold(PRUNE_FRACTION);
    plnr->setTreePruning(true);
    plnr->setName("RRTstar_Prune");

    //Return
    return plnr;
};

//Allocate and configure RRT* with pruning
boost::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar_Trio(const ompl::base::SpaceInformationPtr &si, double steerEta)
{
    //Create an RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
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
boost::shared_ptr<ompl::geometric::RRTstar> allocateInformedRrtStar(const ompl::base::SpaceInformationPtr &si, double steerEta)
{
    //Create an RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta);

    //Configure it to be Informed
    plnr->setPruneThreshold(PRUNE_FRACTION);
    plnr->setInformedRrtStar(true);
    plnr->setName("Informed_RRTstar");

    //Return
    return plnr;
};

boost::shared_ptr<ompl::geometric::RRTstar> allocatePlanner(const PlannerType plnrType, const BaseExperimentPtr& expDefn, const double steerEta)
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

WallGapExperimentPtr allocateGapExperiment(const unsigned int dim, const double gapWidth, const double runTime, const double resolution)
{
    return boost::make_shared<WallGapExperiment> (dim, true, gapWidth, runTime, resolution);
};

CentreSquareExperimentPtr allocateMapExperiment(const unsigned int dim, const double mapWidth, const double goalTol, const double runTime, const double resolution)
{
    //Variable
    //The return value
    CentreSquareExperimentPtr exp;
    // A random number generator
    ompl::RNG rng;

    exp = boost::make_shared<CentreSquareExperiment> (dim, rng.uniformReal(0.25,0.5), mapWidth, runTime, resolution/(0.5*mapWidth));

    exp->setTarget(goalTol);

    return exp;

};

CentreSquareExperimentPtr allocateTargetExperiment(const unsigned int dim, const double runTime, const double resolution)
{
    // A random number generator
    ompl::RNG rng;

    return boost::make_shared<CentreSquareExperiment> (dim, rng.uniformReal(0.25,0.5), 2.0, runTime, resolution);
};


std::vector<BaseExperimentPtr> allocateRunExperiments(const ProblemType probType, const unsigned int dim, const double maxTime, const double resolution, const std::vector<double>& variables, const double goalTol)
{
    //Variables
    //The return value
    std::vector<BaseExperimentPtr> exps;

    //Allocate the experiments depending on problem
    switch (probType)
    {
        case TIME_V_GAPSIZE:
        {
            //Create a problem for each variable:
            for (unsigned int i = 0u; i < variables.size(); ++i)
            {
                exps.push_back(allocateGapExperiment(dim, variables.at(i), maxTime, resolution));
            }
            break;
        }
        case TIME_V_MAPSIZE:
        {
            //Create a problem for each variable:
            for (unsigned int i = 0u; i < variables.size(); ++i)
            {
                exps.push_back(allocateMapExperiment(dim, variables.at(i), goalTol, maxTime, resolution));
            }
            break;
        }
        case TIME_V_TARGET:
        {
            //If we're searching for the time-to-target, we only have to create one experiment on the first trial:
            exps.push_back(allocateTargetExperiment(dim, maxTime, resolution));
            break;
        }
        default:
        {
            throw ompl::Exception("Unrecognized problemType.");
            break;
        }
    }

    return exps;
}

int main(int argc, char **argv)
{
    //Argument Variables
    //The dimension size:
    unsigned int N;
    //The problem
    ProblemType problemType;
    //The number of experiments
    unsigned int numRuns;
    //The time for which to run the planners
    double maxTime;
    //The variable of the experiment world
    std::vector<double> indepVariables;
    //The goal tolerance for MapSize
    double goalTol;
    //The steer / range of RRT
    double steerEta;
    //Whether to make frame-by-frame animations
    bool createAnimationFrames;

    //Get the command line arguments
    if (argParse(argc, argv, &N, &problemType, &indepVariables, &steerEta, &goalTol, &numRuns, &maxTime, &createAnimationFrames) == false)
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

    //The results output file:
    fileName << "R" << N << "S" << masterSeed << ProblemName(problemType) << ".csv";

    //Let people know what's going on:
    std::cout << "Seed: " << masterSeed << std::endl;
    std::cout << "Output: " << fileName.str() << std::endl;

    TargetFile timeToTarget(fileName.str());

    //Perform numRuns
    for (unsigned int q = 0u; q < numRuns; ++q)
    {
        //Variables:
        //The vector of planner types:
        std::vector<PlannerType> plannersToTest;
        //The experiment definitions for this run
        std::vector<BaseExperimentPtr> runExperiments;
        //A vector of RNGs, we will use their seeds for all the planners, but not actually use the RNG...
        std::vector< boost::shared_ptr<ompl::RNG> > seedRNGs;

        //Specify the planners:
        plannersToTest.push_back(PLANNER_RRTSTAR);
        plannersToTest.push_back(PLANNER_RRTSTAR_PRUNE);
        plannersToTest.push_back(PLANNER_RRTSTAR_NEW_REJECT);
        plannersToTest.push_back(PLANNER_RRTSTAR_SAMPLE_REJECT);
        plannersToTest.push_back(PLANNER_RRTSTAR_TRIO);
        plannersToTest.push_back(PLANNER_RRTSTAR_INFORMED);

        //Create all the experiments for this run:
        runExperiments = allocateRunExperiments(problemType, N, maxTime, CHECK_RESOLUTION, indepVariables, goalTol);

        //Iterate over the planners:
        for (unsigned int p = 0u; p < plannersToTest.size(); ++p)
        {
            //Variables
            //The current experiment
            BaseExperimentPtr expDefn;
            //The current planner:
            boost::shared_ptr<ompl::geometric::RRTstar> plnr;
            //The cumulative runtime
            ompl::time::duration runTime(0,0,0,0);
            //The results from this planners run across all the variates:
            target_pair_vector_t runResults;

            //Now iterate over the variables:
            for (unsigned int v = 0u; v < indepVariables.size(); ++v)
            {
                //Variables
                //The start time for a call
                ompl::time::point startTime;
                //The problem defintion used by this planner
                ompl::base::ProblemDefinitionPtr pdef;

                //If this is the first planner of this experiment, create the seed for this target variable
                if (p == 0u)
                {
                    seedRNGs.push_back( boost::make_shared<ompl::RNG>() );
                }

                //Ok, what we do depends on the problem type.
                if (problemType == TIME_V_GAPSIZE || problemType == TIME_V_MAPSIZE)
                {
                    //This is TIME_V_GAPSIZE or TIME_V_MAPSIZE, we always just get the next
                    expDefn = runExperiments.at(v);
                    //allocate,
                    plnr = allocatePlanner(plannersToTest.at(p), expDefn, steerEta);
                    //and reset the run time. Easy.
                    runTime = ompl::time::duration(0,0,0,0);
                }
                else if (problemType == TIME_V_TARGET)
                {
                    //This is TIME_V_TARGET, we only get the experiment, allocate and reset the timer on the first experiment:
                    if (v == 0u)
                    {
                        expDefn = runExperiments.at(0u);
                        plnr = allocatePlanner(plannersToTest.at(p), expDefn, steerEta);
                        runTime = ompl::time::duration(0,0,0,0);
                    }

                    //But we always specify the target:
                    expDefn->setTarget(indepVariables.at(v));
                }
                else
                {
                    throw ompl::Exception("Unsupported problem type in loop");
                }

                //Get the problem definition
                pdef = expDefn->newProblemDefinition();

                //Give to the planner
                plnr->setProblemDefinition(pdef);

                //If necessary, setup
                if (plnr->isSetup() == false)
                {
                    startTime = ompl::time::now();
                    plnr->setup();
                    runTime = ompl::time::now() - startTime;

                    //Set the planner seed:
                    plnr->setLocalSeed(seedRNGs.at(v)->getLocalSeed());
                }

                //This must come after setup to get the steer info!
                //If this is the first run of the first planner, we have info to output
                if (v == 0u)
                {
                    if (p == 0u)
                    {
                        //First, if this is also the first run ever, output start info:
                        if (q == 0u)
                        {
                            std::cout << "First experiments: " << std::endl;
                            for (unsigned int i = 0u; i < runExperiments.size(); ++i)
                            {
                                runExperiments.at(i)->print(false);
                            }
                            std::cout << "Very first steer: " << plnr->getRange() << std::endl;


                            //The first column is 25 wide for planner names
                            std::cout << std::setw(25) << std::setfill(' ') << " ";
                            for (unsigned int i = 0u; i < indepVariables.size(); ++i)
                            {
                                //The subsequent columns are 20 wide for 15 chars of time and 5 of padding
                                std::cout << std::setw(20) << std::setfill(' ') << indepVariables.at(i);
                            }
                            std::cout << std::endl;
                        }
                        //No else

                        //Then, output the trial number
                        std::cout << std::setw(3) << std::setfill('0') << q << std::endl;
                    }
                    //No else

                    //Output the planner name:
                    std::cout << std::setw(25) << std::setfill(' ') << plnr->getName() << std::flush;
                }
                //No else

                //Run the planner:
                if (createAnimationFrames == true)
                {
                    runTime = runTime + createAnimation(expDefn, plannersToTest.at(p), plnr, masterSeed, expDefn->getTargetTime() - runTime, false, false, false, false);
                }
                else
                {
                    //Get the end result:
                    startTime = ompl::time::now();
                    static_cast<ompl::base::PlannerPtr>(plnr)->solve(expDefn->getTargetTime() - runTime);
                    runTime = runTime + (ompl::time::now() - startTime);
                }

                //Store the result:
                if (pdef->hasExactSolution() == true && pdef->getOptimizationObjective()->isSatisfied( pdef->getSolutionPath()->cost(expDefn->getOptimizationObjective()) ) == true)
                {
                    runResults.push_back(std::make_pair(indepVariables.at(v), runTime));
                }
                else
                {
                    runResults.push_back(std::make_pair(indepVariables.at(v), ompl::time::duration(boost::date_time::neg_infin)));
                }

                //Save the map:
                std::stringstream postFix;
                postFix << problemPostfix(problemType) << indepVariables.at(v) << "E" << q;
                writeMatlabMap(expDefn, plannersToTest.at(p), plnr, masterSeed, false, false, false, false, "plots/", postFix.str());

                //Output info to the terminal:
                //If the result is infinite, pad with an extra 6 white spaces, as the word "+infinity" is 9 chars long.:
                if (runResults.back().second.is_special() == true)
                {
                    std::cout << std::setw(6) << std::setfill(' ') << " ";
                }

                //Finally, output the time padded with 5 whitespaces in the front (manually, as it doesn't work on time), as the number is (nominally) of the form 00:00:00.123456
                std::cout << std::setw(5) << std::setfill(' ') << " " << runResults.back().second << std::flush;
            }
            //Get the next independent variable for this planner

            std::cout << std::endl;

            //Store this information in the time file.
            timeToTarget.addResult(plannerName(plannersToTest.at(p)), runResults);
        }
        //Get the next planner
    }
    //Perform the next run

    return 0;
}
