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
THIS CODE ONLY COMPILES ON THE set_seeds BRANCH!!!!
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
#include "tools/plotting_tools.h"

#define ASRL_DBL_INFINITY std::numeric_limits<double>::infinity()


//Others:
const unsigned int OUTPUT_PERIOD = 100u;
const unsigned int MAX_INIT_SOLN_ITERS = 1000u;
const double PRUNE_FRACTION = 0.05;
const double GOAL_BIAS = 0.025;
const bool KNEAREST_RRT = false;
const double INITIAL_REWIRE_SCALE = 1.1; //The factor scaling the RGG term
const double INITIAL_STEER_ETA = 0.25; //Quarter the distance from start to goal.

bool argParse(int argc, char** argv, unsigned int* dimensionPtr, double* steerPtr, double* rewirePtr, unsigned int* numTrialsPtr, unsigned int* numIterationsPtr, bool* animatePtr)
{
    // Declare the supported options.
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("state,r", boost::program_options::value<unsigned int>(), "The state dimension.")
        ("steer-eta,s", boost::program_options::value<double>(), "The steer eta, or maximum edge length, to use. Negative values use Infinity, 0.0 uses the OMPL auto calculation.")
        ("rewire-scale,x", boost::program_options::value<double>(), "The rewire radius scale to use. Negative values use Infinity.")
        ("trials,t", boost::program_options::value<unsigned int>(), "The number of unique trials to run with the same initial condition.")
        ("iterations,i", boost::program_options::value<unsigned int>(), "The number of iterations for which to run the planner, (0,infty)")
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

    if (vm.count("steer-eta"))
    {
        *steerPtr = vm["steer-eta"].as<double>();

        if (*steerPtr  < 0.0)
        {
            *steerPtr = ASRL_DBL_INFINITY;
        }
    }
    else
    {
        std::cout << "steer-eta not set" << std::endl << std::endl << desc << std::endl;
        return false;
    }

    if (vm.count("rewire-scale"))
    {
        *rewirePtr = vm["rewire-scale"].as<double>();

        if (*rewirePtr < 0.0)
        {
            *rewirePtr = ASRL_DBL_INFINITY;
        }
    }
    else
    {
        std::cout << "rewire-scale not set" << std::endl << std::endl << desc << std::endl;
        return false;
    }

    if (vm.count("trials"))
    {
        *numTrialsPtr = vm["trials"].as<unsigned int>();
    }
    else
    {
        std::cout << "Number of trials not set" << std::endl << std::endl << desc << std::endl;
        return false;
    }

    if (vm.count("iterations"))
    {
        *numIterationsPtr = vm["iterations"].as<unsigned int>();
    }
    else
    {
        std::cout << "Number of iterations not set" << std::endl << std::endl << desc << std::endl;
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
boost::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar(const ompl::base::SpaceInformationPtr &si, double steerEta, double rewireScale)
{
    //Create a RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = boost::make_shared<ompl::geometric::RRTstar>(si);

    //Configure it
    plnr->setGoalBias(GOAL_BIAS);
    plnr->setKNearest(KNEAREST_RRT);
    plnr->setRewireFactor(rewireScale);
    plnr->setDelayCC(true);
    plnr->setPruneThreshold(1.0);
    plnr->setTreePruning(false);
    plnr->setSampleRejection(false);
    plnr->setNewStateRejection(false);
    plnr->setAdmissibleCostToCome(true);
    plnr->setInformedRrtStar(false);
    plnr->setNumSamplingAttempts(1u);
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
boost::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar_SampRej(const ompl::base::SpaceInformationPtr &si, double steerEta, double rewireScale)
{
    //Create an RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta, rewireScale);

    //Configure it to be Informed
    plnr->setSampleRejection(true);
    plnr->setName("RRTstar_SampleRejection");

    //Return
    return plnr;
};

//Allocate and configure RRT* with new-state rejection
boost::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar_NewRej(const ompl::base::SpaceInformationPtr &si, double steerEta, double rewireScale)
{
    //Create an RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta, rewireScale);

    //Configure it to be Informed
    plnr->setNewStateRejection(true);
    plnr->setName("RRTstar_NewStateRejection");

    //Return
    return plnr;
};

//Allocate and configure RRT* with pruning
boost::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar_Prune(const ompl::base::SpaceInformationPtr &si, double steerEta, double rewireScale)
{
    //Create an RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta, rewireScale);

    //Configure it to be Informed
    plnr->setPruneThreshold(PRUNE_FRACTION);
    plnr->setTreePruning(true);
    plnr->setName("RRTstar_Prune");

    //Return
    return plnr;
};

//Allocate and configure RRT* with pruning
boost::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar_Trio(const ompl::base::SpaceInformationPtr &si, double steerEta, double rewireScale)
{
    //Create an RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta, rewireScale);

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
boost::shared_ptr<ompl::geometric::RRTstar> allocateInformedRrtStar(const ompl::base::SpaceInformationPtr &si, double steerEta, double rewireScale)
{
    //Create an RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr;
    plnr = allocateRrtStar(si, steerEta, rewireScale);

    //Configure it to be Informed
    plnr->setPruneThreshold(PRUNE_FRACTION);
    plnr->setInformedRrtStar(true);
    plnr->setName("Informed_RRTstar");

    //Return
    return plnr;
};

boost::shared_ptr<ompl::geometric::RRTstar> allocatePlanner(const PlannerType plnrType, const BaseExperimentPtr& expDefn, const double steerEta, double rewireScale)
{
    switch(plnrType)
    {
        case PLANNER_RRTSTAR:
        {
            return allocateRrtStar(expDefn->getSpaceInformation(), steerEta, rewireScale);
            break;
        }
        case PLANNER_RRTSTAR_PRUNE:
        {
            return allocateRrtStar_Prune(expDefn->getSpaceInformation(), steerEta, rewireScale);
            break;
        }
        case PLANNER_RRTSTAR_NEW_REJECT:
        {
            return allocateRrtStar_NewRej(expDefn->getSpaceInformation(), steerEta, rewireScale);
            break;
        }
        case PLANNER_RRTSTAR_SAMPLE_REJECT:
        {
            return allocateRrtStar_SampRej(expDefn->getSpaceInformation(), steerEta, rewireScale);
            break;
        }
        case PLANNER_RRTSTAR_TRIO:
        {
            return allocateRrtStar_Trio(expDefn->getSpaceInformation(), steerEta, rewireScale);
            break;
        }
        case PLANNER_RRTSTAR_INFORMED:
        {
            return allocateInformedRrtStar(expDefn->getSpaceInformation(), steerEta, rewireScale);
            break;
        }
        default:
        {
            throw ompl::Exception("Unrecognized planner type in allocatePlanner()");
        }
    }
};

void outputResult(IterationCostHistory history)
{
    //Output info to the terminal, this column can be up to 25 chars wide:
    //Padding:
    std::cout << std::setw(25-8-6) << std::setfill(' ') << " ";

    //The number of iterations (8-wide with the ": ")
    std::cout << std::setw(8) << std::setfill(' ') << history.back().first << ": ";

    //And the cost:
    std::cout << std::setw(6) << std::setfill(' ') << std::setprecision(4) << history.back().second << std::flush;
}

int main(int argc, char **argv)
{
    //Argument Variables
    //The dimension size:
    unsigned int N;
    //The steer / range of RRT
    double steerEta;
    //The rewire factor of RRT
    double rewireScale;
    //The number of trials
    unsigned int numTrials;
    //The time for which to run the planners
    unsigned int numIterations;
    //Whether to make frame-by-frame animations
    bool createAnimationFrames;

    //Get the command line arguments
    if (argParse(argc, argv, &N, &steerEta, &rewireScale, &numTrials, &numIterations, &createAnimationFrames) == false)
    {
        return 1;
    }

    //Variables
    //ompl::RNG::setSeed(107965365);    std::cout << std::endl << "                   ---------> Seed set! <---------                   " << std::endl << std::endl;
    //Master seed:
    boost::uint32_t masterSeed = ompl::RNG::getSeed();
    //The filename for progress
    std::stringstream fileName;
    //The world name
    std::stringstream worldName;
    //The current experiment
    ObstacleFreeExperimentPtr expDefn = boost::make_shared<ObstacleFreeExperiment>(N, 1u, 1u, ASRL_DBL_INFINITY);
    //The vector of planner types:
    std::vector<PlannerType> plannersToTest;

    //Specify the planners:
//    plannersToTest.push_back(PLANNER_RRTSTAR);
//    plannersToTest.push_back(PLANNER_RRTSTAR_PRUNE);
//    plannersToTest.push_back(PLANNER_RRTSTAR_NEW_REJECT);
//    plannersToTest.push_back(PLANNER_RRTSTAR_SAMPLE_REJECT);
//    plannersToTest.push_back(PLANNER_RRTSTAR_TRIO);
    plannersToTest.push_back(PLANNER_RRTSTAR_INFORMED);

    std::cout << "To make this code the most efficient, make sure that the version of OMPL compares costs directly without adding a margin." << std::endl;

    //The results output filename:
    fileName << "ConvergeR" << N << "S" << masterSeed << "Steer" << steerEta << "Rewire" << rewireScale << ".csv";

    //Let people know what's going on:
    std::cout << "Seed: " << masterSeed << std::endl;
    std::cout << "Output: " << fileName.str() << std::endl;

    //The iteration/cost results:
    ResultsFile<IterationCostHistory> costPerIteration(fileName.str());


    //Perform numTrials
    for (unsigned int q = 0u; q < numTrials; ++q)
    {
        for (unsigned int p = 0u; p < plannersToTest.size(); ++p)
        {
            //Variables
            //The current planner:
            boost::shared_ptr<ompl::geometric::RRTstar> plnr;
            //The optimization objective used by this planner
            ompl::base::OptimizationObjectivePtr opt = expDefn->getOptimizationObjective();
            //The results from this planner:
            IterationCostHistory iterCostResults(numIterations);
            //A 1-iteration PTC:
            ompl::base::IterationTerminationCondition iterationPtc(1u);
            //The target number of extra iterations after the first solution:
            unsigned int targetIters;

            //Allocate the planner:
            plnr = allocatePlanner(plannersToTest.at(p), expDefn, INITIAL_STEER_ETA, INITIAL_REWIRE_SCALE);

            //Give a problem definition to the planner:
            plnr->setProblemDefinition(expDefn->newProblemDefinition());

            //Setup
            plnr->setup();

            //Specify the same seed for the planner:
            plnr->setLocalSeed(masterSeed);

            //Find the initial solution:
            opt->setCostThreshold(opt->infiniteCost());

            //Run as an animation or not until the first solution:
            if (createAnimationFrames == true)
            {
                //Create the animation:
                createAnimation(expDefn, plannersToTest.at(p), plnr, masterSeed + q, boost::date_time::pos_infin, true, false, false, false);

                //Push back the initial value
                iterCostResults.push_back(std::make_pair(plnr->numIterations(), plnr->bestCost().value()));
            }
            else
            {
                //Push back an initial value:
                iterCostResults.push_back(std::make_pair(0u, ASRL_DBL_INFINITY));

                //Run for the specified number of iterations or until a solution is found:
                while ((plnr->numIterations() < MAX_INIT_SOLN_ITERS) && (opt->isSatisfied(plnr->bestCost()) == false))
                {
                    //Reset the iteration PTC
                    iterationPtc.reset();

                    //Run one iteration of the planner:
                    plnr->solve(iterationPtc);

                    //Push back the current value
                    iterCostResults.push_back(std::make_pair(plnr->numIterations(), plnr->bestCost().value()));
                }
            }

            //Output some info, including the intiial solution:
            //If this is the first planner of a trial, output at least the trial number:
            if (p == 0u && q%OUTPUT_PERIOD == 0u)
            {
                std::cout << std::endl;
                //If this is also the first run ever, output start info:
                if (q == 0u)
                {
                    //The first column is 7 wide for a 6-digit trial number and a ":"
                    std::cout << std::setw(7) << std::setfill(' ') << " ";

                    //The subsequent columns are 25 wide for planner names
                    for (unsigned int i = 0u; i < plannersToTest.size(); ++i)
                    {
                        std::cout << std::setw(25) << std::setfill(' ') << plnr->getName() << std::flush;
                    }
                    std::cout << std::endl;

                    //Then the initial solution:
                    std::cout << std::setw(6) << std::setfill(' ') << "init" << ":" << std::flush;

                    outputResult(iterCostResults);
                    std::cout << std::endl;
                }
                //No else

                //The trial number is 6-digits with 3 of padding
                std::cout << std::setw(6) << std::setfill(' ') << q << ":" << std::flush;
            }
            //No else

            //Now, set the planner to a random seed:
            plnr->setLocalSeed(masterSeed + q);

            //Forget the solution it placed in the problem definition:
            plnr->getProblemDefinition()->clearSolutionPaths();

            //And change the rewiring parameters:
            plnr->setRewireFactor(rewireScale);
            plnr->setRange(steerEta);

            //Set the objective to be the optimum:
            opt->setCostThreshold(expDefn->getMinimum());

            //Calculate when to stop:
            targetIters = plnr->numIterations() + numIterations;

            //And either make an (arbitrary) number of further animation frames or run for the specified number of iterations:
            if (createAnimationFrames == true)
            {
                createAnimation(expDefn, plannersToTest.at(p), plnr, masterSeed + q, ompl::time::seconds(1.0), true, false, false, false, plnr->numIterations()+1u);

                //Push back the last value
                iterCostResults.push_back(std::make_pair(plnr->numIterations(), plnr->bestCost().value()));
            }
            else
            {
                //Run for the specified number of iterations:
                while ((plnr->numIterations() < targetIters) && (opt->isSatisfied(plnr->bestCost()) == false))
                {
                    //Reset the iteration PTC
                    iterationPtc.reset();

                    //Run one iteration of the planner:
                    plnr->solve(iterationPtc);

                    //Push back the current value
                    iterCostResults.push_back(std::make_pair(plnr->numIterations(), plnr->bestCost().value()));
                }
            }
            //Add the result:
            costPerIteration.addResult(plannerName(plannersToTest.at(p)), iterCostResults);

            //Save the map:
            std::stringstream postFix;
            postFix << "E" << q;
            writeMatlabMap(expDefn, plannersToTest.at(p), plnr, masterSeed, false, false, false, false, "plots/", postFix.str());

            if (q%OUTPUT_PERIOD == 0u)
            {
                outputResult(iterCostResults);
            }
        }
        //Get next planner
    }
    //Perform experiment with next seed

    std::cout << std::endl;
    return 0;
}
