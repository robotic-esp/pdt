//Me
#include <tools/planner_tools.h>

//For boost::make_shared
#include <boost/make_shared.hpp>

boost::shared_ptr<ompl::geometric::RRT> allocateRrt(const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias)
{
    //Create a RRT planner
    boost::shared_ptr<ompl::geometric::RRT> plnr = boost::make_shared<ompl::geometric::RRT>(si);

    //Configure it
    plnr->setRange(steerEta);
    plnr->setGoalBias(goalBias);
    plnr->setName("RRT");

    //Return
    return plnr;
}

boost::shared_ptr<ompl::geometric::RRTConnect> allocateRrtConnect(const ompl::base::SpaceInformationPtr &si, const double steerEta)
{
    //Create a RRT* planner
    boost::shared_ptr<ompl::geometric::RRTConnect> plnr = boost::make_shared<ompl::geometric::RRTConnect>(si);

    //Configure it
    plnr->setRange(steerEta);
    plnr->setName("RRTConnect");

    //Return
    return plnr;
}

boost::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar(const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias, const bool kNearest, const double rewireScale)
{
    //Create a RRT* planner
    boost::shared_ptr<ompl::geometric::RRTstar> plnr = boost::make_shared<ompl::geometric::RRTstar>(si);

    //Configure it
    plnr->setRange(steerEta);
    plnr->setGoalBias(goalBias);
    plnr->setKNearest(kNearest);
    plnr->setRewireFactor(rewireScale);
    plnr->setDelayCC(true);
    plnr->setTreePruning(false);
    plnr->setPruneThreshold(1.0);
    plnr->setPrunedMeasure(false);
    plnr->setSampleRejection(false);
    plnr->setNewStateRejection(false);
    plnr->setAdmissibleCostToCome(true);
    plnr->setInformedSampling(false);

    plnr->setName("RRTstar");

    //Return
    return plnr;
}

boost::shared_ptr<ompl::geometric::InformedRRTstar> allocateInformedRrtStar(const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias, const bool kNearest, const double rewireScale, const double pruneFraction)
{
    //Create an RRT* planner
    boost::shared_ptr<ompl::geometric::InformedRRTstar> plnr  = boost::make_shared<ompl::geometric::InformedRRTstar>(si);

    //Configure it to be Informed
    plnr->setRange(steerEta);
    plnr->setGoalBias(goalBias);
    plnr->setKNearest(kNearest);
    plnr->setRewireFactor(rewireScale);
    plnr->setPruneThreshold(pruneFraction);
    plnr->setDelayCC(true);

    plnr->setName("Informed_RRTstar");

    //Return
    return plnr;
}

boost::shared_ptr<ompl::geometric::FMT> allocateFmtStar(const ompl::base::SpaceInformationPtr &si, const bool kNearest, const unsigned int rewireScale, const unsigned int numSamples, const bool cacheCC,  const bool useHeuristics)
{
    //Create a RRT* planner
    boost::shared_ptr<ompl::geometric::FMT> plnr = boost::make_shared<ompl::geometric::FMT>(si);

    //Configure it
    plnr->setNearestK(kNearest);
    plnr->setRadiusMultiplier(rewireScale);
    plnr->setNumSamples(numSamples);
    plnr->setCacheCC(cacheCC);
    plnr->setHeuristics(useHeuristics);
    plnr->setFreeSpaceVolume(si->getSpaceMeasure());

    std::stringstream plannerName;
    plannerName << "FMTstar" << numSamples;
    plnr->setName(plannerName.str());

    //Return
    return plnr;
}

boost::shared_ptr<ompl::geometric::BITstar> allocateBitStar(const ompl::base::SpaceInformationPtr &si, const bool kNearest, const unsigned int rewireScale, const unsigned int numSamples, const bool pruneFraction, const bool strictQueue, const bool delayRewire, const bool jit, const bool refreshBatches)
{
    //Create a BIT* planner
    boost::shared_ptr<ompl::geometric::BITstar> plnr = boost::make_shared<ompl::geometric::BITstar>(si);

    //Configure it
    plnr->setKNearest(kNearest);
    plnr->setRewireFactor(rewireScale);
    plnr->setSamplesPerBatch(numSamples);
    plnr->setPruning(true);
    plnr->setPruneThresholdFraction(pruneFraction);
    plnr->setStrictQueueOrdering(strictQueue);
    plnr->setDelayRewiringUntilInitialSolution(delayRewire);
    plnr->setJustInTimeSampling(jit);
    plnr->setDropSamplesOnPrune(refreshBatches);
    plnr->setStopOnSolnImprovement(false);

    std::stringstream plannerName;
    plannerName << "BITstar" << numSamples;
    plnr->setName(plannerName.str());

    //Return
    return plnr;
}

bool isRrt(PlannerType plnrType)
{
    return (plnrType == PLANNER_RRT ||
            plnrType == PLANNER_RRTCONNECT);
}

bool isRrtStar(PlannerType plnrType)
{
    return (plnrType == PLANNER_RRTSTAR ||
            plnrType == PLANNER_RRTSTAR_INFORMED ||
            plnrType == PLANNER_RRTSTAR_NEW_REJECT ||
            plnrType == PLANNER_RRTSTAR_PRUNE ||
            plnrType == PLANNER_RRTSTAR_SAMPLE_REJECT ||
            plnrType == PLANNER_RRTSTAR_SEED ||
            plnrType == PLANNER_RRTSTAR_TRIO);
}
bool isBitStar(PlannerType plnrType)
{
    return (plnrType == PLANNER_BITSTAR ||
            plnrType == PLANNER_HYBRID_BITSTAR ||
            plnrType == PLANNER_DUALTREE_BITSTAR ||
            plnrType == PLANNER_BITSTAR_SEED);
}

std::string plannerName(PlannerType plnrType)
{
    switch (plnrType)
    {
        case PLANNER_RRT:
        {
            return "RRT";
            break;
        }
        case PLANNER_RRTCONNECT:
        {
            return "RRTConnect";
            break;
        }
        case PLANNER_RRTSTAR:
        {
            return "RRTStar";
            break;
        }
        case PLANNER_RRTSTAR_PRUNE:
        {
            return "RRTstar_Prune";
            break;
        }
        case PLANNER_RRTSTAR_NEW_REJECT:
        {
            return "RRTstar_NewStateRejection";
            break;
        }
        case PLANNER_RRTSTAR_SAMPLE_REJECT:
        {
            return "RRTstar_SampleRejection";
            break;
        }
        case PLANNER_RRTSTAR_TRIO:
        {
            return "RRTstar_Trio";
            break;
        }
        case PLANNER_RRTSTAR_INFORMED:
        {
            return "Informed_RRTstar";
            break;
        }
        case PLANNER_FMTSTAR:
        {
            return "FMTstar";
            break;
        }
        case PLANNER_BITSTAR:
        {
            return "BITstar";
            break;
        }
        default:
        {
            throw ompl::Exception("Unimplemented PlannerType in plannerName()");
        }
    }

    return std::string();
}
