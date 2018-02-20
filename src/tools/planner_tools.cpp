//Me
#include <tools/planner_tools.h>

//For std::make_shared
#include <memory>

std::shared_ptr<ompl::geometric::RRT> allocateRrt(const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias)
{
    //Create a RRT planner
    std::shared_ptr<ompl::geometric::RRT> plnr = std::make_shared<ompl::geometric::RRT>(si);

    //Configure it
    plnr->setRange(steerEta);
    plnr->setGoalBias(goalBias);
    plnr->setName("RRT");

    //Return
    return plnr;
}

std::shared_ptr<ompl::geometric::RRTConnect> allocateRrtConnect(const ompl::base::SpaceInformationPtr &si, const double steerEta)
{
    //Create a RRT* planner
    std::shared_ptr<ompl::geometric::RRTConnect> plnr = std::make_shared<ompl::geometric::RRTConnect>(si);

    //Configure it
    plnr->setRange(steerEta);
    plnr->setName("RRTConnect");

    //Return
    return plnr;
}

std::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar(const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias, const bool kNearest, const double rewireScale)
{
    //Create a RRT* planner
    std::shared_ptr<ompl::geometric::RRTstar> plnr = std::make_shared<ompl::geometric::RRTstar>(si);

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

std::shared_ptr<ompl::geometric::RRTsharp> allocateRrtSharp(const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias, const bool kNearest, const double rewireScale, const bool reject, const bool informed, const unsigned int variant)
{
    //Create a RRT* planner
    std::shared_ptr<ompl::geometric::RRTsharp> plnr = std::make_shared<ompl::geometric::RRTsharp>(si);

    //Configure it
    plnr->setRange(steerEta);
    plnr->setGoalBias(goalBias);
    plnr->setKNearest(kNearest);
    plnr->setRewireFactor(rewireScale);
    plnr->setVariant(variant);
    plnr->setAlpha(1.0); //This shouldn't matter when the variant is 0.
    plnr->setInformedSampling(informed);
    plnr->setSampleRejection(reject);
    //plnr->setNumSamplingAttempts(0u)
    plnr->setUpdateChildren(true);

//    std::cout << "RRT# - Informed Sampling: " << (plnr->getInformedSampling() ? "yes" : "no") << std::endl;
//    std::cout << "RRT# - Rejection Sampling: " << (plnr->getSampleRejection() ? "yes" : "no") << std::endl;
//    std::cout << "RRT# - Update Children: " << (plnr->getUpdateChildren() ? "yes" : "no") << std::endl;
//    std::cout << "RRT# - Num. Sampling Attempts: " << plnr->getNumSamplingAttempts()<< std::endl;
//    std::cout << "RRT# - Variant: " << plnr->getVariant()<< std::endl;
//    std::cout << "RRT# - Alpha: " << plnr->getAlpha()<< std::endl;
//    std::cout << "RRT# - Epsilon: " << plnr->getEpsilon()<< std::endl;


    std::stringstream plannerName;

    if (plnr->getInformedSampling() == true)
    {
        plannerName << "Informed_";
    }

    if (plnr->getSampleRejection())
    {
        plannerName << "Reject_";
    }

    plannerName << "RRTsharp";

    if (variant != 0)
    {
        plannerName << variant;
    }

    plnr->setName(plannerName.str());

    //Return
    return plnr;
}

std::shared_ptr<ompl::geometric::InformedRRTstar> allocateInformedRrtStar(const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias, const bool kNearest, const double rewireScale, const double pruneFraction)
{
    //Create an RRT* planner
    std::shared_ptr<ompl::geometric::InformedRRTstar> plnr  = std::make_shared<ompl::geometric::InformedRRTstar>(si);

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

std::shared_ptr<ompl::geometric::SORRTstar> allocateSorrtStar(const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias, const bool kNearest, const double rewireScale, const double pruneFraction, const unsigned int numSamples)
{
    //Create an RRT* planner
    std::shared_ptr<ompl::geometric::SORRTstar> plnr  = std::make_shared<ompl::geometric::SORRTstar>(si);

    //Configure it to be Informed
    plnr->setOrderedSampling(true);
    plnr->setBatchSize(numSamples);
    plnr->setRange(steerEta);
    plnr->setGoalBias(goalBias);
    plnr->setKNearest(kNearest);
    plnr->setRewireFactor(rewireScale);
    plnr->setPruneThreshold(pruneFraction);
    plnr->setDelayCC(true);

    std::stringstream plannerName;

    plannerName << "SORRTstar" << numSamples;

    plnr->setName(plannerName.str());

    //Return
    return plnr;
}

std::shared_ptr<ompl::geometric::FMT> allocateFmtStar(const ompl::base::SpaceInformationPtr &si, const bool kNearest, const double rewireScale, const unsigned int numSamples, const bool cacheCC,  const bool useHeuristics)
{
    //Create a RRT* planner
    std::shared_ptr<ompl::geometric::FMT> plnr = std::make_shared<ompl::geometric::FMT>(si);

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

std::shared_ptr<ompl::geometric::BITstar> allocateBitStar(const ompl::base::SpaceInformationPtr &si, const bool kNearest, const double rewireScale, const unsigned int numSamples, const double pruneFraction, const bool strictQueue, const bool delayRewire, const bool jit, const bool refreshBatches, const double weight /*= 1.0*/, const bool adapt /*= false*/)
{
    //Create a BIT* planner
    std::shared_ptr<ompl::geometric::BITstar> plnr = std::make_shared<ompl::geometric::BITstar>(si);

    //Configure it
    plnr->setUseKNearest(kNearest);
    plnr->setRewireFactor(rewireScale);
    plnr->setSamplesPerBatch(numSamples);
    plnr->setPruning(true);
    plnr->setPruneThresholdFraction(pruneFraction);
    plnr->setStrictQueueOrdering(strictQueue);
    plnr->setDelayRewiringUntilInitialSolution(delayRewire);
    plnr->setJustInTimeSampling(jit);
    plnr->setDropSamplesOnPrune(refreshBatches);
    plnr->setStopOnSolnImprovement(false);
    plnr->setConsiderApproximateSolutions(false);
    #warning "ABIT* support disabled here"
    //plnr->setSearchWeight(weight);
    //plnr->setAdaptWeight(adapt);

    if (weight != 1 || adapt == true)
    {
        throw ompl::Exception("ABIT* has been disabled in this code.");
    }

    std::stringstream plannerName;
    if (delayRewire == true)
    {
        plannerName << "d";
    }
    if (jit == true)
    {
        plannerName << "j";
    }
    if (refreshBatches == true)
    {
        plannerName << "x";
    }
    if (kNearest == true)
    {
        plannerName << "k";
    }
    if (strictQueue == false)
    {
        plannerName << "l";
    }
    if (weight > 1.0)
    {
        plannerName << "A";
    }
    plannerName << "BITstar" << numSamples;
    plnr->setName(plannerName.str());

    //Return
    return plnr;
}

#ifdef BITSTAR_REGRESSION
std::shared_ptr<ompl::geometric::BITstarRegression> allocateBitStarRegression(const ompl::base::SpaceInformationPtr &si, const bool kNearest, const double rewireScale, const unsigned int numSamples, const double pruneFraction, const bool strictQueue, const bool delayRewire, const bool jit, const bool refreshBatches, const double weight /*= 1.0*/, const bool adapt /*= false*/)
{
    //Create a BIT* planner
    std::shared_ptr<ompl::geometric::BITstarRegression> plnr = std::make_shared<ompl::geometric::BITstarRegression>(si);

    //Configure it
    plnr->setUseKNearest(kNearest);
    plnr->setRewireFactor(rewireScale);
    plnr->setSamplesPerBatch(numSamples);
    plnr->setPruning(true);
    plnr->setPruneThresholdFraction(pruneFraction);
    plnr->setStrictQueueOrdering(strictQueue);
    plnr->setDelayRewiringUntilInitialSolution(delayRewire);
    plnr->setJustInTimeSampling(jit);
    plnr->setDropSamplesOnPrune(refreshBatches);
    plnr->setStopOnSolnImprovement(false);
    plnr->setConsiderApproximateSolutions(false);
    #warning "ABIT* support disabled here"
    //plnr->setSearchWeight(weight);
    //plnr->setAdaptWeight(adapt);

    if (weight != 1 || adapt == true)
    {
        throw ompl::Exception("ABIT* has been disabled in this code.");
    }

    std::stringstream plannerName;
    if (delayRewire == true)
    {
        plannerName << "d";
    }
    if (jit == true)
    {
        plannerName << "j";
    }
    if (refreshBatches == true)
    {
        plannerName << "x";
    }
    if (kNearest == true)
    {
        plannerName << "k";
    }
    if (strictQueue == false)
    {
        plannerName << "l";
    }
    if (weight > 1.0)
    {
        plannerName << "A";
    }
    plannerName << "BITstarRegression" << numSamples;
    plnr->setName(plannerName.str());

    //Return
    return plnr;
}
#endif  // BITSTAR_REGRESSION

bool isRrt(PlannerType plnrType)
{
    return (plnrType == PLANNER_RRT ||
            plnrType == PLANNER_RRTCONNECT);
}

bool isRrtStar(PlannerType plnrType)
{
    return (plnrType == PLANNER_RRTSTAR ||
            plnrType == PLANNER_RRTSTAR_INFORMED ||
            plnrType == PLANNER_SORRTSTAR ||
            plnrType == PLANNER_RRTSTAR_NEW_REJECT ||
            plnrType == PLANNER_RRTSTAR_PRUNE ||
            plnrType == PLANNER_RRTSTAR_SAMPLE_REJECT ||
            plnrType == PLANNER_RRTSTAR_SEED ||
            plnrType == PLANNER_RRTSTAR_TRIO);
}

bool isRrtSharp(PlannerType plnrType)
{
    return (plnrType == PLANNER_RRTSHARP ||
            plnrType == PLANNER_RRTSHARP_INFORMED);
}

bool isBitStar(PlannerType plnrType)
{
    return (plnrType == PLANNER_BITSTAR ||
            plnrType == PLANNER_ABITSTAR ||
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
        case PLANNER_RRTSHARP:
        {
            return "RRTsharp";
            break;
        }
        case PLANNER_RRTSHARP_INFORMED:
        {
            return "Informed_RRTsharp";
            break;
        }
        case PLANNER_RRTSTAR_INFORMED:
        {
            return "Informed_RRTstar";
            break;
        }
        case PLANNER_SORRTSTAR:
        {
            return "SORRTstar";
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
        case PLANNER_REGRESSION_BITSTAR:
        {
            return "BITstarRegress";
            break;
        }
        case PLANNER_ABITSTAR:
        {
            return "ABITstar";
            break;
        }
        default:
        {
            throw ompl::Exception("Unimplemented PlannerType in plannerName()");
        }
    }

    return std::string();
}
