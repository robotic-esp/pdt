#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>

// Compile hooks for a planner called BITstarRegression:
//#define BITSTAR_REGRESSION

/*
How to make a copy of BIT* for regression testing:
cd ompl/geometric/planners/
cp -r bitstar bitstar_regression
cd bitstar_regression
mv BITstar.h BITstarRegression.h
mv src/BITstar.cpp src/BITstarRegression.cpp
sed -i.bak -e 's|_BITSTAR|_BITSTARREGRESSION|g' -e 's|/bitstar|/bitstar_regression|g' -e 's|BITstar|BITstarRegression|g' BITstarRegression.h
sed -i.bak -e 's|_BITSTAR|_BITSTARREGRESSION|g' -e 's|/bitstar|/bitstar_regression|g' -e 's|BITstar|BITstarRegression|g' src/BITstarRegression.cpp
cd datastructures
sed -i.bak -e 's|_BITSTAR|_BITSTARREGRESSION|g' -e 's|/bitstar|/bitstar_regression|g' -e 's|BITstar|BITstarRegression|g' *.h
cd src
sed -i.bak -e 's|_BITSTAR|_BITSTARREGRESSION|g' -e 's|/bitstar|/bitstar_regression|g' -e 's|BITstar|BITstarRegression|g' *.cpp
*/

#ifdef BITSTAR_REGRESSION
#include <ompl/geometric/planners/bitstar_regression/BITstarRegression.h>
#endif  // BITSTAR_REGRESSION


enum PlannerType
{
    PLANNER_RRT,
    PLANNER_RRTCONNECT,
    PLANNER_RRTSTAR,
    PLANNER_RRTSTAR_INFORMED,
    PLANNER_RRTSHARP,
    PLANNER_RRTSHARP_INFORMED,
    PLANNER_SORRTSTAR,
    PLANNER_RRTSTAR_PRUNE,
    PLANNER_RRTSTAR_NEW_REJECT,
    PLANNER_RRTSTAR_SAMPLE_REJECT,
    PLANNER_RRTSTAR_TRIO,
    PLANNER_FMTSTAR,
    PLANNER_BITSTAR,
    PLANNER_ABITSTAR,
    PLANNER_HYBRID_BITSTAR,
    PLANNER_DUALTREE_BITSTAR,
    PLANNER_RRTSTAR_SEED,
    PLANNER_BITSTAR_SEED,
    PLANNER_REGRESSION_BITSTAR,
};

/** \brief Allocation function for RRT */
std::shared_ptr<ompl::geometric::RRT> allocateRrt(const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias);

/** \brief Allocation function for RRTConnect */
std::shared_ptr<ompl::geometric::RRTConnect> allocateRrtConnect(const ompl::base::SpaceInformationPtr &si, const double steerEta);

/** \brief Allocation function for RRT* */
std::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar(const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias, const bool kNearest, const double rewireScale);

/** \brief Allocation function for RRT# */
std::shared_ptr<ompl::geometric::RRTsharp> allocateRrtSharp(const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias, const bool kNearest, const double rewireScale, const bool reject, const bool informed, const unsigned int variant);

/** \brief Allocation function for Informed RRT* */
std::shared_ptr<ompl::geometric::InformedRRTstar> allocateInformedRrtStar(const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias, const bool kNearest, const double rewireScale, const double pruneFraction);

/** \brief Allocation function for SORRT* */
std::shared_ptr<ompl::geometric::SORRTstar> allocateSorrtStar(const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias, const bool kNearest, const double rewireScale, const double pruneFraction, const unsigned int numSamples);

/** \brief Allocation function for FMT* */
std::shared_ptr<ompl::geometric::FMT> allocateFmtStar(const ompl::base::SpaceInformationPtr &si, const bool kNearest, const double rewireScale, const unsigned int numSamples, const bool cacheCC,  const bool useHeuristics);

/** \brief Allocation function for BIT* */
std::shared_ptr<ompl::geometric::BITstar> allocateBitStar(const ompl::base::SpaceInformationPtr &si, const bool kNearest, const double rewireScale, const unsigned int numSamples, const double pruneFraction, const bool strictQueue, const bool delayRewire, const bool jit, const bool refreshBatches, const double weight = 1.0, const bool adapt = false);

#ifdef BITSTAR_REGRESSION
/** \brief Allocation function for a regression testing BIT* */
std::shared_ptr<ompl::geometric::BITstarRegression> allocateBitStarRegression(const ompl::base::SpaceInformationPtr &si, const bool kNearest, const double rewireScale, const unsigned int numSamples, const double pruneFraction, const bool strictQueue, const bool delayRewire, const bool jit, const bool refreshBatches, const double weight = 1.0, const bool adapt = false);
#endif  // BITSTAR_REGRESSION

/** \brief Return true if the planner is any of the RRT types */
bool isRrt(PlannerType plnrType);

/** \brief Return true if the planner is any of the RRTstar types */
bool isRrtStar(PlannerType plnrType);

/** \brief Return true if the planner is any of the RRTsharp types */
bool isRrtSharp(PlannerType plnrType);

/** \brief Return true if the planner is any of the BITstar types */
bool isBitStar(PlannerType plnrType);

/** \brief Convert a planner enum into a planner name */
std::string plannerName(PlannerType plnrType);
