#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>

enum PlannerType
{
    PLANNER_RRT,
    PLANNER_RRTCONNECT,
    PLANNER_RRTSTAR,
    PLANNER_RRTSTAR_INFORMED,
    PLANNER_SORRTSTAR,
    PLANNER_RRTSTAR_PRUNE,
    PLANNER_RRTSTAR_NEW_REJECT,
    PLANNER_RRTSTAR_SAMPLE_REJECT,
    PLANNER_RRTSTAR_TRIO,
    PLANNER_FMTSTAR,
    PLANNER_BITSTAR,
    PLANNER_HYBRID_BITSTAR,
    PLANNER_DUALTREE_BITSTAR,
    PLANNER_RRTSTAR_SEED,
    PLANNER_BITSTAR_SEED
};

/** \brief Allocation function for RRT */
boost::shared_ptr<ompl::geometric::RRT> allocateRrt(const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias);

/** \brief Allocation function for RRTConnect */
boost::shared_ptr<ompl::geometric::RRTConnect> allocateRrtConnect(const ompl::base::SpaceInformationPtr &si, const double steerEta);

/** \brief Allocation function for RRT* */
boost::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar(const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias, const bool kNearest, const double rewireScale);

/** \brief Allocation function for Informed RRT* */
boost::shared_ptr<ompl::geometric::InformedRRTstar> allocateInformedRrtStar(const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias, const bool kNearest, const double rewireScale, const double pruneFraction);

/** \brief Allocation function for SORRT* */
boost::shared_ptr<ompl::geometric::SORRTstar> allocateSorrtStar(const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias, const bool kNearest, const double rewireScale, const double pruneFraction, const unsigned int numSamples);

/** \brief Allocation function for FMT* */
boost::shared_ptr<ompl::geometric::FMT> allocateFmtStar(const ompl::base::SpaceInformationPtr &si, const bool kNearest, const double rewireScale, const unsigned int numSamples, const bool cacheCC,  const bool useHeuristics);

/** \brief Allocation function for BIT* */
boost::shared_ptr<ompl::geometric::BITstar> allocateBitStar(const ompl::base::SpaceInformationPtr &si, const bool kNearest, const double rewireScale, const unsigned int numSamples, const double pruneFraction, const bool strictQueue, const bool delayRewire, const bool jit, const bool refreshBatches);

/** \brief Return true if the planner is any of the RRT types */
bool isRrt(PlannerType plnrType);

/** \brief Return true if the planner is any of the RRTstar types */
bool isRrtStar(PlannerType plnrType);

/** \brief Return true if the planner is any of the BITstar types */
bool isBitStar(PlannerType plnrType);

/** \brief Convert a planner enum into a planner name */
std::string plannerName(PlannerType plnrType);
