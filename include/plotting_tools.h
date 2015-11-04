
//For boost time
#include <boost/date_time/posix_time/posix_time.hpp>
//For boost tuple (pre C++11 tuple)
#include <boost/tuple/tuple.hpp>

//The states
#include "ompl/base/State.h"
//The planners:
#include "ompl/base/Planner.h"
//The planner termination conditions
#include "ompl/base/PlannerTerminationCondition.h"

//The experiments:
#include "ExperimentDefinitions.h"

//ompl::time::duration == boost::posix_time::time_duration

enum PlannerType
{
    PLANNER_RRT,
    PLANNER_RRTCONNECT,
    PLANNER_RRTSTAR,
    PLANNER_RRTSTAR_INFORMED,
    PLANNER_RRTSTAR_PRUNE,
    PLANNER_RRTSTAR_NEW_REJECT,
    PLANNER_RRTSTAR_SAMPLE_REJECT,
    PLANNER_RRTSTAR_TRIO,
    PLANNER_FMT,
    PLANNER_BITSTAR,
    PLANNER_HYBRID_BITSTAR,
    PLANNER_DUALTREE_BITSTAR,
    PLANNER_RRTSTAR_SEED,
    PLANNER_BITSTAR_SEED
};

/** \brief Return true if the planner is any of the RRTstar types */
bool isRrtStar(PlannerType plnrType);
/** \brief Return true if the planner is any of the BITstar types */
bool isBitStar(PlannerType plnrType);

std::string plannerName(PlannerType plnrType);

/** \brief Create the matlab plot command for a vertex */
std::string plotVertex(const ompl::base::State* vertex, std::string vertexColour, std::string vertexSize);

/** \brief Create the matlab plot command for an edge*/
std::string plotEdge(const ompl::base::State* vertex, const ompl::base::State* parent, std::string edgeColour, std::string lineStyle, std::string edgeWeight);

/** \brief A helper function to create directories using boost filesystem */
void createDirectories(std::string fileName);

/** \brief A helper function to create the filledCircle.m file */
void createMatlabHelpers(std::string path);

/** \brief Write the matlab plot header */
std::string matlabExtraHeader(std::string plannerName, bool informedWorldEllipse, bool bitStarQueueEllipse, bool bitStarNextEdge, bool bitStarFullQueue);

/** \brief Create the matlab plot commands for a problem */
void writeMatlabMap(BaseExperimentPtr experiment, PlannerType plannerType, ompl::base::PlannerPtr planner, unsigned int worldSeed, bool informedWorldEllipse, bool bitStarQueueEllipse, bool bitStarNextEdge, bool bitStarFullQueue, std::string path = "plots/", std::string postFix = std::string(), bool monochrome = false);

/** \brief Create an iteration-by-iteration animation of the planner */
ompl::time::duration createAnimation(BaseExperimentPtr experiment, PlannerType plannerType, ompl::base::PlannerPtr planner, unsigned int worldSeed, ompl::time::duration timeToRun, bool informedWorldEllipse, bool bitStarEllipse, bool bitStarEdge, bool bitStarQueue, unsigned int initialIterNumber = 0u, bool monochrome = false);



typedef std::pair<boost::posix_time::time_duration, double> progress_pair_t;
typedef boost::tuple<boost::posix_time::time_duration, unsigned int, double> progress_tuple_t;
typedef std::vector<progress_pair_t> progress_pair_vector_t;
typedef std::vector<progress_tuple_t> progress_tuple_vector_t;
/** \brief A class to write planner results to file as they run. */
class ProgressFile
{
    public:
        ProgressFile(const std::string& fullFileName, bool storeTuple);

        void addResult(const std::string& plannerName, const progress_pair_vector_t& progress);
        void addResult(const std::string& plannerName, const progress_tuple_vector_t& progress);

    private:
        std::string filename_;
        bool storeTuple_;
};


typedef std::pair<double, boost::posix_time::time_duration> target_pair_t;
typedef std::vector<target_pair_t> target_pair_vector_t;
class TargetFile
{
    public:

        TargetFile(const std::string& fullFileName);

        void addResult(const std::string& plannerName, const target_pair_vector_t& progress);

    private:
        std::string filename_;
};

typedef std::pair<unsigned int, double> iteration_pair_t;
typedef std::vector<iteration_pair_t> iteration_pair_vector_t;
class IterationFile
{
    public:

        IterationFile(const std::string& fullFileName);

        void addResult(const std::string& plannerName, const iteration_pair_vector_t& progress);

    private:
        std::string filename_;
};
