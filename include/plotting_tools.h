//The states
#include "ompl/base/State.h"
//The planners:
#include "ompl/base/Planner.h"
//The planner termination conditions
#include "ompl/base/PlannerTerminationCondition.h"

//The experiments:
#include "ExperimentDefinitions.h"

//The planner tools
#include "planner_tools.h"

//The data recording
#include "recording_tools.h"

//ompl::time::duration == boost::posix_time::time_duration
/** \brief Create the matlab plot command for a vertex */
std::string plotVertex(const ompl::base::State* vertex, std::string vertexColour, std::string vertexSize);

/** \brief Create the matlab plot command for an edge*/
std::string plotEdge(const ompl::base::State* vertex, const ompl::base::State* parent, std::string edgeColour, std::string lineStyle, std::string edgeWeight);

/** \brief A helper function to create the filledCircle.m file */
void createMatlabHelpers(std::string path);

/** \brief Write the matlab plot header */
std::string matlabExtraHeader(std::string plannerName, bool informedWorldEllipse, bool bitStarQueueEllipse, bool bitStarNextEdge, bool bitStarFullQueue);

/** \brief Create the matlab plot commands for a problem */
void writeMatlabMap(BaseExperimentPtr experiment, PlannerType plannerType, ompl::base::PlannerPtr planner, unsigned int worldSeed, bool informedWorldEllipse, bool bitStarQueueEllipse, bool bitStarNextEdge, bool bitStarFullQueue, std::string path = "plots/", std::string postFix = std::string(), bool monochrome = false);

/** \brief Create an iteration-by-iteration animation of the planner */
ompl::time::duration createAnimation(BaseExperimentPtr experiment, PlannerType plannerType, ompl::base::PlannerPtr planner, unsigned int worldSeed, ompl::time::duration timeToRun, bool informedWorldEllipse, bool bitStarEllipse, bool bitStarEdge, bool bitStarQueue, unsigned int initialIterNumber = 0u, bool monochrome = false);
