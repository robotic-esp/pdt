//The states
#include "ompl/base/State.h"
//The planners:
#include "ompl/base/Planner.h"
//The planner termination conditions
#include "ompl/base/PlannerTerminationCondition.h"

//The experiments:
#include "ExperimentDefinitions.h"

//The planner tools
#include <tools/planner_tools.h>

//The data recording
#include "tools/recording_tools.h"

//Some general time helpers
#include "tools/general_tools.h"

//asrl::time::duration == std::chrono::duration
/** \brief Create the matlab plot command for a vertex */
std::string plotVertex(const ompl::base::State* vertex, std::string vertexColour, std::string vertexSize);

/** \brief Create the matlab plot command for an edge*/
std::string plotEdge(const ompl::base::State* vertex, const ompl::base::State* parent, std::string edgeColour, std::string lineStyle, std::string edgeWeight);

/** \brief A helper function to create the filledCircle.m file */
void createMatlabHelpers(std::string path);

/** \brief Write the matlab plot header */
std::string matlabExtraHeader(std::string plannerName, bool plotVertices, bool informedWorldEllipse, bool bitStarQueueEllipse, bool bitStarNextEdge, bool bitStarFullQueue);

/** \brief Create the matlab plot commands for a problem */
void writeMatlabMap(BaseExperimentPtr experiment, PlannerType plannerType, ompl::base::PlannerPtr planner, unsigned int worldSeed, bool plotVertices, bool informedWorldEllipse, bool bitStarQueueEllipse, bool bitStarNextEdge, bool bitStarFullQueue, std::string path = "plots/", std::string postFix = std::string(), bool monochrome = false);

/** \brief Create an iteration-by-iteration animation of the planner */
asrl::time::duration createAnimation(BaseExperimentPtr experiment, PlannerType plannerType, ompl::base::PlannerPtr planner, unsigned int worldSeed, asrl::time::duration timeToRun, bool plotVertices, bool informedWorldEllipse, bool bitStarEllipse, bool bitStarEdge, bool bitStarQueue, unsigned int initialIterNumber = 0u, bool monochrome = false);
