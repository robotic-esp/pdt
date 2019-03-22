#pragma once

#include <ompl/base/State.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerTerminationCondition.h>

#include "esp_planning_contexts/all_contexts.h"
#include "esp_utilities/planner_tools.h"
#include "esp_utilities/recording_tools.h"
#include "esp_utilities/general_tools.h"

// asrl::time::duration == std::chrono::duration
/** \brief Create the matlab plot command for a vertex */
std::string plotVertex(const ompl::base::State* vertex, std::string vertexColour,
                       std::string vertexSize);

/** \brief Create the matlab plot command for an edge*/
std::string plotEdge(const ompl::base::State* vertex, const ompl::base::State* parent,
                     std::string edgeColour, std::string lineStyle, std::string edgeWeight);

/** \brief A helper function to create the filledCircle.m file */
void createMatlabHelpers(std::string path);

/** \brief Write the matlab plot header */
std::string matlabExtraHeader(std::string plannerName, PlannerType plannerType, bool plotVertices,
                              bool plotIndices, bool informedWorldEllipse, bool bitStarQueueEllipse,
                              bool bitStarNextEdge, bool bitStarFullQueue);

/** \brief Create the matlab plot commands for a problem */
void writeMatlabMap(BaseContextPtr experiment, PlannerType plannerType,
                    ompl::base::PlannerPtr planner, unsigned int worldSeed, double runtime,
                    bool plotVertices, bool plotIndices, bool informedWorldEllipse,
                    bool bitStarQueueEllipse, bool bitStarNextEdge, bool bitStarFullQueue,
                    std::string path = "plots/", std::string postFix = std::string(),
                    bool monochrome = false);

/** \brief Create an iteration-by-iteration animation of the planner */
asrl::time::duration createAnimation(BaseContextPtr experiment, PlannerType plannerType,
                                     ompl::base::PlannerPtr planner, unsigned int worldSeed,
                                     asrl::time::duration timeToRun, bool plotVertices,
                                     bool plotIndices, bool informedWorldEllipse,
                                     bool bitStarEllipse, bool bitStarEdge, bool bitStarQueue,
                                     unsigned int initialIterNumber = 0u, bool monochrome = false);
