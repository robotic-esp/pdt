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

// Authors: Jonathan Gammell

#pragma once

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/State.h>

#include "esp_planning_contexts/all_contexts.h"
#include "esp_utilities/general_tools.h"
#include "esp_utilities/planner_tools.h"
#include "esp_utilities/recording_tools.h"

namespace esp {

namespace ompltools {

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

}  // namespace ompltools

}  // namespace esp
