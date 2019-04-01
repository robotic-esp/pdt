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

// Authors: Marlin Strub

// I really don't like having to do this, but I don't see any way around it. The main issue is that
// the planner base class does not provide access to the current best cost. It does for some
// planners (through the progressProperties) but the planner implementations are free to choose the
// corresponding property name themselves and of course its not the same for all planners (e.g.,
// "best cost REAL" vs "best cost DOUBLE"). Similarly, most anytime planners provide direct access
// to their current best cost, but the corresponding function is not named (e.g., getBestCost() in
// LBTRRT and bestCost() in RRT). On top of that the planner base class does not have a type
// property (only a name). But I don't feel comfortable comparing strings all the time, especially
// when logging the best costs.

#pragma once

#include "nlohmann/json.hpp"

namespace esp {

namespace ompltools {

enum class CONTEXT_TYPE {
  CENTRE_SQUARE,
  DIVIDING_WALLS,
  DOUBLE_ENCLOSURE,
  FLANKING_GAP,
  GOAL_ENCLOSURE,
  MULTI_START_GOAL,
  OBSTACLE_FREE,
  RANDOM_RECTANGLES,
  REGULAR_RECTANGLES,
  SPIRAL,
  START_ENCLOSURE,
  WALL_GAP,
};

NLOHMANN_JSON_SERIALIZE_ENUM(CONTEXT_TYPE,
                             {
                                 {CONTEXT_TYPE::CENTRE_SQUARE, "CentreSquare"},
                                 {CONTEXT_TYPE::DIVIDING_WALLS, "DividingWalls"},
                                 {CONTEXT_TYPE::DOUBLE_ENCLOSURE, "DoubleEnclosure"},
                                 {CONTEXT_TYPE::FLANKING_GAP, "FlankingGap"},
                                 {CONTEXT_TYPE::GOAL_ENCLOSURE, "GoalEnclosure"},
                                 {CONTEXT_TYPE::MULTI_START_GOAL, "MultiStartGoal"},
                                 {CONTEXT_TYPE::OBSTACLE_FREE, "ObstacleFree"},
                                 {CONTEXT_TYPE::RANDOM_RECTANGLES, "RandomRectangles"},
                                 {CONTEXT_TYPE::REGULAR_RECTANGLES, "RegularRectangles"},
                                 {CONTEXT_TYPE::SPIRAL, "Spiral"},
                                 {CONTEXT_TYPE::START_ENCLOSURE, "StartEnclosure"},
                                 {CONTEXT_TYPE::WALL_GAP, "WallGap"},
                             })

}  // namespace ompltools

}  // namespace esp
