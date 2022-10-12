/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014--2022
 *  Estimation, Search, and Planning (ESP) Research Group
 *  All rights reserved
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
 *   * Neither the names of the organizations nor the names of its
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

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "nlohmann/json.hpp"
#pragma GCC diagnostic pop

namespace pdt {

namespace common {

enum class CONTEXT_TYPE {
  CENTER_SQUARE,
  DIVIDING_WALLS,
  DOUBLE_ENCLOSURE,
  FLANKING_GAP,
  FOUR_ROOMS,
  GOAL_ENCLOSURE,
  NARROW_PASSAGE,
  OBSTACLE_FREE,
  OPEN_RAVE_MANIPULATOR,
  OPEN_RAVE_R3,
  OPEN_RAVE_R3XSO2,
  OPEN_RAVE_SE3,
  OPEN_RAVE_CONSTRAINED_SE3,
  RANDOM_RECTANGLES,
  RANDOM_RECTANGLES_MULTI_START_GOAL,
  REEDS_SHEPP_RANDOM_RECTANGLES,
  REPEATING_RECTANGLES,
  SPIRAL,
  START_ENCLOSURE,
  WALL_GAP,
};

NLOHMANN_JSON_SERIALIZE_ENUM(
    CONTEXT_TYPE, {
                      {CONTEXT_TYPE::CENTER_SQUARE, "CenterSquare"},
                      {CONTEXT_TYPE::DIVIDING_WALLS, "DividingWalls"},
                      {CONTEXT_TYPE::DOUBLE_ENCLOSURE, "DoubleEnclosure"},
                      {CONTEXT_TYPE::FLANKING_GAP, "FlankingGap"},
                      {CONTEXT_TYPE::FOUR_ROOMS, "FourRooms"},
                      {CONTEXT_TYPE::GOAL_ENCLOSURE, "GoalEnclosure"},
                      {CONTEXT_TYPE::NARROW_PASSAGE, "NarrowPassage"},
                      {CONTEXT_TYPE::OBSTACLE_FREE, "ObstacleFree"},
                      {CONTEXT_TYPE::OPEN_RAVE_MANIPULATOR, "OpenRaveManipulator"},
                      {CONTEXT_TYPE::OPEN_RAVE_R3, "OpenRaveR3"},
                      {CONTEXT_TYPE::OPEN_RAVE_R3XSO2, "OpenRaveR3xSO2"},
                      {CONTEXT_TYPE::OPEN_RAVE_SE3, "OpenRaveSE3"},
                      {CONTEXT_TYPE::OPEN_RAVE_CONSTRAINED_SE3, "OpenRaveConstrainedSE3"},
                      {CONTEXT_TYPE::RANDOM_RECTANGLES, "RandomRectangles"},
                      {CONTEXT_TYPE::RANDOM_RECTANGLES_MULTI_START_GOAL, "MultiStartGoal"},
                      {CONTEXT_TYPE::REEDS_SHEPP_RANDOM_RECTANGLES, "ReedsSheppRandomRectangles"},
                      {CONTEXT_TYPE::REPEATING_RECTANGLES, "RepeatingRectangles"},
                      {CONTEXT_TYPE::SPIRAL, "Spiral"},
                      {CONTEXT_TYPE::START_ENCLOSURE, "StartEnclosure"},
                      {CONTEXT_TYPE::WALL_GAP, "WallGap"},
                  })

}  // namespace common

}  // namespace pdt
