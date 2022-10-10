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

#include <ompl/base/GoalTypes.h>

#include "nlohmann/json.hpp"

namespace pdt {

namespace common {

// clang-format off
NLOHMANN_JSON_SERIALIZE_ENUM(ompl::base::GoalType,
                             {
                                 {ompl::base::GoalType::GOAL_ANY, "GoalAny"},
                                 {ompl::base::GoalType::GOAL_REGION, "GoalRegion"},
                                 {ompl::base::GoalType::GOAL_SAMPLEABLE_REGION, "GoalSampleableRegion"},
                                 {ompl::base::GoalType::GOAL_STATE, "GoalState"},
                                 {ompl::base::GoalType::GOAL_STATES, "GoalStates"},
                                 {ompl::base::GoalType::GOAL_LAZY_SAMPLES, "GoalLazySamples"},
#ifdef PDT_EXTRA_GOAL_SPACE
                                 {ompl::base::GoalType::GOAL_SPACE, "GoalSpace"},
#endif // #ifdef PDT_EXTRA_GOAL_SPACE
                             });
// clang-format on

}  // namespace common

}  // namespace pdt
