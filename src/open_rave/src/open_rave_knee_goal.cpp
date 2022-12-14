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

#include "pdt/open_rave/open_rave_knee_goal.h"

#include <vector>

namespace pdt {

namespace open_rave {

OpenRaveKneeGoal::OpenRaveKneeGoal(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                                   const std::shared_ptr<const config::Configuration>& config,
                                   const std::string& name) :
    GoalSpace(spaceInfo),
    goalState_(spaceInfo) {
  auto goalPosition = config->get<std::vector<double>>("context/" + name + "/validGoal");
  if (goalPosition.size() != 7u) {
    throw std::runtime_error("Valid goal must be a vector of the form [ x y z qx qy qz qw ].");
  }
  goalState_.get()->setXYZ(goalPosition[0u], goalPosition[1u], goalPosition[2u]);
  goalState_.get()->rotation().x = goalPosition[3u];
  goalState_.get()->rotation().y = goalPosition[4u];
  goalState_.get()->rotation().z = goalPosition[5u];
  goalState_.get()->rotation().w = goalPosition[6u];
}

void OpenRaveKneeGoal::sampleGoal(ompl::base::State* st) const {
  if (numSampledGoals_ == 0u) {
    si_->copyState(st, goalState_.get());
  } else {
    GoalSpace::sampleGoal(st);
  }
  ++numSampledGoals_;
}

}  // namespace open_rave

}  // namespace pdt
