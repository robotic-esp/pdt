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

#include "pdt/open_rave/open_rave_base_context.h"

#include <algorithm>

#include <boost/smart_ptr.hpp>

#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <openrave/environment.h>

#include "pdt/open_rave/open_rave_manipulator_validity_checker.h"

using namespace std::string_literals;

namespace pdt {

namespace open_rave {

OpenRaveBaseContext::OpenRaveBaseContext(
    const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
    const std::shared_ptr<const config::Configuration>& config, const std::string& name) :
    BaseContext(spaceInfo, config, name) {
}

std::vector<std::shared_ptr<obstacles::BaseObstacle>> OpenRaveBaseContext::getObstacles() const {
  return {};
}

std::vector<std::shared_ptr<obstacles::BaseAntiObstacle>> OpenRaveBaseContext::getAntiObstacles()
    const {
  return {};
}

std::shared_ptr<ompl::base::Goal> OpenRaveBaseContext::createGoal() const {
  return std::make_shared<ompl::base::GoalState>(spaceInfo_);
}

}  // namespace open_rave

}  // namespace pdt
