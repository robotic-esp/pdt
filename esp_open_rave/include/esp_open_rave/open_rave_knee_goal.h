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

#pragma once

#include <ompl/base/ScopedState.h>
#include <ompl/base/State.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include "esp_configuration/configuration.h"

namespace esp {

namespace ompltools {

/** \brief A planning context to plugin to the OpenRave simulator. */
class OpenRaveKneeGoal : public ompl::base::GoalSpace {
 public:
  OpenRaveKneeGoal(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                   const std::shared_ptr<const Configuration>& config, const std::string& name);
  virtual ~OpenRaveKneeGoal() = default;

  void sampleGoal(ompl::base::State* st) const override;

 private:
  /** \brief The goal state that is definitely valid. */
  ompl::base::ScopedState<ompl::base::SE3StateSpace> goalState_;

  /** \brief The number of sampled goals so far. */
  mutable unsigned int numSampledGoals_{0u};
};

}  // namespace ompltools

}  // namespace esp