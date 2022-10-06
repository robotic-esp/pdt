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

#include <memory>

#include <boost/smart_ptr.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Woverflow"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <openrave-core.h>
#pragma GCC diagnostic pop

#include "pdt/config/configuration.h"
#include "pdt/obstacles/base_obstacle.h"
#include "pdt/obstacles/hyperrectangle.h"
#include "pdt/obstacles/obstacle_visitor.h"

namespace pdt {

namespace open_rave {

class OpenRaveBaseValidityChecker : public ompl::base::StateValidityChecker {
 public:
  /** \brief The constructor. */
  OpenRaveBaseValidityChecker(const ompl::base::SpaceInformationPtr& spaceInfo,
                              const OpenRAVE::EnvironmentBasePtr& environment,
                              const OpenRAVE::RobotBasePtr& robot,
                              const std::shared_ptr<const config::Configuration>& config);

  /** \brief The destructor. */
  virtual ~OpenRaveBaseValidityChecker() = default;

  /** \brief Check if a state is valid. */
  virtual bool isValid(const ompl::base::State* state) const override = 0;

  /** \brief Returns a pointer to the rave environment. */
  virtual OpenRAVE::EnvironmentBasePtr getOpenRaveEnvironment() const;

 protected:
  /** \brief The rave environment. */
  OpenRAVE::EnvironmentBasePtr environment_;

  /** \brief The rave robot. */
  OpenRAVE::RobotBasePtr robot_;

  /** \brief The state space we are checking states of. */
  const std::shared_ptr<const ompl::base::StateSpace> stateSpace_;

  /** \brief The collision check report to store the clearance of a state. */
  mutable boost::shared_ptr<OpenRAVE::CollisionReport> collisionReport_;

  /** \brief The configuration. */
  const std::shared_ptr<const config::Configuration> config_;
};

}  // namespace open_rave

}  // namespace pdt
