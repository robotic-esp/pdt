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

#include <memory>

#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Woverflow"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <openrave-core.h>
#pragma GCC diagnostic pop

#include "esp_configuration/configuration.h"
#include "esp_open_rave/open_rave_base_context.h"
#include "esp_planning_contexts/base_context.h"
#include "esp_planning_contexts/context_visitor.h"

namespace esp {

namespace ompltools {

/** \brief A planning context to plugin to the OpenRave simulator. */
class OpenRaveManipulator : public OpenRaveBaseContext {
 public:
  OpenRaveManipulator(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                      const std::shared_ptr<const Configuration>& config, const std::string& name);
  virtual ~OpenRaveManipulator();

  /** \brief Instantiate a problem definition for this context. */
  virtual std::shared_ptr<ompl::base::ProblemDefinition> instantiateNewProblemDefinition()
      const override;

  /** \brief Return a copy of the start state. */
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> getStartState() const;

  /** \brief Return a copy of the goal state. */
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> getGoalState() const;

  /** \brief Accepts a context visitor. */
  virtual void accept(const ContextVisitor& visitor) const override final;

 private:
  /** \brief The start state. */
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> startState_;

  /** \brief The goal state. */
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goalState_;
};

}  // namespace ompltools

}  // namespace esp