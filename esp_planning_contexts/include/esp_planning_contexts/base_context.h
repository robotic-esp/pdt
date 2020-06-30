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
#include <string>

#include <ompl/base/Goal.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>

#include "esp_configuration/configuration.h"
#include "esp_planning_contexts/context_visitor.h"
#include "esp_time/time.h"

namespace esp {

namespace ompltools {

/** \brief The base class for an experiment */
class BaseContext {
 public:
  BaseContext(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
              const std::shared_ptr<const Configuration>& config, const std::string& name);

  /** \brief Returns the name of this context. */
  std::string getName() const;

  /** \brief Returns the space information. */
  std::shared_ptr<ompl::base::SpaceInformation> getSpaceInformation() const;

  /** \brief Returns the state space. */
  std::shared_ptr<ompl::base::StateSpace> getStateSpace() const;

  /** \brief Returns the dimension of the state space. */
  std::size_t getDimension() const;

  /** \brief Returns the optimization objective of this context. */
  ompl::base::OptimizationObjectivePtr getObjective() const;

  std::shared_ptr<ompl::base::Goal> getGoal() const;

  /** \brief Returns the maximum duration to solve this context. */
  time::Duration getMaxSolveDuration() const;

  /** \brief Return a newly generated problem definition */
  virtual std::shared_ptr<ompl::base::ProblemDefinition> instantiateNewProblemDefinition()
      const = 0;

  /** \brief Accepts a context visitor. */
  virtual void accept(const ContextVisitor& visitor) const = 0;

 protected:
  /** \brief The space information associated with this context. */
  ompl::base::SpaceInformationPtr spaceInfo_{};

  /** \brief The dimension of the state space. */
  std::size_t dimensionality_;

  /** \brief The name of the context. */
  std::string name_{};

  /** \brief The optimization objective. */
  ompl::base::OptimizationObjectivePtr objective_{};

  /** \brief The goal specification of the planning problem. */
  std::shared_ptr<ompl::base::Goal> goal_{};

  /** \brief The maximum duration to solve this context. */
  time::Duration maxSolveDuration_{};

  /** \brief The configuration. */
  const std::shared_ptr<const Configuration> config_;
};

}  // namespace ompltools

}  // namespace esp
