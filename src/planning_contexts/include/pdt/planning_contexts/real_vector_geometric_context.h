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

// Authors: Jonathan Gammell, Marlin Strub

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <ompl/base/Cost.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/util/Console.h>
#include <ompl/util/Exception.h>

#include "pdt/config/configuration.h"
#include "pdt/obstacles/base_obstacle.h"
#include "pdt/planning_contexts/base_context.h"
#include "pdt/planning_contexts/context_visitor.h"
#include "pdt/time/time.h"

namespace pdt {

namespace planning_contexts {

/** \brief The base class for an experiment */
class RealVectorGeometricContext : public BaseContext {
 public:
  RealVectorGeometricContext(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                             const std::shared_ptr<const config::Configuration>& config,
                             const std::string& name);

  // Use default virtual destructor.
  virtual ~RealVectorGeometricContext() = default;

  /** \brief Get the state-space limit */
  const ompl::base::RealVectorBounds& getBoundaries() const;

  // Get the obstacles.
  std::vector<std::shared_ptr<obstacles::BaseObstacle>> getObstacles() const override;

  // Get the antiobstacles.
  std::vector<std::shared_ptr<obstacles::BaseAntiObstacle>> getAntiObstacles() const override;

  // Accept a visitor.
  virtual void accept(const ContextVisitor& visitor) const override;

  /** \brief Create a new goal. */
  std::shared_ptr<ompl::base::Goal> createGoal() const override;

 protected:
  /** \brief The state space bounds. */
  ompl::base::RealVectorBounds bounds_;

  /** \brief The obstacles. */
  std::vector<std::shared_ptr<obstacles::BaseObstacle>> obstacles_{};

  /** \brief The anti obstacles. */
  std::vector<std::shared_ptr<obstacles::BaseAntiObstacle>> antiObstacles_{};
};

}  // namespace planning_contexts

}  // namespace pdt
