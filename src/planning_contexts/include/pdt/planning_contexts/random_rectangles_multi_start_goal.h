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
#include <string>
#include <vector>

#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/RandomNumbers.h>

#include "pdt/config/configuration.h"
#include "pdt/planning_contexts/context_visitor.h"
#include "pdt/planning_contexts/real_vector_geometric_context.h"

namespace pdt {

namespace planning_contexts {

/** \brief An experiment with a singularly placed square obstacle*/
class RandomRectanglesMultiStartGoal : public RealVectorGeometricContext {
 public:
  /** \brief The constructor. */
  RandomRectanglesMultiStartGoal(const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo,
                                 const std::shared_ptr<const config::Configuration>& config,
                                 const std::string& name);

  /** \brief The destructor. */
  virtual ~RandomRectanglesMultiStartGoal() = default;

  /** \brief Accepts a context visitor. */
  virtual void accept(const ContextVisitor& visitor) const override;

 protected:
  /** \brief Return a start/goal pair. */
  virtual std::vector<StartGoalPair> makeStartGoalPair() const override;

  /** \brief Create the obstacles. */
  void createObstacles();

  /** \brief The number of hyper rectangles. */
  std::size_t numRectangles_{};

  /** \brief The minimum side length of the hyper rectangles. */
  double minSideLength_{};

  /** \brief The maximum side length of the hyper rectangles. */
  double maxSideLength_{};

  /** \brief The number of start states. */
  std::size_t numStarts_{};

  /** \brief The number of goal states. */
  std::size_t numGoals_{};

  /** \brief The random number generator. */
  ompl::RNG rng_{};
};

}  // namespace planning_contexts

}  // namespace pdt
