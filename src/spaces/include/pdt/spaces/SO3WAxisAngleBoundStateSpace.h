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

/* Author: Marlin Strub */

#pragma once

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>

#include <boost/math/constants/constants.hpp>

namespace pdt {

namespace spaces {

/** \brief State space sampler for SO(3) with a rotation bound defined by axis angle. */
class SO3WAxisAngleBoundStateSampler : public ompl::base::SO3StateSampler {
 public:
  /** \brief Constructor */
  SO3WAxisAngleBoundStateSampler(const ompl::base::StateSpace *space);

  /** \brief Sample uniformly. */
  void sampleUniform(ompl::base::State *state) override;

  /** \brief Not yet implemented. */
  void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near,
                         double distance) override;

  /** \brief Not yet implemented. */
  void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean,
                      double stdDev) override;
};

/** \brief A state space representing SO(3) with a rotation bound defined by axis angle. */
class SO3WAxisAngleBoundStateSpace : public ompl::base::SO3StateSpace {
 public:
  SO3WAxisAngleBoundStateSpace() {
    setName("SO3WAxisAngleBound" + getName());
    type_ = ompl::base::STATE_SPACE_SO3;
  }

  ~SO3WAxisAngleBoundStateSpace() override = default;

  /** \brief Set the max rotation. */
  void setMaxRotation(double maxRotation);

  /** \brief Get the max rotation. */
  double getMaxRotation() const;

  ompl::base::StateSamplerPtr allocDefaultStateSampler() const override;

  bool satisfiesBounds(const ompl::base::State *state) const override;

 private:
  double maxRotation_{boost::math::constants::pi<double>()};
};

}  // namespace spaces

}  // namespace pdt
