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
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "pdt/spaces/SO3WAxisAngleBoundStateSpace.h"

namespace pdt {

namespace spaces {

/** \brief A state space representing SE(3) with a rotation bound defined by axis angle. */
class SE3WAxisAngleBoundStateSpace : public ompl::base::CompoundStateSpace {
 public:
  /** \brief A state in SE(3): position = (x, y, z), quaternion = (x, y, z, w) */
  class StateType : public ompl::base::CompoundStateSpace::StateType {
   public:
    StateType() = default;

    /** \brief Get the X component of the state */
    double getX() const { return as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0]; }

    /** \brief Get the Y component of the state */
    double getY() const { return as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1]; }

    /** \brief Get the Z component of the state */
    double getZ() const { return as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2]; }

    /** \brief Get the rotation component of the state */
    const SO3WAxisAngleBoundStateSpace::StateType &rotation() const {
      return *as<SO3WAxisAngleBoundStateSpace::StateType>(1);
    }

    /** \brief Get the rotation component of the state and allow changing it as well */
    SO3WAxisAngleBoundStateSpace::StateType &rotation() {
      return *as<SO3WAxisAngleBoundStateSpace::StateType>(1);
    }

    /** \brief Set the X component of the state */
    void setX(double x) { as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = x; }

    /** \brief Set the Y component of the state */
    void setY(double y) { as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = y; }

    /** \brief Set the Z component of the state */
    void setZ(double z) { as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = z; }

    /** \brief Set the X, Y and Z components of the state */
    void setXYZ(double x, double y, double z) {
      setX(x);
      setY(y);
      setZ(z);
    }
  };

  SE3WAxisAngleBoundStateSpace() {
    setName("SE3WAxisAngleBound" + getName());
    type_ = ompl::base::STATE_SPACE_SE3;
    addSubspace(std::make_shared<ompl::base::RealVectorStateSpace>(3), 1.0);
    addSubspace(std::make_shared<SO3WAxisAngleBoundStateSpace>(), 1.0);
    lock();
  }

  ~SE3WAxisAngleBoundStateSpace() override = default;

  /** Same as ompl::base::RealVectorStateSpace::setBounds() */
  void setBounds(const ompl::base::RealVectorBounds &bounds) {
    as<ompl::base::RealVectorStateSpace>(0)->setBounds(bounds);
  }

  /** Same as ompl::base::RealVectorStateSpace::getBounds() */
  const ompl::base::RealVectorBounds &getBounds() const {
    return as<ompl::base::RealVectorStateSpace>(0)->getBounds();
  }

  /** Set the max rotation. */
  void setMaxRotation(double rotation) {
    as<SO3WAxisAngleBoundStateSpace>(1)->setMaxRotation(rotation);
  }

  /** Get the max rotation. */
  double getMaxRotation() const { return as<SO3WAxisAngleBoundStateSpace>(1)->getMaxRotation(); }

  ompl::base::State *allocState() const override;
  void freeState(ompl::base::State *state) const override;
};

}  // namespace spaces

}  // namespace pdt
