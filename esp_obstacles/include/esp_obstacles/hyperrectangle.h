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

/* Authors: Marlin Strub */

#pragma once

#include <memory>
#include <vector>

#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpaceTypes.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/Exception.h>

#include "esp_obstacles/base_obstacle.h"
#include "esp_obstacles/obstacle_visitor.h"

namespace esp {

namespace ompltools {

// A hyperrectangluar obstacle. Must inherit from BaseObstacle or BaseAntiObstacle.
// Anchor point is the middle of the rectangle.
template <typename T>
class Hyperrectangle : public T {
 public:
  Hyperrectangle(const ompl::base::SpaceInformationPtr& spaceInfo);
  Hyperrectangle(const ompl::base::SpaceInformationPtr& spaceInfo,
                 const ompl::base::ScopedState<>& anchor, const std::vector<double>& widths);
  virtual ~Hyperrectangle() = default;

  // Accept a visitor.
  virtual void accept(const ObstacleVisitor& visitor) const override;

  /** \brief The measure of this hyperrectangle. */
  virtual double computeMeasure() const override;

  // Compute the clearance of a state to this hyperrectangle.
  virtual double clearance(const ompl::base::State* state) const override;

  // Compute the largest diameter of this hyperrectangle.
  virtual double computeMinCircumscribingRadius() const override;

  // Set the widths.
  void setWidths(const std::vector<double>& widths);

  /** \brief Get the widths of the obstacle. */
  const std::vector<double>& getWidths() const;

  // Comparison operators of two hyperrectangles.
  bool operator==(const Hyperrectangle& rhs) const;
  bool operator!=(const Hyperrectangle& rhs) const;

 private:
  // Returns whether the state is in this hyperrectangle or not.
  bool isInside(const ompl::base::State* state) const override;

  // The space information.
  ompl::base::SpaceInformationPtr spaceInfo_;

  // The widhts of this hyperrectangle's sides.
  std::vector<double> widths_{};
};

template <typename T>
Hyperrectangle<T>::Hyperrectangle(const ompl::base::SpaceInformationPtr& spaceInfo) :
    T(spaceInfo),
    spaceInfo_(spaceInfo),
    widths_(spaceInfo->getStateDimension(), 0.0) {
}

template <typename T>
Hyperrectangle<T>::Hyperrectangle(const ompl::base::SpaceInformationPtr& spaceInfo,
                                  const ompl::base::ScopedState<>& anchor,
                                  const std::vector<double>& widths) :
    T(spaceInfo),
    spaceInfo_(spaceInfo),
    widths_(widths) {
  // What's a hyperrectangle in a non-real-vector-state-space?
  if (spaceInfo_->getStateSpace()->getType() !=
      ompl::base::StateSpaceType::STATE_SPACE_REAL_VECTOR) {
    throw std::runtime_error(
        "Hyperrectangles are only defined (or at least tested) for real vector state spaces.");
  }
  T::setAnchor(anchor);
}

template <typename T>
void Hyperrectangle<T>::accept(const ObstacleVisitor& visitor) const {
  visitor.visit(*this);
}

template <typename T>
bool Hyperrectangle<T>::isInside(const ompl::base::State* state) const {
  // Let's be conservative here.
  auto rstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
  for (auto dim = 0u; dim < spaceInfo_->getStateDimension(); ++dim) {
    if (rstate->values[dim] <
            T::anchor_[dim] - widths_[dim] / 2.0 - std::numeric_limits<double>::epsilon() ||
        rstate->values[dim] >
            T::anchor_[dim] + widths_[dim] / 2.0 + std::numeric_limits<double>::epsilon()) {
      return false;
    }
  }

  return true;
}

template <typename T>
double Hyperrectangle<T>::computeMeasure() const {
  // Calculate the Lebesgue measure by compound multiplication.
  double measure = 1.0;
  for (std::size_t i = 0u; i < widths_.size(); ++i) {
    measure = measure * widths_.at(i);
  }
  return measure;
}

template <typename T>
double Hyperrectangle<T>::clearance(const ompl::base::State* state) const {
  // Cast the state to the correct type.
  auto rstate = state->as<ompl::base::RealVectorStateSpace::StateType>();

  // Compute the sum of squares.
  double sumOfSquares = 0.0;
  for (auto dim = 0u; dim < spaceInfo_->getStateDimension(); ++dim) {
    double delta = std::max(
        std::abs(
            rstate->operator[](dim) -
            T::getState()->template as<ompl::base::RealVectorStateSpace::StateType>()->operator[](
                dim)) -
            widths_.at(dim) / 2.0,
        0.0);
    sumOfSquares += delta * delta;
  }

  return std::sqrt(sumOfSquares);
}

template <typename T>
double Hyperrectangle<T>::computeMinCircumscribingRadius() const {
  // The anchor is the center of the hyperrectangle, so the circumscribing radius is half the
  // diagonal between the two corners that are farthest away from each other.
  double sumOfSquares = 0.0;
  for (const auto width : widths_) {
    sumOfSquares += std::pow(width, 2.0);
  }
  return std::sqrt(sumOfSquares) / 2.0;
}

template <typename T>
void Hyperrectangle<T>::setWidths(const std::vector<double>& widths) {
  widths_ = widths;
}

template <typename T>
const std::vector<double>& Hyperrectangle<T>::getWidths() const {
  return widths_;
}

template <typename T>
bool Hyperrectangle<T>::operator==(const Hyperrectangle<T>& rhs) const {
  // Check if the anchor is equal.
  if (T::getAnchor != rhs.getAnchor()) {
    return false;
  }
  // Check if the widhts are equal.
  const auto& rhsWidths = rhs.getWidths();
  if (widths_.size() != rhsWidths.size()) {
    return false;
  }
  for (std::size_t i = 0u; i < widths_.size(); ++i) {
    if (std::abs(widths_[i] - rhsWidths[i]) > std::numeric_limits<double>::epsilon()) {
      return false;
    }
  }
  return true;
}

template <typename T>
bool Hyperrectangle<T>::operator!=(const Hyperrectangle<T>& rhs) const {
  return !(*this == rhs);
}

}  // namespace ompltools

}  // namespace esp
