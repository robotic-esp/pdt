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

/* Authors: Marlin Strub */

#include "pdt/planning_contexts/reeds_shepp_validity_checker.h"

#include <ompl/base/spaces/SE2StateSpace.h>

namespace pdt {

namespace planning_contexts {

ReedsSheppValidityChecker::ReedsSheppValidityChecker(
    const ompl::base::SpaceInformationPtr& spaceInfo) :
    ompl::base::StateValidityChecker(spaceInfo),
    circumradius_(std::sqrt(std::pow(width_, 2.0) + std::pow(length_, 2.0)) / 2.0),
    vectorSpace_(spaceInfo->getStateSpace()
                     ->as<ompl::base::CompoundStateSpace>()
                     ->as<ompl::base::RealVectorStateSpace>(0u)),
    so2Space_(spaceInfo->getStateSpace()
                  ->as<ompl::base::CompoundStateSpace>()
                  ->as<ompl::base::SO2StateSpace>(1u)) {
}

bool ReedsSheppValidityChecker::isValid(const ompl::base::State* state) const {
  // If the state does not satisfy the space bounds, it is not valid.
  if (!si_->satisfiesBounds(state)) {
    return false;
  }

  const auto carSE2State = state->as<ompl::base::SE2StateSpace::StateType>();
  const auto carVecState = carSE2State->as<ompl::base::RealVectorStateSpace::StateType>(0u);
  const auto carSO2State = carSE2State->as<ompl::base::SO2StateSpace::StateType>(1u);

  // A state is invalid if it collides with an obstacle.
  for (const auto& obstacle : obstacles_) {
    const auto obs =
        dynamic_cast<obstacles::Hyperrectangle<obstacles::BaseObstacle>*>(obstacle.get());

    // This is a tiny optimization: If the distance between the centers of both rectangles is larger
    // then the sum of the  circumscribing radii, then the orientation of the rectangles doesn't
    // matter.
    const auto obsState = obs->getAnchor();
    const auto obsCircumradius = obs->getCircumradius();
    if (vectorSpace_->distance(obsState->as<ompl::base::RealVectorStateSpace::StateType>(),
                               carVecState) >= obsCircumradius + circumradius_) {
      continue;
    }

    // We need to do a bit more work if this is not the case. We're using the separation axis
    // theorem, as described here:
    // https://www.gamedev.net/tutorials/_/technical/game-programming/2d-rotated-rectangle-collision-r2604/

    // Get the width and height of the obstacle.
    const auto obsWidth = obs->getWidths().at(0u);
    const auto obsHeight = obs->getWidths().at(1u);

    // Compute the corners of the obstacle in world coordinates and clockwise order, starting from
    // upper left: upper left, upper right, lower right, lower left.
    const std::vector<Point> obsCornersWorld{
        {obsState[0u] - (obsWidth / 2.0), obsState[1u] + (obsHeight / 2.0)},
        {obsState[0u] + (obsWidth / 2.0), obsState[1u] + (obsHeight / 2.0)},
        {obsState[0u] + (obsWidth / 2.0), obsState[1u] - (obsHeight / 2.0)},
        {obsState[0u] - (obsWidth / 2.0), obsState[1u] - (obsHeight / 2.0)}};

    // Compute the corners of the car in car coordinates:
    // front left, front right, rear right, rear left.
    const std::vector<Point> carCornersCar{{length_ / 2.0, -width_ / 2.0},
                                           {length_ / 2.0, width_ / 2.0},
                                           {-length_ / 2.0, width_ / 2.0},
                                           {-length_ / 2.0, -width_ / 2.0}};

    // Compute the corners of the car in world coordinates.
    std::vector<Point> carCornersWorld{{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
    for (auto i = 0u; i < 4u; ++i) {
      const auto psi = std::atan2(carCornersCar[i][1u], carCornersCar[i][0u]);
      const auto rad =
          std::sqrt(std::pow(carCornersCar[i][0u], 2.0) + std::pow(carCornersCar[i][1u], 2.0));

      carCornersWorld.at(i)[0u] = rad * std::cos(carSO2State->value + psi) + (*carVecState)[0u];
      carCornersWorld.at(i)[1u] = rad * std::sin(carSO2State->value + psi) + (*carVecState)[1u];
    }

    // Compute the four axes for projection.
    std::vector<Axis> axes{{obsCornersWorld[1u][0u] - obsCornersWorld[0u][0u],
                            obsCornersWorld[1u][1u] - obsCornersWorld[0u][1u]},
                           {obsCornersWorld[1u][0u] - obsCornersWorld[2u][0u],
                            obsCornersWorld[1u][1u] - obsCornersWorld[2u][1u]},
                           {carCornersWorld[0u][0u] - carCornersWorld[3u][0u],
                            carCornersWorld[0u][1u] - carCornersWorld[3u][1u]},
                           {carCornersWorld[0u][0u] - carCornersWorld[1u][0u],
                            carCornersWorld[0u][1u] - carCornersWorld[1u][1u]}};

    // Project the corners of the car and obstacle onto the four axes.
    for (const auto& axis : axes) {
      std::array<double, 4u> car{0.0, 0.0, 0.0, 0.0};
      for (auto i = 0u; i < 4u; ++i) {
        car[i] = dotProduct(project(carCornersWorld[i], axis), axis);
      }
      auto [minCar, maxCar] = std::minmax_element(car.begin(), car.end());

      std::array<double, 4u> obs{0.0, 0.0, 0.0, 0.0};
      for (auto i = 0u; i < 4u; ++i) {
        obs[i] = dotProduct(project(obsCornersWorld[i], axis), axis);
      }
      auto [minObs, maxObs] = std::minmax_element(obs.begin(), obs.end());

      // If there is no overlap, then the obstacle cannot be in collision with the car.
      if (minCar <= maxObs || minObs <= maxCar) {
        return false;
      }
    }
  }

  // The state satisfies the bounds and is not invalidated by any obstacles.
  return true;
}

double ReedsSheppValidityChecker::clearance(const ompl::base::State* state) const {
  // Compute the distance to all obstacles and take the minimum.
  double minDistance = std::numeric_limits<double>::infinity();
  for (const auto& obstacle : obstacles_) {
    double distance = obstacle->clearance(state);
    minDistance = distance < minDistance ? distance : minDistance;
  }
  return minDistance;
}

std::array<double, 2u> ReedsSheppValidityChecker::project(const Point& point,
                                                          const Axis& axis) const {
  const double frac =
      (point[0u] * axis[0u] + point[1u] * axis[1u]) / (axis[0u] * axis[0u] + axis[1u] * axis[1u]);
  return {frac * axis[0u], frac * axis[1u]};
}

double ReedsSheppValidityChecker::dotProduct(const Point& point, const Axis& axis) const {
  return point[0u] * axis[0u] + point[1u] * axis[1u];
}

void ReedsSheppValidityChecker::addObstacle(
    const std::shared_ptr<obstacles::BaseObstacle>& obstacle) {
  obstacles_.push_back(obstacle);
}

void ReedsSheppValidityChecker::addObstacles(
    const std::vector<std::shared_ptr<obstacles::BaseObstacle>>& obstacles) {
  obstacles_.insert(obstacles_.end(), obstacles.begin(), obstacles.end());
}

std::vector<std::shared_ptr<obstacles::BaseObstacle>> ReedsSheppValidityChecker::getObstacles()
    const {
  return obstacles_;
}

}  // namespace planning_contexts

}  // namespace pdt
