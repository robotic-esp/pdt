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

// Author: Marlin Strub

#include "pdt/factories/context_factory.h"

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

#include "nlohmann/json.hpp"

#include "pdt/common/context_type.h"
#include "pdt/planning_contexts/all_contexts.h"

#ifdef PDT_OPEN_RAVE
#include "pdt/open_rave/open_rave_manipulator.h"
#include "pdt/open_rave/open_rave_r3.h"
#include "pdt/open_rave/open_rave_r3xso2.h"
#include "pdt/open_rave/open_rave_se3.h"
#include "pdt/spaces/SE3WAxisAngleBoundStateSpace.h"
#endif

namespace pdt {

namespace factories {

using namespace std::string_literals;
namespace json = nlohmann;

ContextFactory::ContextFactory(const std::shared_ptr<const config::Configuration>& config) :
    config_(config) {
  if (config_->contains("context") == 0) {
    OMPL_ERROR("Configuration does not contain context data.");
    throw std::runtime_error("Context factory error.");
  }
}

std::shared_ptr<planning_contexts::BaseContext> ContextFactory::create(
    const std::string& contextName) const {
  // Generate the parent key for convenient lookups in the config.
  const std::string parentKey{"context/" + contextName};

  // Ensure the config contains parameters for the parent key.
  if (!config_->contains(parentKey)) {
    throw std::invalid_argument("Requested unknown context '"s + contextName + "'."s);
  }

  // Get the type of the context.
  auto type = config_->get<common::CONTEXT_TYPE>(parentKey + "/type");

  // Allocate the correct context.
  switch (type) {
    case common::CONTEXT_TYPE::CENTER_SQUARE: {
      try {
        return std::make_shared<planning_contexts::CenterSquare>(
            createRealVectorSpaceInfo(parentKey), config_, contextName);
      } catch (const json::detail::type_error& e) {
        auto msg = "Error allocating a CenterSquare context with exception:\n    "s + e.what();
        throw std::runtime_error(msg);
      }
    }
    case common::CONTEXT_TYPE::DIVIDING_WALLS: {
      try {
        return std::make_shared<planning_contexts::DividingWalls>(
            createRealVectorSpaceInfo(parentKey), config_, contextName);
      } catch (const json::detail::type_error& e) {
        auto msg = "Error allocating a DividingWalls context with exception:\n    "s + e.what();
        throw std::runtime_error(msg);
      }
    }
    case common::CONTEXT_TYPE::DOUBLE_ENCLOSURE: {
      try {
        return std::make_shared<planning_contexts::DoubleEnclosure>(
            createRealVectorSpaceInfo(parentKey), config_, contextName);
      } catch (const json::detail::type_error& e) {
        auto msg = "Error allocating a DoubleEnclosure context with exception:\n    "s + e.what();
        throw std::runtime_error(msg);
      }
    }
    case common::CONTEXT_TYPE::FLANKING_GAP: {
      try {
        return std::make_shared<planning_contexts::FlankingGap>(
            createRealVectorSpaceInfo(parentKey), config_, contextName);
      } catch (const json::detail::type_error& e) {
        auto msg = "Error allocating a FlankingGap context with exception:\n    "s + e.what();
        throw std::runtime_error(msg);
      }
    }
    case common::CONTEXT_TYPE::FOUR_ROOMS: {
      try {
        return std::make_shared<planning_contexts::FourRooms>(createRealVectorSpaceInfo(parentKey),
                                                              config_, contextName);
      } catch (const json::detail::type_error& e) {
        auto msg = "Error allocating a FourRooms context with exception:\n    "s + e.what();
        throw std::runtime_error(msg);
      }
    }
    case common::CONTEXT_TYPE::GOAL_ENCLOSURE: {
      try {
        return std::make_shared<planning_contexts::GoalEnclosure>(
            createRealVectorSpaceInfo(parentKey), config_, contextName);
      } catch (const json::detail::type_error& e) {
        auto msg = "Error allocating a GoalEnclosure context with exception:\n    "s + e.what();
        throw std::runtime_error(msg);
      }
    }
    case common::CONTEXT_TYPE::NARROW_PASSAGE: {
      try {
        return std::make_shared<planning_contexts::NarrowPassage>(
            createRealVectorSpaceInfo(parentKey), config_, contextName);
      } catch (const json::detail::type_error& e) {
        auto msg = "Error allocating a NarrowPassage context with exception:\n    "s + e.what();
        throw std::runtime_error(msg);
      }
    }
    case common::CONTEXT_TYPE::OBSTACLE_FREE: {
      try {
        return std::make_shared<planning_contexts::ObstacleFree>(
            createRealVectorSpaceInfo(parentKey), config_, contextName);
      } catch (const json::detail::type_error& e) {
        auto msg = "Error allocating a ObstacleFree context with exception:\n    "s + e.what();
        throw std::runtime_error(msg);
      }
    }
#ifdef PDT_OPEN_RAVE
    case common::CONTEXT_TYPE::OPEN_RAVE_MANIPULATOR: {
      try {
        // Allocate a real vector state space.
        // The state space bounds are set in the context.
        auto stateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(
            config_->get<std::size_t>(parentKey + "/dimensions"));

        // Allocate the state information for this space.
        auto spaceInfo = std::make_shared<ompl::base::SpaceInformation>(stateSpace);
        return std::make_shared<open_rave::OpenRaveManipulator>(spaceInfo, config_, contextName);
      } catch (const json::detail::type_error& e) {
        auto msg =
            "Error allocating a OpenRaveManipulator context with exception:\n    "s + e.what();
        throw std::runtime_error(msg);
      }
    }
    case common::CONTEXT_TYPE::OPEN_RAVE_R3: {
      try {
        // Allocate a real vector state space.
        // The state space bounds are set in the context.
        auto stateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(3u);

        // Allocate the state information for this space.
        auto spaceInfo = std::make_shared<ompl::base::SpaceInformation>(stateSpace);
        return std::make_shared<open_rave::OpenRaveR3>(spaceInfo, config_, contextName);
      } catch (const json::detail::type_error& e) {
        auto msg = "Error allocating a OpenRaveR3 context with exception:\n    "s + e.what();
        throw std::runtime_error(msg);
      }
    }
    case common::CONTEXT_TYPE::OPEN_RAVE_R3XSO2: {
      try {
        // Allocate the R3 component of the state space.
        // The space bounds are set in the context.
        auto r3StateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(3u);

        // Allocate the O2 component of the state space.
        auto so2StateSpace = std::make_shared<ompl::base::SO2StateSpace>();

        // Allocate the compound state space.
        auto stateSpace = std::make_shared<ompl::base::CompoundStateSpace>(
            std::vector<ompl::base::StateSpacePtr>({r3StateSpace, so2StateSpace}),
            std::vector<double>({1.0, 0.01}));

        // Allocate the space information for this space.
        auto spaceInfo = std::make_shared<ompl::base::SpaceInformation>(stateSpace);
        return std::make_shared<open_rave::OpenRaveR3xSO2>(spaceInfo, config_, contextName);
      } catch (const json::detail::type_error& e) {
        auto msg = "Error allocating a OpenRaveR3xSO2 context with exception:\n    "s + e.what();
        throw std::runtime_error(msg);
      }
    }
    case common::CONTEXT_TYPE::OPEN_RAVE_SE3: {
      try {
        // Allocate a real vector state space.
        // The state space bounds are set in the context.
        auto stateSpace = std::make_shared<ompl::base::SE3StateSpace>();

        // Allocate the state information for this space.
        auto spaceInfo = std::make_shared<ompl::base::SpaceInformation>(stateSpace);
        return std::make_shared<open_rave::OpenRaveSE3>(spaceInfo, config_, contextName);
      } catch (const json::detail::type_error& e) {
        auto msg = "Error allocating a OpenRaveSE3 context with exception:\n    "s + e.what();
        throw std::runtime_error(msg);
      }
    }
    case common::CONTEXT_TYPE::OPEN_RAVE_CONSTRAINED_SE3: {
      try {
        // Allocate a real vector state space.
        // The state space bounds are set in the context.
        auto stateSpace = std::make_shared<spaces::SE3WAxisAngleBoundStateSpace>();
        stateSpace->setMaxRotation(config_->get<double>(parentKey + "/maxRotation"));

        // Allocate the state information for this space.
        auto spaceInfo = std::make_shared<ompl::base::SpaceInformation>(stateSpace);
        return std::make_shared<open_rave::OpenRaveSE3>(spaceInfo, config_, contextName);
      } catch (const json::detail::type_error& e) {
        auto msg =
            "Error allocating a ConstrainedOpenRaveSE3 context with exception:\n    "s + e.what();
        throw std::runtime_error(msg);
      }
    }
#endif
    case common::CONTEXT_TYPE::RANDOM_RECTANGLES: {
      try {
        return std::make_shared<planning_contexts::RandomRectangles>(
            createRealVectorSpaceInfo(parentKey), config_, contextName);
      } catch (const json::detail::type_error& e) {
        auto msg = "Error allocating a RandomRectangles context with exception:\n    "s + e.what();
        throw std::runtime_error(msg);
      }
    }
    case common::CONTEXT_TYPE::RANDOM_RECTANGLES_MULTI_START_GOAL: {
      try {
        return std::make_shared<planning_contexts::RandomRectanglesMultiStartGoal>(
            createRealVectorSpaceInfo(parentKey), config_, contextName);
      } catch (const json::detail::type_error& e) {
        auto msg =
            "Error allocating a RandomRectanglesMultiStartGoal context with exception:\n    "s +
            e.what();
        throw std::runtime_error(msg);
      }
    }
    case common::CONTEXT_TYPE::REEDS_SHEPP_RANDOM_RECTANGLES: {
      try {
        return std::make_shared<planning_contexts::ReedsSheppRandomRectangles>(
            createReedsSheppSpaceInfo(parentKey), config_, contextName);
      } catch (const json::detail::type_error& e) {
        auto msg = "Error allocating a ReedsSheppRandomRectangles context with exception:\n    "s +
                   e.what();
        throw std::runtime_error(msg);
      }
    }
    case common::CONTEXT_TYPE::REPEATING_RECTANGLES: {
      try {
        return std::make_shared<planning_contexts::RepeatingRectangles>(
            createRealVectorSpaceInfo(parentKey), config_, contextName);
      } catch (const json::detail::type_error& e) {
        auto msg =
            "Error allocating a RepeatingRectangles context with exception:\n    "s + e.what();
        throw std::runtime_error(msg);
      }
    }
    case common::CONTEXT_TYPE::START_ENCLOSURE: {
      try {
        return std::make_shared<planning_contexts::StartEnclosure>(
            createRealVectorSpaceInfo(parentKey), config_, contextName);
      } catch (const json::detail::type_error& e) {
        auto msg = "Error allocating a StartEnclosure context with exception:\n    "s + e.what();
        throw std::runtime_error(msg);
      }
    }
    case common::CONTEXT_TYPE::WALL_GAP: {
      try {
        return std::make_shared<planning_contexts::WallGap>(createRealVectorSpaceInfo(parentKey),
                                                            config_, contextName);
      } catch (const json::detail::type_error& e) {
        auto msg = "Error allocating a WallGap context with exception:\n    "s + e.what();
        throw std::runtime_error(msg);
      }
    }
    default: {
      throw std::invalid_argument("Context '"s + contextName + "' is of unknown type '"s +
                                  config_->get<std::string>(parentKey + "/type") + "'."s);
    }
  }
}

std::shared_ptr<ompl::base::SpaceInformation> ContextFactory::createRealVectorSpaceInfo(
    const std::string& parentKey) const {
  assert(config_->get<std::vector<double>>(parentKey + "/boundarySideLengths").size() ==
         config_->get<std::size_t>(parentKey + "/dimensions"));
  // Allocate a real vector state space.
  auto stateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(
      config_->get<std::size_t>(parentKey + "/dimensions"));

  // Set the bounds.
  auto sideLengths = config_->get<std::vector<double>>(parentKey + "/boundarySideLengths");
  ompl::base::RealVectorBounds bounds(config_->get<unsigned>(parentKey + "/dimensions"));
  for (std::size_t dim = 0u; dim < config_->get<std::size_t>(parentKey + "/dimensions"); ++dim) {
    bounds.low.at(dim) = -0.5 * sideLengths.at(dim);
    bounds.high.at(dim) = 0.5 * sideLengths.at(dim);
  }
  stateSpace->setBounds(bounds);

  // Allocate the state information for this space.
  return std::make_shared<ompl::base::SpaceInformation>(stateSpace);
}

std::shared_ptr<ompl::base::SpaceInformation> ContextFactory::createReedsSheppSpaceInfo(
    const std::string& parentKey) const {
  assert(config_->get<std::vector<double>>(parentKey + "/boundarySideLengths").size() == 2u);
  // Allocate a real vector state space.
  auto stateSpace = std::make_shared<ompl::base::ReedsSheppStateSpace>(0.1);

  // Set the bounds.
  auto sideLengths = config_->get<std::vector<double>>(parentKey + "/boundarySideLengths");
  ompl::base::RealVectorBounds bounds(2u);
  for (std::size_t dim = 0u; dim < 2u; ++dim) {
    bounds.low.at(dim) = -0.5 * sideLengths.at(dim);
    bounds.high.at(dim) = 0.5 * sideLengths.at(dim);
  }
  stateSpace->setBounds(bounds);

  // Allocate the state information for this space.
  return std::make_shared<ompl::base::SpaceInformation>(stateSpace);
}

}  // namespace factories

}  // namespace pdt
