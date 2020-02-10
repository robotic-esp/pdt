/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Oxford
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
 *   * Neither the name of the University of Oxford nor the names of its
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

#include "esp_factories/context_factory.h"

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

#include "nlohmann/json.hpp"

#include "esp_common/context_type.h"
#include "esp_open_rave/open_rave_manipulator.h"
#include "esp_open_rave/open_rave_r3.h"
#include "esp_open_rave/open_rave_r3xso2.h"
#include "esp_open_rave/open_rave_se3.h"
#include "esp_planning_contexts/all_contexts.h"

namespace esp {

namespace ompltools {

using namespace std::string_literals;

ContextFactory::ContextFactory(const std::shared_ptr<const Configuration>& config) :
    config_(config) {
  if (config_->contains("context") == 0) {
    OMPL_ERROR("Configuration does not contain context data.");
    throw std::runtime_error("Context factory error.");
  }
}

std::shared_ptr<BaseContext> ContextFactory::create(const std::string& contextName) const {
  // Generate the parent key for convenient lookups in the config.
  const std::string parentKey{"context/" + contextName};

  // Ensure the config contains parameters for the parent key.
  if (!config_->contains(parentKey)) {
    throw std::invalid_argument("Requested unknown context '"s + contextName + "'."s);
  }

  // Get the type of the context.
  auto type = config_->get<CONTEXT_TYPE>(parentKey + "/type");

  // Allocate the correct context.
  switch (type) {
    case CONTEXT_TYPE::CENTRE_SQUARE: {
      try {
        return std::make_shared<CentreSquare>(createRealVectorSpaceInfo(parentKey), config_,
                                              contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error("Error allocating a CentreSquare context.");
      }
    }
    case CONTEXT_TYPE::DIVIDING_WALLS: {
      try {
        return std::make_shared<DividingWalls>(createRealVectorSpaceInfo(parentKey), config_,
                                               contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error("Error allocating a DividingWall context.");
      }
    }
    case CONTEXT_TYPE::DOUBLE_ENCLOSURE: {
      try {
        return std::make_shared<DoubleEnclosure>(createRealVectorSpaceInfo(parentKey), config_,
                                                 contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error("Error allocating a DoubleEnclosure context.");
      }
    }
    case CONTEXT_TYPE::FLANKING_GAP: {
      try {
        return std::make_shared<FlankingGap>(createRealVectorSpaceInfo(parentKey), config_,
                                             contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error("Error allocating a FlankingGap context.");
      }
    }
    case CONTEXT_TYPE::FOUR_ROOMS: {
      try {
        return std::make_shared<FourRooms>(createRealVectorSpaceInfo(parentKey), config_,
                                           contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error("Error allocating a FlankingGap context.");
      }
    }
    case CONTEXT_TYPE::GOAL_ENCLOSURE: {
      try {
        return std::make_shared<GoalEnclosure>(createRealVectorSpaceInfo(parentKey), config_,
                                               contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error("Error allocating a GoalEnclosure context.");
      }
    }
    case CONTEXT_TYPE::NARROW_PASSAGE: {
      try {
        return std::make_shared<NarrowPassage>(createRealVectorSpaceInfo(parentKey), config_,
                                               contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error("Error allocating a NarrowPassage context.");
      }
    }
    case CONTEXT_TYPE::OBSTACLE_FREE: {
      try {
        return std::make_shared<ObstacleFree>(createRealVectorSpaceInfo(parentKey), config_,
                                              contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error("Error allocating a ObstacleFree context.");
      }
    }
    case CONTEXT_TYPE::OPEN_RAVE_MANIPULATOR: {
      try {
        // Allocate a real vector state space.
        // The state space bounds are set in the context.
        auto stateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(
            config_->get<std::size_t>(parentKey + "/dimensions"));

        // Allocate the state information for this space.
        auto spaceInfo = std::make_shared<ompl::base::SpaceInformation>(stateSpace);
        return std::make_shared<OpenRaveManipulator>(spaceInfo, config_, contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error("Error allocating a OpenRaveManipulator context.");
      }
    }
    case CONTEXT_TYPE::OPEN_RAVE_R3: {
      try {
        // Allocate a real vector state space.
        // The state space bounds are set in the context.
        auto stateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(3u);

        // Allocate the state information for this space.
        auto spaceInfo = std::make_shared<ompl::base::SpaceInformation>(stateSpace);
        return std::make_shared<OpenRaveR3>(spaceInfo, config_, contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error("Error allocating a OpenRaveMover context.");
      }
    }
    case CONTEXT_TYPE::OPEN_RAVE_R3XSO2: {
      try {
        // Allocate the R3 component of the state space.
        // The space bounds are set in the context.
        auto r3StateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(3u);

        // Allocate the O2 component of the state space.
        auto so2StateSpace = std::make_shared<ompl::base::SO2StateSpace>();

        // Allocate the compound state space.
        auto stateSpace = std::make_shared<ompl::base::CompoundStateSpace>(
            std::vector<ompl::base::StateSpacePtr>({r3StateSpace, so2StateSpace}),
            std::vector<double>({1.0, 1.0}));

        // Allocate the space information for this space.
        auto spaceInfo = std::make_shared<ompl::base::SpaceInformation>(stateSpace);
        return std::make_shared<OpenRaveR3xSO2>(spaceInfo, config_, contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error("Error allocating an OpenRaveR3XO2 context.");
      }
    }
    case CONTEXT_TYPE::OPEN_RAVE_SE3: {
      try {
        // Allocate a real vector state space.
        // The state space bounds are set in the context.
        auto stateSpace = std::make_shared<ompl::base::SE3StateSpace>();

        // Allocate the state information for this space.
        auto spaceInfo = std::make_shared<ompl::base::SpaceInformation>(stateSpace);
        return std::make_shared<OpenRaveSE3>(spaceInfo, config_, contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error("Error allocating a OpenRaveMover context.");
      }
    }
    case CONTEXT_TYPE::RANDOM_RECTANGLES: {
      try {
        return std::make_shared<RandomRectangles>(createRealVectorSpaceInfo(parentKey), config_,
                                                  contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error("Error allocating a RandomRectangles context.");
      }
    }
    case CONTEXT_TYPE::RANDOM_RECTANGLES_MULTI_START_GOAL: {
      try {
        return std::make_shared<RandomRectanglesMultiStartGoal>(
            createRealVectorSpaceInfo(parentKey), config_, contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error("Error allocating a RandomRectanglesMultiStartGoal context.");
      }
    }
    case CONTEXT_TYPE::REPEATING_RECTANGLES: {
      try {
        return std::make_shared<RepeatingRectangles>(createRealVectorSpaceInfo(parentKey), config_,
                                                     contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error("Error allocating a RepeatingRectangles context.");
      }
    }
    case CONTEXT_TYPE::START_ENCLOSURE: {
      try {
        return std::make_shared<StartEnclosure>(createRealVectorSpaceInfo(parentKey), config_,
                                                contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error("Error allocating a StartEnclosure context.");
      }
    }
    case CONTEXT_TYPE::WALL_GAP: {
      try {
        return std::make_shared<WallGap>(createRealVectorSpaceInfo(parentKey), config_,
                                         contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error("Error allocating a WallGap context.");
      }
    }
    default: { throw std::invalid_argument("Context '"s + contextName + "' is of unknown type."s); }
  }
}

std::shared_ptr<ompl::base::SpaceInformation> ContextFactory::createRealVectorSpaceInfo(
    const std::string& parentKey) const {
  // Allocate a real vector state space.
  auto stateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(
      config_->get<std::size_t>(parentKey + "/dimensions"));

  // Set the bounds.
  stateSpace->setBounds(-0.5 * config_->get<double>(parentKey + "/boundarySideLengths"),
                        0.5 * config_->get<double>(parentKey + "/boundarySideLengths"));

  // Allocate the state information for this space.
  return std::make_shared<ompl::base::SpaceInformation>(stateSpace);
}

}  // namespace ompltools

}  // namespace esp
