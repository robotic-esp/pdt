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

#include "nlohmann/json.hpp"

#include "esp_common/context_type.h"
#include "esp_planning_contexts/all_contexts.h"

namespace esp {

namespace ompltools {

ContextFactory::ContextFactory(const std::shared_ptr<Configuration>& config) : config_(config) {
  if (config_->contains("Contexts") == 0) {
    OMPL_ERROR("Configuration does not contain context data.");
    throw std::runtime_error("Context factory error.");
  }
}

std::shared_ptr<BaseContext> ContextFactory::create(const std::string& contextName) const {
  const std::string parentKey{"Contexts/" + contextName};
  if (!config_->contains(parentKey)) {
    OMPL_ERROR("Configuration has no entry for requested context '%s'.", parentKey.c_str());
  }
  auto type = config_->get<CONTEXT_TYPE>(parentKey + "/type");
  switch (type) {
    case CONTEXT_TYPE::CENTRE_SQUARE: {
      try {
        return std::make_shared<CentreSquare>(config_, contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error(
            "Error allocating a CentreSquare context. Check the spelling of the parameters in the "
            "context factory and the json file.");
      }
    }
    case CONTEXT_TYPE::DIVIDING_WALLS: {
      try {
        return std::make_shared<DividingWalls>(config_, contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error(
            "Error allocating a DividingWall context. Check the spelling of the parameters in the "
            "context factory and the json file.");
      }
    }
    case CONTEXT_TYPE::DOUBLE_ENCLOSURE: {
      try {
        return std::make_shared<DoubleEnclosure>(config_, contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error(
            "Error allocating a DoubleEnclosure context. Check the spelling of the parameters in"
            "the context factory and the json file.");
      }
    }
    case CONTEXT_TYPE::FLANKING_GAP: {
      try {
        return std::make_shared<FlankingGap>(config_, contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error(
            "Error allocating a FlankingGap context. Check the spelling of the parameters in"
            "the context factory and the json file.");
      }
    }
    case CONTEXT_TYPE::GOAL_ENCLOSURE: {
      try {
        return std::make_shared<GoalEnclosure>(config_, contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error(
            "Error allocating a GoalEnclosure context. Check the spelling of the parameters in"
            "the context factory and the json file.");
      }
    }
    case CONTEXT_TYPE::RANDOM_RECTANGLES: {
      try {
        return std::make_shared<RandomRectangles>(config_, contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error(
            "Error allocating a RandomRectangles context. Check the spelling of the parameters in "
            "the context factory and the json file.");
      }
    }
    case CONTEXT_TYPE::START_ENCLOSURE: {
      try {
        return std::make_shared<StartEnclosure>(config_, contextName);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error(
            "Error allocating a StartEnclosure context. Check the spelling of the parameters in"
            "the context factory and the json file.");
      }
    }
    default: {
      OMPL_ERROR("Context '%s' has unknown type.", contextName.c_str());
      throw std::runtime_error("Requested to create context of unknown type at factory.");
      return std::make_shared<CentreSquare>(config_, contextName);
    }
  }
}

}  // namespace ompltools

}  // namespace esp
