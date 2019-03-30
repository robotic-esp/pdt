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

ContextFactory::ContextFactory(const std::shared_ptr<Configuration> &config) : config_(config) {
  if (config_->contains("Contexts") == 0) {
    throw std::runtime_error("Configuration does not contain context data.");
  }
}

std::shared_ptr<BaseContext> ContextFactory::create(const std::string &contextName) const {
  auto contextConfig = config_->getContextConfig(contextName);
  auto type = contextConfig["type"].get<CONTEXT_TYPE>();
  switch (type) {
    case CONTEXT_TYPE::CENTRE_SQUARE: {
      try {
        return std::make_shared<CentreSquare>(contextConfig["dimensions"],
                                              contextConfig["obstacleWidth"],
                                              contextConfig["boundarySideLengths"],
                                              contextConfig["maxTime"],
                                              contextConfig["collisionCheckResolution"]);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error(
            "Error allocating a CentreSquare context. Check the spelling of the parameters in the "
            "context factory and the json file.");
      }
    }
    case CONTEXT_TYPE::DIVIDING_WALL: {
      try {
        return std::make_shared<DividingWall>(contextConfig["dimensions"],
                                              contextConfig["wallThickness"],
                                              contextConfig["numGaps"],
                                              contextConfig["gapWidths"],
                                              contextConfig["maxTime"],
                                              contextConfig["collisionCheckResolution"]);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error(
            "Error allocating a DividingWall context. Check the spelling of the parameters in the "
            "context factory and the json file.");
      }
    }
    case CONTEXT_TYPE::DOUBLE_ENCLOSURE: {
      try {
        return std::make_shared<DoubleEnclosure>(contextConfig["dimensions"],
                                                 contextConfig["boundarySideLengths"],
                                                 contextConfig["insideWidth"],
                                                 contextConfig["wallThickness"],
                                                 contextConfig["openingWidth"],
                                                 contextConfig["maxTime"],
                                                 contextConfig["collisionCheckResolution"]);
      } catch (const json::detail::type_error& e) {
        throw std::runtime_error(
            "Error allocating a DoubleEnclosure context. Check the spelling of the parameters in "
            "the context factory and the json file.");
      }
    }
    default: {
      throw std::runtime_error("Requested to create context of unknown type at factory.");
      return std::make_shared<CentreSquare>(0u, 0.0, 0.0, 0.0, 0.0);
    }
  }
}

}  // namespace ompltools

}  // namespace esp
