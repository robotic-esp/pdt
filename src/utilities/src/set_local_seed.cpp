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

// Authors: Jonathan Gammell

#include "pdt/utilities/set_local_seed.h"

#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>

#ifdef PDT_EXTRA_EITSTAR_PR
#include <ompl/geometric/planners/informedtrees/EIRMstar.h>
#include <ompl/geometric/planners/informedtrees/EITstar.h>
#endif  // #ifdef PDT_EXTRA_EITSTAR_PR

#include "pdt/common/planner_type.h"

namespace pdt {

namespace utilities {

void warnSetLocalSeed() {
#ifndef PDT_EXTRA_SET_LOCAL_SEEDS
  static bool warnOnce = false;

  if (!warnOnce) {
    warnOnce = true;
    OMPL_WARN("PDT has not been compiled with support for setting a planner's local seed.");
  }
#endif  // #ifndef PDT_EXTRA_SET_LOCAL_SEEDS
}

void setLocalSeed(const std::shared_ptr<const config::Configuration> config,
                  const ompl::base::PlannerPtr& planner, const common::PLANNER_TYPE plannerType) {
  if (config->contains("experiment/seed")) {
    switch (plannerType) {
      case common::PLANNER_TYPE::ABITSTAR: {
#ifdef PDT_EXTRA_SET_LOCAL_SEEDS
        planner->as<ompl::geometric::ABITstar>()->setLocalSeed(
            config->get<std::size_t>("experiment/seed"));
#else
        warnSetLocalSeed();
#endif  // #ifdef PDT_EXTRA_SET_LOCAL_SEEDS
        break;
      }
      case common::PLANNER_TYPE::AITSTAR: {
#ifdef PDT_EXTRA_SET_LOCAL_SEEDS
        planner->as<ompl::geometric::AITstar>()->setLocalSeed(
            config->get<std::size_t>("experiment/seed"));
#else
        warnSetLocalSeed();
#endif  // #ifdef PDT_EXTRA_SET_LOCAL_SEEDS
        break;
      }
      case common::PLANNER_TYPE::BITSTAR: {
#ifdef PDT_EXTRA_SET_LOCAL_SEEDS
        planner->as<ompl::geometric::BITstar>()->setLocalSeed(
            config->get<std::size_t>("experiment/seed"));
#else
        warnSetLocalSeed();
#endif  // #ifdef PDT_EXTRA_SET_LOCAL_SEEDS
        break;
      }
#ifdef PDT_EXTRA_EITSTAR_PR
      case common::PLANNER_TYPE::EIRMSTAR:
      case common::PLANNER_TYPE::EITSTAR: {
#ifdef PDT_EXTRA_SET_LOCAL_SEEDS
        planner->as<ompl::geometric::EITstar>()->setLocalSeed(
            config->get<std::size_t>("experiment/seed"));
#else
        warnSetLocalSeed();
#endif  // #ifdef PDT_EXTRA_SET_LOCAL_SEEDS
        break;
      }
#endif  // #ifdef PDT_EXTRA_EITSTAR_PR
      default: { static_cast<void>(planner); }
    }
  }
#ifdef PDT_EXTRA_SET_LOCAL_SEEDS
  else {
    throw std::runtime_error("Unknown seed.");
  }
#endif  // #ifdef PDT_EXTRA_SET_LOCAL_SEEDS
}

}  // namespace utilities

}  // namespace pdt
