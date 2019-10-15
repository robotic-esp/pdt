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

// Authors: Marlin Strub

#include "esp_utilities/get_best_cost.h"

#include <limits>
#include <string>

#include <ompl/geometric/planners/aibitstar/AIBITstar.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/bitstar_regression/BITstarRegression.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/tbdstar/TBDstar.h>

#include "esp_common/planner_type.h"

namespace esp {

namespace ompltools {

namespace utilities {

ompl::base::Cost getBestCost(const ompl::base::PlannerPtr& planner, PLANNER_TYPE plannerType) {
  switch (plannerType) {
    case PLANNER_TYPE::AIBITSTAR: {
      return planner->as<ompl::geometric::AIBITstar>()->bestCost();
    }
    case PLANNER_TYPE::BITSTAR: {
      return planner->as<ompl::geometric::BITstar>()->bestCost();
    }
    case PLANNER_TYPE::BITSTARREGRESSION: {
      return planner->as<ompl::geometric::BITstarRegression>()->bestCost();
    }
    case PLANNER_TYPE::INFORMEDRRTSTAR: {
      return planner->as<ompl::geometric::InformedRRTstar>()->bestCost();
    }
    case PLANNER_TYPE::RRTCONNECT: {
      return ompl::base::Cost(std::numeric_limits<double>::infinity());
    }
    case PLANNER_TYPE::RRTSHARP: {
      return planner->as<ompl::geometric::RRTsharp>()->bestCost();
    }
    case PLANNER_TYPE::RRTSTAR: {
      return planner->as<ompl::geometric::RRTstar>()->bestCost();
    }
    case PLANNER_TYPE::SBITSTAR: {
      return planner->as<ompl::geometric::BITstar>()->bestCost();
    }
    case PLANNER_TYPE::TBDSTAR: {
      return planner->as<ompl::geometric::TBDstar>()->bestCost();
    }
    default: {
      throw std::runtime_error("Received request to get best cost of unknown planner type.");
      return ompl::base::Cost(std::numeric_limits<double>::infinity());
    }
  }
}

}  // namespace utilities

}  // namespace ompltools

}  // namespace esp
