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

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "nlohmann/json.hpp"
#pragma GCC diagnostic pop

namespace esp {

namespace ompltools {

enum class PLANNER_TYPE {
  INVALID,
  ABITSTAR,
  EITSTAR,
  EIRMSTAR,
  AITSTAR,
  BITSTAR,
  FMTSTAR,
  INFORMEDRRTSTAR,
  LAZYPRMSTAR,
  LBTRRT,
  PRMSTAR,
  RRT,
  RRTCONNECT,
  RRTSHARP,
  RRTSTAR,
  SPARSTWO,
};

NLOHMANN_JSON_SERIALIZE_ENUM(PLANNER_TYPE, {
                                               {PLANNER_TYPE::INVALID, "invalid"},
                                               {PLANNER_TYPE::ABITSTAR, "ABITstar"},
                                               {PLANNER_TYPE::EITSTAR, "EITstar"},
                                               {PLANNER_TYPE::EIRMSTAR, "EIRMstar"},
                                               {PLANNER_TYPE::AITSTAR, "AITstar"},
                                               {PLANNER_TYPE::BITSTAR, "BITstar"},
                                               {PLANNER_TYPE::FMTSTAR, "FMTstar"},
                                               {PLANNER_TYPE::INFORMEDRRTSTAR, "InformedRRTstar"},
                                               {PLANNER_TYPE::LAZYPRMSTAR, "LazyPRMstar"},
                                               {PLANNER_TYPE::LBTRRT, "LBTRRT"},
                                               {PLANNER_TYPE::PRMSTAR, "PRMstar"},
                                               {PLANNER_TYPE::RRT, "RRT"},
                                               {PLANNER_TYPE::RRTCONNECT, "RRTConnect"},
                                               {PLANNER_TYPE::RRTSHARP, "RRTsharp"},
                                               {PLANNER_TYPE::RRTSTAR, "RRTstar"},
                                               {PLANNER_TYPE::SPARSTWO, "SPARStwo"},
                                           })

}  // namespace ompltools

}  // namespace esp
