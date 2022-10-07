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

// Authors: Marlin Strub

#include <pangolin/pangolin.h>

#include "pdt/common/context_type.h"
#include "pdt/common/planner_type.h"
#include "pdt/config/configuration.h"
#include "pdt/factories/context_factory.h"
#include "pdt/factories/planner_factory.h"
#include "pdt/planning_contexts/all_contexts.h"
#include "pdt/visualization/interactive_visualizer.h"

using namespace std::string_literals;

int main(const int argc, const char** argv) {
  // Load the config.
  auto config = std::make_shared<pdt::config::Configuration>(argc, argv);
  config->registerAsExperiment();

  auto contextFactory = std::make_shared<pdt::factories::ContextFactory>(config);
  auto context = contextFactory->create(config->get<std::string>("experiment/context"));

  auto plannerFactory = std::make_shared<pdt::factories::PlannerFactory>(config, context);
  std::shared_ptr<ompl::base::Planner> planner;
  pdt::common::PLANNER_TYPE plannerType;
  std::tie(planner, plannerType, std::ignore) =
      plannerFactory->create(config->get<std::string>("experiment/planner"));

  // Get the experiment config.
  config->dumpAccessed();

  pdt::visualization::InteractiveVisualizer visualizer(config, context, {planner, plannerType});

  visualizer.run();

  return 0;
}
