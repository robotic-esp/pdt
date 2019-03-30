/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2017     University of Toronto
 *  Copyright (c) 2018-present  University of Oxford
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
 *   * Neither the names of the copyright holders nor the names of its
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

#include "esp_common/context_type.h"
#include "esp_common/planner_type.h"
#include "esp_configuration/configuration.h"
#include "esp_factories/context_factory.h"
#include "esp_factories/planner_factory.h"
#include "esp_planning_contexts/all_contexts.h"
#include "esp_visualization/interactive_visualizer.h"

int main(int argc, char** argv) {
  // Load the config.
  auto config = std::make_shared<esp::ompltools::Configuration>(argc, argv);

  // Get the experiment config.
  auto experimentConfig = config->getExperimentConfig();

  // Create the context for this experiment.
  esp::ompltools::ContextFactory contextFactory(config);
  auto context = contextFactory.create(experimentConfig["context"]);

  // Create a planner factory for planners in this context.
  esp::ompltools::PlannerFactory plannerFactory(config, context);
  auto [planner, plannerType] = plannerFactory.create(experimentConfig["planner"]);

  esp::ompltools::InteractiveVisualizer visualizer(context, {planner, plannerType});

  visualizer.run();

  return 0;
}
