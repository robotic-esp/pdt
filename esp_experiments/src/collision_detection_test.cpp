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

#include <iostream>

#include "esp_configuration/configuration.h"
#include "esp_factories/context_factory.h"
#include "esp_planning_contexts/all_contexts.h"

using namespace std::string_literals;

constexpr double groundTruthResolution = 1e-7;

int main(int argc, char** argv) {
  // Read the config files.
  auto config = std::make_shared<esp::ompltools::Configuration>(argc, argv);
  config->registerAsExperiment();

  // Get the candidate collision detection resolutions.
  auto candidateResolutions = config->get<std::vector<double>>("experiment/candidateResolutions");

  std::cout << "seed: " << config->get<std::size_t>("experiment/seed") << std::endl;

  std::cout << "Testing a total of "
            << config->get<std::size_t>("experiment/numContexts") *
                   config->get<std::size_t>("experiment/numEdges")
            << " edges from " << config->get<std::size_t>("experiment/numContexts") << " contexts."
            << std::endl;

  config->dumpAccessed();

  // Prepare the results. The indices will correlate with the candidate resolution indices.
  std::vector<std::size_t> numFalseNegatives(candidateResolutions.size(), 0u);

  // It seems the best way to check at different resolutions is by setting the 'valid segment count
  // factor' through 'setValidSegmentCountFactor'. The problem with 'setValidSegmentCount is that
  // we'd have to call setup on the state space every time the collision check resolution is
  // changed.
  // If we set the base valid segment length to 1 and take the reciprocal value of the canidate
  // resolutions as the valid segment count factor, we should get what we want.

  std::size_t numTestedEdges = 0u;
  for (std::size_t c = 0u; c < config->get<std::size_t>("experiment/numContexts"); ++c) {
    // Create the context.
    esp::ompltools::ContextFactory contextFactory(config);
    auto context = contextFactory.create(config->get<std::string>("experiment/context"));

    // Get the space info.
    auto spaceInfo = context->getSpaceInformation();

    // Get a state sampler.
    auto sampler = spaceInfo->allocStateSampler();

    // Allocate two states.
    auto state1 = spaceInfo->allocState();
    auto state2 = spaceInfo->allocState();

    // Test edges.
    std::size_t numEdgesOnContext = 0u;
    while (numEdgesOnContext < config->get<std::size_t>("experiment/numEdges")) {
      // Sample the first state.
      do {
        sampler->sampleUniform(state1);
      } while (!spaceInfo->isValid(state1));

      // Sample the second state.
      do {
        sampler->sampleUniformNear(state2, state1, 0.2);
      } while (!spaceInfo->isValid(state2));

      spaceInfo->setStateValidityCheckingResolution(groundTruthResolution);
      spaceInfo->setup();
      if (!spaceInfo->checkMotion(state1, state2)) {
        ++numTestedEdges;
        ++numEdgesOnContext;
        for (std::size_t j = 0u; j < candidateResolutions.size(); ++j) {
          // Check the invalid edge with each candidate resolution.

          spaceInfo->setStateValidityCheckingResolution(candidateResolutions[j]);
          spaceInfo->setup();
          if (spaceInfo->checkMotion(state1, state2)) {
            // The edge is invalid, but this resolution did not catch it.
            ++numFalseNegatives[j];
          }
        }
        if (numTestedEdges % 5 == 0) {
          std::cout << "Tested a total of " << numTestedEdges << " edges from " << c + 1
                    << " contexts." << std::endl;
          for (std::size_t i = 0u; i < candidateResolutions.size(); ++i) {
            std::cout << "  " << std::fixed << candidateResolutions[i] << "  "
                      << numFalseNegatives[i] << " / " << numTestedEdges << " -> "
                      << static_cast<double>(numFalseNegatives[i]) / numTestedEdges << "\n";
          }
        }
      }
    }

    // Free the allocated states.
    spaceInfo->freeState(state1);
    spaceInfo->freeState(state2);
  }

  std::cout << "\nFinal Results:\n\n";
  std::cout.width(10);
  for (std::size_t i = 0u; i < candidateResolutions.size(); ++i) {
    std::cout << "Resolution: " << std::fixed << candidateResolutions[i] << "\t"
              << numFalseNegatives[i] << " / " << config->get<std::size_t>("experiment/numEdges")
              << " -> " << static_cast<double>(numFalseNegatives[i]) / numTestedEdges << "\n\n";
  }

  return 0;
}
