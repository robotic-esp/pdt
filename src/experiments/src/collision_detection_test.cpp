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

#include <iostream>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

#include "pdt/config/configuration.h"
#include "pdt/factories/context_factory.h"
#include "pdt/planning_contexts/all_contexts.h"

using namespace std::string_literals;

// TODO(Marlin): The value type should really be an integer. However, the
// 'max' tag returns the max value that can be safed in the int type instead
// of the max value of the measurements.
using AccumulatorSet = boost::accumulators::accumulator_set<
    double, boost::accumulators::features<
                boost::accumulators::tag::sum, boost::accumulators::tag::min,
                boost::accumulators::tag::max, boost::accumulators::tag::mean,
                boost::accumulators::tag::median, boost::accumulators::tag::lazy_variance>>;

int main(const int argc, const char** argv) {
  // Read the config files.
  auto config = std::make_shared<pdt::config::Configuration>(argc, argv);
  config->registerAsExperiment();

  // Get the ground truth resolution.
  auto groundTruthResolution = config->get<double>("experiment/groundTruthResolution");

  // Get the candidate collision detection resolutions.
  auto candidateResolutions = config->get<std::vector<double>>("experiment/candidateResolutions");

  std::cout << "seed: " << config->get<std::size_t>("experiment/seed") << std::endl;

  std::cout << "Testing a total of "
            << config->get<std::size_t>("experiment/numContexts") *
                   config->get<std::size_t>("experiment/numEdges")
            << " edges from " << config->get<std::size_t>("experiment/numContexts") << " contexts."
            << std::endl;

  // Prepare the false negative results. The indices will correlate with the candidate resolution
  // indices.
  std::vector<std::size_t> falseNegativesResults(candidateResolutions.size(), 0u);

  // Prepare the timing results. The indices will correlate with the candidate resolution indices
  // First element is for valid edges, second for invalid edges.
  std::vector<std::pair<AccumulatorSet, AccumulatorSet>> timingResults(
      candidateResolutions.size(), std::make_pair(AccumulatorSet{}, AccumulatorSet{}));
  std::pair<AccumulatorSet, AccumulatorSet> groundTruthTimingResults;

  std::size_t numTestedEdges = 0u;
  std::size_t numTestedValid = 0u;
  std::size_t numTestedInvalid = 0u;
  for (std::size_t c = 0u; c < config->get<std::size_t>("experiment/numContexts"); ++c) {
    // Create the context.
    pdt::factories::ContextFactory contextFactory(config);
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
      ++numTestedEdges;
      ++numEdgesOnContext;

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

      const auto start = pdt::time::Clock::now();
      bool isValid = spaceInfo->checkMotion(state1, state2);
      const auto stop = pdt::time::Clock::now();

      if (isValid) {
        ++numTestedValid;
        groundTruthTimingResults.first(
            std::chrono::duration_cast<pdt::time::Duration>(stop - start).count());
        for (std::size_t j = 0u; j < candidateResolutions.size(); ++j) {
          spaceInfo->setStateValidityCheckingResolution(candidateResolutions[j]);
          spaceInfo->setup();

          const auto start = pdt::time::Clock::now();
          spaceInfo->checkMotion(state1, state2);
          const auto stop = pdt::time::Clock::now();

          timingResults[j].first(
              std::chrono::duration_cast<pdt::time::Duration>(stop - start).count());
        }
      } else {
        ++numTestedInvalid;
        groundTruthTimingResults.second(
            std::chrono::duration_cast<pdt::time::Duration>(stop - start).count());

        for (std::size_t j = 0u; j < candidateResolutions.size(); ++j) {
          // Check the invalid edge with each candidate resolution.

          spaceInfo->setStateValidityCheckingResolution(candidateResolutions[j]);
          spaceInfo->setup();

          const auto start = pdt::time::Clock::now();
          bool isValid = spaceInfo->checkMotion(state1, state2);
          const auto stop = pdt::time::Clock::now();

          if (isValid) {
            // The edge is invalid, but this resolution did not catch it.
            ++falseNegativesResults[j];
          }

          timingResults[j].second(
              std::chrono::duration_cast<pdt::time::Duration>(stop - start).count());
        }
      }

      if (numTestedEdges % 5 == 0) {
        std::cout << "Tested a total of " << numTestedEdges << " edges (valid: " << numTestedValid
                  << ", invalid: " << numTestedInvalid << ") from " << c + 1 << " contexts."
                  << std::endl;
        for (std::size_t i = 0u; i < candidateResolutions.size(); ++i) {
          std::cout << "  " << std::fixed << candidateResolutions[i] << "  "
                    << falseNegativesResults[i] << " / " << numTestedInvalid << " -> "
                    << static_cast<double>(falseNegativesResults[i]) /
                           static_cast<double>(numTestedInvalid)
                    << "\n";
        }
      }
    }

    // Free the allocated states.
    spaceInfo->freeState(state1);
    spaceInfo->freeState(state2);
  }

  std::cout << "\nFinal Results for " << numTestedEdges << " edges (valid: " << numTestedValid
            << ", invalid: " << numTestedInvalid << ")\n\n";
  std::cout.width(10);
  for (std::size_t i = 0u; i < candidateResolutions.size(); ++i) {
    std::cout << "Resolution: " << std::fixed << candidateResolutions[i]
              << "\n\tFalse Neg: " << falseNegativesResults[i] << "\tFalse Neg [%]: "
              << static_cast<float>(falseNegativesResults[i]) / static_cast<float>(numTestedInvalid)
              << "\tValid Mean: "
              << boost::accumulators::extract_result<boost::accumulators::tag::mean>(
                     timingResults[i].first)
              << "\tValid Max: "
              << boost::accumulators::extract_result<boost::accumulators::tag::max>(
                     timingResults[i].first)
              << "\tValid Min: "
              << boost::accumulators::extract_result<boost::accumulators::tag::min>(
                     timingResults[i].first)
              << "\tValid Std: "
              << boost::accumulators::extract_result<boost::accumulators::tag::variance>(
                     timingResults[i].first)
              << "\tInvalid Mean "
              << boost::accumulators::extract_result<boost::accumulators::tag::mean>(
                     timingResults[i].second)
              << "\tInvalid Max: "
              << boost::accumulators::extract_result<boost::accumulators::tag::max>(
                     timingResults[i].second)
              << "\tInvalid Min: "
              << boost::accumulators::extract_result<boost::accumulators::tag::min>(
                     timingResults[i].second)
              << "\tInvalid Std: "
              << boost::accumulators::extract_result<boost::accumulators::tag::variance>(
                     timingResults[i].second)
              << "\n\n";
  }

  config->dumpAccessed();

  return 0;
}
