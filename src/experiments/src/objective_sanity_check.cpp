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
  // Load the config.
  auto config = std::make_shared<pdt::config::Configuration>(argc, argv);
  config->registerAsExperiment();

  // Create context factory.
  pdt::factories::ContextFactory contextFactory(config);

  // Create an accumulator set to keep track of some statistics.
  AccumulatorSet accuracyStats;

  // Keep track of how many edges were valid.
  auto numEdges = 0u;
  auto numValidEdges = 0u;

  // Let's test.
  for (auto i = 0u; i < config->get<std::size_t>("experiment/numContexts"); ++i) {
    // Create a new context and an associated problem.
    auto context = contextFactory.create(config->get<std::string>("experiment/context"));
    auto problem = context->instantiateNewProblemDefinition();
    auto objective = problem->getOptimizationObjective();

    // Get the space info and state sampler.
    auto spaceInfo = context->getSpaceInformation();
    auto sampler = spaceInfo->allocStateSampler();

    // Allocate two states.
    auto state1 = spaceInfo->allocState();
    auto state2 = spaceInfo->allocState();

    // Test edge costs.
    for (auto ii = 0u; ii < config->get<std::size_t>("experiment/numEdges"); ++ii) {
      if (numEdges % 1000 == 0u) {
        std::cout << "Tested " << numEdges << " edges.\n";
      }

      // Sample two valid states.
      do {
        sampler->sampleUniform(state1);
      } while (!spaceInfo->isValid(state1));
      do {
        sampler->sampleUniform(state2);
      } while (!spaceInfo->isValid(state2));

      // Compute the true and heuristic costs between the states.
      auto trueCost = objective->motionCost(state1, state2);
      auto heuristicCost = objective->motionCostHeuristic(state1, state2);

      if (objective->isCostBetterThan(trueCost, heuristicCost)) {
        config->dumpAccessed();
        std::cout << std::boolalpha << std::setprecision(12);
        std::cout << "Cost: " << trueCost << ", heuristic: " << heuristicCost << '\n';
        std::cout << "Edge index (i/ii): " << i << '/' << ii << '\n';
        std::cout << "Edge valid: " << spaceInfo->getMotionValidator()->checkMotion(state1, state2)
                  << '\n';
        std::cout << "Distance: " << spaceInfo->distance(state1, state2) << '\n';
        std::cout << "Cost 1: " << objective->stateCost(state1) << '\n';
        std::cout << "Cost 2: " << objective->stateCost(state2) << '\n';
        std::cout << "Clearance 1: " << spaceInfo->getStateValidityChecker()->clearance(state1)
                  << '\n';
        std::cout << "Clearance 2: " << spaceInfo->getStateValidityChecker()->clearance(state2)
                  << '\n';
        OMPL_WARN("Found edge with inadmissible cost.");
      }
      if (spaceInfo->checkMotion(state1, state2)) {
        ++numValidEdges;
        accuracyStats(heuristicCost.value() / trueCost.value());
      }

      // Keep track of the tested edges.
      ++numEdges;
    }

    // Free the allocated states.
    spaceInfo->freeState(state1);
    spaceInfo->freeState(state2);
  }

  config->dumpAccessed();

  auto numTestedEdges = config->get<std::size_t>("experiment/numContexts") *
                        config->get<std::size_t>("experiment/numEdges");

  std::cout << "Num tested edges: " << numTestedEdges << ", Num valid edges: " << numValidEdges
            << " ("
            << static_cast<float>(numValidEdges) / static_cast<float>(numTestedEdges) * 100.0f
            << ")\n";
  std::cout << "Accuracy [mean, median, stddev, min, max]:\n"
            << boost::accumulators::extract_result<boost::accumulators::tag::mean>(accuracyStats)
            << ", "
            << boost::accumulators::extract_result<boost::accumulators::tag::median>(accuracyStats)
            << ", "
            << boost::accumulators::extract_result<boost::accumulators::tag::variance>(
                   accuracyStats)
            << ", "
            << boost::accumulators::extract_result<boost::accumulators::tag::min>(accuracyStats)
            << ", "
            << boost::accumulators::extract_result<boost::accumulators::tag::max>(accuracyStats)
            << '\n';

  return 0;
}
