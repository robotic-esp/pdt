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

#include <string>
#include <vector>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/Console.h>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include "pdt/config/configuration.h"
#include "pdt/config/directory.h"
#include "pdt/factories/context_factory.h"
#include "pdt/objectives/reciprocal_clearance_optimization_objective.h"

using namespace std::string_literals;
using namespace ompl::base;
namespace fs = std::experimental::filesystem;

TEST_CASE("Optimization objectives") {
  // Only print warnings and errors.
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_WARN);

  // Instantiate an empty configuration.
  const char* argv[] = {"test_pdt_objectives\0"};
  const int argc = sizeof(argv) / sizeof(char*) - 1;
  auto config = std::make_shared<pdt::config::Configuration>(argc, argv);
  config->clear();

  // Get the path of the test configurations.
  const auto configsDir = pdt::config::Directory::SOURCE / "test/pdt_objectives/configs";

  SUBCASE("Path length") {
    // Load the test configuration.
    CHECK(fs::exists(configsDir / "path_length.json"));
    config->load(configsDir / "path_length.json");

    // Prepare the context creation.
    pdt::factories::ContextFactory factory(config);
    const std::vector<std::string> contextNames{"test2d", "test4d", "test8d", "test16d", "test32d"};

    // Loop over all contexts.
    for (const auto& name : contextNames) {
      // Create the context.
      auto context = factory.create(name);

      // Allocate test states.
      auto s1 = context->getSpaceInformation()->allocState()->as<RealVectorStateSpace::StateType>();
      auto s2 = context->getSpaceInformation()->allocState()->as<RealVectorStateSpace::StateType>();
      for (auto i = 0u; i < context->getDimension(); ++i) {
        (*s1)[i] = 0.0;
        (*s2)[i] = 0.0;
      }

      // Get the optimization objective.
      auto objective = context->getObjective();

      // Check trivial costs.
      CHECK(objective->motionCost(s1, s2).value() == 0.0);
      CHECK(objective->motionCostHeuristic(s1, s2).value() == 0.0);

      // Check cost along each single dimension.
      for (auto i = 0u; i < context->getDimension(); ++i) {
        (*s2)[i] = 0.25;
        CHECK(objective->motionCost(s1, s2).value() == 0.25);
        CHECK(objective->motionCostHeuristic(s1, s2).value() == 0.25);
        (*s2)[i] = 0.0;
      }

      // Check cost along all dimensions.
      double sumOfSquares{0.0};
      for (auto i = 0u; i < context->getDimension(); ++i) {
        (*s2)[i] = 0.25;
        sumOfSquares += 0.25 * 0.25;
        CHECK(objective->motionCost(s1, s2).value() == sqrt(sumOfSquares));
        CHECK(objective->motionCostHeuristic(s1, s2).value() == sqrt(sumOfSquares));
      }
    }
  }

  SUBCASE("Reciprocal obstacle clearance") {
    // Load the test configuration.
    CHECK(fs::exists(configsDir / "obstacle_clearance.json"));
    config->load(configsDir / "obstacle_clearance.json");

    // Prepare the context creation.
    pdt::factories::ContextFactory factory(config);
    const std::vector<std::string> contextNames{"test2d", "test4d", "test8d", "test16d", "test32d"};

    // Loop over all contexts.
    for (const auto& name : contextNames) {
      SUBCASE(name.c_str()) {
        // Create the context.
        auto context = factory.create(name);

        // Allocate test states.
        auto s1 =
            context->getSpaceInformation()->allocState()->as<RealVectorStateSpace::StateType>();
        auto s2 =
            context->getSpaceInformation()->allocState()->as<RealVectorStateSpace::StateType>();
        for (auto i = 0u; i < context->getDimension(); ++i) {
          (*s1)[i] = 0.0;
          (*s2)[i] = 0.0;
        }

        // Get more convenient access to problem components.
        const auto spaceInfo = context->getSpaceInformation();
        const auto checker = spaceInfo->getStateValidityChecker();
        const auto objective1 =
            pdt::objectives::ReciprocalClearanceOptimizationObjective(spaceInfo, {0.0, 1.0});
        const auto objective2 =
            pdt::objectives::ReciprocalClearanceOptimizationObjective(spaceInfo, {0.0, 0.5, 1.0});
        const auto objective3 =
            pdt::objectives::ReciprocalClearanceOptimizationObjective(spaceInfo, 0.5);

        SUBCASE("Trivial costs") {
          SUBCASE("Objective 1") {
            CHECK(objective1.motionCost(s1, s2).value() == 0.0);
            CHECK(objective1.motionCostHeuristic(s1, s2).value() == 0.0);
          }
          SUBCASE("Objective 2") {
            CHECK(objective2.motionCost(s1, s2).value() == 0.0);
            CHECK(objective2.motionCostHeuristic(s1, s2).value() == 0.0);
          }
          SUBCASE("Objective 3") {
            CHECK(objective3.motionCost(s1, s2).value() == 0.0);
            CHECK(objective3.motionCostHeuristic(s1, s2).value() == 0.0);
          }
        }

        SUBCASE("Parallel edge") {
          // Check cost parallel to obstacle.
          /*    s1    s2
           *     x----x
           *   +--------+
           *   |        |
           *   |  obs.  |
           *   |        |
           *   +--------+
           */
          (*s1)[0u] = 0.1;
          (*s2)[0u] = -0.1;
          (*s1)[1u] = 0.3;
          (*s2)[1u] = 0.3;

          // Make sure the setup is actually what's pictured above.
          CHECK(spaceInfo->distance(s1, s2) == doctest::Approx(0.2));
          CHECK(checker->clearance(s1) == doctest::Approx(0.1));
          CHECK(checker->clearance(s2) == doctest::Approx(0.1));

          // Make sure the cost is as expected.
          SUBCASE("Objective 1") {
            // The cost should be equal to the integral from 0 to 0.2 of 1 / 0.1 dt.
            // So 10 * 0.2 = 2.
            CHECK(objective1.motionCost(s1, s2).value() == doctest::Approx(2.0));

            // The heuristic should be equal to
            // ln((0.1 + 0.1 + 0.2)^2 / (4.0 * 0.1 * 0.1))) = ln(0.16 / 0.04) = ln(4).
            CHECK(objective1.motionCostHeuristic(s1, s2).value() == doctest::Approx(std::log(4.0)));

            // Make sure the heuristic is admissible
            CHECK_FALSE(objective1.isCostBetterThan(objective1.motionCost(s1, s2),
                                                    objective1.motionCostHeuristic(s1, s2)));
          }

          SUBCASE("Ojective 2") {
            // The cost should still be the same, so 10 * 0.2 = 2.
            CHECK(objective2.motionCost(s1, s2).value() == doctest::Approx(2.0));

            // The heuristic should be equal to
            // l = 0.2, t1 = 0, t2 = 0.1, t3 = 0.2, c1 = c2 = c3 = 0.1.
            // ln((t1 + c1) / c1) = 0
            // + ln((c1 + c2 - (t1 - t2))^2 / (4 * c1 * c2)) = ln(9/4)
            // + ln((c2 + c3 - (t2 - t3))^2 / (4 * c2 * c3)) = ln(9/4)
            // + ln((l - t3 + c3) / c3) = 0
            // = 2 * ln(9/4)
            CHECK(objective2.motionCostHeuristic(s1, s2).value() ==
                  doctest::Approx(2.0 * std::log(9.0 / 4.0)));

            // Make sure the heuristic is admissible
            CHECK_FALSE(objective1.isCostBetterThan(objective1.motionCost(s1, s2),
                                                    objective1.motionCostHeuristic(s1, s2)));
          }

          // Cost of segment changes for segments where it should not change..
          // 0.00564175268217
          // 0.0056417526832828026
          // 0.0056417526832823611 ?

          SUBCASE("Objective 3") {
            // The cost should still be the same, so 10 * 0.2 = 2.
            CHECK(objective3.motionCost(s1, s2).value() == doctest::Approx(2.0));
            const auto numSegments = spaceInfo->getStateSpace()->validSegmentCount(s1, s2);
            const auto numSamples =
                std::max(2u, static_cast<unsigned>(std::ceil((numSegments + 1u) / 2.0)));

            // The first and last term will be zero, as the first and last state will be at the
            // boundaries. The terms in between will all be the same because the clearance is the
            // same for all states along the edge, and the states are evenly spaced
            // (0.2 / (numSamples - 1) apart).
            // The middle terms are therefore all ln((c1 + c2 - (t1 - t2))^2 / (4 * c1 * c2))
            // = ln((0.1 + 0.1 + 0.2/(numSamples - 1))^2 / (4 * 0.1 * 0.1))
            // The whole approximation therefore must be:
            // numSamples * ln((0.2 + 0.2/(numSamples - 1))^2 / 0.04).
            // Note that the validSegmentCount depends on the collision detection resolution as
            // well as the state space dimension.
            CHECK(objective3.motionCostHeuristic(s1, s2).value() ==
                  doctest::Approx((numSamples - 1) *
                                  std::log(std::pow(0.2 + 0.2 / (numSamples - 1), 2.0) / 0.04)));
          }
        }

        SUBCASE("Perpendicular edge") {
          // Check cost perpendicular to obstacle.
          /*
           *   +--------+
           *   |        | s1    s2
           *   |  obs.  |  x----x
           *   |        |
           *   +--------+
           */
          (*s1)[0u] = -0.3;
          (*s2)[0u] = -0.5;
          (*s1)[1u] = 0.0;
          (*s2)[1u] = 0.0;

          // Make sure the setup is actually what's pictured above.
          CHECK(spaceInfo->distance(s1, s2) == doctest::Approx(0.2));
          CHECK(checker->clearance(s1) == doctest::Approx(0.1));
          CHECK(checker->clearance(s2) == doctest::Approx(0.3));

          SUBCASE("Objective 1") {
            // Make sure the cost is as expected. The cost should be equal to
            // the integral from 0 to 0.2 of 1 / (t + 0.1) dt
            // = [ln(t + 0.1)|_0^0.2 = ln(0.3) - ln(0.1) = ln(3).
            CHECK(objective1.motionCost(s1, s2).value() == doctest::Approx(std::log(3)));

            // Make sure the heuristic is as expected. Since this scenario is exactly
            // the assumption of the heuristic, the heuristic should be equal to the cost.
            CHECK(objective1.motionCostHeuristic(s1, s2).value() == doctest::Approx(std::log(3)));

            // Make sure the heuristic is admissible
            CHECK_FALSE(objective1.isCostBetterThan(objective1.motionCost(s1, s2),
                                                    objective1.motionCostHeuristic(s1, s2)));
          }

          SUBCASE("Objective 2") {
            // Make sure the cost is as expected. The cost should be equal to
            // the integral from 0 to 0.2 of 1 / (t + 0.1) dt
            // = [ln(t + 0.1)|_0^0.2 = ln(0.3) - ln(0.1) = ln(3).
            CHECK(objective2.motionCost(s1, s2).value() == doctest::Approx(std::log(3)));

            // Make sure the heuristic is as expected. Since this scenario is exactly
            // the assumption of the heuristic, the heuristic should be equal to the cost.
            CHECK(objective2.motionCostHeuristic(s1, s2).value() == doctest::Approx(std::log(3)));

            // Make sure the heuristic is admissible
            CHECK_FALSE(objective2.isCostBetterThan(objective1.motionCost(s1, s2),
                                                    objective1.motionCostHeuristic(s1, s2)));
          }

          SUBCASE("Objective 3") {
            // Make sure the cost is as expected. The cost should be equal to
            // the integral from 0 to 0.2 of 1 / (t + 0.1) dt
            // = [ln(t + 0.1)|_0^0.2 = ln(0.3) - ln(0.1) = ln(3).
            CHECK(objective3.motionCost(s1, s2).value() == doctest::Approx(std::log(3)));

            // Make sure the heuristic is as expected. Since this scenario is exactly
            // the assumption of the heuristic, the heuristic should be equal to the cost.
            CHECK(objective2.motionCostHeuristic(s1, s2).value() == doctest::Approx(std::log(3)));

            // Make sure the heuristic is admissible
            CHECK_FALSE(objective2.isCostBetterThan(objective1.motionCost(s1, s2),
                                                    objective1.motionCostHeuristic(s1, s2)));
          }
        }

        SUBCASE("Edge at angle") {
          // Check cost at angle to obstacle.
          /*
           *   +--------+ s1
           *   |        |  x
           *   |  obs.  |   `-_
           *   |        |      `x
           *   +--------+       s2
           */
          (*s1)[0u] = -0.3;
          (*s2)[0u] = -0.5;
          (*s1)[1u] = 0.1;
          (*s2)[1u] = -0.1;

          // Make sure the setup is actually what's pictured above.
          CHECK(spaceInfo->distance(s1, s2) == doctest::Approx(std::sqrt(0.08)));
          CHECK(checker->clearance(s1) == doctest::Approx(0.1));
          CHECK(checker->clearance(s2) == doctest::Approx(0.3));

          SUBCASE("Objective 1") {
            // Make sure the cost is as expected. The cost should be
            // integral from 0 to sqrt(0.08) of 1 / ((sqrt(2)/2) t + 0.1) dt
            // = [2/sqrt(2) * ln((sqrt(2)/2)t + 0.1)|_0^0.2
            // = 2/sqrt(2) * ln(10 * (sqrt(2)/2 * sqrt(0.08) + 0.1)).
            CHECK(objective1.motionCost(s1, s2).value() ==
                  doctest::Approx(2.0 / std::sqrt(2.0) *
                                  std::log(10.0 * (std::sqrt(2) / 2.0 * std::sqrt(0.08) + 0.1))));

            // Make sure the heuristic is as expected.
            const auto heuristicAngle =
                std::log(std::pow(0.1 + 0.3 + std::sqrt(0.08), 2.0) / (4.0 * 0.1 * 0.3));
            CHECK(objective1.motionCostHeuristic(s1, s2).value() ==
                  doctest::Approx(heuristicAngle));

            // Make sure the heuristic is admissible
            CHECK_FALSE(objective1.isCostBetterThan(objective1.motionCost(s1, s2),
                                                    objective1.motionCostHeuristic(s1, s2)));
          }

          SUBCASE("Objective 2") {
            // Make sure the cost is still as expected.
            CHECK(objective2.motionCost(s1, s2).value() ==
                  doctest::Approx(2.0 / std::sqrt(2.0) *
                                  std::log(10.0 * (std::sqrt(2) / 2.0 * std::sqrt(0.08) + 0.1))));

            // Make sure the heuristic is as expected.
            const auto heuristicAngle =
                std::log((std::pow(0.1 + 0.2 + std::sqrt(0.08) / 2.0, 2.0) / (4.0 * 0.1 * 0.2))) +
                std::log((std::pow(0.2 + 0.3 + std::sqrt(0.08) / 2.0, 2.0) / (4.0 * 0.2 * 0.3)));
            CHECK(objective2.motionCostHeuristic(s1, s2).value() ==
                  doctest::Approx(heuristicAngle));

            // Make sure the heuristic is admissible
            CHECK_FALSE(objective2.isCostBetterThan(objective2.motionCost(s1, s2),
                                                    objective2.motionCostHeuristic(s1, s2)));
          }

          SUBCASE("Objective 3") {
            // Make sure the cost is still as expected.
            CHECK(objective3.motionCost(s1, s2).value() ==
                  doctest::Approx(2.0 / std::sqrt(2.0) *
                                  std::log(10.0 * (std::sqrt(2) / 2.0 * std::sqrt(0.08) + 0.1))));

            // Make sure the heuristic is as expected.
            const auto numSegments = spaceInfo->getStateSpace()->validSegmentCount(s1, s2);
            const auto numSamples =
                std::max(2u, static_cast<unsigned>(std::ceil((numSegments + 1u) / 2.0)));

            // The first and last term will be zero, as the first and last state will bet at the
            // boundaries. The term clearance of the terms in between will all linearly scale with
            // the distance along the edge. The states are evenly spaced (sqrt(0.08) / numSamples -
            // 1 apart. The clearance of the i-th state is 0.1 + i * 0.2 / (numSamples - 1.0).
            auto heuristicAngle = 0.0;
            for (auto i = 0u; i < numSamples - 1u; ++i) {
              heuristicAngle += std::log(std::pow(0.1 + i * 0.2 / (numSamples - 1.0) + 0.1 +
                                                      (i + 1) * 0.2 / (numSamples - 1.0) +
                                                      std::sqrt(0.08) / (numSamples - 1),
                                                  2.0) /
                                         (4.0 * (0.1 + i * 0.2 / (numSamples - 1.0)) *
                                          (0.1 + (i + 1) * 0.2 / (numSamples - 1.0))));
            }

            CHECK(objective3.motionCostHeuristic(s1, s2).value() ==
                  doctest::Approx(heuristicAngle));

            // Make sure the heuristic is admissible
            CHECK_FALSE(objective3.isCostBetterThan(objective3.motionCost(s1, s2),
                                                    objective3.motionCostHeuristic(s1, s2)));
          }
        }

        SUBCASE("1000 Random edges") {
          // Check the admissibility of the heuristic for 1000 random edges per objective.
          const auto sampler = spaceInfo->allocStateSampler();
#ifdef PDT_EXTRA_SET_LOCAL_SEEDS
          sampler->setLocalSeed(42u);  // The tests should never fail/succeed randomly.
#else
          OMPL_WARN(
              "PDT was compiled without support for setting local seeds which makes this unit test "
              "random.");
#endif  // #ifdef PDT_EXTRA_SET_LOCAL_SEEDS
          for (auto i = 0u; i < 1000u; ++i) {
            sampler->sampleUniform(s1);
            sampler->sampleUniform(s2);
            // Make sure the heuristic is admissible for all objectives.
            CHECK_FALSE(objective1.isCostBetterThan(objective1.motionCost(s1, s2),
                                                    objective1.motionCostHeuristic(s1, s2)));
            CHECK_FALSE(objective2.isCostBetterThan(objective2.motionCost(s1, s2),
                                                    objective2.motionCostHeuristic(s1, s2)));
            CHECK_FALSE(objective3.isCostBetterThan(objective3.motionCost(s1, s2),
                                                    objective3.motionCostHeuristic(s1, s2)));
          }
        }
      }
    }
  }
}
