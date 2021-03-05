#include <string>
#include <vector>

#include <ompl/util/Console.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include "esp_configuration/configuration.h"
#include "esp_configuration/directory.h"
#include "esp_factories/context_factory.h"

using namespace std::string_literals;
using namespace esp::ompltools;
using namespace ompl::base;
namespace fs = std::experimental::filesystem;

TEST_CASE("Optimization objectives") {
  // Only print warnings and errors.
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_WARN);

  // Instantiate an empty configuration.
  const char* argv[] = {"test_esp_configuration\0"};
  const int   argc   = sizeof(argv) / sizeof(char*) - 1;
  auto config = std::make_shared<Configuration>(argc, argv);
  config->clear();

  // Get the path of the test configurations.
  const auto configsDir =
    Directory::SOURCE / "test/esp_optimization_objectives/configs";

  SUBCASE("Path length") {
    // Load the test configuration.
    CHECK(fs::exists(configsDir / "path_length.json"));
    config->load(configsDir / "path_length.json");

    // Prepare the context creation.
    ContextFactory factory(config);
    const std::vector<std::string> contextNames
      {"test2d", "test4d", "test8d", "test16d", "test32d"};

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
      double sumOfSquares { 0.0 };
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
    ContextFactory factory(config);
    const std::vector<std::string> contextNames
      {"test2d", "test4d", "test8d", "test16d", "test32d"};

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

      // Get more convenient access to problem components.
      const auto objective = context->getObjective();
      const auto spaceInfo = context->getSpaceInformation();
      const auto checker   = spaceInfo->getStateValidityChecker();

      // Check trivial costs.
      CHECK(objective->motionCost(s1, s2).value() == 0.0);
      CHECK(objective->motionCostHeuristic(s1, s2).value() == 0.0);

      // Check cost parallel to obstacle.
      /*    s1    s2    The cost should be:
       *     x----x     integral from 0 to 0.2 of 1 / 0.1 dt.
       *   +--------+   So 10 * 0.2 = 2
       *   |        |
       *   |  obs.  |
       *   |        |
       *   +--------+
       */
      (*s1)[0u] =  0.1;
      (*s2)[0u] = -0.1;
      (*s1)[1u] =  0.3;
      (*s2)[1u] =  0.3;

      // Make sure the setup is actually what's pictured above.
      CHECK(spaceInfo->distance(s1, s2) == doctest::Approx(0.2));
      CHECK(checker->clearance(s1) == doctest::Approx(0.1));
      CHECK(checker->clearance(s2) == doctest::Approx(0.1));

      // Make sure the cost is as expected.
      CHECK(objective->motionCost(s1, s2).value() == doctest::Approx(2.0));

      // Make sure the heuristic is as expected.
      const auto heuristicParallel =
        std::log(std::pow((0.1 + 0.1 + 0.2), 2.0) / (4 * 0.1 * 0.1));
      CHECK(objective->motionCostHeuristic(s1, s2).value() == doctest::Approx(heuristicParallel));

      // Make sure the heuristic is admissible
      CHECK_FALSE(objective->isCostBetterThan(objective->motionCost(s1, s2),
                                              objective->motionCostHeuristic(s1, s2)));

      // Check cost perpendicular to obstacle.
      /*                        The cost should be:
       *   +--------+           integral from 0 to 0.2 of 1 / (t + 0.1) dt.
       *   |        | s1    s2  So [ln(t + 0.1)|_0^0.2 = ln(0.3) - ln(0.1)
       *   |  obs.  |  x----x                          = ln(3)
       *   |        |
       *   +--------+
       */
      (*s1)[0u] = -0.3;
      (*s2)[0u] = -0.5;
      (*s1)[1u] =  0.0;
      (*s2)[1u] =  0.0;

      // Make sure the setup is actually what's pictured above.
      CHECK(spaceInfo->distance(s1, s2) == doctest::Approx(0.2));
      CHECK(checker->clearance(s1) == doctest::Approx(0.1));
      CHECK(checker->clearance(s2) == doctest::Approx(0.3));

      // Make sure the cost is as expected.
      CHECK(objective->motionCost(s1, s2).value() == doctest::Approx(std::log(3)));

      // Make sure the heuristic is as expected.
      const auto heuristicPerpendicular = std::log(3);
      CHECK(objective->motionCostHeuristic(s1, s2).value() == doctest::Approx(heuristicPerpendicular));

      // Make sure the heuristic is admissible
      CHECK_FALSE(objective->isCostBetterThan(objective->motionCost(s1, s2),
                                              objective->motionCostHeuristic(s1, s2)));

      // Check cost at angle to obstacle.
      /*                        The cost should be:
       *   +--------+ s1        integral from 0 to sqrt(0.08) of 1 / ((sqrt(2)/2) t + 0.1) dt.
       *   |        |  x        So [2/sqrt(2) * ln((sqrt(2)/2)t + 0.1)|_0^0.2 
       *   |  obs.  |   `-_           = 2/sqrt(2) * ln(10 * (sqrt(2)/2 * sqrt(0.08) + 0.1)).
       *   |        |      `x
       *   +--------+       s2
       */
      (*s1)[0u] = -0.3;
      (*s2)[0u] = -0.5;
      (*s1)[1u] =  0.1;
      (*s2)[1u] = -0.1;

      // Make sure the setup is actually what's pictured above.
      CHECK(spaceInfo->distance(s1, s2) == doctest::Approx(std::sqrt(0.08)));
      CHECK(checker->clearance(s1) == doctest::Approx(0.1));
      CHECK(checker->clearance(s2) == doctest::Approx(0.3));

      // Make sure the cost is as expected.
      CHECK(objective->motionCost(s1, s2).value() ==
            doctest::Approx(2.0/std::sqrt(2.0) *
                            std::log(10.0 * (std::sqrt(2) /
                                             2.0 * std::sqrt(0.08) + 0.1))));

      // // Make sure the heuristic is as expected.
      const auto heuristicAngle = std::log(std::pow(0.1 + 0.3 + std::sqrt(0.08), 2.0) /
                                           (4.0 * 0.1 * 0.3));
      CHECK(objective->motionCostHeuristic(s1, s2).value() == doctest::Approx(heuristicAngle));

      // Make sure the heuristic is admissible
      CHECK_FALSE(objective->isCostBetterThan(objective->motionCost(s1, s2),
                                              objective->motionCostHeuristic(s1, s2)));

      // Check the admissibility of the heuristic for 1000 random edges.
      const auto sampler = spaceInfo->allocStateSampler();
      sampler->setLocalSeed(42u);  // The tests should never fail/succeed randomly.
      for (auto i = 0u; i < 1000u; ++i) {
        sampler->sampleUniform(s1);
        sampler->sampleUniform(s2);
        // Make sure the heuristic is admissible
        CHECK_FALSE(objective->isCostBetterThan(objective->motionCost(s1, s2),
                                                objective->motionCostHeuristic(s1, s2)));
      }
      
    }
  }
}
