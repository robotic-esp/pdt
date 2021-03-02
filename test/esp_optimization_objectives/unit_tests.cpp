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
}
