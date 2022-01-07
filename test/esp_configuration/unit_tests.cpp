#include <experimental/filesystem>
#include <vector>

#include <ompl/util/Console.h>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include "esp_configuration/configuration.h"
#include "esp_configuration/directory.h"

using namespace std::string_literals;
using namespace esp::ompltools;
namespace fs = std::experimental::filesystem;

TEST_CASE("Configuration") {
  // Only print warnings and errors.
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_WARN);

  // Instantiate an empty configuration.
  const char* argv[] = {"test_esp_configuration\0"};
  const int   argc   = sizeof(argv) / sizeof(char*) - 1;
  Configuration config(argc, argv);
  config.clear();

  // Get the path of the test configurations.
  const auto configsDir =
    Directory::SOURCE / "test/esp_configuration/configs";

  SUBCASE("Empty configuration") {
    // Check query behaviour of nonexistent parameter.
    CHECK(config.contains("No") == false);
    CHECK(config.contains("No/No") == false);
    CHECK(config.contains("No/No/No") == false);

    // Add parameters.
    config.add("first", 1);
    config.add("second/level", 2);
    config.add("third/level/down", 3);
    CHECK(config.contains("first") == true);
    CHECK(config.get<int>("first") == 1);
    CHECK(config.contains("second/level") == true);
    CHECK(config.get<int>("second/level") == 2);
    CHECK(config.contains("third/level/down") == true);
    CHECK(config.get<int>("third/level/down") == 3);
  }

  SUBCASE("Basic configuration") {
    // Load the basic fonciguration.
    CHECK(fs::exists(configsDir / "basic.json"));
    config.load(configsDir / "basic.json");
    
    // Make sure the config was loaded correctly.
    CHECK(config.get<std::string>("Hello") == std::string("World"));
    CHECK(config.get<std::string>("You/are") == std::string("beautiful!"));
    CHECK(config.get<int>("Because of/Integers") == 42);
    CHECK(config.get<float>("Because of/Floats") == 4.2f);
    CHECK(config.get<double>("Because of/Doubles") == 4.2);
    CHECK(config.get<std::vector<int>>("Because of/Vectors") == std::vector<int>{3, 1, 4, 1, 5, 9});
  }
}
