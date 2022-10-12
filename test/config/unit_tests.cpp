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

#include <experimental/filesystem>
#include <vector>

#include <ompl/util/Console.h>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include "pdt/config/configuration.h"
#include "pdt/config/directory.h"

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

TEST_CASE("Configuration") {
  // Only print warnings and errors.
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_WARN);

  // Instantiate an empty configuration.
  const char* argv[] = {"test_pdt_config\0"};
  const int argc = sizeof(argv) / sizeof(char*) - 1;
  pdt::config::Configuration config(argc, argv);
  config.clear();

  // Get the path of the test configurations.
  const auto configsDir = pdt::config::Directory::SOURCE / "test/pdt_config/configs";

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
