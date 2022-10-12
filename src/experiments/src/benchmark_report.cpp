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

#include <functional>
#include <iomanip>
#include <iostream>
#include <vector>

#include <experimental/filesystem>

#include "pdt/config/configuration.h"
#include "pdt/reports/multiquery_report.h"
#include "pdt/reports/single_query_report.h"
#include "pdt/statistics/planning_statistics.h"

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

int main(const int argc, const char** argv) {
  // Read the config files.
  auto config = std::make_shared<pdt::config::Configuration>(argc, argv);

  const std::vector<std::string> resultPaths =
      config->get<std::vector<std::string>>("experiment/results");
  fs::path reportPath;

  // Generate the statistic.
  std::vector<pdt::statistics::PlanningStatistics> stats;

  for (const auto& path : resultPaths) {
    stats.push_back(pdt::statistics::PlanningStatistics(config, path, true));
  }

  // Inform that the report is being compiled.
  std::cout << "Report\n"
            << std::setw(2u) << std::setfill(' ') << ' '
            << "Compiling (this may take a couple of minutes)" << std::flush;
  if (stats.size() == 0u) {
    throw std::runtime_error("No statistics were generated, thus no report can be compiled.");
  } else if (stats.size() == 1u) {  // Single query report
    pdt::reports::SingleQueryReport report(config, stats[0u]);
    report.generateReport();
    reportPath = report.compileReport();
  } else {  // Multiquery report
    pdt::statistics::MultiqueryStatistics mqstats(config, stats, true);
    pdt::reports::MultiqueryReport report(config, mqstats);
    report.generateReport();
    reportPath = report.compileReport();
  }

  // Inform that we are done compiling the report.
  std::cout << '\r' << std::setw(47u) << std::setfill(' ') << ' ' << '\r' << std::setw(2u)
            << std::setfill(' ') << ' ' << "Compilation done\n"
            << std::flush;

  // Inform where we wrote the report to.
  std::cout << std::setw(2u) << std::setfill(' ') << ' ' << "Location " << reportPath << "\n\n";

  return 0;
}
