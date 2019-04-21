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

#include <functional>
#include <iomanip>
#include <iostream>
#include <vector>

#include <experimental/filesystem>

#include "esp_configuration/configuration.h"
#include "esp_statistics/performance_statistics.h"
#include "esp_tikz/performance_plotter.h"

using namespace std::string_literals;

int main(int argc, char **argv) {
  // Read the config files.
  auto config = std::make_shared<esp::ompltools::Configuration>(argc, argv);

  // Get the results.
  std::experimental::filesystem::path resultsPath = config->get<std::string>("Experiment/results");

  // Get the statistics.
  esp::ompltools::PerformanceStatistics stats(resultsPath);

  // Generate the plot.
  auto contextName = config->get<std::string>("Experiment/context");
  std::size_t numMeasurements =
      std::ceil(config->get<double>("Contexts/" + contextName + "/maxTime") *
                config->get<double>("Experiment/logFrequency"));
  double binSize = 1.0 / config->get<double>("Experiment/logFrequency");
  std::vector<double> durations;
  durations.reserve(numMeasurements);
  for (std::size_t i = 0u; i < numMeasurements; ++i) {
    durations.emplace_back(static_cast<double>(i + 1u) * binSize);
  }
  esp::ompltools::PerformancePlotter plotter(config);
  auto plotPath = ((resultsPath.parent_path() / resultsPath.stem()) += "_median_cost_plot.tex"s);
  plotter.generateMedianCostAndSuccessPlot(stats, durations, plotPath);
  plotter.compilePlot(plotPath);

  return 0;
}
