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
#include "esp_statistics/statistics.h"
#include "esp_tikz/experiment_report.h"

using namespace std::string_literals;

int main(const int argc, const char** argv) {
  // Read the config files.
  auto config = std::make_shared<esp::ompltools::Configuration>(argc, argv);

  std::cout << "\nReport\n"
            << std::setw(2u) << std::setfill(' ') << ' '
            << "Compiling (this may take a couple of minutes)" << std::flush;

  // Get the statistics.
  esp::ompltools::Statistics stats(config, true);

  // Create the report.
  esp::ompltools::ExperimentReport report(config, stats);
  report.generateReport();
  report.compileReport();
  
  // Inform that we are done compiling the report.
  std::cout << '\r' << std::setw(47u) << std::setfill(' ') << ' ' << '\r' << std::setw(2u)
            << std::setfill(' ') << ' ' << "Compilation done\n"
            << std::flush;

  return 0;
}
