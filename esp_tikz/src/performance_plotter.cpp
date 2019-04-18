/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, University of Oxford
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
 *   * Neither the name of the University of Oxford nor the names of its
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

#include "esp_tikz/performance_plotter.h"

#include <stdlib.h>
#include <fstream>

#include "esp_tikz/colormap.h"
#include "esp_tikz/pgf_axis.h"
#include "esp_tikz/pgf_plot.h"
#include "esp_tikz/pgf_table.h"
#include "esp_tikz/colormap.h"

namespace esp {

namespace ompltools {

using namespace std::string_literals;

void PerformancePlotter::generateQuantileCostPlot(
    const PerformanceStatistics& stats, double quantile, const std::vector<double>& durations,
    const std::experimental::filesystem::path& filename) {
  // Create an axis for this plot.
  PgfAxisOptions axisOptions;
  axisOptions.xlog = true;
  axisOptions.xminorgrids = true;
  axisOptions.xlabel = "Computation time"s;
  axisOptions.ylabel = "Solution cost"s;
  auto axis = std::make_shared<PgfAxis>();
  axis->setOptions(axisOptions);

  // Create a picture that holds this axis.
  TikzPicture picture;
  picture.addAxis(axis);

  // This seems so hacky. How to make this better?
  std::map<std::string, std::string> plannerColors;
  auto color = espcolors.begin();
  for (const auto& name : stats.getPlannerNames()) {
    plannerColors.emplace(name, color->first);
    ++color;
    // Wrap around?
    if (color == espcolors.end()) {
      color = espcolors.begin();
    }
  }
  
  // Plot the cost evolution, if applicable.
  for (const auto& name : stats.getPlannerNames()) {
    // This doesn't apply to planners that aren't anytime.
    if (name == "RRTConnect"s) {
      continue;
    }
    // Get the data and store it in a pgf table.
    auto costs = stats.getQuantileEvolution(name, quantile, durations);
    auto table = std::make_shared<PgfTable>();
    table->addColumn(durations);
    table->addColumn(costs);

    // Create a pgf plot with this data and add it to the axis.
    PgfPlotOptions plotOptions;
    plotOptions.markSize = 0.0;
    plotOptions.color = plannerColors.at(name);
    auto plot = std::make_shared<PgfPlot>(table);
    plot->setOptions(plotOptions);
    plot->setLegend(name);
    axis->addPlot(plot);
  }

  // // Plot the initial solutions.
  // for (const auto& name : stats.getPlannerNames()) {
  //   // Get the data and store it in a pgf table.
  //   auto [duration, cost] = stats.getInitialSolution(name, quantile);
  //   auto table = std::make_shared<PgfTable>();
  //   table->addColumn({duration});
  //   table->addColumn({cost});

  //   // Create the pgf plot with this data and add it to the axis.
  //   auto plot = std::make_shared<PgfPlot>(table);
  //   axis->addPlot(plot);
  // }

  // Write it to a file.
  writePictureToFile(picture, filename);
}

void PerformancePlotter::writePictureToFile(
    const TikzPicture& picture, const std::experimental::filesystem::path& filename) const {
  // Open a file.
  std::ofstream texFile;
  texFile.open(filename.string());

  // Check on the failbit.
  if (texFile.fail() == true) {
    throw std::ios_base::failure("Could not open performance plot file.");
  }

  // Write the preamble.
  texFile << "\\documentclass{standalone}\n"
          << "\\usepackage{tikz}\n"
          << "\\usepackage{pgfplots}\n"
          << "\\pgfplotsset{compat=1.15}\n"
          << "\\usepackage{xcolor}\n";
  for (const auto& [name, values] : espcolors) {
    texFile << "\\definecolor{" << name << "}{RGB}{" << values[0u] << ',' << values[1u] << ','
            << values[2u] << "}\n";
  }

  // Write the picture.
  texFile << "\n\n\\begin{document}\n\n";
  texFile << picture.string();
  texFile << "\n\n\\end{document}\n";

  // Close the file.
  texFile.close();
}

void PerformancePlotter::compilePlot(const std::experimental::filesystem::path& filename) const {
  // Compile the plot.
  auto currentPath = std::experimental::filesystem::current_path();
  auto cmd = "cd "s + filename.parent_path().string() +
             " && pdflatex -file-line-error -interaction=nonstopmode "s +
             filename.filename().string() + " && cd "s + currentPath.string();
  int retval = std::system(cmd.c_str());
  (void)retval;
}

}  // namespace ompltools

}  // namespace esp
