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

#include <fstream>

#include "esp_tikz/pgf_axis.h"
#include "esp_tikz/pgf_plot.h"
#include "esp_tikz/pgf_table.h"

namespace esp {

namespace ompltools {

PerformancePlotter::PerformancePlotter(const std::experimental::filesystem::path& outDirectory) :
    outDirectory_(outDirectory) {
}

void PerformancePlotter::generateQuantileCostPlot(const AccumulatingCostLog& log, double quantile,
                                                  const std::string& filename) {
  // Create an axis for this plot.
  auto axis = std::make_shared<PgfAxis>();

  // Create a picture that holds this axis.
  TikzPicture picture;
  picture.addAxis(axis);

  for (const auto& name : log.getPlannerNames()) {
    // Get the data and store it in a pgf table.
    auto [times, costs] = log.getQuantile(name, quantile);
    auto table = std::make_shared<PgfTable>();
    table->addColumn(times);
    table->addColumn(costs);

    // Create a pgf plot with this data and add it to the axis.
    auto plot = std::make_shared<PgfPlot>(table);
    axis->addPlot(plot);
  }

  if (filename == std::string("")) {
    writePictureToFile(picture, "quantile_plot.tex");
  } else {
    writePictureToFile(picture, filename);
  }
}

void PerformancePlotter::writePictureToFile(const TikzPicture& picture,
                                            const std::string& filename) const {
  // Open a file.
  std::ofstream texFile;
  texFile.open(outDirectory_ / std::experimental::filesystem::path(filename));

  // Check on the failbit.
  if (texFile.fail() == true) {
    throw std::ios_base::failure("Could not open performance plot file.");
  }

  // Write the preamble.
  texFile << "\\documentclass{standalone}\n"
          << "\\usepackage{tikz}\n"
          << "\\usepackage{pgfplots}\n"
          << "\\pgfplotsset{compat=1.15}\n\n"
          << "\\begin{document}";

  // Write the picture.
  texFile << picture.string() << '\n';

  // End the document.
  texFile << "\\end{document}\n";

  // Close the file.
  texFile.close();
}

}  // namespace ompltools

}  // namespace esp
