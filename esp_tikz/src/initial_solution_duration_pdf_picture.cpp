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

#include "esp_tikz/initial_solution_duration_pdf_picture.h"

#include <stdlib.h>
#include <algorithm>
#include <fstream>

#include <ompl/util/Console.h>

#include "esp_tikz/pgf_axis.h"
#include "esp_tikz/pgf_fillbetween.h"
#include "esp_tikz/pgf_plot.h"
#include "esp_tikz/pgf_table.h"

namespace esp {

namespace ompltools {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

InitialSolutionDurationPdfPicture::InitialSolutionDurationPdfPicture(
    const std::shared_ptr<Configuration>& config) :
    TikzPicture(config),
    config_(config) {
}

fs::path InitialSolutionDurationPdfPicture::generatePlot(const Statistics& stats) {
  // Compute the duration bin size.
  auto defaultBinDurations = stats.getDefaultBinDurations();

  // Determine the min and max durations to be plotted.
  double maxDurationToBePlotted = stats.getMaxNonInfInitialSolutionDuration();
  double minDurationToBePlotted = stats.getMinInitialSolutionDuration();

  // Set the axis options for the median cost plot.
  PgfAxisOptions initialSolutionDurationAxisOptions;
  initialSolutionDurationAxisOptions.name = "InitialSolutionDurationPdf";
  initialSolutionDurationAxisOptions.xmin = minDurationToBePlotted;
  initialSolutionDurationAxisOptions.xmax = maxDurationToBePlotted;
  initialSolutionDurationAxisOptions.xlog = true;
  initialSolutionDurationAxisOptions.xminorgrids = true;
  initialSolutionDurationAxisOptions.xlabel = "Computation time [s]";
  initialSolutionDurationAxisOptions.ybar = true;
  initialSolutionDurationAxisOptions.barWidth = "0.2pt";
  initialSolutionDurationAxisOptions.ylabel = "Initial Solution Counts";
  initialSolutionDurationAxisOptions.ylabelAbsolute = true;
  initialSolutionDurationAxisOptions.ylabelStyle =
      "font=\\footnotesize, text depth=0.0em, text height=0.5em";
  auto initialSolutionDurationAxis = generateInitialSolutionDurationPdfPlot(stats);
  initialSolutionDurationAxis->setOptions(initialSolutionDurationAxisOptions);

  // Create the axis that holds the plot legend.
  PgfAxisOptions legendAxisOptions;
  legendAxisOptions.at = "($(InitialSolutionDurationPdf.south) - (0.0em, 0.3em)$)";
  legendAxisOptions.anchor = "north";
  legendAxisOptions.name = "LegendAxis";
  legendAxisOptions.xmin = minDurationToBePlotted;
  legendAxisOptions.xmax = maxDurationToBePlotted;
  legendAxisOptions.ymin = 1;
  legendAxisOptions.ymax = 10;
  legendAxisOptions.hideAxis = true;
  legendAxisOptions.legendStyle =
      "anchor=north, legend cell align=left, legend columns=6, "
      "at={($(InitialSolutionDurationPdf.south) + "
      "(0.0em," +
      legendAxisOptions.height + " + 5em)$)}";  // Why do I have to shift so far up?
  auto legendAxis = generateLegendAxis();
  legendAxis->setOptions(legendAxisOptions);

  // Add all axis to this plot.
  axes_.emplace_back(initialSolutionDurationAxis);
  axes_.emplace_back(legendAxis);

  // Generate the path to write to.
  auto picturePath = fs::path(config_->get<std::string>("Experiment/results")).parent_path() /
                     fs::path("tikz/"s + config_->get<std::string>("Experiment/name") +
                              "_initial_solution_duration_pdf.tikz");
  write(picturePath);
  return picturePath;
}

std::shared_ptr<PgfAxis> InitialSolutionDurationPdfPicture::generateInitialSolutionDurationPdfPlot(
    const Statistics& stats) const {
  // Create an axis to hold the plots.
  auto axis = std::make_shared<PgfAxis>();

  // Plot the initial solutions.
  for (const auto& name : config_->get<std::vector<std::string>>("Experiment/planners")) {
    // Load the median initial duration and cost into a table.
    auto medianInitialSolutionPath = stats.extractInitialSolutionDurationPdf(name);
    auto initialSolutionTable =
        std::make_shared<PgfTable>(medianInitialSolutionPath, "bin centers", "bin counts");
    initialSolutionTable->setCleanData(false);

    // Create the pgf plot with this data and add it to the axis.
    PgfPlotOptions initialSolutionPlotOptions;
    initialSolutionPlotOptions.constPlot = false;
    initialSolutionPlotOptions.markSize = 0.0;
    initialSolutionPlotOptions.namePath = name + "InitialSolutionDurationPdf"s;
    initialSolutionPlotOptions.color = config_->get<std::string>("PlannerPlotColors/" + name);
    initialSolutionPlotOptions.fill = config_->get<std::string>("PlannerPlotColors/" + name);
    initialSolutionPlotOptions.fillOpacity = 1.0;
    auto initialSolutionPlot = std::make_shared<PgfPlot>(initialSolutionTable);
    initialSolutionPlot->setOptions(initialSolutionPlotOptions);
    axis->addPlot(initialSolutionPlot);
  }
  return axis;
}

fs::path InitialSolutionDurationPdfPicture::generatePdf() const {
  // Generate the path to write to.
  auto picturePath = fs::path(config_->get<std::string>("Experiment/results")).parent_path() /
                     fs::path("tikz/"s + config_->get<std::string>("Experiment/name") +
                              "_initial_solution_duration_pdf.tikz");
  auto texFilePath = fs::path(config_->get<std::string>("Experiment/results")).parent_path() /
                     fs::path("plots/"s + config_->get<std::string>("Experiment/name") +
                              "_initial_solution_duration_pdf.tex");
  fs::create_directories(texFilePath.parent_path());
  std::ofstream texFile;
  texFile.open(texFilePath.c_str());

  // Check on the failbit.
  if (texFile.fail() == true) {
    auto msg = "InitialSolutionDurationPdfPicture could not open picture at '" +
               texFilePath.string() + "'."s;
    throw std::ios_base::failure(msg);
  }

  // Write the preamble.
  texFile << "% The package 'luatex85' is needed for the standalone document class.\n "
             "\\RequirePackage{luatex85}\n";
  texFile << "\\documentclass{standalone}\n"
          << "\\usepackage{tikz}\n"
          << "\\usetikzlibrary{calc,plotmarks}\n"
          << "\\usepackage{pgfplots}\n"
          << "\\pgfplotsset{compat=1.15}\n"
          << "\\usepgfplotslibrary{fillbetween}\n"
          << "\\usepackage{xcolor}\n\n";

  // Include the picture.
  texFile << "\n\n\\begin{document}\n\n";
  texFile << "\n\\input{" << picturePath.string() << "}\n";
  texFile << "\n\n\\end{document}\n";

  // Close the file.
  texFile.close();

  // Compile the plot.
  // Compiling with lualatex is slower than pdflatex but has dynamic memory allocation. Since
  // these plots can be quite large, pdflatex has run into memory issues. Lualatex should be
  // available with all major tex distributions.
  auto currentPath = fs::current_path();
  auto cmd = "cd \""s + texFilePath.parent_path().string() + "\" && lualatex \""s +
             texFilePath.string() + "\" && cd \""s + currentPath.string() + '\"';
  int retval = std::system(cmd.c_str());
  (void)retval;
  return fs::path(texFilePath).replace_extension(".pdf");
}

}  // namespace ompltools

}  // namespace esp
