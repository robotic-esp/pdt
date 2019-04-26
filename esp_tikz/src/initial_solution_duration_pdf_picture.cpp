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
    const std::shared_ptr<const Configuration>& config, const Statistics& stats) :
    TikzPicture(config),
    config_(config),
    stats_(stats) {
}

std::shared_ptr<PgfAxis> InitialSolutionDurationPdfPicture::createInitialSolutionDurationPdfAxis()
    const {
  auto axis = std::make_shared<PgfAxis>();
  setInitialSolutionDurationPdfAxisOptions(axis);

  for (const auto& name : config_->get<std::vector<std::string>>("Experiment/planners")) {
    auto plot = createInitialSolutionDurationPdfPlot(name);
    plot->options.fillOpacity = config_->get<double>("InitialSolutionPlots/combinedFillOpacity");
    plot->options.lineWidth = config_->get<double>("InitialSolutionPlots/lineWidth");
    axis->addPlot(plot);
  }

  return axis;
}

std::shared_ptr<PgfAxis> InitialSolutionDurationPdfPicture::createInitialSolutionDurationPdfAxis(
    const std::string& plannerName) const {
  auto axis = std::make_shared<PgfAxis>();
  setInitialSolutionDurationPdfAxisOptions(axis);
  axis->addPlot(createInitialSolutionDurationPdfPlot(plannerName));
  return axis;
}

fs::path InitialSolutionDurationPdfPicture::createInitialSolutionDurationPdfPicture() const {
  // Create the picture and add the axis.
  TikzPicture picture(config_);
  picture.addAxis(createInitialSolutionDurationPdfAxis());

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("Experiment/results")).parent_path() /
                     fs::path("tikz/all_planners_initial_solution_pdf_plot.tikz");
  picture.write(picturePath);
  return picturePath;
}

fs::path InitialSolutionDurationPdfPicture::createInitialSolutionDurationPdfPicture(
    const std::string& plannerName) const {
  // Create the picture and add the axis.
  TikzPicture picture(config_);
  picture.addAxis(createInitialSolutionDurationPdfAxis(plannerName));

  // Generate the tikz file.
  auto picturePath = fs::path(config_->get<std::string>("Experiment/results")).parent_path() /
                     fs::path("tikz/"s + plannerName + "_initial_solution_pdf_plot.tikz"s);
  picture.write(picturePath);
  return picturePath;
}

std::shared_ptr<PgfPlot> InitialSolutionDurationPdfPicture::createInitialSolutionDurationPdfPlot(
    const std::string& plannerName) const {
  // Load the data into a pgf table.
  auto table = std::make_shared<PgfTable>(stats_.extractInitialSolutionDurationPdf(plannerName),
                                          "bin begin durations", "bin counts");

  // This table should not clean its data (or should it?).
  table->setCleanData(false);

  // We need extra data points to make this look like a proper histogram.
  auto firstRow = table->getRow(0u);
  auto lastRow = table->getRow(table->getNumRows() - 1u);
  table->prependRow({firstRow.at(0u), 0.0});
  table->appendRow({lastRow.at(0u), 0.0});

  // Create the plot.
  auto plot = std::make_shared<PgfPlot>(table);
  plot->options.markSize = 0.0;
  plot->options.lineWidth = 0.0;
  plot->options.namePath = plannerName + "InitialSolutionDurationPdf"s;
  plot->options.color = config_->get<std::string>("PlannerPlotColors/" + plannerName);
  plot->options.fill = config_->get<std::string>("PlannerPlotColors/" + plannerName);
  plot->options.fillOpacity = 1.0;

  return plot;
}

void InitialSolutionDurationPdfPicture::setInitialSolutionDurationPdfAxisOptions(
    std::shared_ptr<PgfAxis> axis) const {
  axis->options.height = config_->get<std::string>("InitialSolutionPlots/axisHeight");
  axis->options.width = config_->get<std::string>("InitialSolutionPlots/axisWidth");
  axis->options.name = "InitialSolutionDurationPdfAxis"s;
  axis->options.xlog = config_->get<bool>("InitialSolutionPlots/xlog");
  axis->options.xminorgrids = config_->get<bool>("InitialSolutionPlots/xminorgrids");
  axis->options.xmajorgrids = config_->get<bool>("InitialSolutionPlots/xmajorgrids");
  axis->options.yminorgrids = config_->get<bool>("InitialSolutionPlots/yminorgrids");
  axis->options.ymajorgrids = config_->get<bool>("InitialSolutionPlots/ymajorgrids");
  axis->options.xlabel = "Computation time [s]";
  axis->options.ylabel = "Counts";
  axis->options.ylabelAbsolute = true;
  axis->options.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
}

fs::path InitialSolutionDurationPdfPicture::compileStandalonePdf(
    const fs::path& tikzPicture) const {
  // Generate the path to write to.
  auto path = fs::path(tikzPicture).replace_extension(".tex");
  std::ofstream filestream;
  filestream.open(path.c_str());

  // Check on the failbit.
  if (filestream.fail() == true) {
    auto msg =
        "InitialSolutionDurationPdfPicture could not open picture at '" + path.string() + "'."s;
    throw std::ios_base::failure(msg);
  }

  // Write the preamble.
  filestream << "% The package 'luatex85' is needed for the standalone document class.\n"
                "\\RequirePackage{luatex85}\n";
  filestream << "\\documentclass{standalone}\n"
             << "\\usepackage{tikz}\n"
             << "\\usetikzlibrary{calc,plotmarks}\n"
             << "\\usepackage{pgfplots}\n"
             << "\\pgfplotsset{compat=1.15}\n"
             << "\\usepgfplotslibrary{fillbetween}\n"
             << "\\usepackage{xcolor}\n\n";

  // Include the picture.
  filestream << "\n\n\\begin{document}\n\n";
  filestream << "\n\\input{" << tikzPicture.string() << "}\n";
  filestream << "\n\n\\end{document}\n";

  // Close the file.
  filestream.close();

  // Compile the plot.
  // Compiling with lualatex is slower than pdflatex but has dynamic memory allocation. Since these
  // plots can be quite large, pdflatex has run into memory issues. Lualatex should be available
  // with all major tex distributions.
  auto currentPath = fs::current_path();
  auto cmd = "cd \""s + path.parent_path().string() + "\" && lualatex \""s + path.string() +
             "\" && cd \""s + currentPath.string() + '\"';
  int retval = std::system(cmd.c_str());
  (void)retval;
  return path;
}

}  // namespace ompltools

}  // namespace esp
