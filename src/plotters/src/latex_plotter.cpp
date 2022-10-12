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

#include "pdt/plotters/latex_plotter.h"

#include <fstream>

namespace pdt {

namespace plotters {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

std::size_t LatexPlotter::plotId_ = 0u;

LatexPlotter::LatexPlotter(const std::shared_ptr<const config::Configuration>& config) :
    config_(config) {
}

std::shared_ptr<pgftikz::PgfAxis> LatexPlotter::createLegendAxis(
    const std::vector<std::string>& plannerNames) const {
  auto legend = std::make_shared<pgftikz::PgfAxis>();
  legend->options.xlog = false;
  legend->options.xmin = 0.0;
  legend->options.xmax = 10.0;
  legend->options.ymin = 0.0;
  legend->options.ymax = 10.0;
  legend->options.hideAxis = true;
  legend->options.legendStyle =
      "anchor=south, legend cell align=left, legend columns=-1, at={(axis cs:5, 6)}";
  for (const auto& name : plannerNames) {
    std::string imageOptions{config_->get<std::string>("planner/"s + name + "/report/color"s) +
                             ", line width = 1.0pt, mark size=1.0pt, mark=square*"};
    legend->addLegendEntry(config_->get<std::string>("planner/" + name + "/report/name"),
                           imageOptions);
  }
  return legend;
}

void LatexPlotter::alignAbszissen(
    const std::vector<std::shared_ptr<pgftikz::PgfAxis>>& axes) const {
  // Align all pairs.
  for (auto a : axes) {
    for (auto b : axes) {
      a->expandRangeOfAbszisse(*b);
      b->expandRangeOfAbszisse(*a);
    }
  }
}

void LatexPlotter::alignOrdinates(
    const std::vector<std::shared_ptr<pgftikz::PgfAxis>>& axes) const {
  // Align all pairs.
  for (auto a : axes) {
    for (auto b : axes) {
      a->expandRangeOfOrdinate(*b);
      b->expandRangeOfOrdinate(*a);
    }
  }
}

void LatexPlotter::stack(const std::vector<std::shared_ptr<pgftikz::PgfAxis>>& axes) const {
  // Take the first axis as the base, stack the rest underneath.
  if (axes.empty()) {
    return;
  }
  for (std::size_t i = 0u; i < axes.size(); ++i) {
    // Remove the xlabel and xticklabel from the axes
    if (i < axes.size() - 2u) {
      axes.at(i)->options.xlabel = "{\\empty}";
      axes.at(i)->options.xticklabel = "{\\empty}";
    } else if (i == axes.size() - 2u) {
      if (!axes.back()->options.hideAxis) {
        axes.at(i)->options.xlabel = "{\\empty}";
        axes.at(i)->options.xticklabel = "{\\empty}";
      }
    }
  }
  for (std::size_t i = 1u; i < axes.size(); ++i) {
    axes.at(i)->options.at = "($("s + axes.at(i - 1u)->options.name + ".south) - (0.0em, 0.6em)$)"s;
    axes.at(i)->options.anchor = "north";
  }
}

std::shared_ptr<pgftikz::TikzPicture> LatexPlotter::collect(
    const std::vector<std::shared_ptr<pgftikz::PgfAxis>>& axes) const {
  auto picture = std::make_shared<pgftikz::TikzPicture>(config_);

  // Collect the axes in a picture.
  for (const auto& axis : axes) {
    picture->addAxis(axis);
  }

  return picture;
}

std::experimental::filesystem::path LatexPlotter::createPicture(
    const std::vector<std::shared_ptr<pgftikz::PgfAxis>>& axes) const {
  auto picture = collect(axes);

  // Create the name of this picture.
  auto path = std::experimental::filesystem::path(
                  config_->get<std::string>("experiment/experimentDirectory")) /
              std::experimental::filesystem::path("tikz/");
  path += "axes_collection_" + std::to_string(plotId_++) + ".tikz";
  picture->write(path);
  return path;
}

fs::path LatexPlotter::compileStandalonePdf(const fs::path& tikzPicture) const {
  // Generate the path to write to.
  auto path = fs::path(tikzPicture).replace_extension(".tex");
  std::ofstream filestream;
  filestream.open(path.c_str());

  // Check on the failbit.
  if (filestream.fail() == true) {
    auto msg = "LatexPlotter could not open picture at '"s + path.string() + "'."s;
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
  filestream.close();

  // Compile the plot.
  // Compiling with lualatex is slower than pdflatex but has dynamic memory allocation. Since
  // these plots can be quite large, pdflatex has run into memory issues. Lualatex should be
  // available with all major tex distributions.
  auto currentPath = fs::current_path();
  auto cmd = "cd \""s + path.parent_path().string() +
             "\" && lualatex --interaction=nonstopmode --shell-escape \""s + path.string() +
             "\" && cd \""s + currentPath.string() + '\"';
  int retval = std::system(cmd.c_str());
  (void)retval;
  return path;
}

}  // namespace plotters

}  // namespace pdt
