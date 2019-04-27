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

#include "esp_tikz/latex_plotter.h"

#include <fstream>

namespace esp {

namespace ompltools {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

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
