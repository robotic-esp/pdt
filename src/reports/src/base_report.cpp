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

#include "pdt/reports/base_report.h"

#include <stdlib.h>
#include <algorithm>
#include <fstream>

#include <ompl/util/Console.h>

#include "pdt/pgftikz/define_latex_colors.h"

namespace pdt {

namespace reports {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

BaseReport::BaseReport(const std::shared_ptr<config::Configuration>& config) :
    latexPlotter_(config),
    config_(config) {
  // Latex doesn't like unescaped underscores in text mode.
  experimentName_ = config_->get<std::string>("experiment/name");
  findAndReplaceAll(&experimentName_, "_", "\\_");
  experimentName_ = "\\texttt{"s + experimentName_ + "}"s;

  // Get the plot planner names.
  for (const auto& name : config_->get<std::vector<std::string>>("experiment/planners")) {
    try {
      plotPlannerNames_[name] = config_->get<std::string>("planner/" + name + "/report/name");
    } catch (const std::invalid_argument& e) {
      plotPlannerNames_[name] = name;
    }
  }
}

std::stringstream BaseReport::preamble() const {
  std::stringstream preamble;
  // First we need the 'Required' packages.
  for (const auto& package : requirePackages_) {
    preamble << "\\RequirePackage{" << package << "}\n";
  }

  // We're ready to declare the document class.
  preamble << "\\documentclass[titlepage]{article}\n";

  // Now we load the 'used' packages.
  preamble << "% This report uses the following packages:\n";
  for (const auto& package : usePackages_) {
    preamble << "\\usepackage{" << package << "}\n";
  }

  // Now we load the used tikz libraries.
  preamble << "% This report uses the following TikZ libraries:\n";
  for (const auto& library : tikzLibraries_) {
    preamble << "\\usetikzlibrary{" << library << "}\n";
  }

  // We activate externalization for the tikz figures.
  preamble
      << "\\immediate\\write18{mkdir -p tikz-externalized}\n"
      << "\\tikzexternalize[prefix=tikz-externalized/]\n"
      << "\\tikzset{external/system call/.add={}{; convert -density 600 -flatten \"\\image.pdf\" "
         "\"\\image-600.png\"}}\n";

  // Now we load the used pgf libraries.
  preamble << "% This report uses the following PGFPlots libraries:\n";
  for (const auto& library : pgfLibraries_) {
    preamble << "\\usepgfplotslibrary{" << library << "}\n";
  }

  // Set the desired listings options.
  preamble << "% This report sets the following listings options:\n";
  for (const auto& option : lstSet_) {
    preamble << "\\lstset{" << option << "}\n";
  }

  // Set the desired pgfplots options.
  preamble << "% This report sets the following PGFPlots options:\n";
  for (const auto& option : pgfPlotsset_) {
    preamble << "\\pgfplotsset{" << option << "}\n";
  }

  // Define a nicer listing for JSON.
  preamble << "\\lstdefinelanguage{json}{%\n"
              "basicstyle={\\normalfont\\ttfamily},%\n"
              "stringstyle=\\color{black!90},%\n"
              "numbers=left,%\n"
              "numberstyle=\\scriptsize,%\n"
              "stepnumber=1,%\n"
              "numbersep=8pt,%\n"
              "showstringspaces=false,%\n"
              "breaklines=true,%\n"
              "backgroundcolor=\\color{gray!10},%\n"
              "string=[s]{\"}{\"},%\n"
              "comment=[l]{:\\ \"},%\n"
              "morecomment=[l]{:\"},%\n"
              "literate=%\n"
              "  *{0}{{{\\color{black!70}0}}}{1}%\n"
              "  {1}{{{\\color{black!70}1}}}{1}%\n"
              "  {2}{{{\\color{black!70}2}}}{1}%\n"
              "  {3}{{{\\color{black!70}3}}}{1}%\n"
              "  {4}{{{\\color{black!70}4}}}{1}%\n"
              "  {5}{{{\\color{black!70}5}}}{1}%\n"
              "  {6}{{{\\color{black!70}6}}}{1}%\n"
              "  {7}{{{\\color{black!70}7}}}{1}%\n"
              "  {8}{{{\\color{black!70}8}}}{1}%\n"
              "  {9}{{{\\color{black!70}9}}}{1}}%\n";
  preamble << "\\lstset{language=json}\n";

  // Include the colors.
  preamble << pgftikz::defineLatexColors(config_);

  // Create the title.
  preamble << "\\title{\\bfseries\\LARGE Experiment \\\\ " << experimentName_ << "}\n";

  // Create the author.
  preamble << "\\author{Planner Developer Tools (PDT)}\n";

  // Set the when this report was compiled.
  preamble << "\\date{\\today}\n";

  // That's it, let's separate the preamble with an extra newline.
  preamble << '\n';
  return preamble;
}

std::stringstream BaseReport::appendix() const {
  std::stringstream appendix;
  appendix << "\n\\pagebreak\n";
  appendix << "\\begin{appendices}\n";
  // At the configuration section.
  appendix << "\\section{Configuration}\\label{sec:configuration}\n";

  // Report the configuration of the experiment first.
  appendix << "\\subsection{Experiment}\\label{sec:experiment-configuration}\n";
  appendix << "\\begin{lstlisting}\n" << config_->dump("experiment") << "\\end{lstlisting}\n";

  // Report the configuration of the context second.
  appendix << "\\subsection{" << config_->get<std::string>("experiment/context")
           << "}\\label{sec:context-configuration}\n";
  appendix << "\\begin{lstlisting}\n"
           << config_->dump("context/" + config_->get<std::string>("experiment/context"))
           << "\\end{lstlisting}\n";

  // Report the configuration of all planners.
  for (const auto& plannerName : config_->get<std::vector<std::string>>("experiment/planners")) {
    appendix << "\\subsection{" << plotPlannerNames_.at(plannerName)
             << "}\\label{sec:" << plannerName << "-configuration}\n";
    appendix << "\\begin{lstlisting}\n"
             << config_->dump("planner/" + plannerName) << "\\end{lstlisting}\n";
  }

  appendix << "\\end{appendices}\n";

  return appendix;
}

fs::path BaseReport::compileReport() const {
  // Compiling with lualatex is slower than pdflatex but has dynamic memory allocation. Since
  // these plots can be quite large, pdflatex has run into memory issues. Lualatex should be
  // available with all major tex distributions.
  auto reportPath =
      fs::path(config_->get<std::string>("experiment/experimentDirectory")) / "report.tex";
  auto currentPath = fs::current_path();
  auto cmd = "cd \""s + reportPath.parent_path().string() +
             "\" && lualatex --interaction=nonstopmode --shell-escape \""s + reportPath.string() +
             "\""s;
  if (!config_->get<bool>("report/verboseCompilation")) {
    cmd += " > /dev/null";
  }
  cmd += " && cd \""s + currentPath.string() + '\"';
  int retval = std::system(cmd.c_str());
  retval = std::system(cmd.c_str());  //  We compile the report twice to get the references right.
  (void)retval;                       // Get rid of warning for unused variable.
  return fs::path(reportPath).replace_extension(".pdf");
}

void BaseReport::findAndReplaceAll(std::string* string, const std::string& key,
                                   const std::string& replacement) const {
  // Get the first occurrence.
  size_t pos = string->find(key);

  // Repeat till end is reached.
  while (pos != std::string::npos) {
    // Replace this occurrence of Sub String.
    string->replace(pos, key.size(), replacement);
    // Get the next occurrence from the current position.
    pos = string->find(key, pos + replacement.size());
  }
}

}  // namespace reports

}  // namespace pdt
