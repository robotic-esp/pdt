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

#include "esp_tikz/experiment_report.h"

#include <stdlib.h>
#include <algorithm>
#include <fstream>

#include <ompl/util/Console.h>

#include "esp_factories/context_factory.h"
#include "esp_tikz/kpi_table.h"
#include "esp_tikz/pgf_axis.h"
#include "esp_tikz/pgf_fillbetween.h"
#include "esp_tikz/pgf_plot.h"
#include "esp_tikz/pgf_table.h"

namespace esp {

namespace ompltools {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

ExperimentReport::ExperimentReport(const std::shared_ptr<Configuration>& config,
                                   const Statistics& stats) :
    latexPlotter_(config),
    costPercentileEvolutionPlotter_(config, stats),
    initialSolutionDurationPdfPlotter_(config, stats),
    initialSolutionScatterPlotter_(config, stats),
    medianCostEvolutionPlotter_(config, stats),
    medianInitialSolutionPlotter_(config, stats),
    successPlotter_(config, stats),
    overviewPlotter_(config, stats),
    config_(config),
    stats_(stats) {
  // Latex doesn't like unescaped underscores in text mode.
  experimentName_ = config_->get<std::string>("Experiment/name");
  findAndReplaceAll(&experimentName_, "_", "\\_");
  experimentName_ = "\\texttt{"s + experimentName_ + "}"s;

  // Get the plot planner names.
  for (const auto& name : config_->get<std::vector<std::string>>("Experiment/planners")) {
    try {
      plotPlannerNames_[name] = config_->get<std::string>("PlotPlannerNames/" + name);
    } catch (const std::invalid_argument& e) {
      plotPlannerNames_[name] = name;
    }
  }
}

fs::path ExperimentReport::generateReport() {
  auto reportPath =
      fs::path(config_->get<std::string>("Experiment/results")).parent_path() / "report.tex"s;
  // Open the filestream.
  std::ofstream report;
  report.open(reportPath.c_str());

  // Check on the failbit.
  if (report.fail() == true) {
    auto msg = "ExperimentReport failed to create a report at '" + reportPath.string() + "'."s;
    throw std::ios_base::failure(msg);
  }

  // Write the preamble.
  report << preamble().str();

  // Start the document.
  report << "\\begin{document}\n";

  // Make the title.
  report << "\\maketitle\n";

  // Start with an overview.
  report << overview().str();

  // Report the individual results.
  report << individualResults().str();

  // Add the appendix.
  report << appendix().str();

  // End the document.
  report << "\\end{document}\n";

  return reportPath;
}

std::stringstream ExperimentReport::preamble() const {
  std::stringstream preamble;
  // First we need the 'Required' packages.
  for (const auto package : requirePackages_) {
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

  // Create the title.
  preamble << "\\title{\\bfseries\\LARGE Experiment \\\\ " << experimentName_ << "}\n";

  // Create the author.
  preamble << "\\author{ESP OMPLTools}\n";

  // Set the when this report was compiled.
  preamble << "\\date{\\today}\n";

  // That's it, let's separate the preamble with an extra newline.
  preamble << '\n';
  return preamble;
}

std::stringstream ExperimentReport::overview() const {
  std::stringstream overview;
  // We often refer to the planner names, this reference just makes it more convenient.
  const auto& plannerNames = config_->get<std::vector<std::string>>("Experiment/planners");

  // Create the section header.
  overview << "\\section{Overview}\\label{sec:overview}\n\n";

  // Provide some basic info about this experiment.
  overview << "This report was automatically generated using ESP "
              "OMPLtools. It presents the results "
              "for the "
           << experimentName_ << " experiment, which executed "
           << config_->get<std::size_t>("Experiment/numRuns") << " runs of ";
  for (std::size_t i = 0u; i < plannerNames.size() - 1u; ++i) {
    overview << plotPlannerNames_.at(plannerNames.at(i)) << ", ";
  }
  overview << " and " << plotPlannerNames_.at(plannerNames.back()) << " on the \\texttt{"
           << config_->get<std::string>("Experiment/context")
           << "} planning context. See appendix~\\ref{sec:experiment-configuration} for more "
              "information about the "
              "experiment setup.\n";

  // Create the results summary section.
  overview << "\\subsection{Results Summary}\\label{sec:overview-results-summary}\n";

  // Create the KPI table.
  KpiTable kpiTable(config_, stats_);
  for (const auto& name : config_->get<std::vector<std::string>>("Experiment/planners")) {
    kpiTable.addKpi(name, plotPlannerNames_.at(name));
  }
  overview << kpiTable.string() << '\n';

  // Create all axes to be displayed in the results summary.
  auto medianCostEvolutionAxis = medianCostEvolutionPlotter_.createMedianCostEvolutionAxis();
  auto medianInitialSolutionAxis = medianInitialSolutionPlotter_.createMedianInitialSolutionAxis();
  auto successAxis = successPlotter_.createSuccessAxis();
  // Merge the intial solution axis into the cost evolution axis.
  medianCostEvolutionAxis->mergePlots(medianInitialSolutionAxis);

  // Align the success and median cost evolution axes.
  latexPlotter_.align(successAxis, medianCostEvolutionAxis);

  // Create the legend axis.
  auto legend =
      latexPlotter_.createLegendAxis(config_->get<std::vector<std::string>>("Experiment/planners"));

  // Stack the axes
  latexPlotter_.stack(successAxis, medianCostEvolutionAxis, legend);

  overview << "\\begin{center}\n\\input{"
           << latexPlotter_.createPicture(successAxis, medianCostEvolutionAxis, legend).string()
           << "}\n\\end{center}\n";

  // Create the initial solution overview section.
  overview << "\\pagebreak\n";
  overview << "\\subsection{Initial Solutions}\\label{sec:overview-initial-solutions}\n";

  // Collect all initial solution duration pdf plots.
  std::vector<std::shared_ptr<PgfAxis>> initialSolutionDurationPdfAxes{};
  for (const auto& name : plannerNames) {
    initialSolutionDurationPdfAxes.emplace_back(
        initialSolutionDurationPdfPlotter_.createInitialSolutionDurationPdfAxis(name));
  }
  latexPlotter_.align(initialSolutionDurationPdfAxes);
  initialSolutionDurationPdfAxes.push_back(legend);
  latexPlotter_.stack(initialSolutionDurationPdfAxes);

  overview << "\\begin{center}\n\\input{"
           << latexPlotter_.createPicture(initialSolutionDurationPdfAxes).string()
           << "}\n\\end{center}";

  return overview;
}

std::stringstream ExperimentReport::individualResults() const {
  std::stringstream results;

  // Create a section for every planner.
  const auto& plannerNames = config_->get<std::vector<std::string>>("Experiment/planners");
  for (const auto& name : plannerNames) {
    // Create the section title on a new page.
    results << "\n\\pagebreak\n";
    results << "\\section{" << plotPlannerNames_.at(name) << "}\\label{sec:" << name << "}\n";

    // First report on the initial solutions.
    results << "\\subsection{Initial Solutions}\\label{sec:" << name << "-initial-solution}\n";

    // Overlay the pdf with the cdf for the first initial durations plot.
    auto cdf = successPlotter_.createSuccessAxis(name);
    cdf->options.xmin = stats_.getMinInitialSolutionDuration(name);
    cdf->options.xmax = stats_.getMaxNonInfInitialSolutionDuration(name);
    auto pdf = initialSolutionDurationPdfPlotter_.createInitialSolutionDurationPdfAxis(name);
    pdf->overlay(cdf.get());
    for (const auto& plot : pdf->getPlots()) {
      plot->options.drawOpacity = 0.2;
      plot->options.fillOpacity = 0.1;
    }

    // Create the scatter axis of all initial solutions.
    auto scatter = initialSolutionScatterPlotter_.createInitialSolutionScatterAxis(name);

    // Create a median plot of all initial solutions.
    auto median = medianInitialSolutionPlotter_.createMedianInitialSolutionAxis(name);
    for (const auto& plot : median->getPlots()) {
      plot->options.markSize = 2.0;
      plot->options.lineWidth = 1.0;
    }

    // Merge the plots from the median axis into the scatter axis.
    scatter->mergePlots(median);

    // Place the scatter plot underneath the cdf/pdf plot
    scatter->matchAbszisse(*cdf);
    latexPlotter_.stack(cdf, scatter);

    // Enlarge all x axis limits such that the earliest and latest initial solutions aren't glued to
    // the axis box.
    pdf->options.enlargeXLimits = "true";
    cdf->options.enlargeXLimits = "true";
    scatter->options.enlargeXLimits = "true";

    // Create a picture out of the three initial solution axes.
    results << "\\begin{center}\n\\input{"
            << latexPlotter_.createPicture(cdf, pdf, scatter).string() << "}\n\\end{center}\n";

    // Show the cost evolution plots for anytime planners.
    if (name != "RRTConnect") {
      // Cost evolution plots.
      auto medianEvolution = medianCostEvolutionPlotter_.createMedianCostEvolutionAxis(name);
      auto percentileEvolution =
          costPercentileEvolutionPlotter_.createCostPercentileEvolutionAxis(name);
      medianEvolution->matchAbszisse(*percentileEvolution);
      latexPlotter_.stack(medianEvolution, percentileEvolution);

      results << "\\subsection{Cost Evolution}\\label{sec:" << name << "-cost-evolution}\n";
      results << "\\begin{center}\n\\input{"
              << latexPlotter_.createPicture(medianEvolution, percentileEvolution).string()
              << "}\n\\end{center}\n";
    }
  }

  return results;
}

std::stringstream ExperimentReport::appendix() const {
  std::stringstream appendix;
  appendix << "\\begin{appendices}\n";
  // At the configuration section.
  appendix << "\\section{Configuration}\\label{sec:configuration}\n";

  // Report the configuration of the experiment first.
  appendix << "\\subsection{Experiment}\\label{sec:experiment-configuration}\n";
  appendix << "\\begin{lstlisting}\n" << config_->dump("Experiment") << "\\end{lstlisting}\n";

  // Report the configuration of the context second.
  appendix << "\\subsection{" << config_->get<std::string>("Experiment/context")
           << "}\\label{sec:context-configuration}\n";
  appendix << "\\begin{lstlisting}\n"
           << config_->dump("Contexts/" + config_->get<std::string>("Experiment/context"))
           << "\\end{lstlisting}\n";

  // Report the configuration of all planners.
  for (const auto& plannerName : config_->get<std::vector<std::string>>("Experiment/planners")) {
    appendix << "\\subsection{" << plotPlannerNames_.at(plannerName)
             << "}\\label{sec:" << plannerName << "-configuration}\n";
    appendix << "\\begin{lstlisting}\n"
             << config_->dump("Planners/" + plannerName) << "\\end{lstlisting}\n";
  }

  appendix << "\\end{appendices}\n";

  return appendix;
}

fs::path ExperimentReport::compileReport() const {
  // Compiling with lualatex is slower than pdflatex but has dynamic memory allocation. Since
  // these plots can be quite large, pdflatex has run into memory issues. Lualatex should be
  // available with all major tex distributions.
  auto reportPath =
      fs::path(config_->get<std::string>("Experiment/results")).parent_path() / "report.tex";
  auto currentPath = fs::current_path();
  auto cmd = "cd \""s + reportPath.parent_path().string() + "\" && lualatex \""s +
             reportPath.string() + "\" && cd \""s + currentPath.string() + '\"';
  int retval = std::system(cmd.c_str());
  retval = std::system(cmd.c_str());
  (void)retval;
  return fs::path(reportPath).replace_extension(".pdf");
}

void ExperimentReport::findAndReplaceAll(std::string* string, const std::string& key,
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

}  // namespace ompltools

}  // namespace esp
