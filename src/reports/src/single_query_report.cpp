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

#include "pdt/reports/single_query_report.h"

#include <stdlib.h>
#include <algorithm>
#include <fstream>

#include <ompl/util/Console.h>

#include "pdt/factories/context_factory.h"
#include "pdt/pgftikz/kpi_table.h"
#include "pdt/pgftikz/pgf_axis.h"
#include "pdt/pgftikz/pgf_fillbetween.h"
#include "pdt/pgftikz/pgf_plot.h"
#include "pdt/pgftikz/pgf_table.h"

namespace pdt {

namespace reports {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

SingleQueryReport::SingleQueryReport(const std::shared_ptr<config::Configuration>& config,
                                     const statistics::PlanningStatistics& stats) :
    BaseReport(config),
    latexPlotter_(config),
    overviewPlotter_(config, stats),
    queryCostAtFirstVsTimeAtFirstScatterPlotter_(config, stats),
    queryMedianCostAtFirstVsMedianTimeAtFirstPointPlotter_(config, stats),
    queryMedianCostVsTimeLinePlotter_(config, stats),
    queryPercentileCostVsTimeLinePlotter_(config, stats),
    querySuccessVsTimeLinePlotter_(config, stats),
    queryTimeAtFirstHistogramPlotter_(config, stats),
    stats_(stats) {
}

fs::path SingleQueryReport::generateReport() {
  auto reportPath =
      fs::path(config_->get<std::string>("experiment/experimentDirectory")) / "report.tex"s;
  // Open the filestream.
  std::ofstream report;
  report.open(reportPath.c_str());

  // Check on the failbit.
  if (report.fail() == true) {
    auto msg = "SingleQueryReport failed to create a report at '" + reportPath.string() + "'."s;
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

std::stringstream SingleQueryReport::overview() const {
  std::stringstream overview;
  // We often refer to the planner names, this reference just makes it more convenient.
  const auto& plannerNames = config_->get<std::vector<std::string>>("experiment/planners");

  // Create the section header.
  overview << "\\section{Overview}\\label{sec:overview}\n\n";

  // Provide some basic info about this experiment.
  overview << "This report was automatically generated using Planner "
              "Developer Tools (PDT). It presents the results "
              "for the "
           << experimentName_ << " experiment, which executed "
           << config_->get<std::size_t>("experiment/numRuns") << " runs of ";
  for (std::size_t i = 0u; i < plannerNames.size() - 1u; ++i) {
    overview << plotPlannerNames_.at(plannerNames.at(i)) << ", ";
  }
  overview << " and " << plotPlannerNames_.at(plannerNames.back()) << " on the \\texttt{"
           << config_->get<std::string>("experiment/context")
           << "} planning context. See appendix~\\ref{sec:experiment-configuration} for more "
              "information about the "
              "experiment setup.\n";

  // Create the results summary section.
  overview << "\\subsection{Results Summary}\\label{sec:overview-results-summary}\n";

  // Create the KPI table.
  pgftikz::KpiTable kpiTable(config_, stats_);
  for (const auto& name : config_->get<std::vector<std::string>>("experiment/planners")) {
    kpiTable.addKpi(name, plotPlannerNames_.at(name));
  }
  overview << kpiTable.string() << '\n';

  // Create all axes to be displayed in the results summary.
  auto medianCostEvolutionAxis = queryMedianCostVsTimeLinePlotter_.createMedianCostEvolutionAxis();
  auto medianInitialSolutionAxis =
      queryMedianCostAtFirstVsMedianTimeAtFirstPointPlotter_.createMedianInitialSolutionAxis();
  auto successAxis = querySuccessVsTimeLinePlotter_.createSuccessAxis();
  // Merge the intial solution axis into the cost evolution axis.
  medianCostEvolutionAxis->mergePlots(medianInitialSolutionAxis);

  // Align the success and median cost evolution axes.
  latexPlotter_.alignAbszissen(successAxis, medianCostEvolutionAxis);

  // Create the legend axis.
  auto legend =
      latexPlotter_.createLegendAxis(config_->get<std::vector<std::string>>("experiment/planners"));

  // Stack the axes
  latexPlotter_.stack(successAxis, medianCostEvolutionAxis, legend);

  std::stringstream medianCiKey;
  medianCiKey << "statistics/percentiles/sampleSize/"s << stats_.getNumRunsPerPlanner()
              << "/populationPercentile/0.50/confidenceInterval/"s << std::fixed
              << std::setfill('0') << std::setw(4) << std::setprecision(2)
              << config_->get<double>("report/medianInitialSolutionPlots/confidence")
              << "/confidence"s;

  overview << "\\begin{center}\n\\input{"
           << latexPlotter_.createPicture(successAxis, medianCostEvolutionAxis, legend).string()
           << "}\n\\captionof{figure}{\\footnotesize \\textbf{Top:} Percentage of runs that found "
              "a solution at any given time with a Clopper-Pearson (nonparametric) "
           << 100.0 * config_->get<double>("report/successPlots/confidence")
           << "\\% confidence interval. \\textbf{Bottom:} Median cost evolution and median of "
              "initial solution with nonparametric "
           << std::floor(100.0 * config_->get<double>(medianCiKey.str()))
           << "\\% confidence intervals.}\n\\end{center}\n";

  // Create the initial solution overview section.
  overview << "\\pagebreak\n";
  overview << "\\subsection{Initial Solutions}\\label{sec:overview-initial-solutions}\n";

  // Collect all initial solution duration histogram plots.
  std::vector<std::shared_ptr<pgftikz::PgfAxis>> initialSolutionDurationHistogramAxes{};
  for (const auto& name : plannerNames) {
    initialSolutionDurationHistogramAxes.push_back(
        queryTimeAtFirstHistogramPlotter_.createInitialSolutionDurationHistogramAxis(name));
  }
  latexPlotter_.alignAbszissen(initialSolutionDurationHistogramAxes);
  latexPlotter_.alignOrdinates(initialSolutionDurationHistogramAxes);
  initialSolutionDurationHistogramAxes.push_back(legend);
  latexPlotter_.stack(initialSolutionDurationHistogramAxes);

  overview << "\\begin{center}\n\\input{"
           << latexPlotter_.createPicture(initialSolutionDurationHistogramAxes).string()
           << "}\n\\captionof{figure}{\\footnotesize Histograms of "
              "initial solution times.}\n\\end{center}";

  return overview;
}

std::stringstream SingleQueryReport::individualResults() const {
  std::stringstream results;

  // Create a section for every planner.
  const auto& plannerNames = config_->get<std::vector<std::string>>("experiment/planners");
  for (const auto& name : plannerNames) {
    // Create the section title on a new page.
    results << "\n\\pagebreak\n";
    results << "\\section{" << plotPlannerNames_.at(name) << "}\\label{sec:" << name << "}\n";

    // First report on the initial solutions.
    results << "\\subsection{Initial Solutions}\\label{sec:" << name << "-initial-solution}\n";

    // Overlay the histogram with the edf for the first initial durations plot.
    auto edf = querySuccessVsTimeLinePlotter_.createSuccessAxis(name);
    edf->options.xmin = stats_.getMinInitialSolutionDuration(name);
    edf->options.xmax = config_->get<double>(
        "context/"s + config_->get<std::string>("experiment/context") + "/maxTime");
    edf->options.ytickPos = "left";
    auto histo = queryTimeAtFirstHistogramPlotter_.createInitialSolutionDurationHistogramAxis(name);
    histo->overlay(edf.get());
    for (const auto& plot : histo->getPlots()) {
      plot->options.drawOpacity = 0.2f;
      plot->options.fillOpacity = 0.1f;
    }

    // Create the scatter axis of all initial solutions.
    auto scatter =
        queryCostAtFirstVsTimeAtFirstScatterPlotter_.createInitialSolutionScatterAxis(name);

    // Create a median plot of all initial solutions.
    auto median =
        queryMedianCostAtFirstVsMedianTimeAtFirstPointPlotter_.createMedianInitialSolutionAxis(
            name);
    for (const auto& plot : median->getPlots()) {
      plot->options.markSize = 2.0;
      plot->options.lineWidth = 1.0;
    }

    // Merge the plots from the median axis into the scatter axis.
    scatter->mergePlots(median);

    // Place the scatter plot underneath the edf/histogram plot
    scatter->matchAbszisse(*edf);
    latexPlotter_.stack(edf, scatter);

    // Enlarge all x axis limits such that the earliest and latest initial solutions aren't glued to
    // the axis box.
    histo->options.enlargeXLimits = "lower";
    edf->options.enlargeXLimits = "lower";
    scatter->options.enlargeXLimits = "lower";

    std::stringstream initialCiKey;
    initialCiKey << "statistics/percentiles/sampleSize/"s << stats_.getNumRunsPerPlanner()
                 << "/populationPercentile/0.50/confidenceInterval/"s << std::fixed
                 << std::setfill('0') << std::setw(4) << std::setprecision(2)
                 << config_->get<double>("report/medianInitialSolutionPlots/confidence")
                 << "/confidence"s;

    // Create a picture out of the three initial solution axes.
    results << "\\begin{center}\n\\input{"
            << latexPlotter_.createPicture(edf, histo, scatter).string()
            << "}\n\\captionof{figure}{\\footnotesize \\textbf{Top:} Histogram and associated "
               "empirical distribution function (EDF) of "
            << plotPlannerNames_.at(name) << " with a Clopper-Pearson (nonparametric) "
            << 100.0 * config_->get<double>("report/successPlots/confidence")
            << "\\% confidence interval for the underlying CDF. \\textbf{Bottom:} All initial "
               "solutions of "
            << plotPlannerNames_.at(name) << " and their median with a nonparametric "
            << std::floor(100.0 * config_->get<double>(initialCiKey.str()))
            << "\\% confidence interval.}\n\\end{center}\n";

    // Show the cost evolution plots for anytime planners.
    if (config_->get<bool>("planner/"s + name + "/isAnytime"s)) {
      // Cost evolution plots.
      auto medianEvolution = queryMedianCostVsTimeLinePlotter_.createMedianCostEvolutionAxis(name);
      auto percentileEvolution =
          queryPercentileCostVsTimeLinePlotter_.createCostPercentileEvolutionAxis(name);
      medianEvolution->matchAbszisse(*percentileEvolution);
      latexPlotter_.stack(medianEvolution, percentileEvolution);

      std::stringstream costCiKey;
      costCiKey << "statistics/percentiles/sampleSize/"s << stats_.getNumRunsPerPlanner()
                << "/populationPercentile/0.50/confidenceInterval/"s << std::fixed
                << std::setfill('0') << std::setw(4) << std::setprecision(2)
                << config_->get<double>("report/medianCostPlots/confidence") << "/confidence"s;

      results << "\\subsection{Cost Evolution}\\label{sec:" << name << "-cost-evolution}\n";
      results << "\\begin{center}\n\\input{"
              << latexPlotter_.createPicture(medianEvolution, percentileEvolution).string()
              << "}\n\\captionof{figure}{\\footnotesize \\textbf{Top:} Median cost evolution of "
              << plotPlannerNames_.at(name) << " with a nonparametric "
              << std::floor(100.0 * config_->get<double>(costCiKey.str()))
              << "\\% confidence interval. \\textbf{Bottom:} Seven percentiles of the cost "
                 "evolution of "
              << plotPlannerNames_.at(name) << ".}\\end{center}\n";
    }
  }

  return results;
}

}  // namespace reports

}  // namespace pdt
