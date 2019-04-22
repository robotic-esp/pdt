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

#include "esp_tikz/median_cost_success_plot.h"

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

MedianCostSuccessPlot::MedianCostSuccessPlot(const std::shared_ptr<Configuration>& config) :
    TikzPicture(config),
    config_(config) {
}

// The lookup table for confidence intervals.
struct Interval {
  std::size_t low{0u}, high{0u};
  float probability{0.0f};
};
static const std::map<std::size_t, std::map<std::size_t, Interval>> medianConfidenceIntervals = {
    {10u, {{95u, {1u, 8u, 0.9511}}, {99u, {0u, 9u, 0.9910}}}},
    {50u, {{95u, {18u, 32u, 0.9511}}, {99u, {15u, 34u, 0.9910}}}},
    {100u, {{95u, {40u, 60u, 0.9540}}, {99u, {37u, 63u, 0.9907}}}},
    {200u, {{95u, {86u, 114u, 0.9520}}, {99u, {81u, 118u, 0.9906}}}},
    {250u, {{95u, {110u, 141u, 0.9503}}, {99u, {104u, 145u, 0.9900}}}},
    {300u, {{95u, {133u, 167u, 0.9502}}, {99u, {127u, 172u, 0.9903}}}},
    {400u, {{95u, {179u, 219u, 0.9522}}, {99u, {174u, 226u, 0.9907}}}},
    {500u, {{95u, {228u, 272u, 0.9508}}, {99u, {221u, 279u, 0.9905}}}},
    {600u, {{95u, {274u, 323u, 0.9508}}, {99u, {267u, 331u, 0.9907}}}},
    {700u, {{95u, {324u, 376u, 0.9517}}, {99u, {314u, 383u, 0.9901}}}},
    {800u, {{95u, {371u, 427u, 0.9511}}, {99u, {363u, 436u, 0.9900}}}},
    {900u, {{95u, {420u, 479u, 0.9503}}, {99u, {410u, 488u, 0.9904}}}},
    {1000u, {{95u, {469u, 531u, 0.9500}}, {99u, {458u, 531u, 0.9905}}}},
    {2000u, {{95u, {955u, 1043u, 0.9504}}, {99u, {940u, 1056u, 0.9901}}}},
    {5000u, {{95u, {2429u, 2568u, 0.9503}}, {99u, {2406u, 2589u, 0.9901}}}},
    {10000u, {{95u, {4897u, 5094u, 0.9500}}, {99u, {4869u, 5127u, 0.9900}}}},
    {100000u, {{95u, {49687u, 50307u, 0.9500}}, {99u, {49588u, 50403u, 0.9900}}}},
    {1000000u, {{95u, {499018u, 500978u, 0.9500}}, {99u, {498707u, 501283u, 0.9900}}}}};

fs::path MedianCostSuccessPlot::generatePlot(const Statistics& stats,
                                             std::size_t confidence) {
  // Compute the duration bin size.
  auto contextName = config_->get<std::string>("Experiment/context");
  std::size_t numMeasurements =
      std::ceil(config_->get<double>("Contexts/" + contextName + "/maxTime") *
                config_->get<double>("Experiment/logFrequency"));
  double binSize = 1.0 / config_->get<double>("Experiment/logFrequency");
  std::vector<double> durations;
  durations.reserve(numMeasurements);
  for (std::size_t i = 0u; i < numMeasurements; ++i) {
    durations.emplace_back(static_cast<double>(i + 1u) * binSize);
  }

  // Determine the min and max durations to be plotted.
  double maxDurationToBePlotted = durations.back();
  double minDurationToBePlotted{std::numeric_limits<double>::infinity()};
  for (const auto& name : stats.getPlannerNames()) {
    double minDuration = stats.getNthInitialSolutionDuration(name, 0u);
    if (minDuration < minDurationToBePlotted) {
      minDurationToBePlotted = minDuration;
    }
  }

  // Set the axis options for the success plot.
  PgfAxisOptions successAxisOptions;
  successAxisOptions.name = "SuccessAxis";
  successAxisOptions.height = "0.4\\textwidth";
  successAxisOptions.xmin = minDurationToBePlotted;
  successAxisOptions.xmax = maxDurationToBePlotted;
  successAxisOptions.xlog = true;
  successAxisOptions.xminorgrids = true;
  successAxisOptions.xlabel = "{\\empty}"s;
  successAxisOptions.xticklabel = "{\\empty}"s;
  successAxisOptions.ymin = 0;
  successAxisOptions.ymax = 100.2;
  successAxisOptions.ytick = "0,25,50,75,100";
  successAxisOptions.ylabel = "Success [\\%]"s;
  successAxisOptions.ylabelAbsolute = true;
  successAxisOptions.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
  auto successAxis = generateSuccessPlot(stats);
  successAxis->setOptions(successAxisOptions);

  // Set the axis options for the median cost plot.
  PgfAxisOptions medianCostAxisOptions;
  medianCostAxisOptions.at = "($(SuccessAxis.south) - (0.0em, 0.3em)$)";
  medianCostAxisOptions.anchor = "north";
  medianCostAxisOptions.name = "MedianCostAxis";
  medianCostAxisOptions.xmin = minDurationToBePlotted;
  medianCostAxisOptions.xmax = maxDurationToBePlotted;
  medianCostAxisOptions.ymax = stats.getMaxNonInfCost();
  medianCostAxisOptions.xlog = true;
  medianCostAxisOptions.xminorgrids = true;
  medianCostAxisOptions.xlabel = "Computation time [s]";
  medianCostAxisOptions.ylabel = "Median cost";
  medianCostAxisOptions.ylabelAbsolute = true;
  medianCostAxisOptions.ylabelStyle = "font=\\footnotesize, text depth=0.0em, text height=0.5em";
  auto medianCostAxis = generateMedianCostPlot(stats, durations, confidence);
  medianCostAxis->setOptions(medianCostAxisOptions);

  // Create the axis that holds the plot legend.
  PgfAxisOptions legendAxisOptions;
  legendAxisOptions.at = "($(MedianCostAxis.south) - (0.0em, 0.3em)$)";
  legendAxisOptions.anchor = "north";
  legendAxisOptions.name = "LegendAxis";
  legendAxisOptions.xmin = minDurationToBePlotted;
  legendAxisOptions.xmax = maxDurationToBePlotted;
  legendAxisOptions.ymin = 1;
  legendAxisOptions.ymax = 10;
  legendAxisOptions.hideAxis = true;
  legendAxisOptions.legendStyle =
      "anchor=north, legend cell align=left, legend columns=6, at={($(MedianCostAxis.south) + "
      "(0.0em," +
      legendAxisOptions.height + " + 5em)$)}";  // Why do I have to shift so far up?
  auto legendAxis = generateLegendAxis(stats);
  legendAxis->setOptions(legendAxisOptions);

  // Add all axis to this plot.
  axes_.emplace_back(successAxis);
  axes_.emplace_back(medianCostAxis);
  axes_.emplace_back(legendAxis);

  // Generate the path to write to.
  auto picturePath = fs::path(config_->get<std::string>("Experiment/results")).parent_path() /=
      fs::path(config_->get<std::string>("Experiment/results")).stem() +=
      "_median_cost_success_plot.tikz"s;
  write(picturePath);
  return picturePath;
}

std::shared_ptr<PgfAxis> MedianCostSuccessPlot::generateMedianCostPlot(
    const Statistics& stats, const std::vector<double>& durations,
    std::size_t confidence) const {
  // Create an axis to hold the plots.
  auto axis = std::make_shared<PgfAxis>();

  // Get the number of runs per planner and compute the bin durations.
  std::size_t numRunsPerPlanner = stats.getNumRunsPerPlanner();

  // Plot the initial solutions.
  for (const auto& name : stats.getPlannerNames()) {
    // Get the median costs.
    double medianInitialDuration{std::numeric_limits<double>::signaling_NaN()};
    double medianInitialCost{std::numeric_limits<double>::signaling_NaN()};
    if (numRunsPerPlanner % 2 == 1) {
      medianInitialDuration =
          stats.getNthInitialSolutionDuration(name, (numRunsPerPlanner + 1u) / 2);
      medianInitialCost = stats.getNthInitialSolutionCost(name, (numRunsPerPlanner + 1u) / 2);
    } else {
      auto lowMedianInitialDuration =
          stats.getNthInitialSolutionDuration(name, numRunsPerPlanner / 2);
      auto highMedianInitialDuration =
          stats.getNthInitialSolutionDuration(name, (numRunsPerPlanner + 2u) / 2);
      auto lowMedianInitialCost = stats.getNthInitialSolutionCost(name, numRunsPerPlanner / 2);
      auto highMedianInitialCost =
          stats.getNthInitialSolutionCost(name, (numRunsPerPlanner + 2u) / 2);
      medianInitialDuration = (lowMedianInitialDuration + highMedianInitialDuration) / 2.0;
      medianInitialCost = (lowMedianInitialCost + highMedianInitialCost) / 2.0;
    }

    // Store the duration and cost in a table.
    auto initialSolutionTable = std::make_shared<PgfTable>();
    initialSolutionTable->addColumn({medianInitialDuration});
    initialSolutionTable->addColumn({medianInitialCost});

    // Create the pgf plot with this data and add it to the axis.
    PgfPlotOptions initialSolutionPlotOptions;
    initialSolutionPlotOptions.markSize = 0.5;
    initialSolutionPlotOptions.color = config_->get<std::string>("PlannerPlotColors/" + name);
    auto initialSolutionPlot = std::make_shared<PgfPlot>(initialSolutionTable);
    initialSolutionPlot->setOptions(initialSolutionPlotOptions);
    axis->addPlot(initialSolutionPlot);

    if (medianConfidenceIntervals.find(numRunsPerPlanner) == medianConfidenceIntervals.end() ||
        (confidence != 95u && confidence != 99u)) {
      OMPL_WARN(
          "Median confidence intervals with %d percent confidence not precomputed for %d runs.",
          confidence, numRunsPerPlanner);
    } else {
      // Get the interval indices.
      auto interval = medianConfidenceIntervals.at(numRunsPerPlanner).at(confidence);

      // Get the corresponding durations and cost.
      auto lowerIntervalDuration = stats.getNthInitialSolutionDuration(name, interval.low);
      auto upperIntervalDuration = stats.getNthInitialSolutionDuration(name, interval.high);
      auto lowerIntervalCost = stats.getNthInitialSolutionCost(name, interval.low);
      auto upperIntervalCost = stats.getNthInitialSolutionCost(name, interval.high);

      // Store the duration interval in a table.
      auto initialSolutionDurationIntervalTable = std::make_shared<PgfTable>();
      initialSolutionDurationIntervalTable->addRow({lowerIntervalDuration, medianInitialCost});
      initialSolutionDurationIntervalTable->addRow({upperIntervalDuration, medianInitialCost});

      // Store the Cost interval in a table.
      auto initialSolutionCostIntervalTable = std::make_shared<PgfTable>();
      initialSolutionCostIntervalTable->addRow({medianInitialDuration, lowerIntervalCost});
      initialSolutionCostIntervalTable->addRow({medianInitialDuration, upperIntervalCost});

      // Create a plot for the duration and add it to the axis.
      PgfPlotOptions initialSolutionDurationIntervalPlotOptions;
      initialSolutionDurationIntervalPlotOptions.markSize = 1.0;
      initialSolutionDurationIntervalPlotOptions.mark = "|";
      initialSolutionDurationIntervalPlotOptions.lineWidth = 0.5;
      initialSolutionDurationIntervalPlotOptions.color =
          config_->get<std::string>("PlannerPlotColors/" + name);
      auto initialSolutionDurationIntervalPlot =
          std::make_shared<PgfPlot>(initialSolutionDurationIntervalTable);
      initialSolutionDurationIntervalPlot->setOptions(initialSolutionDurationIntervalPlotOptions);
      axis->addPlot(initialSolutionDurationIntervalPlot);

      // Create a plot for the cost and add it to the axis.
      PgfPlotOptions initialSolutionCostIntervalPlotOptions;
      initialSolutionCostIntervalPlotOptions.markSize = 1.0;
      initialSolutionCostIntervalPlotOptions.mark = "-";
      initialSolutionCostIntervalPlotOptions.lineWidth = 0.5;
      initialSolutionCostIntervalPlotOptions.color =
          config_->get<std::string>("PlannerPlotColors/" + name);
      auto initialSolutionCostIntervalPlot =
          std::make_shared<PgfPlot>(initialSolutionCostIntervalTable);
      initialSolutionCostIntervalPlot->setOptions(initialSolutionCostIntervalPlotOptions);
      axis->addPlot(initialSolutionCostIntervalPlot);
    }
  }

  // Plot the median costs and confidence intervals.
  for (const auto& name : stats.getPlannerNames()) {
    // This cannot be applied to planners that aren't anytime.
    if (name == "RRTConnect"s) {
      continue;
    }
    // Get the median costs.
    std::vector<double> medianCosts;
    medianCosts.resize(durations.size(), std::numeric_limits<double>::signaling_NaN());
    if (numRunsPerPlanner % 2 == 1) {
      medianCosts = stats.getNthCosts(name, (numRunsPerPlanner - 1u) / 2, durations);
    } else {
      std::vector<double> lowMedianCosts =
          stats.getNthCosts(name, numRunsPerPlanner / 2, durations);
      std::vector<double> highMedianCosts =
          stats.getNthCosts(name, (numRunsPerPlanner + 2u) / 2, durations);
      for (std::size_t i = 0u; i < medianCosts.size(); ++i) {
        medianCosts.at(i) = (lowMedianCosts.at(i) + highMedianCosts.at(i)) / 2.0;
      }
    }

    // Convert this data into a table.
    auto medianCostsTable = std::make_shared<PgfTable>();
    medianCostsTable->addColumn(durations);
    medianCostsTable->addColumn(medianCosts);

    // Let's plot the median costs right away.
    PgfPlotOptions medianCostsPlotOptions;
    medianCostsPlotOptions.markSize = 0.0;
    medianCostsPlotOptions.color = config_->get<std::string>("PlannerPlotColors/" + name);
    medianCostsPlotOptions.namePath = name + "Median"s;
    auto medianCostsPlot = std::make_shared<PgfPlot>(medianCostsTable);
    medianCostsPlot->setOptions(medianCostsPlotOptions);
    axis->addPlot(medianCostsPlot);

    // Get the confidence interval costs.
    if (medianConfidenceIntervals.find(numRunsPerPlanner) == medianConfidenceIntervals.end() ||
        (confidence != 95u && confidence != 99u)) {
      OMPL_WARN(
          "Median confidence intervals with %d percent confidence not precomputed for %d runs.",
          confidence, numRunsPerPlanner);
    } else {
      // Get the interval indices.
      auto interval = medianConfidenceIntervals.at(numRunsPerPlanner).at(confidence);

      // Get the costs for the intervals.
      std::vector<double> intervalLowCosts = stats.getNthCosts(name, interval.low, durations);
      std::vector<double> intervalHighCosts = stats.getNthCosts(name, interval.high, durations);

      // If the median cost is infinity, the low and high costs are as well.
      for (std::size_t i = 0u; i < medianCosts.size(); ++i) {
        if (medianCosts.at(i) == std::numeric_limits<double>::infinity()) {
          intervalLowCosts.at(i) = std::numeric_limits<double>::infinity();
        } else if (intervalHighCosts.at(i) == std::numeric_limits<double>::infinity()) {
          intervalHighCosts.at(i) = stats.getMaxNonInfCost() + 10.0;
        }
      }

      // Plot the lower bound of the confidence interval.
      auto medianLowConfidenceTable = std::make_shared<PgfTable>();
      medianLowConfidenceTable->addColumn(durations);
      medianLowConfidenceTable->addColumn(intervalLowCosts);

      PgfPlotOptions medianLowConfidencePlotOptions;
      medianLowConfidencePlotOptions.markSize = 0.0;
      medianLowConfidencePlotOptions.lineWidth = 0.5;
      medianLowConfidencePlotOptions.color = config_->get<std::string>("PlannerPlotColors/" + name);
      medianLowConfidencePlotOptions.fillOpacity = 0.0;
      medianLowConfidencePlotOptions.drawOpacity = 0.2;
      medianLowConfidencePlotOptions.namePath = name + "LowConfidence"s;
      auto medianLowConfidencePlot = std::make_shared<PgfPlot>(medianLowConfidenceTable);
      medianLowConfidencePlot->setOptions(medianLowConfidencePlotOptions);
      axis->addPlot(medianLowConfidencePlot);

      // Plot the upper bound of the confidence interval.
      auto medianHighConfidenceTable = std::make_shared<PgfTable>();
      medianHighConfidenceTable->addColumn(durations);
      medianHighConfidenceTable->addColumn(intervalHighCosts);

      PgfPlotOptions medianHighConfidencePlotOptions;
      medianHighConfidencePlotOptions.markSize = 0.0;
      medianHighConfidencePlotOptions.lineWidth = 0.5;
      medianHighConfidencePlotOptions.color =
          config_->get<std::string>("PlannerPlotColors/" + name);
      medianHighConfidencePlotOptions.fillOpacity = 0.0;
      medianHighConfidencePlotOptions.drawOpacity = 0.2;
      medianHighConfidencePlotOptions.namePath = name + "HighConfidence"s;
      auto medianHighConfidencePlot = std::make_shared<PgfPlot>(medianHighConfidenceTable);
      medianHighConfidencePlot->setOptions(medianHighConfidencePlotOptions);
      axis->addPlot(medianHighConfidencePlot);

      // Fill the areas between the upper and lower bound
      PgfFillBetweenOptions fillBetweenOptions;
      fillBetweenOptions.name1 = medianHighConfidencePlotOptions.namePath;
      fillBetweenOptions.name2 = medianLowConfidencePlotOptions.namePath;
      auto fillBetween = std::make_shared<PgfFillBetween>();
      fillBetween->setOptions(fillBetweenOptions);
      PgfPlotOptions confidenceIntervalFillPlotOptions;
      confidenceIntervalFillPlotOptions.color =
          config_->get<std::string>("PlannerPlotColors/" + name);
      confidenceIntervalFillPlotOptions.fillOpacity = 0.1;
      confidenceIntervalFillPlotOptions.drawOpacity = 0.0;
      auto confidenceIntervalFillPlot = std::make_shared<PgfPlot>(fillBetween);
      confidenceIntervalFillPlot->setOptions(confidenceIntervalFillPlotOptions);
      axis->addPlot(confidenceIntervalFillPlot);
    }
  }

  return axis;
}

std::shared_ptr<PgfAxis> MedianCostSuccessPlot::generateSuccessPlot(
    const Statistics& stats) const {
  // Create an axis for this plot.
  auto axis = std::make_shared<PgfAxis>();

  // Only plot the sample CDF for now.
  for (const auto& name : stats.getPlannerNames()) {
    // Get the initial solution durations.
    auto initialSolutionDurations = stats.getInitialSolutionDurations(name);

    // I don't think there's a way around sorting them.
    std::sort(initialSolutionDurations.begin(), initialSolutionDurations.end());

    // Prepare variable to calculate solution percentages.
    double successPercentage = 0.0;
    double numSolvedRuns = 0.0;
    auto numRunsPerPlanner = static_cast<double>(stats.getNumRunsPerPlanner());

    // Compute the solution percentages and store them in a pgf table.
    auto successPercentagePlotTable = std::make_shared<PgfTable>();
    successPercentagePlotTable->addRow({0.1e-9, 0.0});
    for (const auto duration : initialSolutionDurations) {
      numSolvedRuns += 1.0;
      successPercentage = numSolvedRuns / numRunsPerPlanner * 100.0;
      successPercentagePlotTable->addRow({duration, successPercentage});
    }
    successPercentagePlotTable->addRow({stats.getMaxDuration(), successPercentage});

    // Create the plot and add it to the axis.
    PgfPlotOptions successPercentagePlotOptions;
    successPercentagePlotOptions.markSize = 0.0;
    successPercentagePlotOptions.color = config_->get<std::string>("PlannerPlotColors/" + name);
    successPercentagePlotOptions.namePath = name + "Success"s;
    auto successPercentagePlot = std::make_shared<PgfPlot>(successPercentagePlotTable);
    successPercentagePlot->setOptions(successPercentagePlotOptions);
    axis->addPlot(successPercentagePlot);
  }

  return axis;
}

std::shared_ptr<PgfAxis> MedianCostSuccessPlot::generateLegendAxis(
    const Statistics& stats) const {
  auto legendAxis = std::make_shared<PgfAxis>();

  // Make sure the names are alphabetic.
  auto plannerNames = stats.getPlannerNames();
  std::sort(plannerNames.begin(), plannerNames.end());
  for (const auto& name : plannerNames) {
    std::string imageOptions{config_->get<std::string>("PlannerPlotColors/" + name) +
                             ", line width = 1.0pt, mark size=1.0pt, mark=square*"};
    legendAxis->addLegendEntry(config_->get<std::string>("PlannerPlotNames/" + name), imageOptions);
  }
  return legendAxis;
}

fs::path MedianCostSuccessPlot::generatePdf() const {
  // Generate the path to write to.
  auto picturePath = fs::path(config_->get<std::string>("Experiment/results")).parent_path() /=
      fs::path(config_->get<std::string>("Experiment/results")).stem() +=
      "_median_cost_success_plot.tikz"s;
  auto texFilePath = fs::path(config_->get<std::string>("Experiment/results")).parent_path() /=
      fs::path(config_->get<std::string>("Experiment/results")).stem() +=
      "_median_cost_success_plot.tex"s;
  std::ofstream texFile;
  texFile.open(texFilePath.c_str());

  // Check on the failbit.
  if (texFile.fail() == true) {
    auto msg = "MedianCostSuccessPlot could not open picture at '" + texFilePath.string() + "'."s;
    throw std::ios_base::failure(msg);
  }

  // Write the preamble.
  texFile << "\\documentclass{standalone}\n"
          << "\\usepackage{tikz}\n"
          << "\\usetikzlibrary{calc,plotmarks}\n"
          << "\\usepackage{pgfplots}\n"
          << "\\pgfplotsset{compat=1.15}\n"
          << "\\usepgfplotslibrary{fillbetween}\n"
          << "\\usepackage{xcolor}\n";

  // Include the picture.
  texFile << "\n\n\\begin{document}\n\n";
  texFile << "\n\\input{" << picturePath.string() << "}\n";
  texFile << "\n\n\\end{document}\n";

  // Close the file.
  texFile.close();

  // Compile the plot.
  auto currentPath = std::experimental::filesystem::current_path();
  auto cmd = "cd "s + texFilePath.parent_path().string() +
             " && pdflatex -file-line-error -interaction=nonstopmode "s +
             texFilePath.filename().string() + " && cd "s + currentPath.string();
  int retval = std::system(cmd.c_str());
  (void)retval;
  return fs::path(texFilePath).replace_extension(".pdf");
}

}  // namespace ompltools

}  // namespace esp