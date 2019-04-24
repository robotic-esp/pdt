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

#include "esp_tikz/median_cost_success_picture.h"

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

MedianCostSuccessPicture::MedianCostSuccessPicture(
    const std::shared_ptr<const Configuration>& config) :
    TikzPicture(config),
    config_(config) {
}

// The lookup table for confidence intervals.
struct Interval {
  std::size_t lower{0u}, upper{0u};
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

fs::path MedianCostSuccessPicture::generatePlot(const Statistics& stats, std::size_t confidence) {
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
  double minDurationToBePlotted = durations.front();

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
  auto medianCostAxis = generateMedianCostPlot(stats, confidence);
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
  auto legendAxis = generateLegendAxis();
  legendAxis->setOptions(legendAxisOptions);

  // Add all axis to this plot.
  axes_.emplace_back(successAxis);
  axes_.emplace_back(medianCostAxis);
  axes_.emplace_back(legendAxis);

  // Generate the path to write to.
  auto picturePath = fs::path(config_->get<std::string>("Experiment/results")).parent_path() /
                     fs::path("tikz/"s + config_->get<std::string>("Experiment/name") +
                              "_median_cost_success_plot.tikz");
  write(picturePath);
  return picturePath;
}

std::shared_ptr<PgfAxis> MedianCostSuccessPicture::generateMedianCostPlot(
    const Statistics& stats, std::size_t confidence) const {
  // Create an axis to hold the plots.
  auto axis = std::make_shared<PgfAxis>();

  // Plot the initial solutions.
  for (const auto& name : config_->get<std::vector<std::string>>("Experiment/planners")) {
    // Load the median initial duration and cost into a table.
    auto medianInitialSolutionPath = stats.extractMedianInitialSolution(name, confidence);
    auto initialSolutionTable =
        std::make_shared<PgfTable>(medianInitialSolutionPath, "median initial solution duration",
                                   "median initial solution cost");

    // Create the pgf plot with this data and add it to the axis.
    PgfPlotOptions initialSolutionPlotOptions;
    initialSolutionPlotOptions.markSize = 0.5;
    initialSolutionPlotOptions.namePath = name + "MedianInitialSolution"s;
    initialSolutionPlotOptions.color = config_->get<std::string>("PlannerPlotColors/" + name);
    auto initialSolutionPlot = std::make_shared<PgfPlot>(initialSolutionTable);
    initialSolutionPlot->setOptions(initialSolutionPlotOptions);
    axis->addPlot(initialSolutionPlot);

    // We need to load the other values as well.
    PgfTable lowerBoundSolutionTable(medianInitialSolutionPath,
                                     "lower initial solution duration confidence bound",
                                     "lower initial solution cost confidence bound");
    PgfTable upperBoundSolutionTable(medianInitialSolutionPath,
                                     "upper initial solution duration confidence bound",
                                     "upper initial solution cost confidence bound");
    double medianDuration = initialSolutionTable->getRow(0u).at(0u);
    double medianCost = initialSolutionTable->getRow(0u).at(1u);
    double lowerDurationBound = lowerBoundSolutionTable.getRow(0u).at(0u);
    double upperDurationBound = upperBoundSolutionTable.getRow(0u).at(0u);
    double lowerCostBound = lowerBoundSolutionTable.getRow(0u).at(1u);
    double upperCostBound = upperBoundSolutionTable.getRow(0u).at(1u);

    // Create a table for the duration confidence intervals.
    auto initialSolutionDurationIntervalTable = std::make_shared<PgfTable>();
    initialSolutionDurationIntervalTable->appendRow({lowerDurationBound, medianCost});
    initialSolutionDurationIntervalTable->appendRow({upperDurationBound, medianCost});

    // Create a plot for the duration and add it to the axis.
    PgfPlotOptions initialSolutionDurationIntervalPlotOptions;
    initialSolutionDurationIntervalPlotOptions.markSize = 1.0;
    initialSolutionDurationIntervalPlotOptions.mark = "|";
    initialSolutionDurationIntervalPlotOptions.lineWidth = 0.5;
    initialSolutionPlotOptions.namePath = name + "MedianInitialSolutionDurationConfidenceInterval"s;
    initialSolutionDurationIntervalPlotOptions.color =
        config_->get<std::string>("PlannerPlotColors/" + name);
    auto initialSolutionDurationIntervalPlot =
        std::make_shared<PgfPlot>(initialSolutionDurationIntervalTable);
    initialSolutionDurationIntervalPlot->setOptions(initialSolutionDurationIntervalPlotOptions);
    axis->addPlot(initialSolutionDurationIntervalPlot);

    // Create a table for the duration confidence intervals.
    auto initialSolutionCostIntervalTable = std::make_shared<PgfTable>();
    initialSolutionCostIntervalTable->appendRow({medianDuration, lowerCostBound});
    initialSolutionCostIntervalTable->appendRow({medianDuration, upperCostBound});

    // Create a plot for the cost and add it to the axis.
    PgfPlotOptions initialSolutionCostIntervalPlotOptions;
    initialSolutionCostIntervalPlotOptions.markSize = 1.0;
    initialSolutionCostIntervalPlotOptions.mark = "-";
    initialSolutionCostIntervalPlotOptions.lineWidth = 0.5;
    initialSolutionPlotOptions.namePath = name + "MedianInitialSolutionCostConfidenceInterval"s;
    initialSolutionCostIntervalPlotOptions.color =
        config_->get<std::string>("PlannerPlotColors/" + name);
    auto initialSolutionCostIntervalPlot =
        std::make_shared<PgfPlot>(initialSolutionCostIntervalTable);
    initialSolutionCostIntervalPlot->setOptions(initialSolutionCostIntervalPlotOptions);
    axis->addPlot(initialSolutionCostIntervalPlot);
  }

  // Plot the median costs and confidence intervals.
  for (const auto& name : config_->get<std::vector<std::string>>("Experiment/planners")) {
    // This cannot be applied to planners that aren't anytime.
    if (name == "RRTConnect"s) {
      continue;
    }

    // Get the median costs file, this needs to be moved.
    auto medianCostsFile = stats.extractMedians(name);

    // Load a table from this file.
    auto medianCostsTable =
        std::make_shared<PgfTable>(medianCostsFile, "durations", "median costs");

    // Let's plot the median costs right away.
    PgfPlotOptions medianCostsPlotOptions;
    medianCostsPlotOptions.markSize = 0.0;
    medianCostsPlotOptions.color = config_->get<std::string>("PlannerPlotColors/" + name);
    medianCostsPlotOptions.namePath = name + "Median"s;
    auto medianCostsPlot = std::make_shared<PgfPlot>(medianCostsTable);
    medianCostsPlot->setOptions(medianCostsPlotOptions);
    axis->addPlot(medianCostsPlot);

    // Plot the lower bound of the confidence interval.
    auto medianLowConfidenceTable =
        std::make_shared<PgfTable>(medianCostsFile, "durations", "lower confidence bound");

    PgfPlotOptions medianLowerConfidencePlotOptions;
    medianLowerConfidencePlotOptions.markSize = 0.0;
    medianLowerConfidencePlotOptions.lineWidth = 0.5;
    medianLowerConfidencePlotOptions.color = config_->get<std::string>("PlannerPlotColors/" + name);
    medianLowerConfidencePlotOptions.fillOpacity = 0.0;
    medianLowerConfidencePlotOptions.drawOpacity = 0.2;
    medianLowerConfidencePlotOptions.namePath = name + "LowConfidence"s;
    auto medianLowConfidencePlot = std::make_shared<PgfPlot>(medianLowConfidenceTable);
    medianLowConfidencePlot->setOptions(medianLowerConfidencePlotOptions);
    axis->addPlot(medianLowConfidencePlot);

    // Plot the upper bound of the confidence interval.
    auto medianHighConfidenceTable =
        std::make_shared<PgfTable>(medianCostsFile, "durations", "upper confidence bound");

    // Replace the infinite values with very high values, otherwise they're not plotted.
    medianHighConfidenceTable->replaceInCodomain(std::numeric_limits<double>::infinity(),
                                                 3 * stats.getMaxNonInfCost());

    PgfPlotOptions medianUpperConfidencePlotOptions;
    medianUpperConfidencePlotOptions.markSize = 0.0;
    medianUpperConfidencePlotOptions.lineWidth = 0.5;
    medianUpperConfidencePlotOptions.color = config_->get<std::string>("PlannerPlotColors/" + name);
    medianUpperConfidencePlotOptions.fillOpacity = 0.0;
    medianUpperConfidencePlotOptions.drawOpacity = 0.2;
    medianUpperConfidencePlotOptions.namePath = name + "HighConfidence"s;
    auto medianHighConfidencePlot = std::make_shared<PgfPlot>(medianHighConfidenceTable);
    medianHighConfidencePlot->setOptions(medianUpperConfidencePlotOptions);
    axis->addPlot(medianHighConfidencePlot);

    // Fill the areas between the upper and lower bound
    PgfFillBetweenOptions fillBetweenOptions;
    fillBetweenOptions.name1 = medianUpperConfidencePlotOptions.namePath;
    fillBetweenOptions.name2 = medianLowerConfidencePlotOptions.namePath;
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

  return axis;
}

std::shared_ptr<PgfAxis> MedianCostSuccessPicture::generateSuccessPlot(const Statistics& stats) const {
  // Create an axis for this plot.
  auto axis = std::make_shared<PgfAxis>();

  // Only plot the sample CDF for now.
  for (const auto& name : config_->get<std::vector<std::string>>("Experiment/planners")) {
    // Get the initial solution durations.
    auto initialSolutionDurationsCdfFile = stats.extractInitialSolutionDurationCdf(name);

    // Compute the solution percentages and store them in a pgf table.
    auto successPercentagePlotTable =
        std::make_shared<PgfTable>(initialSolutionDurationsCdfFile, "durations", "cdf");
    // Multiply the cdf values by 100 to get the percentage.
    successPercentagePlotTable->replaceInCodomain([](double number) { return 100.0 * number; });

    // A zero in the domain will result in a jumped coordinate, because its a logarithmic plot.
    successPercentagePlotTable->replaceInDomain(0.0, 1e-9);

    // Remove if rows for which domain is infinite.
    successPercentagePlotTable->removeRowIfDomainEquals(std::numeric_limits<double>::infinity());

    // Add a row to draw the last element.
    successPercentagePlotTable->appendRow(
        {stats.getMaxDuration(),
         successPercentagePlotTable->getRow(successPercentagePlotTable->getNumRows() - 1u).at(1u)});

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

fs::path MedianCostSuccessPicture::generatePdf() const {
  // Generate the path to write to.
  auto picturePath = fs::path(config_->get<std::string>("Experiment/results")).parent_path() /
                     fs::path("tikz/"s + config_->get<std::string>("Experiment/name") +
                              "_median_cost_success_plot.tikz");
  auto texFilePath = fs::path(config_->get<std::string>("Experiment/results")).parent_path() /
                     fs::path("plots/"s + config_->get<std::string>("Experiment/name") +
                              "_median_cost_success_plot.tex");
  fs::create_directories(texFilePath.parent_path());
  std::ofstream texFile;
  texFile.open(texFilePath.c_str());

  // Check on the failbit.
  if (texFile.fail() == true) {
    auto msg = "MedianCostSuccessPlot could not open picture at '" + texFilePath.string() + "'."s;
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
  // Compiling with lualatex is slower than pdflatex but has dynamic memory allocation. Since these
  // plots can be quite large, pdflatex has run into memory issues. Lualatex should be available
  // with all major tex distributions.
  auto currentPath = fs::current_path();
  auto cmd = "cd \""s + texFilePath.parent_path().string() + "\" && lualatex \""s +
             texFilePath.string() + "\" && cd \""s + currentPath.string() + '\"';
  int retval = std::system(cmd.c_str());
  (void)retval;
  return fs::path(texFilePath).replace_extension(".pdf");
}

}  // namespace ompltools

}  // namespace esp
