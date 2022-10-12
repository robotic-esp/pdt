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

#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "pdt/pgftikz/pgf_plot.h"

namespace pdt {

namespace pgftikz {

class PgfAxis {
 public:
  PgfAxis() = default;
  ~PgfAxis() = default;

  void addPlot(const std::shared_ptr<PgfPlot>& plot);
  void addLegendEntry(const std::string& entry, const std::string& imageOptions = "");

  void setZLevel(std::size_t z);
  std::size_t getZLevel();

  template <typename... Axes>
  void mergePlots(Axes... args);

  std::vector<std::shared_ptr<PgfPlot>> getPlots();

  // Places this axis on top of the other.
  void overlay(PgfAxis* other);

  // Matches the range of this abszisse to that of the other.
  void matchAbszisse(const PgfAxis& other);

  // Expands the range of the abszisse to include the range of the abszisse of the other axis.
  void expandRangeOfAbszisse(const PgfAxis& other);

  // Expands the range of the ordinate to include the range of the ordinate of the other axis.
  void expandRangeOfOrdinate(const PgfAxis& other);

  std::string string() const;

  static void alignAbszissen(PgfAxis* first, PgfAxis* second);
  static void alignOrdinates(PgfAxis* first, PgfAxis* second);

 private:
  std::vector<std::shared_ptr<PgfPlot>> plots_{};
  std::vector<std::pair<std::string, std::string>> legendEntries_{};
  std::size_t zLevel_;

 public:
  struct {
    // General options.
    std::string at{"(0cm, 0cm)"};
    std::string anchor{""};
    std::string width{"\\textwidth"};
    std::string height{"0.5\\textwidth"};
    std::string majorGridStyle{"{densely dotted, black!20}"};
    std::string minorGridStyle{"{densely dotted, black!20}"};
    std::string axisLineStyle{"{solid, black}"};
    std::string majorTickLength{""};
    std::string minorTickLength{""};
    std::string legendStyle{""};
    std::string name{""};
    std::string xshift{""};
    std::string yshift{""};
    std::string unboundedCoords{"jump"};
    std::string barWidth{""};
    bool hideAxis{false};
    bool scaleOnlyAxis{false};

    // X-Axis options.
    bool xbar{false};
    bool xbarInterval{false};
    bool xlog{false};
    bool xmajorgrids{true};
    bool xminorgrids{false};
    double xmin{std::numeric_limits<double>::infinity()};
    double xmax{std::numeric_limits<double>::infinity()};
    std::string axisXLine{""};
    std::string xtick{""};
    std::string xtickAlign{"inside"};
    std::string xticklabel{""};
    std::string xticklabelStyle{"font=\\footnotesize"};
    std::string xlabel{""};
    std::string xlabelStyle{"font=\\footnotesize"};
    std::string enlargeXLimits{"false"};  // "auto, true, false, upper, lower, value=, abs value="
    bool xlabelAbsolute{false};

    // Y-Axis options.
    bool ybar{false};
    bool ybarInterval{false};
    bool ylog{false};
    bool ymajorgrids{true};
    bool yminorgrids{false};
    double ymin{std::numeric_limits<double>::infinity()};
    double ymax{std::numeric_limits<double>::infinity()};
    std::string axisYLine{""};
    std::string ytick{""};
    std::string ytickPos{""};
    std::string ytickAlign{"inside"};
    std::string yticklabel{""};
    std::string yticklabelStyle{"font=\\footnotesize"};
    std::string ylabel{""};
    std::string ylabelStyle{"font=\\footnotesize"};
    std::string enlargeYLimits{"false"};  // "auto, true, false, upper, lower, value=, abs value="
    bool ylabelAbsolute{false};
  } options{};
};

template <typename... Axes>
void PgfAxis::mergePlots(Axes... args) {
  // Collect the args in a vector.
  std::vector<std::shared_ptr<PgfAxis>> axes{};
  (axes.emplace_back(args), ...);

  // Add all plots of all axes, making sure all values are plotted.
  for (const auto& axis : axes) {
    expandRangeOfAbszisse(*axis);
    expandRangeOfOrdinate(*axis);

    for (const auto& plot : axis->getPlots()) {
      addPlot(plot);
    }
  }
}

}  // namespace pgftikz

}  // namespace pdt
