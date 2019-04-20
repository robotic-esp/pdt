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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "esp_tikz/pgf_plot.h"

namespace esp {

namespace ompltools {

struct PgfAxisOptions {
  std::string string() const;
  // General options.
  std::string at{"(0cm, 0cm)"};
  std::string anchor{""};
  std::string width{"\\textwidth"};
  std::string height{"0.5\\textwidth"};
  bool hideAxis{false};
  std::string majorGridStyle{"{densely dotted, black!20}"};
  std::string minorGridStyle{"{densely dotted, black!20}"};
  bool scaleOnlyAxis{false};
  std::string legendStyle{""};
  std::string name{""};
  std::string xshift{""};
  std::string yshift{""};
  std::string unboundedCoords{"jump"};

  // X-Axis options.
  bool xlog{false};
  bool xmajorgrids{true};
  bool xminorgrids{false};
  double xmin{std::numeric_limits<double>::infinity()};
  double xmax{std::numeric_limits<double>::infinity()};
  std::string xtick{""};
  std::string xticklabel{""};
  std::string xticklabelStyle{"font=\\footnotesize"};
  std::string xlabel{""};
  std::string xlabelStyle{"font=\\footnotesize"};

  // Y-Axis options.
  bool ylog{false};
  bool ymajorgrids{true};
  bool yminorgrids{false};
  double ymin{std::numeric_limits<double>::infinity()};
  double ymax{std::numeric_limits<double>::infinity()};
  std::string ytick{""};
  std::string yticklabel{""};
  std::string yticklabelStyle{"font=\\footnotesize"};
  std::string ylabel{""};
  std::string ylabelStyle{"font=\\footnotesize"};
};

class PgfAxis {
 public:
  PgfAxis() = default;
  ~PgfAxis() = default;

  void setOptions(const PgfAxisOptions& options);
  void addPlot(const std::shared_ptr<PgfPlot>& plot);
  void addLegendEntry(const std::string& entry, const std::string& imageOptions = "");

  std::string string() const;

 private:
  std::vector<std::shared_ptr<const PgfPlot>> plots_{};
  std::vector<std::pair<std::string, std::string>> legendEntries_{};
  
  PgfAxisOptions options_{};
};

}  // namespace ompltools

}  // namespace esp