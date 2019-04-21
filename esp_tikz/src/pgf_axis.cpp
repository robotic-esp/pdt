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

#include "esp_tikz/pgf_axis.h"

#include <sstream>

namespace esp {

namespace ompltools {

using namespace std::string_literals;

std::string PgfAxisOptions::string() const {
  std::ostringstream stream{};
  stream << "\n  width=" << width << ",\n  height=" << height << ",\n  at={" << at << '}'
         << ",\n  unbounded coords=" << unboundedCoords;
  if (name != ""s) {
    stream << ",\n  name=" << name;
  }
  if (anchor != ""s) {
    stream << ",\n  anchor=" << anchor;
  }
  if (hideAxis) {
    stream << ",\n  hide axis";
  }
  if (scaleOnlyAxis) {
    stream << ",\n  scale only axis";
  }
  if (xmajorgrids) {
    stream << ",\n  xmajorgrids";
  }
  if (xminorgrids) {
    stream << ",\n  xminorgrids";
  }
  if (ymajorgrids) {
    stream << ",\n  ymajorgrids";
  }
  if (yminorgrids) {
    stream << ",\n  yminorgrids";
  }
  if (xmajorgrids || ymajorgrids) {
    stream << ",\n  major grid style=" << majorGridStyle;
  }
  if (xminorgrids || yminorgrids) {
    stream << ",\n  minor grid style=" << minorGridStyle;
  }
  if (xmin != std::numeric_limits<double>::infinity()) {
    stream << ",\n  xmin=" << xmin;
  }
  if (xmax != std::numeric_limits<double>::infinity()) {
    stream << ",\n  xmax=" << xmax;
  }
  if (ymin != std::numeric_limits<double>::infinity()) {
    stream << ",\n  ymin=" << ymin;
  }
  if (ymax != std::numeric_limits<double>::infinity()) {
    stream << ",\n  ymax=" << ymax;
  }
  if (xlog) {
    stream << ",\n  xmode=log";
  }
  if (ylog) {
    stream << ",\n  ymode=log";
  }
  if (xlabel != ""s) {
    stream << ",\n  xlabel={" << xlabel << '}';
  }
  if (xlabelStyle != ""s) {
    stream << ",\n  xlabel style={" << xlabelStyle << '}';
  }
  if (xlabelAbsolute) {
    stream << ",\n  xlabel absolute";
  }
  if (xtick != ""s) {
    stream << ",\n  xtick={" << xtick << '}';
  }
  if (xticklabel != ""s) {
    stream << ",\n  xticklabel={" << xticklabel << '}';
  }
  if (xticklabelStyle != ""s) {
    stream << ",\n  xticklabel style={" << xticklabelStyle << '}';
  }
  if (ylabel != ""s) {
    stream << ",\n  ylabel={" << ylabel << '}';
  }
  if (ylabelStyle != ""s) {
    stream << ",\n  ylabel style={" << ylabelStyle << '}';
  }
  if (ylabelAbsolute) {
    stream << ",\n  ylabel absolute";
  }
  if (ytick != ""s) {
    stream << ",\n  ytick={" << ytick << '}';
  }
  if (yticklabel != ""s) {
    stream << ",\n  yticklabel={" << yticklabel << '}';
  }
  if (yticklabelStyle != ""s) {
    stream << ",\n  yticklabel style={" << yticklabelStyle << '}';
  }
  if (legendStyle != ""s) {
    stream << ",\n  legend style={" << legendStyle << '}';
  }
  if (xshift != ""s) {
    stream << ",\n  xshift=" << xshift;
  }
  if (yshift != ""s) {
    stream << ",\n  yshift=" << yshift;
  }
  return stream.str();
}

void PgfAxis::setOptions(const PgfAxisOptions& options) {
  options_ = options;
}

void PgfAxis::addPlot(const std::shared_ptr<PgfPlot>& plot) {
  plots_.emplace_back(plot);
}

void PgfAxis::addLegendEntry(const std::string& entry, const std::string& imageOptions) {
  legendEntries_.emplace_back(entry, imageOptions);
}

std::string PgfAxis::string() const {
  std::ostringstream stream{};
  stream << "\\begin{axis} [" << options_.string() << "\n]\n\n";
  for (const auto& plot : plots_) {
    stream << plot->string() << '\n';
  }
  for (const auto& entry : legendEntries_) {
    if (entry.second != ""s) {
      stream << "\\addlegendimage{" << entry.second << "}\n";
    }
    stream << "\\addlegendentry{" << entry.first << "}\n";
  }
  stream << "\\end{axis}\n";
  return stream.str();
}

}  // namespace ompltools

}  // namespace esp
