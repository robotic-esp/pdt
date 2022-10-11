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

#include "pdt/pgftikz/pgf_axis.h"

#include <sstream>

namespace pdt {

namespace pgftikz {

using namespace std::string_literals;

void PgfAxis::addPlot(const std::shared_ptr<PgfPlot>& plot) {
  plots_.push_back(plot);
}

void PgfAxis::addLegendEntry(const std::string& entry, const std::string& imageOptions) {
  legendEntries_.emplace_back(entry, imageOptions);
}

void PgfAxis::setZLevel(std::size_t z) {
  zLevel_ = z;
}

std::size_t PgfAxis::getZLevel() {
  return zLevel_;
}

std::vector<std::shared_ptr<PgfPlot>> PgfAxis::getPlots() {
  return plots_;
}

void PgfAxis::overlay(PgfAxis* other) {
  // Align the abszissen.
  matchAbszisse(*other);

  // Make sure they are the same height and width.
  options.height = other->options.height;
  options.width = other->options.width;

  // Make sure they are at the same position
  options.at = other->options.at;
  options.anchor = other->options.anchor;

  // Don't display this axis' abszisse.
  options.xtick = "{\\empty}";
  options.xticklabel = "{\\empty}";
  options.axisXLine = "none";
  options.ylabelAbsolute = false;
  options.axisYLine = "right";
}

void PgfAxis::matchAbszisse(const PgfAxis& other) {
  options.xmin = other.options.xmin;
  options.xmax = other.options.xmax;
}

void PgfAxis::expandRangeOfAbszisse(const PgfAxis& other) {
  if (options.xmin > other.options.xmin) {
    options.xmin = other.options.xmin;
  }
  if (options.xmax < other.options.xmax) {
    options.xmax = other.options.xmax;
  }
}

void PgfAxis::expandRangeOfOrdinate(const PgfAxis& other) {
  if (options.ymin < other.options.ymin) {
    options.ymin = other.options.ymin;
  }
  if (options.ymax < other.options.ymax) {
    options.ymax = other.options.ymax;
  }
}

void PgfAxis::alignAbszissen(PgfAxis* first, PgfAxis* second) {
  first->expandRangeOfAbszisse(*second);
  second->expandRangeOfAbszisse(*first);
}

void PgfAxis::alignOrdinates(PgfAxis* first, PgfAxis* second) {
  first->expandRangeOfOrdinate(*second);
  second->expandRangeOfOrdinate(*first);
}

std::string PgfAxis::string() const {
  std::ostringstream stream{};
  stream << "\\begin{axis} [";
  stream << "\n  width=" << options.width << ",\n  height=" << options.height << ",\n  at={"
         << options.at << '}' << ",\n  unbounded coords=" << options.unboundedCoords
         << ",\n  xtick align=" << options.xtickAlign << ",\n  ytick align=" << options.ytickAlign
         << ",\n  axis line style=" << options.axisLineStyle;
  if (options.name != ""s) {
    stream << ",\n  name=" << options.name;
  }
  if (options.anchor != ""s) {
    stream << ",\n  anchor=" << options.anchor;
  }
  if (options.hideAxis) {
    stream << ",\n  hide axis";
  }
  if (options.barWidth != ""s) {
    stream << ",\n  bar width=" << options.barWidth;
  }
  if (options.scaleOnlyAxis) {
    stream << ",\n  scale only axis";
  }
  if (options.xmajorgrids) {
    stream << ",\n  xmajorgrids";
  }
  if (options.xminorgrids) {
    stream << ",\n  xminorgrids";
  }
  if (options.ymajorgrids) {
    stream << ",\n  ymajorgrids";
  }
  if (options.yminorgrids) {
    stream << ",\n  yminorgrids";
  }
  if (options.xmajorgrids || options.ymajorgrids) {
    stream << ",\n  major grid style=" << options.majorGridStyle;
  }
  if (options.xminorgrids || options.yminorgrids) {
    stream << ",\n  minor grid style=" << options.minorGridStyle;
  }
  if (options.majorTickLength != ""s) {
    stream << ",\n major tick length=" << options.majorTickLength;
  }
  if (options.minorTickLength != ""s) {
    stream << ",\n minor tick length=" << options.minorTickLength;
  }
  if (options.xmin != std::numeric_limits<double>::infinity()) {
    stream << ",\n  xmin=" << options.xmin;
  }
  if (options.xmax != std::numeric_limits<double>::infinity()) {
    stream << ",\n  xmax=" << options.xmax;
  }
  if (options.enlargeXLimits != "false"s) {
    stream << ",\n  enlarge x limits=" << options.enlargeXLimits;
  }
  if (options.ymin != std::numeric_limits<double>::infinity()) {
    stream << ",\n  ymin=" << options.ymin;
  }
  if (options.ymax != std::numeric_limits<double>::infinity()) {
    stream << ",\n  ymax=" << options.ymax;
  }
  if (options.enlargeYLimits != "false"s) {
    stream << ",\n  enlarge y limits=" << options.enlargeYLimits;
  }
  if (options.xbarInterval) {
    stream << ",\n  xbar interval";
  }
  if (options.ybarInterval) {
    stream << ",\n  ybar interval";
  }
  if (options.xbar) {
    stream << ",\n  xbar";
  }
  if (options.ybar) {
    stream << ",\n  ybar";
  }
  if (options.xlog) {
    stream << ",\n  xmode=log";
  }
  if (options.ylog) {
    stream << ",\n  ymode=log";
  }
  if (options.axisXLine != ""s) {
    stream << ",\n  axis x line=" << options.axisXLine;
  }
  if (options.axisYLine != ""s) {
    stream << ",\n  axis y line*=" << options.axisYLine;
  }
  if (options.xlabel != ""s) {
    stream << ",\n  xlabel={" << options.xlabel << '}';
  }
  if (options.xlabelStyle != ""s) {
    stream << ",\n  xlabel style={" << options.xlabelStyle << '}';
  }
  if (options.xlabelAbsolute) {
    stream << ",\n  xlabel absolute";
  }
  if (options.xtick != ""s) {
    stream << ",\n  xtick={" << options.xtick << '}';
  }
  if (options.xticklabel != ""s) {
    stream << ",\n  xticklabel={" << options.xticklabel << '}';
  }
  if (options.xticklabelStyle != ""s) {
    stream << ",\n  xticklabel style={" << options.xticklabelStyle << '}';
  }
  if (options.ylabel != ""s) {
    stream << ",\n  ylabel={" << options.ylabel << '}';
  }
  if (options.ylabelStyle != ""s) {
    stream << ",\n  ylabel style={" << options.ylabelStyle << '}';
  }
  if (options.ylabelAbsolute) {
    stream << ",\n  ylabel absolute";
  }
  if (options.ytick != ""s) {
    stream << ",\n  ytick={" << options.ytick << '}';
  }
  if (options.ytickPos != ""s) {
    stream << ",\n ytick pos={" << options.ytickPos << '}';
  }
  if (options.yticklabel != ""s) {
    stream << ",\n  yticklabel={" << options.yticklabel << '}';
  }
  if (options.yticklabelStyle != ""s) {
    stream << ",\n  yticklabel style={" << options.yticklabelStyle << '}';
  }
  if (options.legendStyle != ""s) {
    stream << ",\n  legend style={" << options.legendStyle << '}';
  }
  if (options.xshift != ""s) {
    stream << ",\n  xshift=" << options.xshift;
  }
  if (options.yshift != ""s) {
    stream << ",\n  yshift=" << options.yshift;
  }
  stream << "\n]\n";
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

}  // namespace pgftikz

}  // namespace pdt
