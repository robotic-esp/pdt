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

#include "pdt/pgftikz/pgf_plot.h"

#include <sstream>

namespace pdt {

namespace pgftikz {

using namespace std::string_literals;

PgfPlot::PgfPlot(const std::shared_ptr<PlottableInterface>& plottable) : plottable_(plottable) {
}

void PgfPlot::setLegend(const std::string& legend) {
  legend_ = legend;
}

void PgfPlot::setPlottable(const std::shared_ptr<PlottableInterface>& plottable) {
  plottable_ = plottable;
}

std::string PgfPlot::string() const {
  if (empty()) {
    return {};
  }
  std::ostringstream stream{};
  stream << "\\addplot [\n";
  stream << "  line width=" << options.lineWidth << ",\n  color=" << options.color
         << ",\n  mark=" << options.mark;
  if (options.mark != "\"none\""s) {
    stream << ",\n  mark size=" << options.markSize;
  }
  if (options.onlyMarks) {
    stream << ",\n  only marks";
  }
  if (options.forgetPlot) {
    stream << ",\n  forget plot";
  }
  if (options.constPlot) {
    stream << ",\n  const plot";
  }
  if (options.dashed) {
    stream << ",\n  dashed";
  }
  if (options.dotted) {
    stream << ",\n  dotted";
  }
  if (options.denselyDashed) {
    stream << ",\n  densely dashed";
  }
  if (options.denselyDotted) {
    stream << ",\n  densely dotted";
  }
  if (options.looselyDashed) {
    stream << ",\n  loosely dashed";
  }
  if (options.looselyDotted) {
    stream << ",\n  loosely dotted";
  }
  if (options.namePath != ""s) {
    stream << ",\n  name path={" << options.namePath << '}';
  }
  if (options.fill != ""s) {
    stream << ",\n  fill=" << options.fill;
  }
  if (options.fillOpacity != 1.0) {
    stream << ",\n  fill opacity=" << options.fillOpacity;
  }
  if (options.drawOpacity != 1.0) {
    stream << ",\n  draw opacity=" << options.drawOpacity;
  }
  stream << "\n] ";
  stream << plottable_->string();
  if (legend_ != ""s) {
    stream << "\\addlegendentry{" << legend_ << "}\n";
  }
  return stream.str();
}

bool PgfPlot::empty() const {
  return !static_cast<bool>(plottable_);
}

}  // namespace pgftikz

}  // namespace pdt
