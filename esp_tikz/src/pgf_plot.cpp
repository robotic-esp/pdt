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

#include "esp_tikz/pgf_plot.h"

#include <sstream>

namespace esp {

namespace ompltools {

using namespace std::string_literals;

std::string PgfPlotOptions::string() const {
  std::ostringstream stream{};
  stream << "\n  line width=" << lineWidth << ",\n  color=" << color << ",\n  mark=" << mark
         << ",\n  mark size=" << markSize;
  if (onlyMarks) {
    stream << ",\n  only marks";
  }
  if (forgetPlot) {
    stream << ",\n  forget plot";
  }
  if (constPlot) {
    stream << ",\n  const plot";
  }
  if (namePath != ""s) {
    stream << ",\n  name path={" << namePath << '}';
  }
  if (fillOpacity != 1.0) {
    stream << ",\n  fill opacity=" << fillOpacity;
  }
  if (drawOpacity != 1.0) {
    stream << ",\n  draw opacity=" << drawOpacity;
  }
  return stream.str();
}

PgfPlot::PgfPlot(const std::shared_ptr<PlottableInterface>& plottable) : plottable_(plottable) {
}

void PgfPlot::setOptions(const PgfPlotOptions& options) {
  options_ = options;
}

void PgfPlot::setLegend(const std::string& legend) {
  legend_ = legend;
}

void PgfPlot::setPlottable(const std::shared_ptr<PlottableInterface>& plottable) {
  plottable_ = plottable;
}

std::string PgfPlot::string() const {
  if (!plottable_) {
    return {};
  }
  std::ostringstream stream{};
  stream << "\\addplot [" << options_.string() << "\n] ";
  stream << plottable_->string();
  if (legend_ != ""s) {
    stream << "\\addlegendentry{" << legend_ << "}\n";
  }
  return stream.str();
}

}  // namespace ompltools

}  // namespace esp
