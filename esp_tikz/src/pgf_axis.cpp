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

std::string PgfAxisOptions::string() const {
  std::ostringstream stream{};
  stream << "\n  width=" << width
         << ",\n  height=" << height
         << ",\n  at={" << at << '}';
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
    stream << ",\n  ymay=" << ymax;
  }
  if (xlog) {
    stream << ",\n  xmode=log";
  }
  if (ylog) {
    stream << ",\n  ymode=log";
  }
  if (xlabel != std::string("")) {
    stream << ",\n  xlabel={" << xlabel << '}';
  }
  if (ylabel != std::string("")) {
    stream << ",\n  ylabel={" << ylabel << '}';
  }
  return stream.str();
}

void PgfAxis::setOptions(const PgfAxisOptions& options) {
  options_ = options;
}

void PgfAxis::addPlot(const std::shared_ptr<PgfPlot>& plot) {
  plots_.emplace_back(plot);
}

std::string PgfAxis::string() const {
  if (plots_.empty()) {
    return {};
  }
  std::ostringstream stream{};
  stream << "\\begin{axis} [" << options_.string() << "\n]\n\n";
  for (const auto& plot : plots_) {
    stream << plot->string() << '\n';
  }
  stream << "\\end{axis}\n";
  return stream.str();
}

}  // namespace ompltools

}  // namespace esp
