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

#include <memory>
#include <string>
#include <vector>

#include "pdt/pgftikz/pgf_plottable.h"

namespace pdt {

namespace pgftikz {

class PgfPlot {
 public:
  PgfPlot() = default;
  PgfPlot(const std::shared_ptr<PlottableInterface>& plottable);
  ~PgfPlot() = default;

  void setLegend(const std::string& legend);
  void setPlottable(const std::shared_ptr<PlottableInterface>& plottable);

  std::string string() const;
  bool empty() const;

 private:
  std::shared_ptr<PlottableInterface> plottable_{};
  std::string legend_{""};

 public:
  // The supported plot options.
  struct {
    // Line options.
    double lineWidth{1.0};
    std::string color{"black"};
    std::string fill{""};
    bool dashed{false};
    bool dotted{false};
    bool denselyDashed{false};
    bool denselyDotted{false};
    bool looselyDashed{false};
    bool looselyDotted{false};

    // Mark options.
    double markSize{2.0};
    std::string mark{"square*"};
    bool onlyMarks{false};

    // Plot options.
    bool forgetPlot{false};
    bool constPlot{true};
    std::string namePath{""};
    float fillOpacity{1.0};
    float drawOpacity{1.0};
  } options{};
};

}  // namespace pgftikz

}  // namespace pdt
