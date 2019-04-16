/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2017     University of Toronto
 *  Copyright (c) 2018-present  University of Oxford
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
 *   * Neither the names of the copyright holders nor the names of its
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

#include <iostream>
#include <string>
#include <vector>

#include "esp_tikz/pgf_axis.h"
#include "esp_tikz/pgf_plot.h"
#include "esp_tikz/pgf_table.h"
#include "esp_tikz/tikz_picture.h"

int main() {
  std::cout << "\\documentclass{standalone}\n\\usepackage{pgfplots}\n\\pgfplotsset{compat=1.15}\n\\usepackage{tikz}"
               "\n\n\\begin{document}";

  auto table = std::make_shared<esp::ompltools::PgfTable>();
  std::vector<double> times = {1.0, 2.0, 3.0, 4.0};
  std::vector<double> costs = {4.4123, 4.4123, 3.331, 3.111};

  table->addColumn(times);
  table->addColumn(costs);

  table->addRow({5.0, 3.001});

  auto plot = std::make_shared<esp::ompltools::PgfPlot>(table);
  auto axis = std::make_shared<esp::ompltools::PgfAxis>();
  axis->addPlot(plot);

  auto picture = std::make_shared<esp::ompltools::TikzPicture>();
  picture->addAxis(axis);

  std::cout << picture->string() << '\n';

  std::cout << "\\end{document}\n";

  return 0;
}
