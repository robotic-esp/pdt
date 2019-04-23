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

#include "esp_tikz/tikz_picture.h"

#include <sstream>

namespace esp {

namespace ompltools {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

std::string TikzPictureOptions::string() const {
  std::ostringstream stream{};
  stream << "\n  xscale=" << xscale << ",\n  yscale=" << yscale;
  return stream.str();
}

TikzPicture::TikzPicture(const std::shared_ptr<Configuration>& config) : config_(config) {
  // Load colors from config.
  espColors_.emplace("espblack", config_->get<std::array<int, 3>>("Colors/espblack"));
  espColors_.emplace("espwhite", config_->get<std::array<int, 3>>("Colors/espwhite"));
  espColors_.emplace("espgray", config_->get<std::array<int, 3>>("Colors/espgray"));
  espColors_.emplace("espblue", config_->get<std::array<int, 3>>("Colors/espblue"));
  espColors_.emplace("espred", config_->get<std::array<int, 3>>("Colors/espred"));
  espColors_.emplace("espyellow", config_->get<std::array<int, 3>>("Colors/espyellow"));
  espColors_.emplace("espgreen", config_->get<std::array<int, 3>>("Colors/espgreen"));
  espColors_.emplace("esppurple", config_->get<std::array<int, 3>>("Colors/esppurple"));
  espColors_.emplace("esplightblue", config_->get<std::array<int, 3>>("Colors/esplightblue"));
  espColors_.emplace("espdarkred", config_->get<std::array<int, 3>>("Colors/espdarkred"));
}

void TikzPicture::setOptions(const TikzPictureOptions& options) {
  options_ = options;
}

void TikzPicture::addAxis(const std::shared_ptr<PgfAxis>& axis) {
  axes_.emplace_back(axis);
}

std::shared_ptr<PgfAxis> TikzPicture::generateLegendAxis() const {
  auto legendAxis = std::make_shared<PgfAxis>();

  // Make sure the names are alphabetic.
  auto plannerNames = config_->get<std::vector<std::string>>("Experiment/planners");
  std::sort(plannerNames.begin(), plannerNames.end());
  for (const auto& name : plannerNames) {
    std::string imageOptions{config_->get<std::string>("PlannerPlotColors/" + name) +
                             ", line width = 1.0pt, mark size=1.0pt, mark=square*"};
    legendAxis->addLegendEntry(config_->get<std::string>("PlannerPlotNames/" + name), imageOptions);
  }
  return legendAxis;
}

std::string TikzPicture::string() const {
  if (axes_.empty()) {
    return {};
  }
  std::ostringstream stream{};
  // Make sure the colors are defined.
  for (const auto& [name, values] : espColors_) {
    stream << "\\definecolor{" << name << "}{RGB}{" << values[0u] << ',' << values[1u] << ','
           << values[2u] << "}\n";
  }

  // Write the picture.
  stream << "\\begin{tikzpicture} [" << options_.string() << "\n]\n\n";
  for (const auto& axis : axes_) {
    stream << axis->string() << '\n';
  }
  stream << "\\end{tikzpicture}\n";
  return stream.str();
}

void TikzPicture::write(const std::experimental::filesystem::path& path) const {
  // Make sure the directories to the path exist.
  fs::create_directories(path.parent_path());

  // Open a file.
  std::ofstream texFile;
  texFile.open(path.string());

  // Check on the failbit.
  if (texFile.fail() == true) {
    auto msg = "TikzPicture could not write to file '"s + path.string() + "'."s;
    throw std::ios_base::failure(msg);
  }

  texFile << string();
}

}  // namespace ompltools

}  // namespace esp
