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

#include "pdt/pgftikz/tikz_picture.h"

#include <sstream>

namespace pdt {

namespace pgftikz {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

std::string TikzPictureOptions::string() const {
  std::ostringstream stream{};
  stream << "\n  xscale=" << xscale << ",\n  yscale=" << yscale;
  return stream.str();
}

TikzPicture::TikzPicture(const std::shared_ptr<const config::Configuration>& config) : config_(config) {
}

void TikzPicture::clear() {
  axes_.clear();
  nodes_.clear();
  draws_.clear();
  texts_.clear();
}

void TikzPicture::setOptions(const TikzPictureOptions& options) {
  options_ = options;
}

void TikzPicture::addAxis(const std::shared_ptr<PgfAxis>& axis) {
  axes_.emplace_back(axis);
}

void TikzPicture::addNode(const std::shared_ptr<TikzNode>& node) {
  nodes_.emplace_back(node);
}

void TikzPicture::addDraw(const std::shared_ptr<TikzDraw>& draw) {
  draws_.emplace_back(draw);
}

void TikzPicture::addText(const std::string& line) {
  texts_.emplace_back(line);
}

void TikzPicture::setClipCommand(const std::string& clip) {
  clip_ = clip;
}

std::vector<std::shared_ptr<PgfAxis>> TikzPicture::getAxes() {
  return axes_;
}

std::shared_ptr<PgfAxis> TikzPicture::generateLegendAxis() const {
  auto legendAxis = std::make_shared<PgfAxis>();

  // Make sure the names are alphabetic.
  auto plannerNames = config_->get<std::vector<std::string>>("experiment/planners");
  std::sort(plannerNames.begin(), plannerNames.end());
  for (const auto& name : plannerNames) {
    std::string imageOptions{config_->get<std::string>("planner/"s + name + "/report/color"s) +
                             ", line width = 1.0pt, mark size=1.0pt, mark=square*"};
    legendAxis->addLegendEntry(config_->get<std::string>("planner/"s + name + "/report/name"s),
                               imageOptions);
  }
  return legendAxis;
}

std::string TikzPicture::string() const {
  if (axes_.empty() && nodes_.empty()) {
    return {};
  }
  std::ostringstream stream{};
  // Write the picture.
  stream << "\\begin{tikzpicture} [" << options_.string() << "\n]\n";

  if (clip_ != ""s) {
    stream << clip_ << '\n';
  }

  for (const auto& axis : axes_) {
    stream << axis->string() << '\n';
  }
  for (const auto& node : nodes_) {
    stream << node->string() << '\n';
  }
  for (const auto& draw : draws_) {
    stream << draw->string() << '\n';
  }
  for (const auto& text : texts_) {
    stream << text << '\n';
  }
  stream << "\\end{tikzpicture}%";
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

}  // namespace pgftikz

}  // namespace pdt
