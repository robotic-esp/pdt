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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "pdt/config/configuration.h"
#include "pdt/pgftikz/pgf_axis.h"
#include "pdt/pgftikz/tikz_draw.h"
#include "pdt/pgftikz/tikz_node.h"

namespace pdt {

namespace pgftikz {

struct TikzPictureOptions {
  std::string string() const;
  double xscale{1.0};
  double yscale{1.0};
};

namespace zlevels {
constexpr std::size_t UNSET = 0;
constexpr std::size_t OBSTACLE = 1;
constexpr std::size_t ANTIOBSTACLE = 2;
constexpr std::size_t BOUNDARY = 3;
constexpr std::size_t EDGE_LOWLIGHT = 10;
constexpr std::size_t EDGE = 11;
constexpr std::size_t EDGE_HIGHLIGHT = 12;
constexpr std::size_t SOLUTION = 20;
constexpr std::size_t VERTEX = 30;
constexpr std::size_t START = 31;
constexpr std::size_t GOAL = 32;
}  // namespace zlevels

class TikzPicture {
 public:
  TikzPicture(const std::shared_ptr<const config::Configuration>& config);
  ~TikzPicture() = default;

  // Clears the contents of this picture.
  void clear();

  // Sets the options for this tikz picture
  void setOptions(const TikzPictureOptions& options);

  // Adds an axis to this picture.
  void addAxis(const std::shared_ptr<PgfAxis>& axis);

  // Adds a node to this picture.
  void addNode(const std::shared_ptr<TikzNode>& node);

  // Adds a draw command to this picture.
  void addDraw(const std::shared_ptr<TikzDraw>& draw);

  // Adds a text line to this picture.
  void addText(const std::string& line);

  // Sets the clip command.
  void setClipCommand(const std::string& clip);

  // Get all axes of this picture.
  std::multimap<std::size_t, std::shared_ptr<PgfAxis>> getAxes();

  // Returns this tikz picture as a string.
  std::string string() const;

  // Writes this tikz picture to a file.
  void write(const std::experimental::filesystem::path& path) const;

 protected:
  std::shared_ptr<PgfAxis> generateLegendAxis() const;
  std::multimap<std::size_t, std::shared_ptr<PgfAxis>> axes_{};
  std::multimap<std::size_t, std::shared_ptr<TikzNode>> nodes_{};
  std::multimap<std::size_t, std::shared_ptr<TikzDraw>> draws_{};
  std::vector<std::string> texts_{};
  std::string clip_{""};
  TikzPictureOptions options_{};
  const std::shared_ptr<const config::Configuration> config_;
};

}  // namespace pgftikz

}  // namespace pdt
