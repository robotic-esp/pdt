/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Toronto
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
 *   * Neither the name of the University of Toronto nor the names of its
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

#include <experimental/filesystem>

#include "esp_configuration/configuration.h"
#include "esp_tikz/tikz_picture.h"

namespace esp {

namespace ompltools {

class LatexPlotter {
 public:
  LatexPlotter(const std::shared_ptr<const Configuration>& config);
  virtual ~LatexPlotter() = default;

  // Create the legend axis.
  std::shared_ptr<PgfAxis> createLegendAxis(const std::vector<std::string>& plannerNames) const;

  // Align multiple axes' abszissen.
  template <typename... Axes>
  void alignAbszissen(Axes... args) const;
  void alignAbszissen(const std::vector<std::shared_ptr<PgfAxis>>& axes) const;

  // Align multiple axes' ordinates.
  template <typename... Axes>
  void alignOrdinates(Axes... args) const;
  void alignOrdinates(const std::vector<std::shared_ptr<PgfAxis>>& axes) const;

  // Stack multiple axes.
  template <typename... Axes>
  void stack(Axes... args) const;
  void stack(const std::vector<std::shared_ptr<PgfAxis>>& axes) const;

  // Collect multiple axes in a tikzpicture.
  template <typename... Axes>
  std::shared_ptr<TikzPicture> collect(Axes... args) const;
  std::shared_ptr<TikzPicture> collect(const std::vector<std::shared_ptr<PgfAxis>>& axes) const;

  // Create a tikzpicture from multiple axes.
  template <typename... Axes>
  std::experimental::filesystem::path createPicture(Axes... args) const;
  std::experimental::filesystem::path createPicture(
      const std::vector<std::shared_ptr<PgfAxis>>& axes) const;

  // Compiles the given tikzpicture to a pdf document.
  std::experimental::filesystem::path compileStandalonePdf(
      const std::experimental::filesystem::path& tikzPicture) const;

 protected:
  const std::shared_ptr<const Configuration> config_;
};

template <typename... Axes>
void LatexPlotter::alignAbszissen(Axes... args) const {
  // Collect the axes in a vector
  std::vector<std::shared_ptr<PgfAxis>> axes{};
  (axes.push_back(args), ...);

  // Align all axes.
  alignAbszissen(axes);
}

template <typename... Axes>
void LatexPlotter::alignOrdinates(Axes... args) const {
  // Collect the axes in a vector
  std::vector<std::shared_ptr<PgfAxis>> axes{};
  (axes.push_back(args), ...);

  // Align all axes.
  alignOrdinates(axes);
}

template <typename... Axes>
void LatexPlotter::stack(Axes... args) const {
  using namespace std::string_literals;
  // Collect the axes in a vector
  std::vector<std::shared_ptr<PgfAxis>> axes{};
  (axes.push_back(args), ...);

  // Stack them.
  stack(axes);
}

template <typename... Axes>
std::shared_ptr<TikzPicture> LatexPlotter::collect(Axes... args) const {
  auto picture = std::make_shared<TikzPicture>(config_);

  // Collect the axes in a picture.
  (picture->addAxis(args), ...);

  return picture;
}

template <typename... Axes>
std::experimental::filesystem::path LatexPlotter::createPicture(Axes... args) const {
  // Collect all axes in a tikz picture.
  auto picture = collect(args...);

  // Create the name of this picture.
  auto path = std::experimental::filesystem::path(config_->get<std::string>("experiment/results"))
                  .parent_path() /
              std::experimental::filesystem::path("tikz/");
  for (const auto& axis : picture->getAxes()) {
    path += std::experimental::filesystem::path(axis->options.name + '_');
  }
  path += ".tikz";
  picture->write(path);
  return path;
}

}  // namespace ompltools

}  // namespace esp
