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

namespace esp {

namespace ompltools {

class LatexPlotter {
 public:
  LatexPlotter() = default;
  virtual ~LatexPlotter() = default;

  // Creates a picture from axes and a file name.
  template <typename... Axes>
  std::shared_ptr<TikzPicture> createPicture(std::shared_ptr<PgfAxis> axis, Axes... axes) const;

  // Compiles the given tikzpicture to a pdf document.
  std::experimental::filesystem::path compileStandalonePdf(
      const std::experimental::filesystem::path& tikzPicture) const;

 private:
  template <typename... Axes>
  std::shared_ptr<TikzPicture> createPicture(std::shared_ptr<TikzPicture> picture,
                                             std::shared_ptr<PgfAxis> axis, Axes... axis) const;
};

template <typename... Axes>
std::shared_ptr<TikzPicture> LatexPlotter::createPicture(std::shared_ptr<PgfAxis> axis,
                                                         Axes..axes) const {
  auto picture = std::make_shared<TikzPicture>();
  picture.addAxis(axis);
  if constexpr (sizeof...(axes) > 0) {
      return createPicture()
  }
}

}  // namespace ompltools

}  // namespace esp
