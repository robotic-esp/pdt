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

#include "pdt/pgftikz/tikz_draw.h"

#include <sstream>

namespace pdt {

namespace pgftikz {

using namespace std::string_literals;

void TikzDraw::setFromPosition(double x, double y) {
  from_ = std::to_string(x) + "cm, " + std::to_string(y) + "cm";
}

void TikzDraw::setFromPosition(const std::string& position) {
  from_ = position;
}

void TikzDraw::setToPosition(double x, double y) {
  to_ = std::to_string(x) + "cm, " + std::to_string(y) + "cm";
}

void TikzDraw::setToPosition(const std::string& position) {
  to_ = position;
}

void TikzDraw::setConnection(const std::string& connection) {
  connection_ = connection;
}

void TikzDraw::setOptions(const std::string& options) {
  options_ = options;
}

void TikzDraw::setZLevel(std::size_t z) {
  zLevel_ = z;
}

std::size_t TikzDraw::getZLevel() {
  return zLevel_;
}

std::string TikzDraw::string() const {
  std::ostringstream stream{};
  stream << "\\draw ";
  if (options_ != ""s) {
    stream << "[" << options_ << "] ";
  }
  stream << "(" << from_ << ") " << connection_ << " (" << to_ << ");\n";
  return stream.str();
}

}  // namespace pgftikz

}  // namespace pdt
