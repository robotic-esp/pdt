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

#include "pdt/pgftikz/tikz_node.h"

#include <sstream>

namespace pdt {

namespace pgftikz {

using namespace std::string_literals;

void TikzNode::setPosition(double x, double y) {
  at_ = std::to_string(x) + ", " + std::to_string(y);
}

void TikzNode::setPosition(const std::string& position) {
  at_ = position;
}

void TikzNode::setName(const std::string& name) {
  name_ = name;
}

void TikzNode::setOptions(const std::string& options) {
  options_ = options;
}

void TikzNode::setLabel(const std::string& label) {
  label_ = label;
}

void TikzNode::setZLevel(std::size_t z) {
  zLevel_ = z;
}

std::size_t TikzNode::getZLevel() {
  return zLevel_;
}

std::string TikzNode::string() const {
  std::ostringstream stream{};
  stream << "\\node ";
  if (name_ != ""s) {
    stream << "(" << name_ << ") ";
  }
  if (options_ != ""s) {
    stream << "[" << options_ << "] ";
  }
  if (at_ != ""s) {
    stream << "at (" << at_ << ") ";
  }
  stream << "{" << label_ << "};\n";
  return stream.str();
}

}  // namespace pgftikz

}  // namespace pdt
