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

// Authors: Jonathan Gammell

#include "pdt/pgftikz/define_latex_colors.h"

#include <map>
#include <sstream>

namespace pdt {

namespace pgftikz {

std::string defineLatexColors(const std::shared_ptr<const config::Configuration>& config)
{
  std::stringstream rval;
  std::map<std::string, std::array<int, 3>> colors;

  // Load the colors from the config.
  colors.emplace("pdtblack", config->get<std::array<int, 3>>("colors/pdtblack"));
  colors.emplace("pdtwhite", config->get<std::array<int, 3>>("colors/pdtwhite"));
  colors.emplace("pdtgray", config->get<std::array<int, 3>>("colors/pdtgray"));
  colors.emplace("pdtblue", config->get<std::array<int, 3>>("colors/pdtblue"));
  colors.emplace("pdtlightblue", config->get<std::array<int, 3>>("colors/pdtlightblue"));
  colors.emplace("pdtdarkblue", config->get<std::array<int, 3>>("colors/pdtdarkblue"));
  colors.emplace("pdtred", config->get<std::array<int, 3>>("colors/pdtred"));
  colors.emplace("pdtlightred", config->get<std::array<int, 3>>("colors/pdtlightred"));
  colors.emplace("pdtdarkred", config->get<std::array<int, 3>>("colors/pdtdarkred"));
  colors.emplace("pdtyellow", config->get<std::array<int, 3>>("colors/pdtyellow"));
  colors.emplace("pdtgreen", config->get<std::array<int, 3>>("colors/pdtgreen"));
  colors.emplace("pdtlightgreen", config->get<std::array<int, 3>>("colors/pdtlightgreen"));
  colors.emplace("pdtdarkgreen", config->get<std::array<int, 3>>("colors/pdtdarkgreen"));
  colors.emplace("pdtpurple", config->get<std::array<int, 3>>("colors/pdtpurple"));
  colors.emplace("pdtlightpurple", config->get<std::array<int, 3>>("colors/pdtlightpurple"));
  colors.emplace("pdtdarkpurple", config->get<std::array<int, 3>>("colors/pdtdarkpurple"));

  // Output the colors
  for (const auto& [name, values] : colors) {
    rval << "\\definecolor{" << name << "}{RGB}{" << values[0u] << ',' << values[1u] << ','
             << values[2u] << "}\n";
  }

  return rval.str();
}

}  // namespace pgftikz

}  // namespace pdt
