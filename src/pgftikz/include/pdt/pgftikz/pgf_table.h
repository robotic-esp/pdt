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

#include <deque>
#include <experimental/filesystem>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "pdt/pgftikz/pgf_plottable.h"

namespace pdt {

namespace pgftikz {

class PgfTable : public PlottableInterface {
 public:
  PgfTable() = default;
  PgfTable(const std::experimental::filesystem::path& path, const std::string& domain,
           const std::string& codomain);
  ~PgfTable() = default;

  void loadFromPath(const std::experimental::filesystem::path& path, const std::string& domain,
                    const std::string& codomain);

  void setCleanData(bool cleanData);

  // Check if the table empty.
  bool empty() const;

  // Add numbers.
  void prependRow(const std::vector<double>& row);
  void appendRow(const std::vector<double>& row);

  // Replace numbers.
  void replaceInDomain(double number, double replacement);
  void replaceInDomain(const std::function<double(double)>& replacement);
  void replaceInCodomain(double number, double replacement);
  void replaceInCodomain(const std::function<double(double)>& replacement);

  // Remove numbers.
  void removeRowIfDomainEquals(double number);
  void removeRowIfCodomainEquals(double number);
  void removeRowIfDomainIsNan();
  void removeRowIfCodomainIsNan();

  // Get rows.
  std::size_t getNumRows() const;
  std::vector<double> getRow(std::size_t index) const;

  // Get min/max values.
  double getMaxValue() const;
  double getMinValue() const;
  double getMaxValueInRow(std::size_t index) const;
  double getMinValueInRow(std::size_t index) const;
  double getMaxValueInCol(std::size_t index) const;
  double getMinValueInCol(std::size_t index) const;

  // Add a col. (Deprecated?)
  void addColumn(const std::deque<double>& column);

  // Convert this table to a string.
  std::string string() const override;

 private:
  std::vector<std::deque<double>> data_{};
  bool cleanData_{true};

 public:
  struct {
    std::string colSep{"&"};
    std::string rowSep{"\\\\"};
  } options{};
};

}  // namespace pgftikz

}  // namespace pdt
