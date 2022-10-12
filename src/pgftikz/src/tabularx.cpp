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

#include "pdt/pgftikz/tabularx.h"

#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <sstream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Weffc++"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "csv/parser.hpp"
#pragma GCC diagnostic pop

namespace pdt {

namespace pgftikz {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

TabularX::TabularX(const std::experimental::filesystem::path& path,
                   const std::vector<std::string>& rows) {
  loadFromPath(path, rows);
}

void TabularX::loadFromPath(const std::experimental::filesystem::path& path,
                            const std::vector<std::string>& rows) {
  // Open the file.
  std::ifstream filestream(path.string());
  if (filestream.fail()) {
    auto msg = "TabularX cannot open file at '"s + path.string() + "'."s;
    throw std::runtime_error(msg);
  }

  // Parse the file.
  aria::csv::CsvParser parser(filestream);
  for (const auto& row : parser) {
    if (row.size() == 0) {
      throw std::runtime_error("Empty row.");
    }
    if (row.at(0).find('#') != std::string::npos) {
      continue;  // Skipping comments.
    }
    std::string rowName;
    double unused = std::numeric_limits<double>::signaling_NaN();
    if (boost::conversion::try_lexical_convert<double>(row.at(0), unused)) {
      auto msg = "Pgf Table encountered unnamed row in '"s + path.string() +
                 "'. The first entry in every row is expected to be a string."s;
      throw std::invalid_argument(msg);
    } else {
      rowName = row.at(0);
    }

    // Check if we supposed to load this row.
    bool loadThisRow = false;
    for (const auto& request : rows) {
      if (request == rowName) {
        loadThisRow = true;
        break;
      }
    }

    if (!loadThisRow) {
      continue;  // Skipping rows that aren't of interest.
    }

    std::vector<double> rowAsDoubles{};
    rowAsDoubles.reserve(row.size());
    data_.push_back(std::deque<double>());
    for (std::size_t i = 1u; i < row.size(); ++i) {
      try {
        data_.back().push_back(std::stod(row.at(i)));
      } catch (const std::invalid_argument& e) {
        auto msg = "TabularX cannot convert entry '"s + row.at(i) + "' from file '"s +
                   path.string() + "' to double:\n    "s + e.what();
        throw std::invalid_argument(msg);
      }
    }
  }

  // Check that the requested data was present.
  if (data_.size() != rows.size()) {
    auto msg = "File at '"s + path.string() + "' does not contain the requested rows."s;
    throw std::invalid_argument(msg);
  }
}

void TabularX::appendCol(const std::vector<double>& col) {
  data_.push_back(std::deque<double>());
  data_.back().insert(data_.back().end(), col.begin(), col.end());
}

void TabularX::prependCol(const std::vector<double>& col) {
  data_.push_back(std::deque<double>());
  data_.back().insert(data_.back().begin(), col.begin(), col.end());
}

void TabularX::appendRow(const std::vector<double>& row) {
  if (!data_.empty() && row.size() != data_.size()) {
    auto msg = "In Tabluarx, all rows must have the same number of columns."s;
    throw std::invalid_argument(msg);
  }
  if (data_.empty()) {
    data_.resize(row.size(), {});
  }
  for (std::size_t i = 0u; i < row.size(); ++i) {
    data_.at(i).push_back(row.at(i));
  }
}

void TabularX::prependRow(const std::vector<double>& row) {
  if (row.size() != data_.size()) {
    auto msg = "In Tabularx, all rows must have the same number of columns."s;
    throw std::invalid_argument(msg);
  }
  if (data_.empty()) {
    data_.resize(row.size(), {});
  }
  for (std::size_t i = 0u; i < row.size(); ++i) {
    data_.at(i).emplace_front(row.at(i));
  }
}

void TabularX::replaceInCol(std::size_t col, double number, double replacement) {
  std::replace(data_.at(col).begin(), data_.at(col).end(), number, replacement);
}

void TabularX::replaceInCol(std::size_t col, const std::function<double(double)>& replacement) {
  for (auto& number : data_.at(col)) {
    number = replacement(number);
  }
}

std::size_t TabularX::getNumRows() const {
  return data_.at(0u).size();
}

std::vector<double> TabularX::getRow(std::size_t index) const {
  std::vector<double> row;
  row.reserve(data_.size());
  for (const auto& col : data_) {
    row.push_back(col.at(index));
  }
  return row;
}

std::size_t TabularX::getNumCols() const {
  return data_.size();
}

std::vector<double> TabularX::getCol(std::size_t index) const {
  std::vector<double> row;
  row.insert(row.end(), data_.at(index).begin(), data_.at(index).end());
  return row;
}

std::string TabularX::string() const {
  // Perform some sanity checks.
  if (data_.size() < 2) {
    throw std::runtime_error("Pgf Table holds less than 2 columns.");
  }
  // Throw if the cols don't have the same number of entries.
  if (data_.at(0u).size() != data_.at(1u).size()) {
    auto msg = "Pgf Table columns have different sizes."s;
    throw std::invalid_argument(msg);
  }
  // No output if the table is empty.
  if (data_.at(0u).empty() || data_.at(1u).empty()) {
    return {};
  }

  // We clean the table here. Values that are sandwiched are omitted.
  std::ostringstream stream{};
  stream << "table [\n"
         << "  row sep=" << options.rowSep << ",\n  col sep=" << options.colSep << "\n]{\n";
  for (std::size_t row = 0u; row < data_.at(0u).size(); ++row) {
    auto x = data_.at(0u).at(row);
    auto y = data_.at(1u).at(row);
    stream << x << ' ' << options.colSep << ' ' << y << options.rowSep << '\n';
  }
  stream << "};\n";

  return stream.str();
}

}  // namespace pgftikz

}  // namespace pdt
