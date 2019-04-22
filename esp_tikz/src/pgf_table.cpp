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

#include "esp_tikz/pgf_table.h"

#include <algorithm>
#include <fstream>
#include <sstream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Weffc++"
#include "csv/parser.hpp"
#pragma GCC diagnostic pop

namespace esp {

namespace ompltools {

using namespace std::string_literals;
namespace fs = std::experimental::filesystem;

std::string PgfTableOptions::string() const {
  std::ostringstream stream{};
  stream << "\n  row sep=" << rowSep << ",\n  col sep=" << colSep;
  return stream.str();
}

PgfTable::PgfTable(const std::experimental::filesystem::path& path, const std::string& domain,
                   const std::string& codomain) {
  loadFromPath(path, domain, codomain);
}

void PgfTable::loadFromPath(const std::experimental::filesystem::path& path,
                            const std::string& domain, const std::string& codomain) {
  // Open the file.
  std::ifstream filestream(path.string());
  if (filestream.fail()) {
    auto msg = "PgfTable cannot open file at '"s + path.string() + "'."s;
    throw std::runtime_error(msg);
  }

  // Prepare data.
  if (data_.size() == 0u) {
    data_.resize(2u, {});
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
    std::string rowName{"invalid row"};
    try {  // This *should* throw, we expect a row name at first position.
      std::stod(row.at(0));
    } catch (const std::invalid_argument& e) {
      rowName = row.at(0);
    }
    if (rowName == "invalid row"s) {
      auto msg = "Pgf Table encountered unnamed row in '"s + path.string() +
                 "'. The first entry in every row is expected to be a string."s;
      throw std::invalid_argument(msg);
    }
    if (rowName != domain && rowName != codomain) {
      continue;  // Skipping rows that aren't of interest.
    } else if (rowName == domain && !data_.at(0u).empty()) {
      auto msg = "Pgf Table encountered multiple rows with the name '"s + domain +
                 "' in the file '"s + path.string() + "'."s;
      throw std::invalid_argument(msg);
    } else if (rowName == codomain && !data_.at(1u).empty()) {
      auto msg = "Pgf Table encountered multiple rows with the name '"s + codomain +
                 "' in the file '"s + path.string() + "'."s;
      throw std::invalid_argument(msg);
    }

    std::vector<double> rowAsDoubles{};
    rowAsDoubles.reserve(row.size());
    for (std::size_t i = 1u; i < row.size(); ++i) {
      try {
        if (rowName == domain) {
          data_.at(0u).emplace_back(std::stod(row.at(i)));
        } else if (rowName == codomain) {
          data_.at(1u).emplace_back(std::stod(row.at(i)));
        }
      } catch (const std::invalid_argument& e) {
        auto msg = "Pgf Table cannot convert entry '"s + row.at(i) + "' from file '"s +
                   path.string() + "' to double."s;
        throw std::invalid_argument(msg);
      }
    }
  }

  // Check that the requested data was present.
  if (data_.at(0u).size() + data_.at(1u).size() == 0u) {
    auto msg = "File at '"s + path.string() + "' does not contain any data for '"s + domain +
               "' and '" + codomain + "'."s;
    throw std::invalid_argument(msg);
  }
  if (data_.at(0u).size() == 0u) {
    auto msg = "File at '"s + path.string() + "' does not contain any data for '"s + domain + "'."s;
    throw std::invalid_argument(msg);
  }
  if (data_.at(1u).size() == 0u) {
    auto msg =
        "File at '"s + path.string() + "' does not contain any data for '"s + codomain + "'."s;
    throw std::invalid_argument(msg);
  }
}

void PgfTable::addColumn(const std::deque<double>& column) {
  if (!data_.empty() && column.size() != data_.at(0u).size()) {
    throw std::runtime_error("Number of elements in column does not match table.");
  }
  if (data_.size() > 1) {
    throw std::runtime_error("Table currently only implemented for 2 dimensional data.");
  }
  data_.push_back(column);
}

void PgfTable::setOptions(const PgfTableOptions& options) {
  options_ = options;
}

void PgfTable::replaceInDomain(double number, double replacement) {
  std::replace(data_.at(0u).begin(), data_.at(0u).end(), number, replacement);
}

void PgfTable::replaceInDomain(const std::function<double(double)>& replacement) {
  for (auto& number : data_.at(0u)) {
    number = replacement(number);
  }
}

void PgfTable::replaceInCodomain(double number, double replacement) {
  std::replace(data_.at(1u).begin(), data_.at(1u).end(), number, replacement);
}

void PgfTable::replaceInCodomain(const std::function<double(double)>& replacement) {
  for (auto& number : data_.at(1u)) {
    number = replacement(number);
  }
}

void PgfTable::appendRow(const std::vector<double>& row) {
  if (!data_.empty() && row.size() != data_.size()) {
    auto msg = "In Pgf Tables, all rows must have the same number of columns."s;
    throw std::invalid_argument(msg);
  }
  if (data_.empty()) {
    data_.resize(row.size(), {});
  }
  for (std::size_t i = 0u; i < row.size(); ++i) {
    data_.at(i).emplace_back(row.at(i));
  }
}

void PgfTable::prependRow(const std::vector<double>& row) {
  if (row.size() != data_.size()) {
    auto msg = "In Pgf Tables, all rows must have the same number of columns."s;
    throw std::invalid_argument(msg);
  }
  for (std::size_t i = 0u; i < row.size(); ++i) {
    data_.at(i).emplace_front(row.at(i));
  }
}

void PgfTable::removeRowIfDomainEquals(double number) {
  while (std::find(data_.at(0u).begin(), data_.at(0u).end(), number) != data_.at(0u).end()) {
    auto itDomain = std::find(data_.at(0u).begin(), data_.at(0u).end(), number);
    auto dist = std::distance(data_.at(0u).begin(), itDomain);
    auto itCodomain = data_.at(1u).begin() + dist;
    data_.at(0u).erase(itDomain);
    data_.at(1u).erase(itCodomain);
  }
}

void PgfTable::removeRowIfCodomainEquals(double number) {
  while (std::find(data_.at(1u).begin(), data_.at(1u).end(), number) != data_.at(1u).end()) {
    auto itCodomain = std::find(data_.at(1u).begin(), data_.at(1u).end(), number);
    auto dist = std::distance(data_.at(1u).begin(), itCodomain);
    auto itDomain = data_.at(0u).begin() + dist;
    data_.at(0u).erase(itDomain);
    data_.at(1u).erase(itCodomain);
  }
}

std::size_t PgfTable::getNumRows() const {
  return data_.at(0u).size();
}

std::vector<double> PgfTable::getRow(std::size_t index) const {
  std::vector<double> row;
  row.reserve(data_.size());
  for (const auto col : data_) {
    row.emplace_back(col.at(index));
  }
  return row;
}

std::string PgfTable::string() const {
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
  double lowX = data_.at(0u).at(0);
  double lowY = data_.at(1u).at(0);
  double lastX = data_.at(0u).at(0);
  stream << "table [" << options_.string() << "\n]{\n";
  stream << lowX << ' ' << options_.colSep << ' ' << lowY << options_.rowSep << '\n';
  for (std::size_t row = 1u; row < data_.at(0u).size(); ++row) {
    auto x = data_.at(0u).at(row);
    auto y = data_.at(1u).at(row);
    if (y == lowY && row != data_.at(0u).size() - 1u) {  // We need the last result.
      lowX = x;
      continue;
    } else {
      if (lastX != lowX) {
        stream << lowX << ' ' << options_.colSep << ' ' << lowY << options_.rowSep << '\n';
      }
      stream << x << ' ' << options_.colSep << ' ' << y << options_.rowSep << '\n';
      lastX = x;
      lowX = x;
      lowY = y;
    }
  }
  stream << "};\n";

  return stream.str();
}

}  // namespace ompltools

}  // namespace esp
