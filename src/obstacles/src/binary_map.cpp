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

/* Authors: Jonathan Gammell */

#include "pdt/obstacles/binary_map.h"

#include <fstream>

#include <boost/lexical_cast.hpp>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/Exception.h>

namespace pdt {

namespace obstacles {

BinaryMap::BinaryMap(const ompl::base::SpaceInformationPtr& si, const double obsThreshold,
                     const std::string& fullCsvFileName, const bool plotCsv,
                     const bool flipCsvRows /*= false*/,
                     const std::string& fullPngFileName /*= ""*/) :
    BaseObstacle(),
    spaceInfo_(si),
    threshold_(obsThreshold),
    csvFile_(fullCsvFileName),
    pngFile_(fullPngFileName),
    plotCsv_(plotCsv) {
  this->construct(flipCsvRows);
}

void BinaryMap::construct(const bool flipRows) {
  // Variable
  // The number of rows
  unsigned int numRows;
  // The number of columns
  unsigned numCols;
  // The parsed CSV file:
  std::vector<std::vector<double> > csvParsed;

  // Parse the csv file
  parseCsv(&numRows, &numCols, &csvParsed);

  // Resize the eigen matrix
  obsData_.resize(numRows, numCols);

  // Iterate over rows
  for (int r = 0u; r < obsData_.rows(); ++r) {
    // and columns
    for (unsigned int c = 0u; c < obsData_.cols(); ++c) {
      // Storing data
      obsData_(r, c) = (csvParsed.at(r).at(c) >= threshold_);
    }
  }

  // Are we flipping the rows?
  if (flipRows == true) {
    obsData_ = obsData_.colwise().reverse().eval();
  }
}

// Adapted from http://stackoverflow.com/a/1120224
void BinaryMap::parseCsv(unsigned int* numRows, unsigned int* numCols,
                         std::vector<std::vector<double> >* data) {
  // Variables
  // The input csv file
  std::ifstream csvStream;
  // Each line of the csv as a string
  std::string csvLine;

  // Open the file
  csvStream.open(csvFile_.c_str());

  // Check
  if (csvStream.is_open() == false) {
    throw ompl::Exception("Could not open csv file");
  }

  // Initialize the number of rows and columns
  *numRows = 0u;
  *numCols = 0u;

  // Load the csv file into the string by getting it one line at a time:
  while (std::getline(csvStream, csvLine)) {
    data->push_back(std::vector<double>());
    // Variables
    // A string stream of the line
    std::stringstream lineStream(csvLine);
    // The actual value
    std::string cellValue;

    // Iterate along the tokens
    while (std::getline(lineStream, cellValue, ',')) {
      data->back().push_back(boost::lexical_cast<double>(cellValue));
    }

    // If this is the first row, store the number of columns, otherwise check
    if (data->size() == 1u) {
      *numCols = data->back().size();
    } else if (data->back().size() != *numCols) {
      throw ompl::Exception("Found an inconsistent number of columns in the csv file.");
    }
  }

  // Store the number of rows:
  *numRows = data->size();

  // Close the file
  csvStream.close();
}

bool BinaryMap::isValid(const ompl::base::State* state) const {
  // Variable
  // The return value
  bool validState;

  // Check if the state satisfies the bounds
  validState = StateValidityChecker::si_->satisfiesBounds(state);

  // If it does, compare it to the CSV data
  if (validState == true) {
    // Variables
    // The indices of the state:
    unsigned int rowIdx;
    unsigned int colIdx;

    // Get the indices, this will truncate the double
    colIdx = static_cast<unsigned int>(
        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0u]);
    rowIdx = static_cast<unsigned int>(
        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1u]);

    // If both the column and row index are in range, use the data
    if (rowIdx < obsData_.rows() && colIdx < obsData_.cols()) {
      validState = !obsData_(rowIdx, colIdx);
    } else {
      // Return true
      validState = true;
    }
  }
  // No else, we're done

  return validState;
}

// Discard arguments
std::string BinaryMap::mfile(const std::string& /*obsColour*/,
                             const std::string& /*spaceColour*/) const {
  // Variables
  // The string stream:
  std::stringstream rval;

  // Output the csv if desired
  if (plotCsv_ == true) {
    rval << "obstacle_data = [" << obsData_ << "];" << std::endl;
    rval << "surf(obstacle_data,'EdgeColor','None');" << std::endl;
    rval << "view(2);" << std::endl;
    rval << "colormap([1 1 1; 0 0 0]);" << std::endl;
  }

  // and the overlay if present
  if (pngFile_.empty() == false) {
    rval << "isHeld = ishold();" << std::endl;
    rval << "[img, map, alpha] = imread('" << pngFile_ << "');" << std::endl;
    rval << "imgHndl = imshow(img, map);" << std::endl;
    rval << "set(imgHndl, 'AlphaData', alpha);" << std::endl;
    rval << "if isHeld" << std::endl;
    rval << "    hold on;" << std::endl;
    rval << "end" << std::endl;
  }

  return rval.str();
}

}  // namespace obstacles

}  // namespace pdt
