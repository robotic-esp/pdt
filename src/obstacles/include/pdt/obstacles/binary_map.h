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

#pragma once

#include <vector>

// GCC complains about eigen when compiling with these warnings.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Weffc++"
#pragma GCC diagnostic ignored "-Wuninitialized"
#include <Eigen/Core>
#pragma GCC diagnostic pop

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>

#include "pdt/obstacles/base_obstacle.h"

namespace pdt {

namespace obstacles {

class BinaryMap : public BaseObstacle {
 public:
  BinaryMap(ompl::base::SpaceInformation* si, const double obsThreshold,
            const std::string& fullCsvFileName, const bool plotCsv, const bool flipCsvRows = false,
            const std::string& fullPngFileName = "");
  BinaryMap(const ompl::base::SpaceInformationPtr& si, const double obsThreshold,
            const std::string& fullFileName, const bool plotCsv, const bool flipCsvRows = false,
            const std::string& fullPngFileName = "");
  ~BinaryMap() = default;

  /** \brief Check for state validity */
  virtual bool isValid(const ompl::base::State* state) const;

  /** \brief The obstacle map as a series of Matlab plot functions. (Discard arguments */
  virtual std::string mfile(const std::string& /*obsColour*/,
                            const std::string& /*spaceColour*/) const;

 protected:
 private:
  /// \brief A common constructor
  void construct(const bool flipRows);
  /// \brief Parse the csv
  void parseCsv(unsigned int* numRows, unsigned int* numCols,
                std::vector<std::vector<double> >* data);

  // Variables
  ompl::base::SpaceInformationPtr spaceInfo_;
  /// \brief The cost threshold
  double threshold_{0.0};
  /// \brief The csv file of costs
  std::string csvFile_{""};
  /// \brief The (optional) image file to overlay in the plot
  std::string pngFile_{""};
  /// \brief Whether to plot the csv
  bool plotCsv_{false};
  /// \brief The obs data
  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> obsData_{0, 0};
};

}  // namespace obstacles

}  // namespace pdt
