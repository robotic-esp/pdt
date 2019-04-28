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
#include <memory>
#include <string>

#include "esp_configuration/configuration.h"
#include "esp_statistics/statistics.h"
#include "esp_tikz/latex_plotter.h"

namespace esp {

namespace ompltools {

class InitialSolutionDurationPdfPlotter : public LatexPlotter {
 public:
  InitialSolutionDurationPdfPlotter(const std::shared_ptr<const Configuration>& config,
                                    const Statistics& stats);
  ~InitialSolutionDurationPdfPlotter() = default;

  // Creates a pgf axis that hold the initial solution duration pdf of all planners.
  std::shared_ptr<PgfAxis> createInitialSolutionDurationPdfAxis() const;

  // Creates a pgf axis that hold the initial solution duration pdf of the specified planner.
  std::shared_ptr<PgfAxis> createInitialSolutionDurationPdfAxis(
      const std::string& plannerName) const;

  // Creates a tikz picture that contains the initial solution duration pdf axis of all planners.
  std::experimental::filesystem::path createInitialSolutionDurationPdfPicture() const;

  // Creates a tikz picture that contains the initial solution duration pdf axis of all planners.
  std::experimental::filesystem::path createInitialSolutionDurationPdfPicture(
      const std::string& plannerName) const;

 private:
  std::shared_ptr<PgfPlot> createInitialSolutionDurationPdfPlot(
      const std::string& plannerName) const;

  void setInitialSolutionDurationPdfAxisOptions(std::shared_ptr<PgfAxis> axis) const;

  const Statistics& stats_;
};

}  // namespace ompltools

}  // namespace esp