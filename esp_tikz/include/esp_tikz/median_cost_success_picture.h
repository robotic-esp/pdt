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
#include "esp_tikz/tikz_picture.h"

namespace esp {

namespace ompltools {

class MedianCostSuccessPicture : public TikzPicture {
 public:
  MedianCostSuccessPicture(const std::shared_ptr<const Configuration>& config,
                           const Statistics& stats);
  ~MedianCostSuccessPicture() = default;

  // Returns a pgf axis that holds the median cost at binned durations for all planners.
  std::shared_ptr<PgfAxis> createMedianCostAxis() const;

  // Returns a pgf axis that holds the median cost at binned durations for the specified planner.
  std::shared_ptr<PgfAxis> createMedianCostAxis(const std::string& plannerName) const;

  // Returns a pgf axis that holds the success percentage over time for all planners.
  std::shared_ptr<PgfAxis> createSuccessAxis() const;

  // Returns a pgf axis that holds the success percentage over time for the specified planner.
  std::shared_ptr<PgfAxis> createSuccessAxis(const std::string& plannerName) const;

  // Creates a tikz picture that contains the median cost axis of all planners.
  std::experimental::filesystem::path createMedianCostPicture() const;

  // Creates a tikz picture that contains the median cost axis of the specified planner.
  std::experimental::filesystem::path createMedianCostPicture(const std::string& plannerName) const;

  // Creates a tikz picture that contains the success axis of all planners.
  std::experimental::filesystem::path createSuccessPicture() const;

  // Creates a tikz picture that contains the success axis of the specified planner.
  std::experimental::filesystem::path createSuccessPicture(const std::string& plannerName) const;

  // Creates a combined tikz picture with success and median costs of all planners.
  std::experimental::filesystem::path createCombinedPicture() const;

  // Creates a combined tikz picture with success and median costs of the specified planner.
  std::experimental::filesystem::path createCombinedPicture(const std::string& plannerName) const;

  // Compiles the given tikzpicture to a pdf document.
  std::experimental::filesystem::path compileStandalonePdf(
      const std::experimental::filesystem::path& tikzPicture) const;

 private:
  // Plots with all planners.
  std::shared_ptr<PgfAxis> generateMedianCostPlot(const Statistics& stats,
                                                  std::size_t confidence) const;
  std::shared_ptr<PgfAxis> generateSuccessPlot(const Statistics& stats) const;

  // Plots of individual planners.
  std::shared_ptr<PgfAxis> generateMedianCostPlot(const Statistics& stats,
                                                  const std::string& plannerName,
                                                  std::size_t confidence) const;
  std::shared_ptr<PgfAxis> generateSuccessPlot(const Statistics& stats,
                                               const std::string& plannerName) const;

  const std::shared_ptr<const Configuration> config_{};
};

}  // namespace ompltools

}  // namespace esp
