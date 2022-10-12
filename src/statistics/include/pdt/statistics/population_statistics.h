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

#pragma once

#include <iterator>

#include "pdt/config/configuration.h"

namespace pdt {

namespace statistics {

struct ConfidenceInterval {
  // The lower and upper bounds are zero-based indices. If I have 10 measurements in a sorted
  // vector, e.g.,
  //   v = { 0.1 0.3 0.4 0.5 0.8 0.9 1.1 1.3 1.7 1.9 },
  // then lower and upper bound indices of 1 and 8, respectively, mean that the percentile is with
  // <confidence> certainty in the open interval (v[1], v[8]) = (0.3, 1.7).
  std::size_t lower{0u}, upper{0u};
  double confidence{0.0};
};

bool operator<(const ConfidenceInterval& ci, double confidence);

class PopulationStatistics {
 public:
  // The direction to round calculations for index corresponding to an estimate of a percentile.
  enum class INDEX_ROUNDING {
    UP,
    DOWN,
  };

  // Construct/initialize
  PopulationStatistics(const std::shared_ptr<config::Configuration>& config,
                       const INDEX_ROUNDING rounding);
  ~PopulationStatistics() = default;

  void setSampleSize(const std::size_t sampleSize);
  std::size_t getSampleSize() const;

  // Returns the ordered index that estimates the provided percentile.
  std::size_t estimatePercentileAsIndex(const double percentile) const;
  // Finds an estimate for the specified percentile with the specified confidence. Specifically
  // finds a symmetric interval centered about the best-estimate of the percentile that contains the
  // true percentile with the specified confidence.
  ConfidenceInterval findPercentileConfidenceInterval(const double percentile,
                                                      const double confidence) const;
  // Calculates the confidence with which an interval estimates a percentile
  double calcPercentileConfidence(const double percentile, const std::size_t lowerIdx,
                                  const std::size_t upperIdx) const;

 private:
  class ConfidenceIntervalIterator {
   public:
    // Traits
    using iterator_category = std::random_access_iterator_tag;
    // Taken from std::vector<double>::iterator::difference_type:
    using difference_type = long int;
    using value_type = ConfidenceInterval const;
    using pointer = value_type*;
    using reference = value_type&;

    // A subset of iterator methods for std::lower_bound and other conveniences.
    ConfidenceIntervalIterator(const PopulationStatistics* parent, const double percentile,
                               difference_type offset);
    // Without an offset value, constructs end() (i.e., offset_ = maxDereferenceOffset_ + 1u).
    ConfidenceIntervalIterator(const PopulationStatistics* parent, const double percentile);
    reference operator*() const;
    pointer operator->() const;
    // Note that these define ++iter and --iter but not iter++ and iter--
    ConfidenceIntervalIterator& operator++();
    ConfidenceIntervalIterator& operator--();
    ConfidenceIntervalIterator& operator+=(const difference_type offset);
    difference_type operator-(const ConfidenceIntervalIterator& other);
    bool operator==(const ConfidenceIntervalIterator& other) const;
    bool operator!=(const ConfidenceIntervalIterator& other) const;

   private:
    // Member variables
    const PopulationStatistics* parent_;
    double percentile_;
    std::size_t centerIdx_, maxDereferenceOffset_;
    std::size_t offset_{0u};
    ConfidenceInterval value_{0u, 0u, 0.0};
  };

  // Convenience key base
  std::string percentileKey(double percentile) const;

  // Iterator interface for finding a specified CI for a percentile.
  ConfidenceIntervalIterator begin(const double percentile) const;
  ConfidenceIntervalIterator end(const double percentile) const;

  std::shared_ptr<config::Configuration> config_;
  INDEX_ROUNDING round_;
  std::size_t sampleSize_{0u};
};

}  // namespace statistics

}  // namespace pdt
