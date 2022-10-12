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

#include "pdt/statistics/population_statistics.h"

#include <boost/math/special_functions/beta.hpp>
#include <iostream>
#include <string>

namespace pdt {

namespace statistics {

using namespace std::string_literals;

bool operator<(const ConfidenceInterval& ci, const double confidence) {
  return ci.confidence < confidence;
}

PopulationStatistics::PopulationStatistics(const std::shared_ptr<config::Configuration>& config,
                                           const INDEX_ROUNDING round) :
    config_{config},
    round_{round} {
}

void PopulationStatistics::setSampleSize(const std::size_t sampleSize) {
  sampleSize_ = sampleSize;
}

std::size_t PopulationStatistics::getSampleSize() const {
  return sampleSize_;
}

std::size_t PopulationStatistics::estimatePercentileAsIndex(const double percentile) const {
  if (sampleSize_ == 0u) {
    throw std::out_of_range("Requested a percentile estimate for a sample size of 0.");
  }
  if (percentile < 0.0 || percentile > 1.0) {
    auto msg = "The requested percentile ("s + std::to_string(percentile) +
               ") is not in the interval [0, 1]."s;
    throw std::out_of_range(msg);
  }

  std::string key = percentileKey(percentile) + "/orderedIndex"s;

  if (!config_->contains(key)) {
    double orderedDouble = percentile * static_cast<double>(sampleSize_ - 1u);

    // Which direction is conservative will depend on the cost ordering.
    std::size_t orderedIndex;
    if (round_ == INDEX_ROUNDING::UP) {
      orderedIndex = static_cast<std::size_t>(std::ceil(orderedDouble));
    } else if (round_ == INDEX_ROUNDING::DOWN) {
      orderedIndex = static_cast<std::size_t>(std::floor(orderedDouble));
    } else {
      throw std::runtime_error("Unimplemented INDEX_ROUNDING choice.");
    }

    config_->add<std::size_t>(key, orderedIndex);
  }

  return config_->get<std::size_t>(key);
}

double PopulationStatistics::calcPercentileConfidence(const double percentile,
                                                      const std::size_t lowerIdx,
                                                      const std::size_t upperIdx) const {
  if (sampleSize_ == 0u) {
    throw std::out_of_range("Requested the percentile confidence for a sample size of 0.");
  }
  if (percentile < 0.0 || percentile > 1.0) {
    auto msg = "The requested percentile ("s + std::to_string(percentile) +
               ") is not in the interval [0, 1]."s;
    throw std::out_of_range(msg);
  }
  if (lowerIdx >= sampleSize_ || upperIdx >= sampleSize_) {
    auto msg = "The 0-based interval ("s + std::to_string(lowerIdx) + ", "s +
               std::to_string(upperIdx) + ") is out of range for "s + std::to_string(sampleSize_) +
               " samples.\n"s;
    throw std::out_of_range(msg);
  }

  /*****
    From https://probabilityandstats.wordpress.com/2010/02/22/confidence-intervals-for-percentiles/
      CI(value-percentile-p) = Prob(ith-value < value-percentile-p < jth-value)
                             = \sum_{k=i}^{j-1} C(n, k) p^k (1 - p)^{n-k}
    where i, j \in [1, n] and C(n, k) is the binomial coefficient.
    This can be split into two sums starting at zero
      CI(p) = \sum_{k=0}^{j-1} C(n, k) p^k (1-p)^{n-k} - \sum_{k=0}^{i-1} C(n, k) p^k (1-p)^{n-k})
    and then noting that from
    https://en.wikipedia.org/wiki/Binomial_distribution#Cumulative_distribution_function
      \sum_{k=0}^{q} C(n, k) p^{k} (1-p)^{n-k} = CDF(q; n, p)
                                               = I_{1-p}(n-q, q+1)
    we get
      CI(p) = CDF(j-1; n, p) - CDF(i-1; n, p)
            = I_{1-p}(n-j+1, j) - I_{1-p}(n-i+1, i)
    which when we use I_{x}(a,b) = boost::math::ibeta(T1 a, T2 b, T3 x)
    results in:
      CI(p) = boost::math::ibeta(n-j+1, j, 1-p) - boost::math::ibeta(n-i+1, i, 1-p)
    where still i, j \in [1, n]. Finally lowerIdx, upperIdx \in [0, n-1] so
      i = lowerIdx + 1u;
      j = upperIdx + 1u;
      n = sampleSize_;
      p = percentile_;
    and the final result is:
  *****/
  return boost::math::ibeta(sampleSize_ - upperIdx, upperIdx + 1u, 1.0 - percentile) -
         boost::math::ibeta(sampleSize_ - lowerIdx, lowerIdx + 1u, 1.0 - percentile);
}

ConfidenceInterval PopulationStatistics::findPercentileConfidenceInterval(
    const double percentile, const double confidence) const {
  if (sampleSize_ == 0u) {
    throw std::out_of_range("Requested a confidence interval for a sample size of 0.");
  }
  if (percentile < 0.0 || percentile > 1.0) {
    auto msg = "The requested percentile ("s + std::to_string(percentile) +
               ") is not in the interval [0, 1]."s;
    throw std::out_of_range(msg);
  }
  if (confidence < 0.0 || confidence > 1.0) {
    auto msg = "The requested confidence ("s + std::to_string(confidence) +
               ") is not in the interval [0, 1]."s;
    throw std::out_of_range(msg);
  }

  std::stringstream key;
  key << percentileKey(percentile) << "/confidenceInterval/"s << std::fixed << std::setfill('0')
      << std::setw(4) << std::setprecision(2) << confidence;

  if (!config_->contains(key.str())) {
    // Find the specified CI
    auto ciIter = std::lower_bound(begin(percentile), end(percentile), confidence);

    if (ciIter == end(percentile)) {
      --ciIter;
      std::cout << std::endl
                << "    Warning: Could not find an interval with a confidence of "
                << 100.0 * confidence << "% for estimating the " << 100.0 * percentile
                << "th percentile from " << sampleSize_ << " samples. Using (" << ciIter->lower
                << ", " << ciIter->upper << ") instead, which gives " << 100.0 * ciIter->confidence
                << "% confidence." << std::endl;
    }

    config_->add<std::size_t>(key.str() + "/lowerOrderedIndex", ciIter->lower);
    config_->add<std::size_t>(key.str() + "/upperOrderedIndex", ciIter->upper);
    config_->add<double>(key.str() + "/confidence", ciIter->confidence);
  }

  return {config_->get<std::size_t>(key.str() + "/lowerOrderedIndex"),
          config_->get<std::size_t>(key.str() + "/upperOrderedIndex"),
          config_->get<double>(key.str() + "/confidence")};
}

std::string PopulationStatistics::percentileKey(const double percentile) const {
  std::stringstream key;
  key << "statistics/percentiles/sampleSize/"s << sampleSize_ << "/populationPercentile/"s
      << std::fixed << std::setfill('0') << std::setw(4) << std::setprecision(2) << percentile;
  return key.str();
}

PopulationStatistics::ConfidenceIntervalIterator PopulationStatistics::begin(
    const double percentile) const {
  return ConfidenceIntervalIterator(this, percentile, 0);
}

PopulationStatistics::ConfidenceIntervalIterator PopulationStatistics::end(
    const double percentile) const {
  return ConfidenceIntervalIterator(this, percentile);
}

PopulationStatistics::ConfidenceIntervalIterator::ConfidenceIntervalIterator(
    const PopulationStatistics* parent, const double percentile, const difference_type offset) :
    parent_(parent),
    percentile_(percentile),
    centerIdx_(parent_->estimatePercentileAsIndex(percentile_)),
    maxDereferenceOffset_(std::max(centerIdx_, parent_->getSampleSize() - 1u - centerIdx_)) {
  *this += offset;
}

PopulationStatistics::ConfidenceIntervalIterator::ConfidenceIntervalIterator(
    const PopulationStatistics* parent, const double percentile) :
    parent_(parent),
    percentile_(percentile),
    centerIdx_(parent_->estimatePercentileAsIndex(percentile_)),
    maxDereferenceOffset_(std::max(centerIdx_, parent_->getSampleSize() - 1u - centerIdx_)) {
  offset_ = maxDereferenceOffset_ + 1u;
}

PopulationStatistics::ConfidenceIntervalIterator::reference
    PopulationStatistics::ConfidenceIntervalIterator::operator*() const {
  if (offset_ > maxDereferenceOffset_) {
    auto msg = "ConfidenceIntervalIterator is out of range at index "s + std::to_string(offset_) +
               " > "s + std::to_string(maxDereferenceOffset_) + "."s;
    throw std::out_of_range(msg);
  }
  return value_;
}

PopulationStatistics::ConfidenceIntervalIterator::pointer
    PopulationStatistics::ConfidenceIntervalIterator::operator->() const {
  if (offset_ > maxDereferenceOffset_) {
    auto msg = "ConfidenceIntervalIterator is out of range at index "s + std::to_string(offset_) +
               " > "s + std::to_string(maxDereferenceOffset_) + "."s;
    throw std::out_of_range(msg);
  }
  return &value_;
}

PopulationStatistics::ConfidenceIntervalIterator& PopulationStatistics::ConfidenceIntervalIterator::
operator++() {
  *this += 1;
  return *this;
}

PopulationStatistics::ConfidenceIntervalIterator& PopulationStatistics::ConfidenceIntervalIterator::
operator--() {
  *this += -1;
  return *this;
}

PopulationStatistics::ConfidenceIntervalIterator& PopulationStatistics::ConfidenceIntervalIterator::
operator+=(const difference_type delta) {
  // Delta cannot move the iterator further than 0 (i.e., begin()) or maxDereferenceOffset_ + 1
  // (i.e., end())
  if (delta < 0) {
    if (static_cast<std::size_t>(std::abs(delta)) > offset_) {
      // Moving beyond the begin(), i.e., to an offset_ less than 0.
      auto msg = "Attempting to move ConfidenceIntervalIterator from " + std::to_string(offset_) +
                 " by "s + std::to_string(delta) + " which would be less than begin()."s;
      throw std::out_of_range(msg);
    }
  } else {
    if (static_cast<std::size_t>(delta) > maxDereferenceOffset_ - offset_ + 1u) {
      // Moving beyond end(), i.e., to an offset_ beyond maxDereferenceOffset_ + 1.
      auto msg = "Attempting to move ConfidenceIntervalIterator from " + std::to_string(offset_) +
                 " by "s + std::to_string(delta) + " which would be more than end()."s;
      throw std::out_of_range(msg);
    }
  }

  // Update the offset
  if (delta < 0) {
    offset_ = offset_ - static_cast<std::size_t>(std::abs(delta));
  } else {
    offset_ = offset_ + static_cast<std::size_t>(delta);
  }

  // Offsets are valid until _both_ indices are at the limit in order to support asymmetric center
  // points.
  std::size_t lowerIdx = 0u;
  if (offset_ < centerIdx_) {
    lowerIdx = centerIdx_ - offset_;
  }

  std::size_t upperIdx = parent_->getSampleSize() - 1u;
  if (offset_ < parent_->getSampleSize() - 1u - centerIdx_) {
    upperIdx = centerIdx_ + offset_;
  }

  // Calculate value if not out of bounds
  if (offset_ <= maxDereferenceOffset_) {
    value_ = {lowerIdx, upperIdx,
              parent_->calcPercentileConfidence(percentile_, lowerIdx, upperIdx)};
  } else {
    value_ = {0u, 0u, 0.0};
  }
  return *this;
}

PopulationStatistics::ConfidenceIntervalIterator::difference_type
PopulationStatistics::ConfidenceIntervalIterator::operator-(
    const ConfidenceIntervalIterator& other) {
  return static_cast<difference_type>(offset_) - static_cast<difference_type>(other.offset_);
}

bool PopulationStatistics::ConfidenceIntervalIterator::operator==(
    const ConfidenceIntervalIterator& other) const {
  return parent_ == other.parent_ && percentile_ == other.percentile_ &&
         centerIdx_ == other.centerIdx_ && maxDereferenceOffset_ == other.maxDereferenceOffset_ &&
         offset_ == other.offset_;
}

bool PopulationStatistics::ConfidenceIntervalIterator::operator!=(
    const ConfidenceIntervalIterator& other) const {
  return !(*this == other);
}

}  // namespace statistics

}  // namespace pdt
