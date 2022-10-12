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

#include <limits>
#include <string>
#include <vector>

namespace pdt {

namespace statistics {

// A linear interpolator that handles infinity values.
template <typename X, typename Y>
class LinearInterpolator {
 public:
  LinearInterpolator(const std::vector<X>& x, const std::vector<Y>& y);
  LinearInterpolator(const std::vector<std::pair<X, Y>>& valuePairs);
  ~LinearInterpolator() = default;

  Y operator()(X x) const;

 private:
  std::map<X, Y> data_{};
};

template <typename X, typename Y>
LinearInterpolator<X, Y>::LinearInterpolator(const std::vector<X>& x, const std::vector<Y>& y) {
  // Check input.
  if (x.size() < 2u || y.size() < 2u) {
    auto msg = std::string("Interpolator cannot interpolate fewer than 2 elements");
    throw std::runtime_error(msg);
  }
  if (x.size() != y.size()) {
    auto msg = std::string("Interpolator cannot interpolate fewer than 2 elements");
    throw std::runtime_error(msg);
  }

  // Create map. Some form of data copy seems sensible, otherwise we'd have to either
  // - Force the data to live on the heap (and be passed through a shared_ptr); or
  // - Make sure the caller manages the livetime correctly.
  // Copying seems like the best option.
  for (std::size_t i = 0u; i < x.size(); ++i) {
    if (std::numeric_limits<X>::has_infinity && x[i] == std::numeric_limits<X>::infinity()) {
      auto msg = std::string("Domain cannot contain infinity.");
      throw std::runtime_error(msg);
    }
    if (data_.find(x[i]) != data_.end()) {
      if (y[i] != data_[x[i]]) {
        auto msg = std::string("The same argument cannot map to a different value.");
        throw std::runtime_error(msg);
      }
    }
    data_.emplace(x[i], y[i]);
  }
}

template <typename X, typename Y>
LinearInterpolator<X, Y>::LinearInterpolator(const std::vector<std::pair<X, Y>>& valuePairs) {
  // Check input.
  if (valuePairs.size() < 2u) {
    auto msg = std::string("Interpolator cannot interpolate fewer than 2 elements");
    throw std::runtime_error(msg);
  }

  // Create map. Some form of data copy seems sensible, otherwise we'd have to either
  // - Force the data to live on the heap (and be passed through a shared_ptr); or
  // - Make sure the caller manages the livetime correctly.
  // Copying seems like the best option.
  for (const auto& pair : valuePairs) {
    if (std::numeric_limits<X>::has_infinity && pair.first == std::numeric_limits<X>::infinity()) {
      auto msg = std::string("Domain cannot contain infinity.");
      throw std::runtime_error(msg);
    }
    if (auto it = data_.find(pair.first); it != data_.end() && it->second != pair.second) {
      auto msg = std::string("The same argument cannot map to a different value.");
      throw std::runtime_error(msg);
    }
    data_.emplace(pair);
  }
}

template <typename X, typename Y>
Y LinearInterpolator<X, Y>::operator()(X x) const {
  // Get the sandwiching iterators.
  auto lower = data_.lower_bound(x);
  auto upper = data_.upper_bound(x);

  // the lower iterator is smaller equal, the upper is _greater_.
  // Thus we can check if we can simply return the lower iterator here.
  if (std::abs(lower->first - x) < 1e-6) {
    return lower->second;
  }

  if (lower == data_.end() || upper == data_.end()) {
    auto msg = std::string("Interpolator refuses to extrapolate.");
    throw std::runtime_error(msg);
  }

  // Get the values.
  auto [xLow, yLow] = *--lower;
  auto [xHigh, yHigh] = *upper;

  // If one of the two values are infinity, return infinity.
  if constexpr (std::numeric_limits<Y>::has_infinity) {
    if (yLow == std::numeric_limits<Y>::infinity() || yHigh == std::numeric_limits<Y>::infinity()) {
      return std::numeric_limits<Y>::infinity();
    }
  }

  // If the interpolation is trivial, let it be trivial.
  if (yLow == yHigh) {
    return yHigh;
  }

  // Some basic checks for overflows.
  if ((xLow < static_cast<X>(0) && xHigh > std::numeric_limits<X>::max() + xLow) ||
      (xLow > static_cast<X>(0) && xHigh < std::numeric_limits<X>::lowest() + xLow)) {
    auto msg = std::string("xHigh - xLow overflows.");
    throw std::runtime_error(msg);
  } else if ((xLow < static_cast<X>(0) && x > std::numeric_limits<X>::max() + xLow) ||
             (xLow > static_cast<X>(0) && x < std::numeric_limits<X>::lowest() + xLow)) {
    auto msg = std::string("x - xLow overflows.");
    throw std::runtime_error(msg);
  } else if ((yLow < static_cast<Y>(0) && yHigh > std::numeric_limits<Y>::max() + yLow) ||
             (yLow > static_cast<Y>(0) && yHigh < std::numeric_limits<Y>::lowest() + yLow)) {
    auto msg = std::string("yHigh - yLow overflows.");
    throw std::runtime_error(msg);
  }

  return yLow + (x - xLow) / (xHigh - xLow) * (yHigh - yLow);
}

}  // namespace statistics

}  // namespace pdt
