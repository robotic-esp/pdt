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

// Authors: Jonathan Gammell, Marlin Strub

#pragma once

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <type_traits>

namespace esp {

namespace pdt {

namespace time {

// There are two fundamentally different types of clocks, called steady and unsteady. Steady clocks
// are monotonically increasing, which makes them good at measuring duration. Unsteady clocks
// sometimes move backward in time to correct for accumulated drift, which makes them good for
// telling the exact time. We need a steady clock because we care about measuring duration.
using Clock = std::conditional<std::chrono::high_resolution_clock::is_steady,
                               std::chrono::high_resolution_clock, std::chrono::steady_clock>::type;

// Apparently sometimes even stead_clock is not steady. For example GCC 4.7 must be built with
// --enable-libstdcxx-time=rt to get a steady std::steady_clock. Let's prevent nasty surprises.
static_assert(Clock::is_steady);

// Storing durations as seconds in doubles.
using Duration = std::chrono::duration<double, std::ratio<1>>;

// Convert a TimePoint to a string.
std::string toDateString(const std::chrono::system_clock::time_point& timePoint);

// Convert a duration to a string.
std::string toDurationString(const Duration& duration);

// Convert a double to a duration.
Duration seconds(double sec);

// Convert a duration to a double.
double seconds(Duration sec);

}  // namespace time

}  // namespace pdt

}  // namespace esp

// A pretty output operator for durations.
std::ostream& operator<<(std::ostream& out, const esp::pdt::time::Duration& duration);

// A pretty output operator for times (dates).
std::ostream& operator<<(std::ostream& out, const std::chrono::system_clock::time_point timePoint);
