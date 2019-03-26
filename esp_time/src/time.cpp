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

// Authors: Jonathan Gammell, Marlin Strub

#include "esp_time/time.h"

namespace esp {

namespace ompltools {

namespace time {

// Convert a TimePoint to a string.
// See https://rextester.com/HADNXK16356
std::string toDateString(const std::chrono::system_clock::time_point& timePoint,
                         const std::string& format) {
  const std::time_t t = std::chrono::system_clock::to_time_t(timePoint);
  const std::tm tm = *std::localtime(std::addressof(t));
  std::ostringstream stream;
  stream << std::put_time(std::addressof(tm), format.c_str());
  return stream.str();
}

// Convert a double to a duration.
Duration seconds(double sec) {
  return Duration(sec);
}

// Convert a duration to a double.
double seconds(Duration sec) {
  return std::chrono::duration<double, std::ratio<1>>(sec).count();
}

}  // namespace time

}  // namespace ompltools

}  // namespace esp

std::ostream& operator<<(std::ostream& out, const esp::ompltools::time::Duration& duration) {
  std::stringstream stream;
  // Handle the case of time with infinite value.
  if (duration.count() == std::numeric_limits<double>::infinity()) {
    stream << "inf";
  } else {
    // Convert to HH:MM:SS:XXXXXX format (15 chars wide).
    long hr = std::chrono::duration_cast<std::chrono::hours>(duration).count();
    long min = std::chrono::duration_cast<std::chrono::minutes>(duration).count() - 60 * hr;
    long s =
        std::chrono::duration_cast<std::chrono::seconds>(duration).count() - 60 * (min + 60 * hr);
    long us = std::chrono::duration_cast<std::chrono::microseconds>(duration).count() -
              1e6 * (s + 60 * (min + 60 * hr));

    stream << std::setw(2) << std::setfill('0') << hr << ":" << std::setw(2) << std::setfill('0')
           << min << ":" << std::setw(2) << std::setfill('0') << s << "." << std::setw(6)
           << std::setfill('0') << us;
  }

  out << stream.str();
  return out;
}