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

#ifndef GENERAL_TOOLS
#define GENERAL_TOOLS

//For io streams
#include <iostream>
//For stringstream
#include <sstream>
//For setw and setprecision, etc.
#include <iomanip>
//For infinities
#include <cmath>
//For time
#include <chrono>

#define ASRL_DURATION_INFINITY asrl::time::clock::duration::max()


//Stream operator defined after the namespace
template <class CLOCK, class DUR>
std::ostream& operator<<(std::ostream& out, const std::chrono::time_point<CLOCK, DUR>& point);

//Stream operator defined after the namespace
template<class REP, class PER>
std::ostream& operator<< (std::ostream& out, const std::chrono::duration<REP, PER>& dur);

namespace asrl
{
  namespace time
  {
    ///The specific clock to use
    typedef std::chrono::high_resolution_clock clock;

    ///This was copied from ompl/util/Time.h and modified to use the specified clock
    typedef asrl::time::clock::time_point point;

    ///This was copied from ompl/util/Time.h and modified to use the specified clock
    typedef asrl::time::clock::duration duration;

    ///This is new, and it's untested.
    inline bool isfinite(const asrl::time::duration& dur)
    {
      return (dur.count() != ASRL_DURATION_INFINITY.count());
    }

    ///This was copied from ompl/util/Time.h and modified to use the specified clock
    inline asrl::time::point now()
    {
        return asrl::time::clock::now();
    }

    /** \brief Return string representation of point in time */
    inline std::string as_string(const asrl::time::point &p)
    {
      std::stringstream ss;
      ss << p;
      return ss.str();
    }

    ///This was copied from ompl/util/Time.h and modified to use ns instead of us
    /** \brief Return the time duration representing a given number of seconds */
    inline asrl::time::duration seconds(double sec)
    {
      long s = static_cast<long>(sec);
      long ns = static_cast<long>((sec - static_cast<double>(s)) * 1e9);
      return std::chrono::seconds(s) + std::chrono::nanoseconds(ns);
    }

    ///This was copied from ompl/util/Time.h
    /** \brief Return the number of seconds that a time duration represents */
    inline double seconds(const asrl::time::duration &d)
    {
      return std::chrono::duration<double>(d).count();
    }
  }
}

//An output operator for std::chrono::duration that mimics the old boost::posix format, i.e., it is 15 characters wide of the format: HH:MM:SS.XXXXXX
template<class REP, class PER>
std::ostream& operator<< (std::ostream& out, const std::chrono::duration<REP, PER>& dur)
{
  //Variables
  //The stream to output
  std::stringstream out_stream;

  if (asrl::time::isfinite(dur) == true)
  {
    long hr = std::chrono::duration_cast<std::chrono::hours>(dur).count();
    long min = std::chrono::duration_cast<std::chrono::minutes>(dur).count() - 60*hr;
    long s = std::chrono::duration_cast<std::chrono::seconds>(dur).count() - 60*(min + 60*hr);
    long us = std::chrono::duration_cast<std::chrono::microseconds>(dur).count() - 1e6*(s + 60*(min + 60*hr));

    out_stream << std::setw(2) << std::setfill('0') << hr << ":"
               << std::setw(2) << std::setfill('0') << min << ":"
               << std::setw(2) << std::setfill('0') << s << "."
               << std::setw(6) << std::setfill('0') << us;
  }
  else
  {
    out_stream << " infinite";
  }

  out << out_stream.str();

  return out;
};

//An output operator for std::chrono::point
//Similar code exists in ompl/util/Time.h as as_string()
template <class CLOCK, class DUR>
std::ostream& operator<<(std::ostream& out, const std::chrono::time_point<CLOCK, DUR>& point)
{
    // Variables
    // The c representation of the time
    std::time_t cTime;
    // The duration since epoch
    DUR epoch_dur;
    // The microseconds since epoch
    typename DUR::rep epoch_us;

    //Get the duration since epoch
    epoch_dur = point.time_since_epoch();

    //The us since epoch
    epoch_us = std::chrono::duration_cast<std::chrono::microseconds>(epoch_dur).count() - 1e6*std::chrono::duration_cast<std::chrono::seconds>(epoch_dur).count();

    //Get the ctime
    cTime = CLOCK::to_time_t(point);

    //Output it
    out << std::put_time(std::localtime(&cTime), "%F %T.") << epoch_us;

    return out;
};
#endif //GENERAL_TOOLS
