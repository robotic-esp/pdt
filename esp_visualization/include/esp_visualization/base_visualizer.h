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

#pragma once

#include <atomic>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include <ompl/base/Planner.h>

#include "esp_common/planner_type.h"
#include "esp_planning_contexts/base_context.h"
#include "esp_time/time.h"

namespace esp {

namespace ompltools {

// The base class for a visualizer.
class BaseVisualizer {
 public:
  BaseVisualizer(const std::shared_ptr<BaseContext> &context,
                 const std::pair<std::shared_ptr<ompl::base::Planner>, PLANNER_TYPE> plannerPair);
  virtual ~BaseVisualizer();

 public:
  // Setters for context and planner. These are final because they need to be threadsafe, so
  // care has to be taken when implementing them.
  virtual void setContext(
      const std::shared_ptr<BaseContext> &context) final;
  virtual void setPlanner(
      const std::pair<std::shared_ptr<ompl::base::Planner>, PLANNER_TYPE> &plannerPair) final;

 protected:
  // Make data available to derived classes.
  std::shared_ptr<const ompl::base::PlannerData> getPlannerData(std::size_t iteration) const;
  time::Duration getIterationDuration(std::size_t iteration) const;
  const ompl::base::PathPtr getSolutionPath(std::size_t iteration) const;
  time::Duration getTotalElapsedDuration(std::size_t iteration) const;

  // The current context.
  std::shared_ptr<BaseContext> context_{};

  // The current planner.
  std::shared_ptr<ompl::base::Planner> planner_{};
  PLANNER_TYPE plannerType_{};

  // The currently viewed and largest iterations.
  std::atomic<std::size_t> viewedIteration_{0u};
  std::atomic<std::size_t> largestIteration_{0u};

  // Some nice colors.
  float black[3] = {0.0, 0.0, 0.0};
  float white[3] = {1.0, 1.0, 1.0};
  float gray[3] = {0.471, 0.471, 0.471};
  float red[3] = {0.808, 0.243, 0.082};
  float blue[3] = {0.043, 0.365, 0.682};
  float yellow[3] = {0.910, 0.639, 0.102};
  float green[3] = {0.392, 0.631, 0.106};
  float purple[3] = {0.416, 0.078, 0.490};
  float lightblue[3] = {0.231, 0.686, 0.925};
  float darkred[3] = {0.569, 0.000, 0.129};

 private:
  // Create the data to visualize. This should be called on a separate thread.
  void createData();

  // This is how many iterations we'll create ahead of the viewed iteration.
  static constexpr std::size_t iterationBuffer_{2000u};

  // The planner data, indexed by the iteration.
  std::vector<std::shared_ptr<ompl::base::PlannerData>> plannerData_{};
  mutable std::mutex plannerDataMutex_{};

  // The iteration durations, indexed by the iteration.
  std::vector<time::Duration> durations_{};
  mutable std::mutex durationsMutex_{};

  // The total elapsed duration.
  time::Duration setupDuration_{};
  mutable std::mutex setupDurationMutex_{};

  std::vector<ompl::base::PathPtr> solutionPaths_{};
  mutable std::mutex solutionPathsMutex_{};
  // TODO add a generic datastructure to hold planner specific data.

  // The thread that creates the data, i.e., solves the given planning problem.
  std::thread dataThread_{};

  // The promise and future to gracefully join the data thread upon destruction.
  std::promise<void> dataThreadPromise_{};
  std::future<void> dataThreadStopSignal_{};
};

}  // namespace ompltools

}  // namespace esp
