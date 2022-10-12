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

#include <atomic>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerData.h>

#include "pdt/common/planner_type.h"
#include "pdt/config/configuration.h"
#include "pdt/planning_contexts/real_vector_geometric_context.h"
#include "pdt/time/time.h"
#include "pdt/visualization/planner_specific_data.h"

namespace pdt {

namespace visualization {

// The base class for a visualizer.
class BaseVisualizer {
 public:
  BaseVisualizer(
      const std::shared_ptr<config::Configuration> &config,
      const std::shared_ptr<planning_contexts::BaseContext> &context,
      const std::pair<std::shared_ptr<ompl::base::Planner>, common::PLANNER_TYPE> plannerPair);
  virtual ~BaseVisualizer();

 public:
  // Setters for context and planner. These are final because they need to be threadsafe, so
  // care has to be taken when implementing them.
  virtual void setContext(
      const std::shared_ptr<planning_contexts::RealVectorGeometricContext> &context) final;
  virtual void setPlanner(const std::pair<std::shared_ptr<ompl::base::Planner>,
                                          common::PLANNER_TYPE> &plannerPair) final;

 protected:
  // Make data available to derived classes.
  std::shared_ptr<const ompl::base::PlannerData> getPlannerData(const std::size_t iteration) const;
  std::shared_ptr<const PlannerSpecificData> getPlannerSpecificData(
      const std::size_t iteration) const;
  time::Duration getIterationDuration(const std::size_t iteration) const;
  const ompl::base::PathPtr getSolutionPath(const std::size_t iteration) const;
  ompl::base::Cost getSolutionCost(const std::size_t iteration) const;
  time::Duration getTotalElapsedDuration(const std::size_t iteration) const;
  std::size_t getQueryNumber(const std::size_t iteration) const;

  // The current context.
  std::shared_ptr<planning_contexts::BaseContext> context_{};

  // The current planner.
  std::shared_ptr<ompl::base::Planner> planner_{};
  common::PLANNER_TYPE plannerType_{};

  // The currently viewed and largest iterations.
  std::atomic<std::size_t> displayIteration_{0u};
  std::atomic<std::size_t> largestIteration_{0u};

  // Some nice colors.
  float black[4] = {0.0f, 0.0f, 0.0f, 1.0f};
  float white[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  float transparent[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float gray[4] = {0.471f, 0.471f, 0.471f, 1.0f};
  float red[4] = {0.808f, 0.243f, 0.082f, 1.0f};
  float blue[4] = {0.043f, 0.365f, 0.682f, 1.0f};
  float yellow[4] = {0.910f, 0.639f, 0.102f, 1.0f};
  float green[4] = {0.392f, 0.631f, 0.106f, 1.0f};
  float purple[4] = {0.416f, 0.078f, 0.490f, 1.0f};
  float lightblue[4] = {0.231f, 0.686f, 0.925f, 1.0f};
  float darkred[4] = {0.569f, 0.000f, 0.129f, 1.0f};

 private:
  // Create the data to visualize. This should be called on a separate thread.
  void createData();

  // The configuration for this visualization.
  const std::shared_ptr<const config::Configuration> config_;

  // This is how many iterations we'll create ahead of the viewed iteration.
  std::size_t iterationBuffer_{1000u};

  // The planner data, indexed by the iteration.
  std::vector<std::shared_ptr<ompl::base::PlannerData>> plannerData_{};
  mutable std::mutex plannerDataMutex_{};

  // The planner specific data, indexed by the iteration.
  std::vector<std::shared_ptr<PlannerSpecificData>> plannerSpecificData_{};
  mutable std::mutex plannerSpecificDataMutex_{};

  // The iteration durations, indexed by the iteration.
  std::vector<time::Duration> durations_{};
  mutable std::mutex durationsMutex_{};

  // The total elapsed duration.
  time::Duration setupDuration_{};
  mutable std::mutex setupDurationMutex_{};

  // The solution paths, indexed by the iteration.
  std::vector<ompl::base::PathPtr> solutionPaths_{};
  mutable std::mutex solutionPathsMutex_{};

  // The costs of the solution paths, indexed by the iteration.
  std::vector<ompl::base::Cost> solutionCosts_{};
  mutable std::mutex solutionCostsMutex_{};

  // The query-numbers, indexed by the iteration.
  std::vector<std::size_t> queryNumbers_{};
  mutable std::mutex queryNumbersMutex_{};

  // The thread that creates the data, i.e., solves the given planning problem.
  std::thread dataThread_{};

  // The promise and future to gracefully join the data thread upon destruction.
  std::promise<void> dataThreadPromise_{};
  std::future<void> dataThreadStopSignal_{};
};

}  // namespace visualization

}  // namespace pdt
