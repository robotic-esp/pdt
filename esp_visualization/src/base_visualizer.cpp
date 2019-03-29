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

#include "esp_visualization/base_visualizer.h"

#include <chrono>
#include <exception>

#include <ompl/base/PlannerTerminationCondition.h>

#include "esp_time/time.h"

namespace esp {

namespace ompltools {

BaseVisualizer::BaseVisualizer(
    const std::pair<std::shared_ptr<BaseContext>, CONTEXT_TYPE> &contextPair,
    const std::pair<std::shared_ptr<ompl::base::Planner>, PLANNER_TYPE> plannerPair) :
    context_(contextPair.first),
    contextType_(contextPair.second),
    planner_(plannerPair.first),
    plannerType_(plannerPair.second),
    dataThreadPromise_(),
    dataThreadStopSignal_(dataThreadPromise_.get_future()) {
  dataThread_ = std::thread(&BaseVisualizer::createData, this);
}

BaseVisualizer::~BaseVisualizer() {
  dataThreadPromise_.set_value();
  dataThread_.join();
}

void BaseVisualizer::setContext(
    const std::pair<std::shared_ptr<BaseContext>, CONTEXT_TYPE> &contextPair) {
  // Setting a new context means all the data is invalid.

  // Stop iterating.
  dataThreadPromise_.set_value();
  dataThread_.join();

  // Set the new context.
  context_ = contextPair.first;
  contextType_ = contextPair.second;

  // Reset the promise and stop signal.
  dataThreadPromise_ = std::promise<void>();
  dataThreadStopSignal_ = dataThreadPromise_.get_future();

  // Reset all data.
  plannerData_.clear();
  durations_.clear();
  setupDuration_ = time::Duration(0.0);

  // Create data.
  dataThread_ = std::thread(&BaseVisualizer::createData, this);
}

void BaseVisualizer::setPlanner(
    const std::pair<std::shared_ptr<ompl::base::Planner>, PLANNER_TYPE> &plannerPair) {
  // Setting a new context means all the data is invalid.

  // Stop iterating.
  dataThreadPromise_.set_value();
  dataThread_.join();

  // Set the new planner.
  planner_ = plannerPair.first;
  plannerType_ = plannerPair.second;

  // Reset the promise and stop signal.
  dataThreadPromise_ = std::promise<void>();
  dataThreadStopSignal_ = dataThreadPromise_.get_future();

  // Reset all data.
  plannerData_.clear();
  durations_.clear();
  viewedIteration_ = 0u;
  largestIteration_ = 0u;
  setupDuration_ = time::Duration(0.0);

  // Create data.
  dataThread_ = std::thread(&BaseVisualizer::createData, this);
}

std::shared_ptr<const ompl::base::PlannerData> BaseVisualizer::getPlannerData(
    std::size_t iteration) const {
  std::scoped_lock lock(plannerDataMutex_);
  if (iteration >= plannerData_.size()) {
    std::cout << "Requested iteration: " << iteration << ", available: " << plannerData_.size()
              << '\n';
    throw std::runtime_error("Requested planner data of iteration that has not yet been processed");
  }
  return plannerData_.at(iteration);
}

time::Duration BaseVisualizer::getIterationDuration(std::size_t iteration) const {
  std::scoped_lock lock(durationsMutex_);
  if (iteration >= plannerData_.size()) {
    throw std::runtime_error(
        "Requested iteration duration of iteration that has not yet been processed");
  }
  return durations_.at(iteration);
}

time::Duration BaseVisualizer::getTotalElapsedDuration(std::size_t iteration) const {
  std::scoped_lock lock(durationsMutex_, setupDurationMutex_);
  if (iteration >= durations_.size()) {
    throw std::runtime_error(
        "Requested elapsed duration of iteration that has not yet been processed");
  }
  return std::accumulate(durations_.begin(), durations_.begin() + iteration, setupDuration_);
}

void BaseVisualizer::createData() {
  {  // Setup the planner.
    std::scoped_lock lock(setupDurationMutex_);
    if (!context_) {
      throw std::runtime_error("Requested to create data, but no context has been set.");
    }
    if (!planner_) {
      throw std::runtime_error("Requested to create data, but no planner has been set.");
    }
    auto setupStartTime = time::Clock::now();
    planner_->setup();
    setupDuration_ = time::Clock::now() - setupStartTime;
  }

  while (dataThreadStopSignal_.wait_for(std::chrono::milliseconds(1)) ==
         std::future_status::timeout) {
    // Create a new iteration if we we're viewing one thats uncomfortably close.
    if (viewedIteration_ + iterationBuffer_ > largestIteration_) {
      // Create a termination condition that stops the planner after one iteration.
      ompl::base::IterationTerminationCondition terminationCondition(1u);

      // Advance one iteration.
      auto iterStartTime = time::Clock::now();
      planner_->solve(terminationCondition);
      auto iterationDuration = time::Clock::now() - iterStartTime;

      // Get the planner data.
      auto plannerData = std::make_shared<ompl::base::PlannerData>(context_->getSpaceInformation());
      planner_->getPlannerData(*plannerData);

      {  // Store the iteration duration.
        std::scoped_lock lock(durationsMutex_);
        durations_.emplace_back(iterationDuration);
      }

      {  // Store the planner data.
        std::scoped_lock lock(plannerDataMutex_);
        plannerData_.emplace_back(plannerData);
        largestIteration_ = plannerData_.size() - 1u;
      }
    }
  }
}

}  // namespace ompltools

}  // namespace esp
