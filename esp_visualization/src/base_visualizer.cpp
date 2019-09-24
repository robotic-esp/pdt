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
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/tbdstar/TBDstar.h>

#include "esp_time/time.h"

namespace esp {

namespace ompltools {

BaseVisualizer::BaseVisualizer(
    const std::shared_ptr<Configuration> &config, const std::shared_ptr<BaseContext> &context,
    const std::pair<std::shared_ptr<ompl::base::Planner>, PLANNER_TYPE> plannerPair) :
    context_(context),
    planner_(plannerPair.first),
    plannerType_(plannerPair.second),
    config_(config),
    dataThreadPromise_(),
    dataThreadStopSignal_(dataThreadPromise_.get_future()) {
  dataThread_ = std::thread(&BaseVisualizer::createData, this);
}

BaseVisualizer::~BaseVisualizer() {
  dataThreadPromise_.set_value();
  dataThread_.join();
}

void BaseVisualizer::setContext(const std::shared_ptr<BaseContext> &context) {
  // Setting a new context means all the data is invalid.
  displayIteration_ = 0u;

  // Stop iterating.
  dataThreadPromise_.set_value();
  dataThread_.join();

  // Set the new context.
  context_ = context;

  // Reset the promise and stop signal.
  dataThreadPromise_ = std::promise<void>();
  dataThreadStopSignal_ = dataThreadPromise_.get_future();

  // Reset all data.
  plannerData_.clear();
  durations_.clear();
  largestIteration_ = 0u;
  setupDuration_ = time::Duration(0.0);

  // Create data.
  dataThread_ = std::thread(&BaseVisualizer::createData, this);
}

void BaseVisualizer::setPlanner(
    const std::pair<std::shared_ptr<ompl::base::Planner>, PLANNER_TYPE> &plannerPair) {
  // Setting a new context means all the data is invalid.
  displayIteration_ = 0u;

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

std::shared_ptr<const PlannerSpecificData> BaseVisualizer::getPlannerSpecificData(
    std::size_t iteration) const {
  std::scoped_lock lock(plannerSpecificDataMutex_);
  if (iteration >= plannerSpecificData_.size()) {
    std::cout << "Requested iteration: " << iteration
              << ", available: " << plannerSpecificData_.size() << '\n';
    throw std::runtime_error(
        "Requested planner specific data of iteration that has not yet been processed");
  }
  return plannerSpecificData_.at(iteration);
}

const ompl::base::PathPtr BaseVisualizer::getSolutionPath(std::size_t iteration) const {
  std::scoped_lock lock(solutionPathsMutex_);
  if (iteration >= solutionPaths_.size()) {
    std::cout << "Requested iteration: " << iteration << ", available: " << plannerData_.size()
              << '\n';
    throw std::runtime_error("Requested planner data of iteration that has not yet been processed");
  }
  return solutionPaths_.at(iteration);
}

ompl::base::Cost BaseVisualizer::getSolutionCost(std::size_t iteration) const {
  std::scoped_lock lock(solutionCostsMutex_);
  if (iteration >= solutionCosts_.size()) {
    std::cout << "Requested iteration: " << iteration << ", available: " << plannerData_.size()
              << '\n';
    throw std::runtime_error("Requested planner data of iteration that has not yet been processed");
  }
  return solutionCosts_.at(iteration);
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

    if (config_->contains("Experiment/seed")) {
      if (plannerType_ == esp::ompltools::PLANNER_TYPE::BITSTAR) {
        planner_->as<ompl::geometric::BITstar>()->setLocalSeed(
            config_->get<std::size_t>("Experiment/seed"));
      } else if (plannerType_ == esp::ompltools::PLANNER_TYPE::TBDSTAR) {
        // planner_->as<ompl::geometric::TBDstar>()->setLocalSeed(
        // config_->get<std::size_t>("Experiment/seed"));
      }
    } else {
      throw std::runtime_error("Unknown seed.");
    }
  }

  while (dataThreadStopSignal_.wait_for(std::chrono::nanoseconds(1)) ==
         std::future_status::timeout) {
    // Create a new iteration if we we're viewing one thats uncomfortably close.
    if (displayIteration_ + iterationBuffer_ > largestIteration_) {
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

      {  // Store the solution path.
        std::scoped_lock lock(solutionPathsMutex_);
        if (planner_->getProblemDefinition()->hasExactSolution()) {
          solutionPaths_.emplace_back(planner_->getProblemDefinition()->getSolutionPath());
        } else {
          solutionPaths_.emplace_back(nullptr);
        }
      }

      {  // Store the solution cost.
        std::scoped_lock lock(solutionCostsMutex_);
        if (planner_->getProblemDefinition()->hasExactSolution()) {
          solutionCosts_.emplace_back(planner_->getProblemDefinition()->getSolutionPath()->cost(
              context_->getOptimizationObjective()));
        } else {
          solutionCosts_.emplace_back(std::numeric_limits<double>::infinity());
        }
      }

      // Store the planner specific data.
      switch (plannerType_) {
        case PLANNER_TYPE::BITSTAR:
        case PLANNER_TYPE::SBITSTAR: {
          auto bitstarData = std::make_shared<BITstarData>(context_->getSpaceInformation());

          // Store the BIT* edge queue.
          std::vector<BITstarData::BITstarEdge> edgeQueue;
          planner_->as<ompl::geometric::BITstar>()->getEdgeQueue(&edgeQueue);
          bitstarData->setEdgeQueue(edgeQueue);

          // Store the BIT* next edge.
          bitstarData->setNextEdge(planner_->as<ompl::geometric::BITstar>()->getNextEdgeInQueue());

          // Store the BIT* next edge queue value.
          bitstarData->setNextEdgeValueInQueue(
              planner_->as<ompl::geometric::BITstar>()->getNextEdgeValueInQueue());

          // Store the data.
          std::scoped_lock lock(plannerSpecificDataMutex_);
          plannerSpecificData_.emplace_back(bitstarData);
          break;
        }
        case PLANNER_TYPE::TBDSTAR: {
          auto tbdstarData = std::make_shared<TBDstarData>(context_->getSpaceInformation());

          // Store the TBD* forward queue.
          tbdstarData->setForwardQueue(planner_->as<ompl::geometric::TBDstar>()->getEdgesInQueue());

          // Store the TBD* backward queue.
          tbdstarData->setBackwardQueue(
              planner_->as<ompl::geometric::TBDstar>()->getVerticesInQueue());

          // Store the next edge.
          const auto &edge = planner_->as<ompl::geometric::TBDstar>()->getNextEdgeInQueue();
          if (edge.getParent() && edge.getChild()) {
            tbdstarData->setNextEdge(
                std::make_pair(edge.getParent()->getState(), edge.getChild()->getState()));
          }

          // Store the next vertex.
          tbdstarData->setNextVertex(
              planner_->as<ompl::geometric::TBDstar>()->getNextVertexInQueue());

          // Store the backward search tree.
          tbdstarData->setVerticesInBackwardSearchTree(
              planner_->as<ompl::geometric::TBDstar>()->getVerticesInBackwardSearchTree());

          // Store the data.
          std::scoped_lock lock(plannerSpecificDataMutex_);
          plannerSpecificData_.emplace_back(tbdstarData);
          break;
        }
        case PLANNER_TYPE::AIBITSTAR: {
          auto aibitstarData = std::make_shared<AIBITstarData>(context_->getSpaceInformation());

          // Store the AIBIT* reverse tree.
          aibitstarData->setReverseTree(
              planner_->as<ompl::geometric::AIBITstar>()->getReverseTree());

          // Store the AIBIT* forward queue.
          aibitstarData->setForwardQueue(
              planner_->as<ompl::geometric::AIBITstar>()->getForwardQueue());

          // Store the AIBIT* reverse queue.
          aibitstarData->setReverseQueue(
              planner_->as<ompl::geometric::AIBITstar>()->getReverseQueue());

          // Store the next forward edge.
          try {
            aibitstarData->setNextForwardEdge(
                planner_->as<ompl::geometric::AIBITstar>()->getNextForwardEdge());
          } catch (const std::out_of_range &e) {
            // Throws if there is no forward edge. This is fine, the edge is default constructed.
          }

          // Store the next reverse edge.
          try {
            aibitstarData->setNextReverseEdge(
                planner_->as<ompl::geometric::AIBITstar>()->getNextReverseEdge());
          } catch (const std::out_of_range &e) {
            // Throws if there is no forward edge. This is fine, the edge is default constructed.
          }

          // Store the data.
          std::scoped_lock lock(plannerSpecificDataMutex_);
          plannerSpecificData_.emplace_back(aibitstarData);
          break;
        }
        default:
          // Defaults to not getting any data.
          break;
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
