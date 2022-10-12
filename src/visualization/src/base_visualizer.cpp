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

#include "pdt/visualization/base_visualizer.h"

#include <chrono>
#include <exception>

#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>

#include "pdt/time/time.h"
#include "pdt/utilities/set_local_seed.h"

namespace pdt {

namespace visualization {

BaseVisualizer::BaseVisualizer(
    const std::shared_ptr<config::Configuration> &config,
    const std::shared_ptr<planning_contexts::BaseContext> &context,
    const std::pair<std::shared_ptr<ompl::base::Planner>, common::PLANNER_TYPE> plannerPair) :
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

void BaseVisualizer::setContext(
    const std::shared_ptr<planning_contexts::RealVectorGeometricContext> &context) {
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
    const std::pair<std::shared_ptr<ompl::base::Planner>, common::PLANNER_TYPE> &plannerPair) {
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
    const std::size_t iteration) const {
  std::scoped_lock lock(plannerDataMutex_);
  if (iteration >= plannerData_.size()) {
    std::cout << "Requested iteration: " << iteration << ", available: " << plannerData_.size()
              << '\n';
    throw std::runtime_error("Requested planner data of iteration that has not yet been processed");
  }
  return plannerData_.at(iteration);
}

std::shared_ptr<const PlannerSpecificData> BaseVisualizer::getPlannerSpecificData(
    const std::size_t iteration) const {
  std::scoped_lock lock(plannerSpecificDataMutex_);
  if (iteration >= plannerSpecificData_.size()) {
    std::cout << "Requested iteration: " << iteration
              << ", available: " << plannerSpecificData_.size() << '\n';
    throw std::runtime_error(
        "Requested planner specific data of iteration that has not yet been processed");
  }
  return plannerSpecificData_.at(iteration);
}

const ompl::base::PathPtr BaseVisualizer::getSolutionPath(const std::size_t iteration) const {
  std::scoped_lock lock(solutionPathsMutex_);
  if (iteration >= solutionPaths_.size()) {
    std::cout << "Requested iteration: " << iteration << ", available: " << plannerData_.size()
              << '\n';
    throw std::runtime_error("Requested planner data of iteration that has not yet been processed");
  }
  return solutionPaths_.at(iteration);
}

ompl::base::Cost BaseVisualizer::getSolutionCost(const std::size_t iteration) const {
  std::scoped_lock lock(solutionCostsMutex_);
  if (iteration >= solutionCosts_.size()) {
    std::cout << "Requested iteration: " << iteration << ", available: " << plannerData_.size()
              << '\n';
    throw std::runtime_error("Requested planner data of iteration that has not yet been processed");
  }
  return solutionCosts_.at(iteration);
}

time::Duration BaseVisualizer::getIterationDuration(const std::size_t iteration) const {
  std::scoped_lock lock(durationsMutex_);
  if (iteration >= durations_.size()) {
    throw std::runtime_error(
        "Requested iteration duration of iteration that has not yet been processed");
  }
  return durations_.at(iteration);
}

time::Duration BaseVisualizer::getTotalElapsedDuration(const std::size_t iteration) const {
  std::scoped_lock lock(durationsMutex_, setupDurationMutex_);
  if (iteration >= durations_.size()) {
    throw std::runtime_error(
        "Requested elapsed duration of iteration that has not yet been processed");
  }
  return std::accumulate(durations_.begin(), durations_.begin() + static_cast<long int>(iteration),
                         setupDuration_);
}

std::size_t BaseVisualizer::getQueryNumber(const std::size_t iteration) const {
  std::scoped_lock lock(queryNumbersMutex_);
  if (iteration >= queryNumbers_.size()) {
    throw std::runtime_error(
        "Requested query number of an iteration that has not yet been processed");
  }
  return queryNumbers_.at(iteration);
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

    utilities::setLocalSeed(config_, planner_, plannerType_);
  }

  double timePerQuery = 0.0;
  if (config_->contains("experiment/time")) {
    timePerQuery = config_->get<double>("experiment/time");
  }

  std::size_t queryNumber = 0;

  planner_->clear();

  const auto problemDefinition = context_->instantiateNthProblemDefinition(queryNumber);
  planner_->setProblemDefinition(problemDefinition);

  double currentIterationStartTime = 0.0;

  while (dataThreadStopSignal_.wait_for(std::chrono::nanoseconds(1)) ==
         std::future_status::timeout) {
    // Create a new iteration if we we're viewing one thats uncomfortably close.
    if (displayIteration_ + iterationBuffer_ > largestIteration_) {
      /*
       * If we are not at the last query, there are two cases under which we continue to the next
       * query:
       * - The time per query is smaller than 0, and the planner found a solution
       * - the time per query is larger than 0, the planner found a solution, and the
       *   time used for the current query is larger than the allowed time budget
       *
       * If we arrived at the last query, we run that one indefinitely.
       */
      if (queryNumber + 1 < context_->getNumQueries() &&
          ((planner_->getProblemDefinition()->hasExactSolution() && timePerQuery <= 0.0) ||
           (largestIteration_ > 0 && timePerQuery > 0.0 &&
            getTotalElapsedDuration(largestIteration_).count() - currentIterationStartTime >
                timePerQuery))) {
        planner_->clearQuery();
        ++queryNumber;

        const auto problemDefinition = context_->instantiateNthProblemDefinition(queryNumber);
        planner_->setProblemDefinition(problemDefinition);

        currentIterationStartTime = getTotalElapsedDuration(largestIteration_).count();
      }

      // Create a termination condition that stops the planner after one iteration.
      ompl::base::IterationTerminationCondition terminationCondition(1u);

      // Advance one iteration.
      auto iterStartTime = time::Clock::now();
      planner_->solve(terminationCondition);
      auto iterationDuration = time::Clock::now() - iterStartTime;

      // Get the planner data.
      auto plannerData = std::make_shared<ompl::base::PlannerData>(context_->getSpaceInformation());
      planner_->getPlannerData(*plannerData);

      {  // Store the duration.
        std::scoped_lock lock(durationsMutex_);
        durations_.push_back(iterationDuration);
      }

      {  // Store the query number.
        std::scoped_lock lock(queryNumbersMutex_);
        queryNumbers_.push_back(queryNumber);
      }

      {  // Store the solution path.
        std::scoped_lock lock(solutionPathsMutex_);
        if (planner_->getProblemDefinition()->hasExactSolution()) {
          solutionPaths_.push_back(planner_->getProblemDefinition()->getSolutionPath());
        } else {
          solutionPaths_.push_back(nullptr);
        }
      }

      {  // Store the solution cost.
        std::scoped_lock lock(solutionCostsMutex_);
        if (planner_->getProblemDefinition()->hasExactSolution()) {
          solutionCosts_.push_back(
              planner_->getProblemDefinition()->getSolutionPath()->cost(context_->getObjective()));
        } else {
          solutionCosts_.push_back(context_->getObjective()->infiniteCost());
        }
      }

      // Store the planner specific data.
      switch (plannerType_) {
        case common::PLANNER_TYPE::BITSTAR:
        case common::PLANNER_TYPE::ABITSTAR: {
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
          plannerSpecificData_.push_back(bitstarData);
          break;
        }
        case common::PLANNER_TYPE::AITSTAR: {
          auto aitstarData = std::make_shared<AITstarData>(context_->getSpaceInformation());

          // Store the TBD* forward queue.
          aitstarData->setForwardQueue(planner_->as<ompl::geometric::AITstar>()->getEdgesInQueue());

          // Store the TBD* backward queue.
          aitstarData->setBackwardQueue(
              planner_->as<ompl::geometric::AITstar>()->getVerticesInQueue());

          // Store the next edge.
          const auto &edge = planner_->as<ompl::geometric::AITstar>()->getNextEdgeInQueue();
          if (edge.getParent() && edge.getChild()) {
            aitstarData->setNextEdge(
                std::make_pair(edge.getParent()->getState(), edge.getChild()->getState()));
          }

          // Store the next vertex.
          aitstarData->setNextVertex(
              planner_->as<ompl::geometric::AITstar>()->getNextVertexInQueue());

          // Store the backward search tree.
          aitstarData->setVerticesInBackwardSearchTree(
              planner_->as<ompl::geometric::AITstar>()->getVerticesInReverseSearchTree());

          // Store the data.
          std::scoped_lock lock(plannerSpecificDataMutex_);
          plannerSpecificData_.push_back(aitstarData);
          break;
        }
#ifdef PDT_EXTRA_EITSTAR_PR
        case common::PLANNER_TYPE::EIRMSTAR:
        case common::PLANNER_TYPE::EITSTAR: {
          auto eitstarData = std::make_shared<EITstarData>(context_->getSpaceInformation());

          // Store the EIT* reverse tree.
          eitstarData->setReverseTree(planner_->as<ompl::geometric::EITstar>()->getReverseTree());

          // Store the EIT* forward queue.
          eitstarData->setForwardQueue(planner_->as<ompl::geometric::EITstar>()->getForwardQueue());

          // Store the EIT* reverse queue.
          eitstarData->setReverseQueue(planner_->as<ompl::geometric::EITstar>()->getReverseQueue());

          // Store the next forward edge.
          if (!planner_->as<ompl::geometric::EITstar>()->isForwardQueueEmpty()) {
            eitstarData->setNextForwardEdge(
                planner_->as<ompl::geometric::EITstar>()->getNextForwardEdge());
          }
          // No else, the edge is default constructed.

          // Store the next reverse edge.
          if (!planner_->as<ompl::geometric::EITstar>()->isReverseQueueEmpty()) {
            eitstarData->setNextReverseEdge(
                planner_->as<ompl::geometric::EITstar>()->getNextReverseEdge());
          }
          // No else, the edge is default constructed.

          // Store the data.
          std::scoped_lock lock(plannerSpecificDataMutex_);
          plannerSpecificData_.push_back(eitstarData);
          break;
        }
        case common::PLANNER_TYPE::LAZYPRMSTAR: {
          auto lPRMstarData = std::make_shared<LazyPRMstarData>(context_->getSpaceInformation());

          lPRMstarData->setValidEdges(
              planner_->as<ompl::geometric::LazyPRMstar>()->getValidEdges());

          // Store the data.
          std::scoped_lock lock(plannerSpecificDataMutex_);
          plannerSpecificData_.push_back(lPRMstarData);
          break;
        }
#endif  // #ifdef PDT_EXTRA_EITSTAR_PR
        default:
          // Defaults to not getting any data.
          break;
      }

      {  // Store the planner data.
        std::scoped_lock lock(plannerDataMutex_);
        plannerData->decoupleFromPlanner();
        plannerData_.push_back(plannerData);
        largestIteration_ = plannerData_.size() - 1u;
      }
    }
  }
}

}  // namespace visualization

}  // namespace pdt
