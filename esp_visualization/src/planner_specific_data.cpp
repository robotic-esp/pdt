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

#include "esp_visualization/planner_specific_data.h"

namespace esp {

namespace ompltools {

std::vector<BITstarData::BITstarEdge> BITstarData::getEdgeQueue() const {
  return edgeQueue_;
}

std::pair<const ompl::base::State*, const ompl::base::State*> BITstarData::getNextEdge() const {
  return std::make_pair(parentStateNextEdge_, childStateNextEdge_);
}

ompl::base::Cost BITstarData::getNextEdgeValueInQueue() const {
  return nextEdgeValueInQueue_;
}

void BITstarData::setEdgeQueue(const std::vector<BITstarEdge>& edges) {
  edgeQueue_ = edges;
}

void BITstarData::setNextEdge(
    const std::pair<const ompl::base::State*, const ompl::base::State*>& edge) {
  if (edge.first == nullptr || edge.second == nullptr) {
    return;
  }
  spaceInfo_->copyState(parentStateNextEdge_, edge.first);
  spaceInfo_->copyState(childStateNextEdge_, edge.second);
}

void BITstarData::setNextEdgeValueInQueue(const ompl::base::Cost& cost) {
  nextEdgeValueInQueue_ = cost;
}

std::vector<ompl::geometric::tbdstar::Edge> TBDstarData::getForwardQueue() const {
  return forwardQueue_;
}

std::vector<std::shared_ptr<ompl::geometric::tbdstar::Vertex>> TBDstarData::getBackwardQueue()
    const {
  return backwardQueue_;
}

std::shared_ptr<ompl::geometric::tbdstar::Vertex> TBDstarData::getNextVertex() const {
  return nextVertex_;
}

std::vector<std::shared_ptr<ompl::geometric::tbdstar::Vertex>>
TBDstarData::getVerticesInBackwardSearchTree() const {
  return verticesInBackwardSearchTree_;
}

std::pair<const ompl::base::State*, const ompl::base::State*> TBDstarData::getNextEdge() const {
  return std::make_pair(parentStateNextEdge_, childStateNextEdge_);
}

void TBDstarData::setNextEdge(
    const std::pair<const ompl::base::State*, const ompl::base::State*>& edge) {
  if (edge.first == nullptr || edge.second == nullptr) {
    return;
  }
  spaceInfo_->copyState(parentStateNextEdge_, edge.first);
  spaceInfo_->copyState(childStateNextEdge_, edge.second);
}

void TBDstarData::setForwardQueue(const std::vector<ompl::geometric::tbdstar::Edge>& queue) {
  forwardQueue_ = queue;
}

void TBDstarData::setBackwardQueue(
    const std::vector<std::shared_ptr<ompl::geometric::tbdstar::Vertex>>& queue) {
  backwardQueue_ = queue;
}

void TBDstarData::setNextVertex(const std::shared_ptr<ompl::geometric::tbdstar::Vertex>& vertex) {
  nextVertex_ = vertex;
}

void TBDstarData::setVerticesInBackwardSearchTree(
    const std::vector<std::shared_ptr<ompl::geometric::tbdstar::Vertex>>& vertices) {
  // We need to create a deep copy here.
  verticesInBackwardSearchTree_.reserve(vertices.size());
  for (const auto& vertex : vertices) {
    verticesInBackwardSearchTree_.emplace_back(
        std::make_shared<ompl::geometric::tbdstar::Vertex>(*vertex));
  }
}

std::vector<ompl::geometric::aibitstar::Edge> AIBITstarData::getReverseTree() const {
  return reverseTree_;
}

std::vector<ompl::geometric::aibitstar::Edge> AIBITstarData::getForwardQueue() const {
  return forwardQueue_;
}

std::vector<ompl::geometric::aibitstar::Edge> AIBITstarData::getReverseQueue() const {
  return reverseQueue_;
}

ompl::geometric::aibitstar::Edge AIBITstarData::getNextForwardEdge() const {
  return nextForwardEdge_;
}

ompl::geometric::aibitstar::Edge AIBITstarData::getNextReverseEdge() const {
  return nextReverseEdge_;
}

void AIBITstarData::setReverseTree(const std::vector<ompl::geometric::aibitstar::Edge>& tree) {
  reverseTree_ = tree;
}

void AIBITstarData::setForwardQueue(const std::vector<ompl::geometric::aibitstar::Edge>& queue) {
  forwardQueue_ = queue;
}

void AIBITstarData::setReverseQueue(const std::vector<ompl::geometric::aibitstar::Edge>& queue) {
  reverseQueue_ = queue;
}

void AIBITstarData::setNextForwardEdge(const ompl::geometric::aibitstar::Edge& edge) {
  nextForwardEdge_ = edge;
}

void AIBITstarData::setNextReverseEdge(const ompl::geometric::aibitstar::Edge& edge) {
  nextReverseEdge_ = edge;
}

}  // namespace ompltools

}  // namespace esp
