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

#include "pdt/visualization/planner_specific_data.h"

namespace pdt {

namespace visualization {

std::vector<LazyPRMstarData::Edge> LazyPRMstarData::getValidEdges() const {
  return validEdges_;
}

void LazyPRMstarData::setValidEdges(const std::vector<LazyPRMstarData::Edge>& edges) {
  validEdges_ = edges;
}

std::vector<std::size_t> LazyPRMstarData::getNewEdgeIndices() const {
  return newEdgeIndices_;
}

void LazyPRMstarData::setNewEdgeIndices(const std::vector<std::size_t>& newEdgeIndices) {
  newEdgeIndices_ = newEdgeIndices;
}

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

std::vector<ompl::geometric::aitstar::Edge> AITstarData::getForwardQueue() const {
  return forwardQueue_;
}

std::vector<std::shared_ptr<ompl::geometric::aitstar::Vertex>> AITstarData::getBackwardQueue()
    const {
  return backwardQueue_;
}

std::shared_ptr<ompl::geometric::aitstar::Vertex> AITstarData::getNextVertex() const {
  return nextVertex_;
}

std::vector<std::shared_ptr<ompl::geometric::aitstar::Vertex>>
AITstarData::getVerticesInBackwardSearchTree() const {
  return verticesInBackwardSearchTree_;
}

std::pair<const ompl::base::State*, const ompl::base::State*> AITstarData::getNextEdge() const {
  return std::make_pair(parentStateNextEdge_, childStateNextEdge_);
}

void AITstarData::setNextEdge(
    const std::pair<const ompl::base::State*, const ompl::base::State*>& edge) {
  if (edge.first == nullptr || edge.second == nullptr) {
    return;
  }
  spaceInfo_->copyState(parentStateNextEdge_, edge.first);
  spaceInfo_->copyState(childStateNextEdge_, edge.second);
}

void AITstarData::setForwardQueue(const std::vector<ompl::geometric::aitstar::Edge>& queue) {
  forwardQueue_ = queue;
}

void AITstarData::setBackwardQueue(
    const std::vector<std::shared_ptr<ompl::geometric::aitstar::Vertex>>& queue) {
  backwardQueue_ = queue;
}

void AITstarData::setNextVertex(const std::shared_ptr<ompl::geometric::aitstar::Vertex>& vertex) {
  nextVertex_ = vertex;
}

void AITstarData::setVerticesInBackwardSearchTree(
    const std::vector<std::shared_ptr<ompl::geometric::aitstar::Vertex>>& vertices) {
  // We need to create a deep copy here.
  verticesInBackwardSearchTree_.reserve(vertices.size());
  for (const auto& vertex : vertices) {
    verticesInBackwardSearchTree_.push_back(
        std::make_shared<ompl::geometric::aitstar::Vertex>(vertex));
  }
}

#ifdef PDT_EXTRA_EITSTAR_PR
std::vector<ompl::geometric::eitstar::Edge> EITstarData::getReverseTree() const {
  return reverseTree_;
}

std::vector<ompl::geometric::eitstar::Edge> EITstarData::getForwardQueue() const {
  return forwardQueue_;
}

std::vector<ompl::geometric::eitstar::Edge> EITstarData::getReverseQueue() const {
  return reverseQueue_;
}

ompl::geometric::eitstar::Edge EITstarData::getNextForwardEdge() const {
  return nextForwardEdge_;
}

ompl::geometric::eitstar::Edge EITstarData::getNextReverseEdge() const {
  return nextReverseEdge_;
}

void EITstarData::setReverseTree(const std::vector<ompl::geometric::eitstar::Edge>& tree) {
  reverseTree_ = tree;
}

void EITstarData::setForwardQueue(const std::vector<ompl::geometric::eitstar::Edge>& queue) {
  forwardQueue_ = queue;
}

void EITstarData::setReverseQueue(const std::vector<ompl::geometric::eitstar::Edge>& queue) {
  reverseQueue_ = queue;
}

void EITstarData::setNextForwardEdge(const ompl::geometric::eitstar::Edge& edge) {
  nextForwardEdge_ = edge;
}

void EITstarData::setNextReverseEdge(const ompl::geometric::eitstar::Edge& edge) {
  nextReverseEdge_ = edge;
}
#endif  // #ifdef PDT_EXTRA_EITSTAR_PR

}  // namespace visualization

}  // namespace pdt
