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

#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/aitstar/Edge.h>

#ifdef PDT_EXTRA_EITSTAR_PR
#include <ompl/geometric/planners/informedtrees/EITstar.h>
#include <ompl/geometric/planners/informedtrees/eitstar/Edge.h>
#endif  // #ifdef PDT_EXTRA_EITSTAR_PR

#include "pdt/common/planner_type.h"

namespace pdt {

namespace visualization {

class PlannerSpecificData {
 public:
  explicit PlannerSpecificData(const ompl::base::SpaceInformationPtr& spaceInfo) :
      spaceInfo_(spaceInfo) {}
  virtual ~PlannerSpecificData() = default;

 protected:
  ompl::base::SpaceInformationPtr spaceInfo_{};
};

class LazyPRMstarData : public PlannerSpecificData {
 public:
  LazyPRMstarData(const ompl::base::SpaceInformationPtr& spaceInfo) :
      PlannerSpecificData(spaceInfo) {}

  using Edge = std::pair<ompl::base::PlannerDataVertex, ompl::base::PlannerDataVertex>;

  // getter
  std::vector<Edge> getValidEdges() const;
  std::vector<std::size_t> getNewEdgeIndices() const;

  // setter
  void setValidEdges(const std::vector<Edge>& edges);
  void setNewEdgeIndices(const std::vector<std::size_t>& newEdges);

 private:
  std::vector<Edge> validEdges_{};
  std::vector<std::size_t> newEdgeIndices_{};
};

class BITstarData : public PlannerSpecificData {
 public:
  using BITstarEdge = std::pair<std::shared_ptr<const ompl::geometric::BITstar::Vertex>,
                                std::shared_ptr<const ompl::geometric::BITstar::Vertex>>;
  BITstarData(const ompl::base::SpaceInformationPtr& spaceInfo) :
      PlannerSpecificData(spaceInfo),
      parentStateNextEdge_(spaceInfo->allocState()),
      childStateNextEdge_(spaceInfo->allocState()) {}
  ~BITstarData() {
    spaceInfo_->freeState(parentStateNextEdge_);
    spaceInfo_->freeState(childStateNextEdge_);
  }

  // Getters.
  std::vector<BITstarEdge> getEdgeQueue() const;
  std::pair<const ompl::base::State*, const ompl::base::State*> getNextEdge() const;
  ompl::base::Cost getNextEdgeValueInQueue() const;

  // Setters.
  void setEdgeQueue(const std::vector<BITstarEdge>& edges);
  void setNextEdge(const std::pair<const ompl::base::State*, const ompl::base::State*>& edge);
  void setNextEdgeValueInQueue(const ompl::base::Cost& cost);

 private:
  std::vector<BITstarEdge> edgeQueue_{};
  ompl::base::State* parentStateNextEdge_{};
  ompl::base::State* childStateNextEdge_{};
  ompl::base::Cost nextEdgeValueInQueue_{std::numeric_limits<double>::signaling_NaN()};
};

class AITstarData : public PlannerSpecificData {
 public:
  AITstarData(const ompl::base::SpaceInformationPtr& spaceInfo) :
      PlannerSpecificData(spaceInfo),
      parentStateNextEdge_(spaceInfo->allocState()),
      childStateNextEdge_(spaceInfo->allocState()) {}
  ~AITstarData() {
    spaceInfo_->freeState(parentStateNextEdge_);
    spaceInfo_->freeState(childStateNextEdge_);
  }

  // Getters.
  std::vector<ompl::geometric::aitstar::Edge> getForwardQueue() const;
  std::pair<const ompl::base::State*, const ompl::base::State*> getNextEdge() const;
  std::vector<std::shared_ptr<ompl::geometric::aitstar::Vertex>> getBackwardQueue() const;
  std::shared_ptr<ompl::geometric::aitstar::Vertex> getNextVertex() const;
  std::vector<std::shared_ptr<ompl::geometric::aitstar::Vertex>> getVerticesInBackwardSearchTree()
      const;

  // Setters.
  void setForwardQueue(const std::vector<ompl::geometric::aitstar::Edge>& edges);
  void setNextEdge(const std::pair<const ompl::base::State*, const ompl::base::State*>& edge);
  void setBackwardQueue(
      const std::vector<std::shared_ptr<ompl::geometric::aitstar::Vertex>>& vertices);
  void setNextVertex(const std::shared_ptr<ompl::geometric::aitstar::Vertex>& vertex);
  void setVerticesInBackwardSearchTree(
      const std::vector<std::shared_ptr<ompl::geometric::aitstar::Vertex>>& vertices);

 private:
  std::vector<ompl::geometric::aitstar::Edge> forwardQueue_{};
  std::vector<std::shared_ptr<ompl::geometric::aitstar::Vertex>> backwardQueue_{};
  std::vector<std::shared_ptr<ompl::geometric::aitstar::Vertex>> verticesInBackwardSearchTree_{};
  ompl::base::State* parentStateNextEdge_{};
  ompl::base::State* childStateNextEdge_{};
  std::shared_ptr<ompl::geometric::aitstar::Vertex> nextVertex_{};
};

#ifdef PDT_EXTRA_EITSTAR_PR
class EITstarData : public PlannerSpecificData {
 public:
  EITstarData(const ompl::base::SpaceInformationPtr& spaceInfo) : PlannerSpecificData(spaceInfo) {}
  // Getters
  std::vector<ompl::geometric::eitstar::Edge> getReverseTree() const;
  std::vector<ompl::geometric::eitstar::Edge> getForwardQueue() const;
  std::vector<ompl::geometric::eitstar::Edge> getReverseQueue() const;
  ompl::geometric::eitstar::Edge getNextForwardEdge() const;
  ompl::geometric::eitstar::Edge getNextReverseEdge() const;

  // Setters
  void setReverseTree(const std::vector<ompl::geometric::eitstar::Edge>& tree);
  void setForwardQueue(const std::vector<ompl::geometric::eitstar::Edge>& queue);
  void setReverseQueue(const std::vector<ompl::geometric::eitstar::Edge>& queue);
  void setNextForwardEdge(const ompl::geometric::eitstar::Edge& edge);
  void setNextReverseEdge(const ompl::geometric::eitstar::Edge& edge);

 private:
  std::vector<ompl::geometric::eitstar::Edge> reverseTree_{};
  std::vector<ompl::geometric::eitstar::Edge> forwardQueue_{};
  std::vector<ompl::geometric::eitstar::Edge> reverseQueue_{};
  ompl::geometric::eitstar::Edge nextForwardEdge_{};
  ompl::geometric::eitstar::Edge nextReverseEdge_{};
};
#endif  // #ifdef PDT_EXTRA_EITSTAR_PR

}  // namespace visualization

}  // namespace pdt
