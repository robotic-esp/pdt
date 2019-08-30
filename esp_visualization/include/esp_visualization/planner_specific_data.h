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

#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/tbdstar/TBDstar.h>
#include <ompl/geometric/planners/tbdstar/datastructures/Edge.h>

#include "esp_common/planner_type.h"

namespace esp {

namespace ompltools {

class PlannerSpecificData {
 public:
  explicit PlannerSpecificData(const ompl::base::SpaceInformationPtr& spaceInfo) :
      spaceInfo_(spaceInfo) {}
  virtual ~PlannerSpecificData() = default;

 protected:
  ompl::base::SpaceInformationPtr spaceInfo_{};
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

class TBDstarData : public PlannerSpecificData {
 public:
  TBDstarData(const ompl::base::SpaceInformationPtr& spaceInfo) :
      PlannerSpecificData(spaceInfo),
      parentStateNextEdge_(spaceInfo->allocState()),
      childStateNextEdge_(spaceInfo->allocState()) {}
  ~TBDstarData() {
    spaceInfo_->freeState(parentStateNextEdge_);
    spaceInfo_->freeState(childStateNextEdge_);
  }

  // Getters.
  std::vector<ompl::geometric::tbdstar::Edge> getForwardQueue() const;
  std::pair<const ompl::base::State*, const ompl::base::State*> getNextEdge() const;
  std::vector<std::shared_ptr<ompl::geometric::tbdstar::Vertex>> getBackwardQueue() const;
  std::shared_ptr<ompl::geometric::tbdstar::Vertex> getNextVertex() const;
  std::vector<std::shared_ptr<ompl::geometric::tbdstar::Vertex>> getVerticesInBackwardSearchTree() const;

  // Setters.
  void setForwardQueue(const std::vector<ompl::geometric::tbdstar::Edge>& edges);
  void setNextEdge(const std::pair<const ompl::base::State*, const ompl::base::State*>& edge);
  void setBackwardQueue(const std::vector<std::shared_ptr<ompl::geometric::tbdstar::Vertex>>& vertices);
  void setNextVertex(const std::shared_ptr<ompl::geometric::tbdstar::Vertex>& vertex);
  void setVerticesInBackwardSearchTree(const std::vector<std::shared_ptr<ompl::geometric::tbdstar::Vertex>>& vertices);

 private:
  std::vector<ompl::geometric::tbdstar::Edge> forwardQueue_{};
  std::vector<std::shared_ptr<ompl::geometric::tbdstar::Vertex>> backwardQueue_{};
  std::vector<std::shared_ptr<ompl::geometric::tbdstar::Vertex>> verticesInBackwardSearchTree_{};
  ompl::base::State* parentStateNextEdge_{};
  ompl::base::State* childStateNextEdge_{};
  std::shared_ptr<ompl::geometric::tbdstar::Vertex> nextVertex_{};
};

}  // namespace ompltools

}  // namespace esp
