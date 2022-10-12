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

#include <experimental/filesystem>

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerData.h>

#include "pdt/common/context_type.h"
#include "pdt/common/planner_type.h"
#include "pdt/config/configuration.h"
#include "pdt/obstacles/obstacle_visitor.h"
#include "pdt/pgftikz/tikz_picture.h"
#include "pdt/planning_contexts/all_contexts.h"
#include "pdt/planning_contexts/context_visitor.h"
#include "pdt/visualization/planner_specific_data.h"

namespace pdt {

namespace visualization {

class TikzVisualizer : public planning_contexts::ContextVisitor, public obstacles::ObstacleVisitor {
 public:
  TikzVisualizer(
      const std::shared_ptr<const config::Configuration>& config,
      const std::shared_ptr<planning_contexts::BaseContext>& context,
      const std::pair<std::shared_ptr<ompl::base::Planner>, common::PLANNER_TYPE>& plannerPair);
  ~TikzVisualizer() = default;

  void render(const ompl::base::PlannerData& plannerData, const std::size_t iteration,
              const std::size_t queryNumber, const ompl::base::PathPtr path,
              const std::shared_ptr<const PlannerSpecificData>& plannerSpecificData,
              const double iterationTime, const double totalTime, const double solutionCost);

 private:
  // Compile a picture to a standalone document.
  std::experimental::filesystem::path compile(const std::experimental::filesystem::path& texPath,
                                              const double cost, const double time,
                                              const std::size_t queryNumber);

  // Log to the frame times file.
  void logToFrameTimes(const std::experimental::filesystem::path& pngPath, double iterationTime);

  // The configuration.
  const std::shared_ptr<const config::Configuration> config_;

  // Implement visualizations of contexts.
  void visit(const planning_contexts::CenterSquare& context) const override;
  void visit(const planning_contexts::DividingWalls& context) const override;
  void visit(const planning_contexts::DoubleEnclosure& context) const override;
  void visit(const planning_contexts::FlankingGap& context) const override;
  void visit(const planning_contexts::FourRooms& context) const override;
  void visit(const planning_contexts::GoalEnclosure& context) const override;
  void visit(const planning_contexts::NarrowPassage& context) const override;
  void visit(const planning_contexts::ObstacleFree& context) const override;
  void visit(const planning_contexts::RandomRectangles& context) const override;
  void visit(const planning_contexts::RandomRectanglesMultiStartGoal& context) const override;
  void visit(const planning_contexts::ReedsSheppRandomRectangles& context) const override;
  void visit(const planning_contexts::RepeatingRectangles& context) const override;
  void visit(const planning_contexts::StartEnclosure& context) const override;
  void visit(const planning_contexts::WallGap& context) const override;

  // Implement visualizations of obstacles.
  void visit(const obstacles::Hyperrectangle<obstacles::BaseObstacle>& obstacle) const override;
  void visit(
      const obstacles::Hyperrectangle<obstacles::BaseAntiObstacle>& antiObstacle) const override;

  // Helper functions.
  void drawBoundary(const planning_contexts::RealVectorGeometricContext& context) const;
  void drawBoundary(const planning_contexts::ReedsSheppRandomRectangles& context) const;
  void drawGoal(const std::shared_ptr<ompl::base::Goal>& context) const;
  void drawStartVertex(const ompl::base::PlannerDataVertex& vertex) const;
  void drawStartState(const ompl::base::ScopedState<ompl::base::RealVectorStateSpace>& state) const;
  void drawStartStates(const std::vector<ompl::base::ScopedState<>>& states) const;
  void drawGoalVertex(const ompl::base::PlannerDataVertex& vertex) const;
  void drawGoalState(const ompl::base::ScopedState<ompl::base::RealVectorStateSpace>& state) const;
  void drawGoalStates(
      const std::vector<ompl::base::ScopedState<ompl::base::RealVectorStateSpace>>& states) const;
  void drawVertex(const ompl::base::PlannerDataVertex& vertex) const;
  void drawVertex(const ompl::base::RealVectorStateSpace::StateType* state,
                  const std::string& options) const;
  void drawEdge(const ompl::base::PlannerDataVertex& parent,
                const ompl::base::PlannerDataVertex& child, std::size_t zLevel,
                const std::string& options) const;
  void drawEdge(const ompl::base::RealVectorStateSpace::StateType* parent,
                const ompl::base::RealVectorStateSpace::StateType* child, std::size_t zLevel,
                const std::string& options) const;
  void drawSolution(const ompl::base::PathPtr path) const;
  void drawEllipse(double cost) const;
  void drawRectangle(double midX, double midY, double widthX, double widthY, std::size_t zLevel,
                     const std::string& options) const;
  void drawPlannerSpecificVisualizations(
      const std::shared_ptr<const PlannerSpecificData>& plannerSpecificData) const;
  void drawBITstarSpecificVisualizations(
      const std::shared_ptr<const BITstarData>& bitstarData) const;
  void drawAITstarSpecificVisualizations(
      const std::shared_ptr<const AITstarData>& aitstarData) const;
#ifdef PDT_EXTRA_EITSTAR_PR
  void drawEITstarSpecificVisualizations(
      const std::shared_ptr<const EITstarData>& aitstarData) const;
#endif  // #ifdef PDT_EXTRA_EITSTAR_PR
  void drawLazyPRMstarSpecificVisualizations(
      const std::shared_ptr<const LazyPRMstarData>& lprmstarData) const;

  // Planner and context to be visualized.
  std::shared_ptr<planning_contexts::BaseContext> context_;
  common::PLANNER_TYPE plannerType_{common::PLANNER_TYPE::INVALID};
  std::string name_{"invalid planner"};

  // The tikz picture holding the visualization.
  mutable pgftikz::TikzPicture picture_;

  // The frame times file.
  std::ofstream frameTimes_;
};

}  // namespace visualization

}  // namespace pdt
