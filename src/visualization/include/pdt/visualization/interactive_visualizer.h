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

#include <pangolin/pangolin.h>

#include "pdt/common/context_type.h"
#include "pdt/common/planner_type.h"
#include "pdt/config/configuration.h"
#include "pdt/objectives/optimization_objective_visitor.h"
#include "pdt/objectives/potential_field_optimization_objective.h"
#include "pdt/objectives/reciprocal_clearance_optimization_objective.h"
#include "pdt/obstacles/obstacle_visitor.h"
#include "pdt/planning_contexts/all_contexts.h"
#include "pdt/planning_contexts/context_visitor.h"
#include "pdt/planning_contexts/real_vector_geometric_context.h"
#include "pdt/visualization/base_visualizer.h"
#include "pdt/visualization/tikz_visualizer.h"

namespace pdt {

namespace visualization {

class InteractiveVisualizer : public BaseVisualizer,
                              public planning_contexts::ContextVisitor,
                              public obstacles::ObstacleVisitor,
                              public objectives::ObjectiveVisitor {
 public:
  InteractiveVisualizer(
      const std::shared_ptr<config::Configuration>& config,
      const std::shared_ptr<planning_contexts::BaseContext>& context,
      const std::pair<std::shared_ptr<ompl::base::Planner>, common::PLANNER_TYPE> plannerPair);
  ~InteractiveVisualizer() = default;

  void run();

 private:
  // General helpers.
  void incrementIteration(std::size_t num = 1u);
  void decrementIteration(std::size_t num = 1u);
  std::size_t lastDisplayIteration_{0u};

  // Play to view and record to view.
  bool playToIteration_{false};
  bool exporting_{false};
  std::size_t iterationToPlayTo_{0u};
  time::Duration desiredDisplayDuration_{};
  time::Duration actualDisplayDuration_{};
  time::Clock::time_point displayStartTime_{};
  std::size_t screencaptureId_{0u};

  // Plotting.
  pangolin::DataLog costLog_{};
  void updateCostLog();
  float minCost_{0.0};
  float maxCost_{0.0};
  float plotBackgroundColor_[4]{0.9f, 0.9f, 0.9f, 1.0f};
  int largestPlottedIteration_{-1};
  void logCost();

  // Highlevel drawing.
  void drawVerticesAndEdges(const std::size_t iteration);
  void drawVertices(const std::size_t iteration);
  void drawEdges(const std::size_t iteration);
  void drawSolution(const std::size_t iteration);
  void drawStateIds(const std::size_t iteration);

  // Planner specific visualizations.
  void drawPlannerSpecificVisualizations(const std::size_t iteration) const;
  void drawBITstarSpecificVisualizations(const std::size_t iteration) const;
  void drawAITstarSpecificVisualizations(const std::size_t iteration) const;
#ifdef PDT_EXTRA_EITSTAR_PR
  void drawEITstarSpecificVisualizations(const std::size_t iteration) const;
#endif  // #ifdef PDT_EXTRA_EITSTAR_PR
  void drawLazyPRMstarSpecificVisualizations(const std::size_t iteration) const;

  // Lowlevel drawing.
  void drawRectangle(const std::vector<float>& midpoint, const std::vector<float>& widths,
                     const float* faceColor, const float* edgeColor) const;
  void drawRectangle2D(const std::vector<float>& midpoint, const std::vector<float>& widths,
                       const float* faceColor, const float* edgeColor) const;
  void drawRectangle3D(const std::vector<float>& midpoint, const std::vector<float>& widths,
                       const float* faceColor, const float* edgeColor) const;
  void drawStarts() const;
  void drawGoal() const;
  void drawBoundary() const;
  void drawPoint(const Eigen::Vector2f& point, const float* color, float size) const;
  void drawPoint(const Eigen::Vector3f& point, const float* color, float size) const;
  void drawPoint(const ompl::base::ScopedState<ompl::base::RealVectorStateSpace>& state,
                 const float* color, float size) const;
  void drawPoints(const std::vector<Eigen::Vector2f>& points, const float* color, float size) const;
  void drawPoints(const std::vector<Eigen::Vector3f>& points, const float* color, float size) const;
  void drawPoints(const std::vector<ompl::base::ScopedState<>>& states, const float* color,
                  float size) const;
  void drawLines(const std::vector<Eigen::Vector2f>& points, float width, const float* color,
                 float alpha = 1.0) const;
  void drawLines(const std::vector<Eigen::Vector3f>& points, float width, const float* color,
                 float alpha = 1.0) const;
  void drawPath(const std::vector<Eigen::Vector2f>& points, float width, const float* color,
                float alpha = 1.0) const;
  void drawPath(const std::vector<Eigen::Vector3f>& points, float width, const float* color,
                float alpha = 1.0) const;
  void drawCars(const std::vector<Eigen::Vector3f>& points, float width, const float* color,
                float alpha = 1.0) const;

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

  // Implement visualizations of objectives.
  void visit(const objectives::PotentialFieldOptimizationObjective& objective) const override;
  void visit(const objectives::ReciprocalClearanceOptimizationObjective& objective) const override;
  void visit(const objectives::MaxMinClearanceOptimizationObjective& objective) const override;

  // Objective coloring helpers.
  mutable float minOptimizationCost_{std::numeric_limits<float>::max()};
  mutable float maxOptimizationCost_{std::numeric_limits<float>::lowest()};
  std::array<float, 4u> interpolateColors(const float* color1, const float* color2,
                                          const float step) const;

  // Helpers.
  std::pair<std::vector<Eigen::Vector2f>, std::vector<Eigen::Vector2f>> getVerticesAndEdges2D(
      const std::size_t iteration) const;
  std::pair<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>> getVerticesAndEdges3D(
      const std::size_t iteration) const;
  std::vector<Eigen::Vector2f> getVertices2D(const std::size_t iteration) const;
  std::vector<Eigen::Vector3f> getVertices3D(const std::size_t iteration) const;
  std::vector<Eigen::Vector2f> getEdges2D(const std::size_t iteration) const;
  std::vector<Eigen::Vector3f> getEdges3D(const std::size_t iteration) const;
  std::vector<Eigen::Vector2f> getPath2D(const std::size_t iteration) const;
  std::vector<Eigen::Vector3f> getPath3D(const std::size_t iteration) const;
  std::vector<Eigen::Vector3f> getPathSE2(const std::size_t iteration) const;

  // The bounds of the context (the real-vector part of it).
  ompl::base::RealVectorBounds bounds_;

  // The tikz visualizer.
  TikzVisualizer tikzVisualizer_;

  // The configuration.
  std::shared_ptr<const config::Configuration> config_;
};

}  // namespace visualization

}  // namespace pdt
