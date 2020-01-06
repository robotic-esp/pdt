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

#include <pangolin/pangolin.h>

#include "esp_common/context_type.h"
#include "esp_common/planner_type.h"
#include "esp_configuration/configuration.h"
#include "esp_obstacles/obstacle_visitor.h"
#include "esp_optimization_objectives/optimization_objective_visitor.h"
#include "esp_optimization_objectives/potential_field_optimization_objective.h"
#include "esp_optimization_objectives/reciprocal_clearance_optimization_objective.h"
#include "esp_planning_contexts/all_contexts.h"
#include "esp_planning_contexts/context_visitor.h"
#include "esp_planning_contexts/real_vector_geometric_context.h"
#include "esp_visualization/base_visualizer.h"
#include "esp_visualization/tikz_visualizer.h"

namespace esp {

namespace ompltools {

class InteractiveVisualizer : public BaseVisualizer,
                              public ContextVisitor,
                              public ObstacleVisitor,
                              public ObjectiveVisitor {
 public:
  InteractiveVisualizer(
      const std::shared_ptr<Configuration>& config,
      const std::shared_ptr<RealVectorGeometricContext>& context,
      const std::pair<std::shared_ptr<ompl::base::Planner>, PLANNER_TYPE> plannerPair);
  ~InteractiveVisualizer() = default;

  void run();

 private:
  // General helpers.
  void incrementIteration(std::size_t num = 1u);
  void decrementIteration(std::size_t num = 1u);

  // Play to view and record to view.
  bool playToIteration_{false};
  bool recording_{false};
  bool exporting_{false};
  std::size_t iterationToPlayTo_{0u};
  time::Duration desiredDisplayDuration_{};
  time::Duration actualDisplayDuration_{};
  time::Clock::time_point displayStartTime_{};
  std::size_t screencaptureId_{0u};

  // Plotting.
  pangolin::DataLog costLog_{};
  void updateCostLog();
  double minCost_{0.0};
  double maxCost_{0.0};
  float plotBackgroundColor_[4]{0.9, 0.9, 0.9, 1.0};
  int largestPlottedIteration_{-1};
  void logCost();

  // Highlevel drawing.
  void drawVerticesAndEdges(std::size_t iteration);
  void drawVertices(std::size_t iteration);
  void drawEdges(std::size_t iteration);
  void drawSolution(std::size_t iteration);
  void drawStateIds(std::size_t iteration);

  // Planner specific visualizations.
  void drawPlannerSpecificVisualizations(std::size_t iteration) const;
  void drawBITstarSpecificVisualizations(std::size_t iteration) const;
  void drawTBDstarSpecificVisualizations(std::size_t iteration) const;
  void drawAIBITstarSpecificVisualizations(std::size_t iteration) const;

  // Lowlevel drawing.
  void drawRectangle(const std::vector<double>& midpoint, const std::vector<double>& widths,
                     const float* faceColor, const float* edgeColor) const;
  void drawRectangle2D(const std::vector<double>& midpoint, const std::vector<double>& widths,
                       const float* faceColor, const float* edgeColor) const;
  void drawRectangle3D(const std::vector<double>& midpoint, const std::vector<double>& widths,
                       const float* faceColor, const float* edgeColor) const;
  void drawBoundary(const RealVectorGeometricContext& context) const;
  void drawPoint(const Eigen::Vector2d& point, const float* color, float size) const;
  void drawPoint(const Eigen::Vector3d& point, const float* color, float size) const;
  void drawPoint(const ompl::base::ScopedState<ompl::base::RealVectorStateSpace>& state,
                 const float* color, float size) const;
  void drawPoints(const std::vector<Eigen::Vector2d>& points, const float* color, float size) const;
  void drawPoints(const std::vector<Eigen::Vector3d>& points, const float* color, float size) const;
  void drawPoints(
      const std::vector<ompl::base::ScopedState<ompl::base::RealVectorStateSpace>>& states,
      const float* color, float size) const;
  void drawLines(const std::vector<Eigen::Vector2d>& points, float width, const float* color,
                 float alpha = 1.0) const;
  void drawLines(const std::vector<Eigen::Vector3d>& points, float width, const float* color,
                 float alpha = 1.0) const;
  void drawPath(const std::vector<Eigen::Vector2d>& points, float width, const float* color,
                float alpha = 1.0) const;
  void drawPath(const std::vector<Eigen::Vector3d>& points, float width, const float* color,
                float alpha = 1.0) const;

  // Implement visualizations of contexts.
  void visit(const CentreSquare& context) const override;
  void visit(const DividingWalls& context) const override;
  void visit(const DoubleEnclosure& context) const override;
  void visit(const FlankingGap& context) const override;
  void visit(const FourRooms& context) const override;
  void visit(const GoalEnclosure& context) const override;
  void visit(const NarrowPassage& context) const override;
  void visit(const ObstacleFree& context) const override;
  void visit(const RandomRectangles& context) const override;
  void visit(const RandomRectanglesMultiStartGoal& context) const override;
  void visit(const RepeatingRectangles& context) const override;
  void visit(const StartEnclosure& context) const override;
  void visit(const WallGap& context) const override;

  // Implement visualizations of obstacles.
  void visit(const Hyperrectangle<BaseObstacle>& obstacle) const override;
  void visit(const Hyperrectangle<BaseAntiObstacle>& antiObstacle) const override;

  // Implement visualizations of objectives.
  void visit(const PotentialFieldOptimizationObjective& objective) const override;
  void visit(const ReciprocalClearanceOptimizationObjective& objective) const override;
  void visit(const MaxMinClearanceOptimizationObjective& objective) const override;

  // Objective coloring helpers.
  mutable double minOptimizationCost_{std::numeric_limits<double>::max()};
  mutable double maxOptimizationCost_{std::numeric_limits<double>::lowest()};
  std::array<float, 4u> interpolateColors(const float* color1, const float* color2,
                                          double step) const;

  // Helpers.
  std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> getVerticesAndEdges2D(
      std::size_t iteration) const;
  std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> getVerticesAndEdges3D(
      std::size_t iteration) const;
  std::vector<Eigen::Vector2d> getVertices2D(std::size_t iteration) const;
  std::vector<Eigen::Vector3d> getVertices3D(std::size_t iteration) const;
  std::vector<Eigen::Vector2d> getEdges2D(std::size_t iteration) const;
  std::vector<Eigen::Vector3d> getEdges3D(std::size_t iteration) const;
  std::vector<Eigen::Vector2d> getPath2D(std::size_t iteration) const;
  std::vector<Eigen::Vector3d> getPath3D(std::size_t iteration) const;

  // The tikz visualizer.
  TikzVisualizer tikzVisualizer_;

  // The configuration.
  std::shared_ptr<const Configuration> config_;
};

}  // namespace ompltools

}  // namespace esp
