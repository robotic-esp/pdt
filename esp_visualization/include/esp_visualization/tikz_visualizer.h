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

#include <experimental/filesystem>

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerData.h>

#include "esp_common/context_type.h"
#include "esp_common/planner_type.h"
#include "esp_configuration/configuration.h"
#include "esp_obstacles/obstacle_visitor.h"
#include "esp_planning_contexts/all_contexts.h"
#include "esp_planning_contexts/context_visitor.h"
#include "esp_tikz/tikz_picture.h"
#include "esp_visualization/planner_specific_data.h"

namespace esp {

namespace ompltools {

class TikzVisualizer : public ContextVisitor, public ObstacleVisitor {
 public:
  TikzVisualizer(const std::shared_ptr<const Configuration>& config,
                 const std::shared_ptr<BaseContext>& context,
                 const std::pair<std::shared_ptr<ompl::base::Planner>, PLANNER_TYPE>& plannerPair);
  ~TikzVisualizer() = default;

  void render(const ompl::base::PlannerData& plannerData, std::size_t iteration,
              const ompl::base::PathPtr path,
              const std::shared_ptr<const PlannerSpecificData>& plannerSpecificData,
              double iterationTime, double totalTime, double solutionCost);

 private:
  // Compile a picture to a standalone document.
  std::experimental::filesystem::path compile(const std::experimental::filesystem::path& texPath,
                                              double cost, double time);

  // Log to the frame times file.
  void logToFrameTimes(const std::experimental::filesystem::path& pngPath, double iterationTime);

  // The configuration.
  const std::shared_ptr<const Configuration> config_;

  // The colors.
  std::map<std::string, std::array<int, 3>> espColors_{};

  // Implement visualizations of contexts.
  void visit(const CentreSquare& context) const override;
  void visit(const DividingWalls& context) const override;
  void visit(const DoubleEnclosure& context) const override;
  void visit(const FlankingGap& context) const override;
  void visit(const FourRooms& context) const override;
  void visit(const GoalEnclosure& context) const override;
  void visit(const ObstacleFree& context) const override;
  void visit(const RandomRectangles& context) const override;
  void visit(const RandomRectanglesMultiStartGoal& context) const override;
  void visit(const RepeatingRectangles& context) const override;
  void visit(const StartEnclosure& context) const override;
  void visit(const WallGap& context) const override;

  // Implement visualizations of obstacles.
  void visit(const Hyperrectangle<BaseObstacle>& obstacle) const override;
  void visit(const Hyperrectangle<BaseAntiObstacle>& antiObstacle) const override;

  // Helper functions.
  void drawBoundary(const BaseContext& context) const;
  void drawStartStates(const BaseContext& context) const;
  void drawGoalStates(const BaseContext& context) const;
  void drawVertex(const ompl::base::PlannerDataVertex& vertex) const;
  void drawVertex(const ompl::base::RealVectorStateSpace::StateType* state,
                  const std::string& options) const;
  void drawEdge(const ompl::base::PlannerDataVertex& parent,
                const ompl::base::PlannerDataVertex& child) const;
  void drawEdge(const ompl::base::RealVectorStateSpace::StateType* parent,
                const ompl::base::RealVectorStateSpace::StateType* child,
                const std::string& options) const;
  void drawSolution(const ompl::base::PathPtr path) const;
  void drawEllipse(double cost) const;
  void drawRectangle(double midX, double midY, double widthX, double widthY,
                     const std::string& name, const std::string& lineColor,
                     const std::string& fillColor) const;
  void drawPlannerSpecificVisualizations(
      const std::shared_ptr<const PlannerSpecificData>& plannerSpecificData) const;
  void drawBITstarSpecificVisualizations(
      const std::shared_ptr<const BITstarData>& bitstarData) const;
  void drawTBDstarSpecificVisualizations(
      const std::shared_ptr<const TBDstarData>& tbdstarData) const;

  // Planner and context to be visualized.
  std::shared_ptr<BaseContext> context_;
  PLANNER_TYPE plannerType_{PLANNER_TYPE::INVALID};
  std::string name_{"invalid planner"};

  // The tikz picture holding the visualization.
  mutable TikzPicture picture_;

  // The frame times file.
  std::ofstream frameTimes_;
};

}  // namespace ompltools

}  // namespace esp