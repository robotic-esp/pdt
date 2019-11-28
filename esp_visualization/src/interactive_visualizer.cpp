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

#include "esp_visualization/interactive_visualizer.h"

#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/bitstar/datastructures/Vertex.h>

#include <pangolin/display/display_internal.h>

#include "esp_visualization/fonts.h"

namespace esp {

namespace ompltools {

InteractiveVisualizer::InteractiveVisualizer(
    const std::shared_ptr<Configuration>& config,
    const std::shared_ptr<RealVectorGeometricContext>& context,
    const std::pair<std::shared_ptr<ompl::base::Planner>, PLANNER_TYPE> plannerPair) :
    BaseVisualizer(config, context, plannerPair),
    tikzVisualizer_(config, context, plannerPair),
    config_(config) {
  if (context_->getStateSpace()->getType() != ompl::base::StateSpaceType::STATE_SPACE_REAL_VECTOR) {
    OMPL_ERROR("Visualizer only tested for real vector state spaces.");
    throw std::runtime_error("Visualizer error.");
  }
}

void InteractiveVisualizer::run() {
  // The default window width and height.
  constexpr std::size_t windowWidth = 800;
  constexpr std::size_t windowHeight = 800;

  costLog_.SetLabels(std::vector<std::string>({planner_->getName()}));

  // Create a window and bind it to the current OpenGL context.
  pangolin::CreateWindowAndBind("esp ompltools", windowWidth, windowHeight);

  // Set a more pleasureable font.
  auto GlContext = pangolin::GetCurrentContext();
  GlContext->font = std::make_shared<pangolin::GlFont>(Fonts::ROBOTO_REGULAR.string(), 16);

  // Set up a viewport. A viewport is where OpenGL draws to. We can
  // have multiple viewports for different parts of a window. Any draw
  // command will draw to the active viewport.
  pangolin::View& contextView = pangolin::CreateDisplay();
  pangolin::View& plotView = pangolin::CreateDisplay();

  // We can set the bounds of this viewport. The coordinates are with
  // respect to the window boundaries (0 is left/bottom, 1 is right/top).
  // We can also set an aspect ratio such that even if the window gets
  // rescaled the drawings in the viewport do not get distorted.
  auto bounds = context_->getBoundaries();
  if (context_->getDimension() == 2u) {
    contextView.SetBounds(
        pangolin::Attach::Pix(205), pangolin::Attach::ReversePix(5), pangolin::Attach::Pix(205),
        pangolin::Attach::ReversePix(5),
        (bounds.high.at(0) - bounds.low.at(0)) / (bounds.high.at(1) - bounds.low.at(1)));
  } else {
    contextView.SetBounds(
        pangolin::Attach::Pix(205), pangolin::Attach::ReversePix(5), pangolin::Attach::Pix(205),
        pangolin::Attach::ReversePix(5),
        -(bounds.high.at(0) - bounds.low.at(0)) / (bounds.high.at(1) - bounds.low.at(1)));
  }
  plotView.SetBounds(0.0, pangolin::Attach::Pix(200), pangolin::Attach::Pix(200), 1.0);
  pangolin::Plotter plotter(&costLog_, 0.0, 1.0, 0.0, 5.0);
  plotter.SetBackgroundColour(
      pangolin::Colour(plotBackgroundColor_[0], plotBackgroundColor_[1], plotBackgroundColor_[2]));
  plotView.AddDisplay(plotter);

  // Create an OpenGL render state. This controls how coordinates are
  // processed before they are drawn in OpenGL's [-1, 1] x [-1, 1] coordinates.
  pangolin::OpenGlRenderState renderState;

  // We can instantiate a handler used to modify the view in 3D.
  pangolin::Handler3D handler(renderState);

  // Get the bounds of the context to setup a correct renderstate.
  if (context_->getDimension() == 2u) {
    // glShadeModel(GL_FLAT);
    // glEnable(GL_LINE_SMOOTH);
    // glEnable(GL_BLEND);
    // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    // glDisable(GL_DEPTH_TEST);
    // glDisable(GL_LIGHTING);
    renderState.SetProjectionMatrix(pangolin::ProjectionMatrixOrthographic(
        bounds.low.at(0),   // The left boundary of the problem
        bounds.high.at(0),  // The right boundary of the problem
        bounds.low.at(1),   // The bottom boundary of the porblem
        bounds.high.at(1),  // The top boundary of the problem
        -1.0f,              // Shouldn't have to change this in 2D
        1.0f                // Shouldn't have to change this in 2D
        ));
  } else if (context_->getDimension() == 3u) {
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_DEPTH_TEST);
    renderState.SetProjectionMatrix(
        pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100));
    renderState.SetModelViewMatrix(pangolin::ModelViewLookAt(-2, 3, -2, 0, 0, 0, pangolin::AxisY));
    contextView.SetHandler(&handler);
  } else {
    throw std::runtime_error(
        "Interactive visualizer can currently only visualize 2D or 3D-context.");
  }

  // Create the options panel.
  const std::string optionsName{"options"};
  pangolin::CreatePanel(optionsName).SetBounds(0.0f, 1.0f, 0.0f, pangolin::Attach::Pix(200));

  // Tickboxes.
  pangolin::Var<bool> optionDrawContext(optionsName + ".Context", true, true);
  pangolin::Var<bool> optionDrawObstacles(optionsName + ".Obstacles", true, true);
  pangolin::Var<bool> optionDrawObjective(optionsName + ".Objective", true, true);
  pangolin::Var<bool> optionDrawVertices(optionsName + ".Vertices", true, true);
  pangolin::Var<bool> optionDrawEdges(optionsName + ".Edges", true, true);
  pangolin::Var<bool> optionDrawPlannerSpecificData(optionsName + ".Planner Specific", true, true);
  pangolin::Var<bool> optionDrawSolution(optionsName + ".Solution", true, true);
  pangolin::Var<bool> optionTrack(optionsName + ".Track", true, true);
  // Buttons.
  pangolin::Var<bool> optionScreenshot(optionsName + ".Screenshot", false, false);
  pangolin::Var<double> optionSlowdown(optionsName + ".Replay Factor", 1, 1e-3, 1e2, true);
  pangolin::Var<bool> optionPlay(optionsName + ".Play", false, false);
  pangolin::Var<bool> optionRecord(optionsName + ".Record", false, false);
  pangolin::Var<bool> optionExport(optionsName + ".Export", false, false);

  // Register some keypresses.
  pangolin::RegisterKeyPressCallback('f', [this]() { incrementIteration(1u); });
  pangolin::RegisterKeyPressCallback('b', [this]() { decrementIteration(1u); });
  pangolin::RegisterKeyPressCallback('F', [this]() {
    incrementIteration(std::ceil(0.1 * static_cast<double>(largestIteration_)));
  });
  pangolin::RegisterKeyPressCallback('B', [this]() {
    decrementIteration(std::ceil(0.1 * static_cast<double>(largestIteration_)));
  });
  pangolin::RegisterKeyPressCallback(' ', [&optionTrack]() { optionTrack = !optionTrack; });

  // This sets the color used when clearing the screen.
  glClearColor(1.0, 1.0, 1.0, 1.0);

  maxCost_ = 0.1;
  minCost_ = 0.0;
  while (!pangolin::ShouldQuit()) {
    // Register input.
    if (pangolin::Pushed(optionScreenshot)) {
      contextView.SaveOnRender(std::to_string(screencaptureId_++) + '_' + context_->getName() +
                               '_' + planner_->getName());
    }
    if (pangolin::Pushed(optionPlay)) {
      optionTrack = false;
      playToIteration_ = true;
      iterationToPlayTo_ = displayIteration_;
      displayIteration_ = 0u;
      desiredDisplayDuration_ = getIterationDuration(displayIteration_);
      actualDisplayDuration_ = time::Duration(0.0);
      displayStartTime_ = time::Clock::now();
    }
    if (pangolin::Pushed(optionRecord)) {
      optionTrack = false;
      playToIteration_ = true;
      iterationToPlayTo_ = displayIteration_;
      displayIteration_ = 0u;
      desiredDisplayDuration_ = getIterationDuration(displayIteration_);
      actualDisplayDuration_ = time::Duration(0.0);
      displayStartTime_ = time::Clock::now();
      recording_ = true;
      contextView.RecordOnRender("ffmpeg:[fps=60,bps=100000000,unique_filename]//" +
                                 std::to_string(screencaptureId_++) + '_' + context_->getName() +
                                 '_' + planner_->getName() + ".avi");
    }
    if (pangolin::Pushed(optionExport)) {
      optionTrack = false;
      exporting_ = true;
      iterationToPlayTo_ = displayIteration_;
      displayIteration_ = 0u;
    }

    // Set values if we're playing in realtime.
    if (playToIteration_) {
      if (displayIteration_ >= iterationToPlayTo_) {
        displayIteration_ = iterationToPlayTo_;
        playToIteration_ = false;
        if (recording_) {
          contextView.RecordOnRender("ffmpeg:[fps=60,bps=100000000,unique_filename]//" +
                                     std::to_string(screencaptureId_) + '_' + context_->getName() +
                                     '_' + planner_->getName() + ".avi");
        }
      } else {
        actualDisplayDuration_ += time::Duration(
            time::Duration((time::Clock::now() - displayStartTime_)).count() * optionSlowdown);
        if (actualDisplayDuration_ >= desiredDisplayDuration_) {
          // Compute how much we overshot.
          auto overshoot = actualDisplayDuration_ - desiredDisplayDuration_;
          time::Duration totalSkippingDuration(0.0);
          do {
            incrementIteration(1u);
            totalSkippingDuration += getIterationDuration(displayIteration_);
          } while (overshoot > totalSkippingDuration);
          desiredDisplayDuration_ = getIterationDuration(displayIteration_);
          actualDisplayDuration_ = time::Duration(0.0);
        }
        displayStartTime_ = time::Clock::now();
      }
    }

    if (exporting_) {
      if (displayIteration_ > iterationToPlayTo_) {
        exporting_ = false;
      } else {
        OMPL_WARN("Exporting iteration %zu of %zu.", static_cast<unsigned>(displayIteration_),
                  static_cast<unsigned>(iterationToPlayTo_));
        if (optionDrawPlannerSpecificData) {
          tikzVisualizer_.render(*getPlannerData(displayIteration_), displayIteration_,
                                 getSolutionPath(displayIteration_),
                                 getPlannerSpecificData(displayIteration_),
                                 getIterationDuration(displayIteration_).count(),
                                 getTotalElapsedDuration(displayIteration_).count(),
                                 getSolutionCost(displayIteration_).value());
        } else {
          tikzVisualizer_.render(*getPlannerData(displayIteration_), displayIteration_,
                                 getSolutionPath(displayIteration_), nullptr,
                                 getIterationDuration(displayIteration_).count(),
                                 getTotalElapsedDuration(displayIteration_).count(),
                                 getSolutionCost(displayIteration_).value());
        }
        incrementIteration();
      }
    }

    // Activate this render state for the canvas.
    contextView.Activate(renderState);

    // Clear the viewport.
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Only draw obstacles and objective if context is drawn.
    if (!optionDrawContext) {
      optionDrawObstacles = false;
      optionDrawObjective = false;
    }

    // Draw the objective.
    if (optionDrawObjective) {
      if (auto objective =
              std::dynamic_pointer_cast<BaseOptimizationObjective>(context_->getObjective())) {
        objective->accept(*this);
      }
    }

    // Draw the obstacles.
    if (optionDrawObstacles) {
      for (auto obstacle : context_->getObstacles()) {
        obstacle->accept(*this);
      }
      for (auto antiObstacle : context_->getAntiObstacles()) {
        antiObstacle->accept(*this);
      }
    }

    // Track the current iteration if selected.
    if (optionTrack) {
      while (displayIteration_ < largestIteration_) {
        incrementIteration(100u);
      }
    }

    // Draw the vertices and edges.
    if (optionDrawVertices && optionDrawEdges) {
      drawVerticesAndEdges(displayIteration_);
    } else if (optionDrawVertices) {
      drawVertices(displayIteration_);
    } else if (optionDrawEdges) {
      drawEdges(displayIteration_);
    }

    // Draw the planner specific data.
    if (optionDrawPlannerSpecificData) {
      drawPlannerSpecificVisualizations(displayIteration_);
    }

    // Draw the solution.
    if (optionDrawSolution) {
      drawSolution(displayIteration_);
    }

    // Draw the context.
    if (optionDrawContext) {
      context_->accept(*this);
    }

    // Set the correct view of the plot.
    plotter.SetView(pangolin::XYRangef(0, 1.06 * static_cast<float>(largestPlottedIteration_),
                                       0.98 * minCost_, 1.02 * maxCost_));
    // Add the marker for the current iteration to the plot.
    plotter.ClearMarkers();
    plotter.AddMarker(pangolin::Marker(
        pangolin::Marker::Direction::Vertical, static_cast<float>(displayIteration_),
        pangolin::Marker::Equality::Equal, pangolin::Colour(black[0], black[1], black[2])));

    // Tell pangolin to render the frame.
    pangolin::FinishFrame();
  }
}

void InteractiveVisualizer::incrementIteration(std::size_t num) {
  if (num == 1u) {
    if (displayIteration_ < largestIteration_) {
      ++displayIteration_;
      updateCostLog();
    }
  } else {
    while (displayIteration_ < largestIteration_ && num > 0u) {
      ++displayIteration_;
      --num;
      updateCostLog();
    }
  }
}

void InteractiveVisualizer::decrementIteration(std::size_t num) {
  if (num == 1u) {
    if (displayIteration_ > 0u) {
      --displayIteration_;
      updateCostLog();
    }
  } else {
    while (displayIteration_ > 0u && num > 0u) {
      --displayIteration_;
      --num;
      updateCostLog();
    }
  }
}

void InteractiveVisualizer::updateCostLog() {
  while (largestPlottedIteration_ < static_cast<int>(displayIteration_)) {
    auto path = getSolutionPath(++largestPlottedIteration_)->as<ompl::geometric::PathGeometric>();
    if (path != nullptr) {
      double cost =
          path->cost(planner_->getProblemDefinition()->getOptimizationObjective()).value();
      if (cost > maxCost_) {
        maxCost_ = cost;
      }
      if (cost < minCost_) {
        minCost_ = cost;
      }
      costLog_.Log(cost);
    } else {
      costLog_.Log(std::numeric_limits<double>::max());
    }
  }
}

void InteractiveVisualizer::drawVerticesAndEdges(std::size_t iteration) {
  if (context_->getDimension() == 2u) {
    auto [vertices, edges] = getVerticesAndEdges2D(iteration);
    drawPoints(vertices, blue, 2.0);
    drawLines(edges, 3.0, gray);
  } else if (context_->getDimension() == 3u) {
    auto [vertices, edges] = getVerticesAndEdges3D(iteration);
    drawPoints(vertices, blue, 2.0);
    drawLines(edges, 3.0, gray, 0.8);
  }
}

void InteractiveVisualizer::drawVertices(std::size_t iteration) {
  if (context_->getDimension() == 2u) {
    auto vertices = getVertices2D(iteration);
    drawPoints(vertices, blue, 2.0);
  } else if (context_->getDimension() == 3u) {
    auto vertices = getVertices3D(iteration);
    drawPoints(vertices, blue, 2.0);
  }
}

void InteractiveVisualizer::drawEdges(std::size_t iteration) {
  if (context_->getDimension() == 2u) {
    auto edges = getEdges2D(iteration);
    drawLines(edges, 3.0, gray);
  } else if (context_->getDimension() == 3u) {
    auto edges = getEdges3D(iteration);
    drawLines(edges, 3.0, gray, 0.8);
  }
}

void InteractiveVisualizer::drawSolution(std::size_t iteration) {
  if (context_->getDimension() == 2u) {
    auto path = getPath2D(iteration);
    drawPath(path, 3.0, purple);
  } else if (context_->getDimension() == 3u) {
    auto path = getPath3D(iteration);
    drawPath(path, 3.0, purple, 1.0);
  }
}

void InteractiveVisualizer::visit(const CentreSquare& context) const {
  // Draw the start states.
  drawPoint(context.getStartState(), green, 9.0);
  // Draw the goal states.
  drawPoint(context.getGoalState(), red, 9.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const DividingWalls& context) const {
  // Draw the start states.
  drawPoint(context.getStartState(), green, 9.0);
  // Draw the goal states.
  drawPoint(context.getGoalState(), red, 9.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const DoubleEnclosure& context) const {
  // Draw the start states.
  drawPoint(context.getStartState(), green, 9.0);
  // Draw the goal states.
  drawPoint(context.getGoalState(), red, 9.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const FlankingGap& context) const {
  // Draw the start states.
  drawPoint(context.getStartState(), green, 9.0);
  // Draw the goal states.
  drawPoint(context.getGoalState(), red, 9.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const FourRooms& context) const {
  // Draw the start states.
  drawPoint(context.getStartState(), green, 9.0);
  // Draw the goal states.
  drawPoint(context.getGoalState(), red, 9.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const GoalEnclosure& context) const {
  // Draw the start states.
  drawPoint(context.getStartState(), green, 9.0);
  // Draw the goal states.
  drawPoint(context.getGoalState(), red, 9.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const ObstacleFree& context) const {
  // Draw the start states.
  drawPoint(context.getStartState(), green, 9.0);
  // Draw the goal states.
  drawPoint(context.getGoalState(), red, 9.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const RandomRectangles& context) const {
  // Draw the start states.
  drawPoint(context.getStartState(), green, 9.0);
  // Draw the goal states.
  drawPoint(context.getGoalState(), red, 9.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const RandomRectanglesMultiStartGoal& context) const {
  // Draw the start states.
  drawPoints(context.getStartStates(), green, 9.0);
  // Draw the goal states.
  drawPoints(context.getGoalStates(), red, 9.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const RepeatingRectangles& context) const {
  // Draw the start states.
  drawPoint(context.getStartState(), green, 9.0);
  // Draw the goal states.
  drawPoint(context.getGoalState(), red, 9.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const StartEnclosure& context) const {
  // Draw the start states.
  drawPoint(context.getStartState(), green, 9.0);
  // Draw the goal states.
  drawPoint(context.getGoalState(), red, 9.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const WallGap& context) const {
  // Draw the start states.
  drawPoint(context.getStartState(), green, 9.0);
  // Draw the goal states.
  drawPoint(context.getGoalState(), red, 9.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::drawBoundary(const RealVectorGeometricContext& context) const {
  glColor4fv(black);
  glLineWidth(3.0);
  if (context.getDimension() == 2u) {
    auto bounds = context.getBoundaries();
    pangolin::glDrawRectPerimeter(bounds.low.at(0), bounds.low.at(1), bounds.high.at(0),
                                  bounds.high.at(1));
  } else if (context.getDimension() == 3u) {
    auto bounds = context.getBoundaries();
    std::vector<Eigen::Vector3d> points{
        {bounds.low.at(0), bounds.low.at(1), bounds.low.at(2)},
        {bounds.low.at(0), bounds.low.at(1), bounds.high.at(2)},
        {bounds.low.at(0), bounds.low.at(1), bounds.low.at(2)},
        {bounds.low.at(0), bounds.high.at(1), bounds.low.at(2)},
        {bounds.low.at(0), bounds.low.at(1), bounds.low.at(2)},
        {bounds.high.at(0), bounds.low.at(1), bounds.low.at(2)},
        {bounds.high.at(0), bounds.high.at(1), bounds.high.at(2)},
        {bounds.high.at(0), bounds.high.at(1), bounds.low.at(2)},
        {bounds.high.at(0), bounds.high.at(1), bounds.high.at(2)},
        {bounds.high.at(0), bounds.low.at(1), bounds.high.at(2)},
        {bounds.high.at(0), bounds.high.at(1), bounds.high.at(2)},
        {bounds.low.at(0), bounds.high.at(1), bounds.high.at(2)},
        {bounds.low.at(0), bounds.low.at(1), bounds.high.at(2)},
        {bounds.low.at(0), bounds.high.at(1), bounds.high.at(2)},
        {bounds.low.at(0), bounds.low.at(1), bounds.high.at(2)},
        {bounds.high.at(0), bounds.low.at(1), bounds.high.at(2)},
        {bounds.low.at(0), bounds.high.at(1), bounds.low.at(2)},
        {bounds.low.at(0), bounds.high.at(1), bounds.high.at(2)},
        {bounds.low.at(0), bounds.high.at(1), bounds.low.at(2)},
        {bounds.high.at(0), bounds.high.at(1), bounds.low.at(2)},
        {bounds.high.at(0), bounds.low.at(1), bounds.low.at(2)},
        {bounds.high.at(0), bounds.high.at(1), bounds.low.at(2)},
        {bounds.high.at(0), bounds.low.at(1), bounds.low.at(2)},
        {bounds.high.at(0), bounds.low.at(1), bounds.high.at(2)},
    };
    pangolin::glDrawLines(points);
  }
}

void InteractiveVisualizer::drawRectangle(const std::vector<double>& midpoint,
                                          const std::vector<double>& widths, const float* faceColor,
                                          const float* edgeColor) const {
  if (midpoint.size() == 2u && widths.size() == 2u) {
    drawRectangle2D(midpoint, widths, faceColor, edgeColor);
  } else if (midpoint.size() == 3u && widths.size() == 3u) {
    drawRectangle3D(midpoint, widths, faceColor, edgeColor);
  } else {
    OMPL_ERROR("Interactive visualizer can only visualize 2D or 3D contexts.");
    throw std::runtime_error("Visualization error.");
  }
}

void InteractiveVisualizer::drawRectangle2D(const std::vector<double>& midpoint,
                                            const std::vector<double>& widths,
                                            const float* faceColor, const float* edgeColor) const {
  glColor4fv(faceColor);
  pangolin::glDrawRect(midpoint.at(0) - widths.at(0) / 2.0, midpoint.at(1) - widths.at(1) / 2.0,
                       midpoint.at(0) + widths.at(0) / 2.0, midpoint.at(1) + widths.at(1) / 2.0);
  glColor4fv(edgeColor);
  pangolin::glDrawRectPerimeter(
      midpoint.at(0) - widths.at(0) / 2.0, midpoint.at(1) - widths.at(1) / 2.0,
      midpoint.at(0) + widths.at(0) / 2.0, midpoint.at(1) + widths.at(1) / 2.0);
}

void InteractiveVisualizer::drawRectangle3D(const std::vector<double>& midpoint,
                                            const std::vector<double>& widths,
                                            const float* faceColor, const float* edgeColor) const {
  float xhalf = widths[0] / 2.0;
  float yhalf = widths[1] / 2.0;
  float zhalf = widths[2] / 2.0;
  const GLfloat xmin = midpoint[0] - xhalf;
  const GLfloat xmax = midpoint[0] + xhalf;
  const GLfloat ymin = midpoint[1] - yhalf;
  const GLfloat ymax = midpoint[1] + yhalf;
  const GLfloat zmin = midpoint[2] - zhalf;
  const GLfloat zmax = midpoint[2] + zhalf;
  const GLfloat vertices[] = {
      xmin, ymin, zmax, xmax, ymin, zmax, xmin, ymax, zmax, xmax, ymax, zmax, xmin, ymin, zmin,
      xmin, ymax, zmin, xmax, ymin, zmin, xmax, ymax, zmin, xmin, ymin, zmax, xmin, ymax, zmax,
      xmin, ymin, zmin, xmin, ymax, zmin, xmax, ymin, zmin, xmax, ymax, zmin, xmax, ymin, zmax,
      xmax, ymax, zmax, xmin, ymax, zmax, xmax, ymax, zmax, xmin, ymax, zmin, xmax, ymax, zmin,
      xmin, ymin, zmax, xmin, ymin, zmin, xmax, ymin, zmax, xmax, ymin, zmin};
  glVertexPointer(3, GL_FLOAT, 0, vertices);
  glEnableClientState(GL_VERTEX_ARRAY);
  glColor4fv(faceColor);
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  glDrawArrays(GL_TRIANGLE_STRIP, 4, 4);
  glDrawArrays(GL_TRIANGLE_STRIP, 8, 4);
  glDrawArrays(GL_TRIANGLE_STRIP, 12, 4);
  glDrawArrays(GL_TRIANGLE_STRIP, 16, 4);
  glDrawArrays(GL_TRIANGLE_STRIP, 20, 4);
  glColor4fv(edgeColor);
  glDrawArrays(GL_LINE_STRIP, 0, 4);
  glDrawArrays(GL_LINE_STRIP, 4, 4);
  glDrawArrays(GL_LINE_STRIP, 8, 4);
  glDrawArrays(GL_LINE_STRIP, 12, 4);
  glDrawArrays(GL_LINE_STRIP, 16, 4);
  glDrawArrays(GL_LINE_STRIP, 20, 4);
  glDisableClientState(GL_VERTEX_ARRAY);
}

void InteractiveVisualizer::drawPoint(const Eigen::Vector2d& point, const float* color,
                                      float size) const {
  glColor4fv(color);
  pangolin::glDrawCircle(point[0], point[1], size / 1000.0);
}

void InteractiveVisualizer::drawPoint(const Eigen::Vector3d& point, const float* color,
                                      float size) const {
  std::vector<Eigen::Vector3d> points;
  points.emplace_back(point);
  drawPoints(points, color, size);
}

void InteractiveVisualizer::drawPoint(
    const ompl::base::ScopedState<ompl::base::RealVectorStateSpace>& state, const float* color,
    float size) const {
  if (state.getSpace()->getDimension() == 2u) {
    drawPoint(Eigen::Vector2d(state[0u], state[1u]), color, size);
  } else if (state.getSpace()->getDimension() == 3u) {
    drawPoint(Eigen::Vector3d(state[0u], state[1u], state[2u]), color, size);
  } else {
    throw std::runtime_error("Can only visualize 2d and 3d states.");
  }
}

void InteractiveVisualizer::drawPoints(const std::vector<Eigen::Vector2d>& points,
                                       const float* color, float size) const {
  glColor4fv(color);
  for (const auto& point : points) {
    pangolin::glDrawCircle(point[0], point[1], size / 1000.0);
  }
}

void InteractiveVisualizer::drawPoints(const std::vector<Eigen::Vector3d>& points,
                                       const float* color, float size) const {
  glColor4fv(color);
  glPointSize(size);
  pangolin::glDrawPoints(points);
}

void InteractiveVisualizer::drawPoints(
    const std::vector<ompl::base::ScopedState<ompl::base::RealVectorStateSpace>>& states,
    const float* color, float size) const {
  if (states.front().getSpace()->getDimension() == 2u) {
    std::vector<Eigen::Vector2d> points;
    for (const auto& state : states) {
      points.emplace_back(state[0], state[1]);
    }
    drawPoints(points, color, size);
  } else if (states.front().getSpace()->getDimension() == 3u) {
    std::vector<Eigen::Vector3d> points;
    for (const auto& state : states) {
      points.emplace_back(state[0], state[1], state[2]);
    }
    drawPoints(points, color, size);
  }
}

void InteractiveVisualizer::drawLines(const std::vector<Eigen::Vector2d>& points, float width,
                                      const float* color, float alpha) const {
  assert(points.size() % 2 == 0u);
  glColor4f(color[0], color[1], color[2], alpha);
  glLineWidth(width);
  pangolin::glDrawLines(points);
}

void InteractiveVisualizer::drawLines(const std::vector<Eigen::Vector3d>& points, float width,
                                      const float* color, float alpha) const {
  assert(points.size() % 2 == 0u);
  glColor4f(color[0], color[1], color[2], alpha);
  glLineWidth(width);
  pangolin::glDrawLines(points);
}

void InteractiveVisualizer::drawPath(const std::vector<Eigen::Vector2d>& points, float width,
                                     const float* color, float alpha) const {
  glColor4f(color[0], color[1], color[2], alpha);
  glLineWidth(width);
  pangolin::glDrawLineStrip(points);
}

void InteractiveVisualizer::drawPath(const std::vector<Eigen::Vector3d>& points, float width,
                                     const float* color, float alpha) const {
  glColor4f(color[0], color[1], color[2], alpha);
  glLineWidth(width);
  pangolin::glDrawLineStrip(points);
}

void InteractiveVisualizer::visit(const Hyperrectangle<BaseObstacle>& obstacle) const {
  drawRectangle(obstacle.getAnchorCoordinates(), obstacle.getWidths(), black, black);
}

void InteractiveVisualizer::visit(const Hyperrectangle<BaseAntiObstacle>& antiObstacle) const {
  drawRectangle(antiObstacle.getAnchorCoordinates(), antiObstacle.getWidths(), white, white);
}

void InteractiveVisualizer::visit(const PotentialFieldOptimizationObjective& objective) const {
  if (context_->getDimension() == 2u) {
    // Get the widths of the context.
    const double minX = context_->getBoundaries().low.at(0u);
    const double maxX = context_->getBoundaries().high.at(0u);
    const double minY = context_->getBoundaries().low.at(1u);
    const double maxY = context_->getBoundaries().high.at(1u);
    const double widthX = maxX - minX;
    const double widthY = maxY - minY;

    // Define the number of points per axis.
    constexpr std::size_t numPointsPerAxis{40u};

    // Compute the resulting step sizes.
    const double stepX = widthX / (numPointsPerAxis);
    const double stepY = widthY / (numPointsPerAxis);

    // Prepare a state to probe the cost at the discretized locations.
    auto costState =
        context_->getStateSpace()->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();

    // Determine the min and max costs if not already determined.
    if (minOptimizationCost_ == std::numeric_limits<double>::max()) {
      for (std::size_t i = 0u; i < numPointsPerAxis; ++i) {
        costState->operator[](0u) = minX + stepX / 2.0 + i * stepX;
        for (std::size_t j = 0u; j < numPointsPerAxis; ++j) {
          costState->operator[](1u) = minY + stepY / 2.0 + j * stepY;

          // Compute the cost at this state.
          auto cost = objective.stateCost(costState);
          if (objective.isCostBetterThan(cost, ompl::base::Cost(minOptimizationCost_))) {
            minOptimizationCost_ = cost.value();
          }
          if (objective.isCostBetterThan(ompl::base::Cost(maxOptimizationCost_), cost)) {
            maxOptimizationCost_ = cost.value();
          }
        }
      }
    }

    // Iterate over the grid to visualize the resulting cost.
    for (std::size_t i = 0u; i < numPointsPerAxis; ++i) {
      double x = minX + stepX / 2.0 + i * stepX;
      costState->operator[](0u) = x;
      for (std::size_t j = 0u; j < numPointsPerAxis; ++j) {
        double y = minY + stepY / 2.0 + j * stepY;
        costState->operator[](1u) = y;

        // Compute the cost at this state.
        auto cost = objective.stateCost(costState).value();

        // Compute the color for this cost.
        auto color = interpolateColors(
            green, red,
            (cost - minOptimizationCost_) / (maxOptimizationCost_ - minOptimizationCost_));

        // Draw the rectangle.
        drawRectangle2D(std::vector<double>{x, y}, std::vector<double>{stepX, stepY}, color.data(),
                        color.data());
      }
    }

    context_->getStateSpace()->freeState(costState);
  } else {
    throw std::runtime_error(
        "PotentialFieldOptimizationObjective not yet implemented for 3D contexts.");
  }
}

void InteractiveVisualizer::visit(
    const MaxMinClearanceOptimizationObjective& /* objective */) const {
}

std::array<float, 4u> InteractiveVisualizer::interpolateColors(const float* color1,
                                                               const float* color2,
                                                               double t) const {
  std::array<float, 4u> color;
  for (std::size_t i = 0u; i < 4u; ++i) {
    color[i] = color1[i] + (color2[i] - color1[i]) * std::pow(t, 0.25);
  }
  return color;
}

void InteractiveVisualizer::drawPlannerSpecificVisualizations(std::size_t iteration) const {
  switch (plannerType_) {
    case PLANNER_TYPE::BITSTAR:
    case PLANNER_TYPE::SBITSTAR: {
      drawBITstarSpecificVisualizations(iteration);
      return;
    }
    case PLANNER_TYPE::TBDSTAR: {
      drawTBDstarSpecificVisualizations(iteration);
      return;
    }
    case PLANNER_TYPE::AIBITSTAR: {
      drawAIBITstarSpecificVisualizations(iteration);
      return;
    }
    default:
      return;
  }
}

void InteractiveVisualizer::drawBITstarSpecificVisualizations(std::size_t iteration) const {
  // Get the BIT* specific data.
  auto bitstarData =
      std::dynamic_pointer_cast<const BITstarData>(getPlannerSpecificData(iteration));
  if (context_->getDimension() == 2u) {
    // Get the edge queue.
    auto edgeQueue = bitstarData->getEdgeQueue();
    std::vector<Eigen::Vector2d> edges{};
    for (const auto& edge : edgeQueue) {
      auto parentState = edge.first->state()->as<ompl::base::RealVectorStateSpace::StateType>();
      edges.push_back(Eigen::Vector2d((*parentState)[0u], (*parentState)[1u]));
      auto childState = edge.second->state()->as<ompl::base::RealVectorStateSpace::StateType>();
      edges.push_back(Eigen::Vector2d((*childState)[0u], (*childState)[1u]));
    }

    // Draw the edge queue.
    drawLines(edges, 1.5, lightblue);

    // Get the next edge in the queue.
    auto nextEdgeStates = bitstarData->getNextEdge();

    // If there are no more edges in the queue, this will return nullptrs.
    if (nextEdgeStates.first == nullptr || nextEdgeStates.second == nullptr) {
      return;
    }
    auto parentState = nextEdgeStates.first->as<ompl::base::RealVectorStateSpace::StateType>();
    auto childState = nextEdgeStates.second->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<Eigen::Vector2d> nextEdge{Eigen::Vector2d((*parentState)[0u], (*parentState)[1u]),
                                          Eigen::Vector2d((*childState)[0u], (*childState)[1u])};

    // Draw the next edge.
    drawLines(nextEdge, 3.0, red);
  } else if (context_->getDimension() == 3u) {
    // Get the edge queue.
    auto edgeQueue = bitstarData->getEdgeQueue();
    std::vector<Eigen::Vector3d> edges{};
    for (const auto& edge : edgeQueue) {
      auto parentState = edge.first->state()->as<ompl::base::RealVectorStateSpace::StateType>();
      edges.push_back(Eigen::Vector3d((*parentState)[0u], (*parentState)[1u], (*parentState)[2u]));
      auto childState = edge.second->state()->as<ompl::base::RealVectorStateSpace::StateType>();
      edges.push_back(Eigen::Vector3d((*childState)[0u], (*childState)[1u], (*childState)[2u]));
    }

    // Draw the edge queue.
    drawLines(edges, 1.5, lightblue);

    // Get the next edge in the queue.
    auto nextEdgeStates = bitstarData->getNextEdge();

    // If there are no more edges in the queue, this will return nullptrs.
    if (nextEdgeStates.first == nullptr || nextEdgeStates.second == nullptr) {
      return;
    }
    auto parentState = nextEdgeStates.first->as<ompl::base::RealVectorStateSpace::StateType>();
    auto childState = nextEdgeStates.second->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<Eigen::Vector3d> nextEdge{
        Eigen::Vector3d((*parentState)[0u], (*parentState)[1u], (*parentState)[2u]),
        Eigen::Vector3d((*childState)[0u], (*childState)[1u], (*childState)[2u])};

    // Draw the next edge.
    drawLines(nextEdge, 3.0, red);
  } else {
    throw std::runtime_error(
        "BITstar specific visualizations only implemented for 2d or 3d contexts.");
  }
}

void InteractiveVisualizer::drawTBDstarSpecificVisualizations(std::size_t iteration) const {
  // Get the TBD* specific data.
  auto tbdstarData =
      std::dynamic_pointer_cast<const TBDstarData>(getPlannerSpecificData(iteration));
  if (context_->getDimension() == 2u) {
    // Get the edge queue.
    auto forwardQueue = tbdstarData->getForwardQueue();
    std::vector<Eigen::Vector2d> forwardQueueEdges;
    forwardQueueEdges.reserve(2u * forwardQueue.size());
    for (const auto& edge : forwardQueue) {
      auto parentState =
          edge.getParent()->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      forwardQueueEdges.emplace_back((*parentState)[0u], (*parentState)[1u]);
      auto childState =
          edge.getChild()->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      forwardQueueEdges.emplace_back((*childState)[0u], (*childState)[1u]);
    }

    drawLines(forwardQueueEdges, 1.5, lightblue);

    // Get the vertex queue.
    auto backwardQueue = tbdstarData->getBackwardQueue();
    std::vector<Eigen::Vector2d> backwardQueueVertices{};
    for (const auto& vertex : backwardQueue) {
      auto state = vertex->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      backwardQueueVertices.emplace_back((*state)[0u], (*state)[1u]);
    }

    drawPoints(backwardQueueVertices, yellow, 10.0);

    // Get the next vertex in the queue.
    auto nextVertex = tbdstarData->getNextVertex();
    if (nextVertex) {
      auto state = nextVertex->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      drawPoints(std::vector<Eigen::Vector2d>{Eigen::Vector2d((*state)[0u], (*state)[1u])}, red,
                 25.0);
    }

    // Draw the backward search tree.
    auto backwardSearchTree = tbdstarData->getVerticesInBackwardSearchTree();
    std::vector<Eigen::Vector2d> backwardSearchTreeEdges;
    for (const auto& vertex : backwardSearchTree) {
      // Add the edge to the parent.
      if (vertex->hasBackwardParent()) {
        auto state = vertex->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
        auto parent = vertex->getBackwardParent()
                          ->getState()
                          ->as<ompl::base::RealVectorStateSpace::StateType>();
        backwardSearchTreeEdges.emplace_back((*state)[0u], (*state)[1u]);
        backwardSearchTreeEdges.emplace_back((*parent)[0u], (*parent)[1u]);
      }
    }

    drawLines(backwardSearchTreeEdges, 1.0, yellow);

    // Get the next edge in the queue.
    auto nextEdgeStates = tbdstarData->getNextEdge();

    // If there are no more edges in the queue, this will return an edge with nullptrs.
    if (!nextEdgeStates.first || !nextEdgeStates.second) {
      return;
    }
    auto parentState = nextEdgeStates.first->as<ompl::base::RealVectorStateSpace::StateType>();
    auto childState = nextEdgeStates.second->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<Eigen::Vector2d> nextEdge{Eigen::Vector2d((*parentState)[0u], (*parentState)[1u]),
                                          Eigen::Vector2d((*childState)[0u], (*childState)[1u])};
    // Draw the next edge.
    drawLines(nextEdge, 3.0, red);
  } else if (context_->getDimension() == 3u) {
    // Get the edge queue.
    auto edgeQueue = tbdstarData->getForwardQueue();
    std::vector<Eigen::Vector3d> edges{};
    for (const auto& edge : edgeQueue) {
      auto parentState =
          edge.getParent()->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      edges.push_back(Eigen::Vector3d((*parentState)[0u], (*parentState)[1u], (*parentState)[2u]));
      auto childState =
          edge.getChild()->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      edges.push_back(Eigen::Vector3d((*childState)[0u], (*childState)[1u], (*childState)[2u]));
    }

    // Draw the edge queue.
    drawLines(edges, 1.5, lightblue);

    // Get the next edge in the queue.
    auto nextEdgeStates = tbdstarData->getNextEdge();

    // If there are no more edges in the queue, this will return an edge with nullptrs.
    if (!nextEdgeStates.first || !nextEdgeStates.second) {
      return;
    }
    auto parentState = nextEdgeStates.first->as<ompl::base::RealVectorStateSpace::StateType>();
    auto childState = nextEdgeStates.second->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<Eigen::Vector3d> nextEdgeVector{
        Eigen::Vector3d((*parentState)[0u], (*parentState)[1u], (*parentState)[2u]),
        Eigen::Vector3d((*childState)[0u], (*childState)[1u], (*childState)[2u])};
    // Draw the next edge.
    drawLines(nextEdgeVector, 3.0, red);
  }
}

void InteractiveVisualizer::drawAIBITstarSpecificVisualizations(std::size_t iteration) const {
  // Get the AIBIT* specific data.
  auto aibitstarData =
      std::dynamic_pointer_cast<const AIBITstarData>(getPlannerSpecificData(iteration));
  if (context_->getDimension() == 2u) {
    // Get the forward queue.
    auto forwardQueue = aibitstarData->getForwardQueue();
    std::vector<Eigen::Vector2d> forwardQueueEdges;
    forwardQueueEdges.reserve(2u * forwardQueue.size());
    for (const auto& edge : forwardQueue) {
      auto parentState = edge.parent->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      forwardQueueEdges.emplace_back((*parentState)[0u], (*parentState)[1u]);
      auto childState = edge.child->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      forwardQueueEdges.emplace_back((*childState)[0u], (*childState)[1u]);
    }

    // Draw the forward queue.
    drawLines(forwardQueueEdges, 1.5, lightblue);

    // Get the reverse queue.
    auto reverseQueue = aibitstarData->getReverseQueue();
    std::vector<Eigen::Vector2d> reverseQueueEdges;
    forwardQueueEdges.reserve(2u * reverseQueue.size());
    for (const auto& edge : reverseQueue) {
      auto parentState = edge.parent->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      reverseQueueEdges.emplace_back((*parentState)[0u], (*parentState)[1u]);
      auto childState = edge.child->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      reverseQueueEdges.emplace_back((*childState)[0u], (*childState)[1u]);
    }

    // Draw the reverse queue.
    drawLines(reverseQueueEdges, 1.5, yellow);

    // Get the reverse tree.
    auto reverseTree = aibitstarData->getReverseTree();
    std::vector<Eigen::Vector2d> reverseTreeEdges;
    reverseTreeEdges.reserve(2u * reverseTree.size());
    for (const auto& edge : reverseTree) {
      auto parentState = edge.parent->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      reverseTreeEdges.emplace_back((*parentState)[0u], (*parentState)[1u]);
      auto childState = edge.child->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      reverseTreeEdges.emplace_back((*childState)[0u], (*childState)[1u]);
    }

    // Draw the reverse tree.
    drawLines(reverseTreeEdges, 2.0, blue);

    // Get the next edge in the forward queue.
    auto nextForwardEdge = aibitstarData->getNextForwardEdge();

    // If there are no more edges in the queue, this will return an edge with nullptrs.
    if (nextForwardEdge.parent && nextForwardEdge.child) {
      std::vector<Eigen::Vector2d> nextEdge;
      auto parentState =
          nextForwardEdge.parent->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      auto childState =
          nextForwardEdge.child->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      nextEdge.emplace_back(parentState->operator[](0), parentState->operator[](1));
      nextEdge.emplace_back(childState->operator[](0), childState->operator[](1));
      // Draw the next edge.
      drawLines(nextEdge, 3.0, red);
    }

    // Get the next edge in the reverse queue.
    auto nextReverseEdge = aibitstarData->getNextReverseEdge();

    // If there are no more edges in the queue, this will return an edge with nullptrs.
    if (nextReverseEdge.parent && nextReverseEdge.child) {
      std::vector<Eigen::Vector2d> nextEdge;
      auto parentState =
          nextReverseEdge.parent->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      auto childState =
          nextReverseEdge.child->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      nextEdge.emplace_back(parentState->operator[](0), parentState->operator[](1));
      nextEdge.emplace_back(childState->operator[](0), childState->operator[](1));
      // Draw the next edge.
      drawLines(nextEdge, 3.0, darkred);
    }
  }
}

std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>>
InteractiveVisualizer::getVerticesAndEdges2D(std::size_t iteration) const {
  const auto& currentPlannerData = getPlannerData(iteration);
  // Get the vertices and edges in the format supported by Panglin.
  std::vector<Eigen::Vector2d> vertices{};
  std::vector<Eigen::Vector2d> edges{};  // Size must be multiple of two.
  for (std::size_t i = 0u; i < currentPlannerData->numVertices(); ++i) {
    auto vertex = currentPlannerData->getVertex(i);
    // Check the vertex is valid.
    if (vertex != ompl::base::PlannerData::NO_VERTEX) {
      const auto* vertexState =
          static_cast<const ompl::base::RealVectorStateSpace::StateType*>(vertex.getState());
      vertices.emplace_back(vertexState->values[0], vertexState->values[1]);

      // Get the outgoing edges of this vertex.
      std::vector<unsigned int> outgoingEdges{};
      currentPlannerData->getEdges(i, outgoingEdges);
      for (std::size_t j = 0; j < outgoingEdges.size(); ++j) {
        // Check that the vertex is valid.
        auto child = currentPlannerData->getVertex(outgoingEdges.at(j));
        if (child != ompl::base::PlannerData::NO_VERTEX) {
          // Add the parent.
          edges.emplace_back(vertexState->values[0], vertexState->values[1]);

          // Add the child.
          const auto* childState =
              static_cast<const ompl::base::RealVectorStateSpace::StateType*>(child.getState());
          edges.emplace_back(childState->values[0], childState->values[1]);
        }
      }
    }
  }
  return {vertices, edges};
}

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>
InteractiveVisualizer::getVerticesAndEdges3D(std::size_t iteration) const {
  const auto& currentPlannerData = getPlannerData(iteration);
  // Get the vertices and edges in the format supported by Panglin.
  std::vector<Eigen::Vector3d> vertices{};
  std::vector<Eigen::Vector3d> edges{};  // Size must be multiple of two.
  for (std::size_t i = 0u; i < currentPlannerData->numVertices(); ++i) {
    auto vertex = currentPlannerData->getVertex(i);
    // Check the vertex is valid.
    if (vertex != ompl::base::PlannerData::NO_VERTEX) {
      const auto* vertexState =
          static_cast<const ompl::base::RealVectorStateSpace::StateType*>(vertex.getState());
      vertices.emplace_back(vertexState->values[0], vertexState->values[1], vertexState->values[2]);

      // Get the outgoing edges of this vertex.
      std::vector<unsigned int> outgoingEdges{};
      currentPlannerData->getEdges(i, outgoingEdges);
      for (std::size_t j = 0; j < outgoingEdges.size(); ++j) {
        // Check that the vertex is valid.
        auto child = currentPlannerData->getVertex(outgoingEdges.at(j));
        if (child != ompl::base::PlannerData::NO_VERTEX) {
          // Add the parent.
          edges.emplace_back(vertexState->values[0], vertexState->values[1],
                             vertexState->values[2]);

          // Add the child.
          const auto* childState =
              static_cast<const ompl::base::RealVectorStateSpace::StateType*>(child.getState());
          edges.emplace_back(childState->values[0], childState->values[1], childState->values[2]);
        }
      }
    }
  }
  return {vertices, edges};
}

std::vector<Eigen::Vector2d> InteractiveVisualizer::getVertices2D(std::size_t iteration) const {
  const auto& currentPlannerData = getPlannerData(iteration);
  // Get the vertices in a format supported by Pangolin.
  std::vector<Eigen::Vector2d> vertices{};
  for (std::size_t i = 0u; i < currentPlannerData->numVertices(); ++i) {
    auto vertex = currentPlannerData->getVertex(i);
    // Check the vertex is valid.
    if (vertex != ompl::base::PlannerData::NO_VERTEX) {
      const auto* vertexState =
          static_cast<const ompl::base::RealVectorStateSpace::StateType*>(vertex.getState());
      vertices.emplace_back(vertexState->values[0], vertexState->values[1]);
    }
  }
  return vertices;
}

std::vector<Eigen::Vector3d> InteractiveVisualizer::getVertices3D(std::size_t iteration) const {
  const auto& currentPlannerData = getPlannerData(iteration);
  // Get the vertices in a format supported by Pangolin.
  std::vector<Eigen::Vector3d> vertices{};
  for (std::size_t i = 0u; i < currentPlannerData->numVertices(); ++i) {
    auto vertex = currentPlannerData->getVertex(i);
    // Check the vertex is valid.
    if (vertex != ompl::base::PlannerData::NO_VERTEX) {
      const auto* vertexState =
          static_cast<const ompl::base::RealVectorStateSpace::StateType*>(vertex.getState());
      vertices.emplace_back(vertexState->values[0], vertexState->values[1], vertexState->values[2]);
    }
  }
  return vertices;
}

std::vector<Eigen::Vector2d> InteractiveVisualizer::getEdges2D(std::size_t iteration) const {
  const auto& currentPlannerData = getPlannerData(iteration);
  std::vector<Eigen::Vector2d> edges{};  // Size must be multiple of two.
  for (std::size_t i = 0u; i < currentPlannerData->numVertices(); ++i) {
    auto parent = currentPlannerData->getVertex(i);
    // Check the vertex is valid.
    if (parent != ompl::base::PlannerData::NO_VERTEX) {
      const auto* parentState =
          static_cast<const ompl::base::RealVectorStateSpace::StateType*>(parent.getState());

      // Get the outgoing edges of this vertex.
      std::vector<unsigned int> outgoingEdges{};
      currentPlannerData->getEdges(i, outgoingEdges);
      for (std::size_t j = 0; j < outgoingEdges.size(); ++j) {
        // Check that the vertex is valid.
        auto child = currentPlannerData->getVertex(outgoingEdges.at(j));
        if (child != ompl::base::PlannerData::NO_VERTEX) {
          // Add the parent.
          edges.emplace_back(parentState->values[0], parentState->values[1]);

          // Add the child.
          const auto* childState =
              static_cast<const ompl::base::RealVectorStateSpace::StateType*>(child.getState());
          edges.emplace_back(childState->values[0], childState->values[1]);
        }
      }
    }
  }
  return edges;
}

std::vector<Eigen::Vector3d> InteractiveVisualizer::getEdges3D(std::size_t iteration) const {
  const auto& currentPlannerData = getPlannerData(iteration);
  std::vector<Eigen::Vector3d> edges{};  // Size must be multiple of two.
  for (std::size_t i = 0u; i < currentPlannerData->numVertices(); ++i) {
    auto parent = currentPlannerData->getVertex(i);
    // Check the vertex is valid.
    if (parent != ompl::base::PlannerData::NO_VERTEX) {
      const auto* parentState =
          static_cast<const ompl::base::RealVectorStateSpace::StateType*>(parent.getState());

      // Get the outgoing edges of this vertex.
      std::vector<unsigned int> outgoingEdges{};
      currentPlannerData->getEdges(i, outgoingEdges);
      for (std::size_t j = 0; j < outgoingEdges.size(); ++j) {
        // Check that the vertex is valid.
        auto child = currentPlannerData->getVertex(outgoingEdges.at(j));
        if (child != ompl::base::PlannerData::NO_VERTEX) {
          // Add the parent.
          edges.emplace_back(parentState->values[0], parentState->values[1],
                             parentState->values[2]);

          // Add the child.
          const auto* childState =
              static_cast<const ompl::base::RealVectorStateSpace::StateType*>(child.getState());
          edges.emplace_back(childState->values[0], childState->values[1], childState->values[2]);
        }
      }
    }
  }
  return edges;
}

std::vector<Eigen::Vector2d> InteractiveVisualizer::getPath2D(std::size_t iteration) const {
  std::vector<Eigen::Vector2d> points{};
  auto solution = getSolutionPath(iteration);
  if (solution != nullptr) {
    auto path = solution->as<ompl::geometric::PathGeometric>()->getStates();
    points.reserve(path.size());
    for (const auto state : path) {
      const auto* rstate = static_cast<const ompl::base::RealVectorStateSpace::StateType*>(state);
      points.emplace_back(rstate->values[0], rstate->values[1]);
    }
  }
  return points;
}

std::vector<Eigen::Vector3d> InteractiveVisualizer::getPath3D(std::size_t iteration) const {
  std::vector<Eigen::Vector3d> points{};
  auto solution = getSolutionPath(iteration);
  if (solution != nullptr) {
    auto path = solution->as<ompl::geometric::PathGeometric>()->getStates();
    points.reserve(path.size());
    for (const auto state : path) {
      const auto* rstate = static_cast<const ompl::base::RealVectorStateSpace::StateType*>(state);
      points.emplace_back(rstate->values[0], rstate->values[1], rstate->values[2]);
    }
  }
  return points;
}

}  // namespace ompltools

}  // namespace esp
