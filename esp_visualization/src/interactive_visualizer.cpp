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

namespace esp {

namespace ompltools {

InteractiveVisualizer::InteractiveVisualizer(
    const std::shared_ptr<BaseContext>& context,
    const std::pair<std::shared_ptr<ompl::base::Planner>, PLANNER_TYPE> plannerPair) :
    BaseVisualizer(context, plannerPair) {
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
  if (bounds.size() == 2u) {
    contextView.SetBounds(
        pangolin::Attach::Pix(205), pangolin::Attach::ReversePix(5), pangolin::Attach::Pix(205),
        pangolin::Attach::ReversePix(5),
        (bounds.at(0).second - bounds.at(0).first) / (bounds.at(1).second - bounds.at(1).first));
  } else {
    contextView.SetBounds(
        pangolin::Attach::Pix(205), pangolin::Attach::ReversePix(5), pangolin::Attach::Pix(205),
        pangolin::Attach::ReversePix(5),
        -(bounds.at(0).second - bounds.at(0).first) / (bounds.at(1).second - bounds.at(1).first));
  }
  plotView.SetBounds(pangolin::Attach::Pix(5), pangolin::Attach::Pix(200),
                     pangolin::Attach::Pix(205), pangolin::Attach::ReversePix(5));
  pangolin::Plotter plotter(&costLog_, 0.0, 1.0, 0.0, 5.0);
  plotter.SetBackgroundColour(pangolin::Colour::White());
  plotView.AddDisplay(plotter);

  // Create an OpenGL render state. This controls how coordinates are
  // processed before they are drawn in OpenGL's [-1, 1] x [-1, 1] coordinates.
  pangolin::OpenGlRenderState renderState;

  // We can instantiate a handler used to modify the view in 3D.
  pangolin::Handler3D handler(renderState);

  // Get the bounds of the context to setup a correct renderstate.
  if (context_->getDimensions() == 2u) {
    glShadeModel(GL_FLAT);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    renderState.SetProjectionMatrix(pangolin::ProjectionMatrixOrthographic(
        bounds.at(0).first,   // The left boundary of the problem
        bounds.at(0).second,  // The right boundary of the problem
        bounds.at(1).first,   // The bottom boundary of the porblem
        bounds.at(1).second,  // The top boundary of the problem
        -1.0f,                // Shouldn't have to change this in 2D
        1.0f                  // Shouldn't have to change this in 2D
        ));
  } else if (context_->getDimensions() == 3u) {
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
  pangolin::Var<bool> optionDrawContext(optionsName + ".Draw Context", true, true);
  pangolin::Var<bool> optionDrawObstacles(optionsName + ".Draw Obstacles", true, true);
  pangolin::Var<bool> optionDrawVertices(optionsName + ".Draw Vertices", true, true);
  pangolin::Var<bool> optionDrawEdges(optionsName + ".Draw Edges", true, true);
  pangolin::Var<bool> optionDrawSolution(optionsName + ".Draw Solution", true, true);
  pangolin::Var<bool> optionTrack(optionsName + ".Track", true, true);
  // Buttons.
  pangolin::Var<bool> optionScreenshot(optionsName + ".Screenshot Iteration", false, false);
  pangolin::Var<double> optionSlowdown(optionsName + ".Replay Factor", 1, 1e-3, 1e1, true);
  pangolin::Var<bool> optionPlay(optionsName + ".Play to Iteration", false, false);
  pangolin::Var<bool> optionRecord(optionsName + ".Record to Iteration", false, false);

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

  maxCost_ = context_->computeMinPossibleCost().value();
  minCost_ = context_->computeMinPossibleCost().value();
  while (!pangolin::ShouldQuit()) {
    // Register input.
    if (pangolin::Pushed(optionScreenshot)) {
      contextView.SaveOnRender("screenshot" + std::to_string(screenshotId_++) + '_' +
                               context_->getName() + '_' + planner_->getName());
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

    // Set values if we're playing in realtime.
    if (playToIteration_) {
      if (displayIteration_ >= iterationToPlayTo_) {
        displayIteration_ = iterationToPlayTo_;
        playToIteration_ = false;
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

    // Activate this render state for the canvas.
    contextView.Activate(renderState);

    // Clear the viewport.
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Only draw obstacles if context is drawn.
    if (!optionDrawContext) {
      optionDrawObstacles = false;
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
      if (context_->getDimensions() == 2u) {
        auto [vertices, edges] = getVerticesAndEdges2D(displayIteration_);
        drawPoints(vertices, blue, 2.0);
        drawLines(edges, 1.0, gray);
      } else if (context_->getDimensions() == 3u) {
        auto [vertices, edges] = getVerticesAndEdges3D(displayIteration_);
        drawPoints(vertices, blue, 2.0);
        drawLines(edges, 1.0, gray, 0.8);
      }
    } else if (optionDrawVertices) {
      if (context_->getDimensions() == 2u) {
        auto vertices = getVertices2D(displayIteration_);
        drawPoints(vertices, blue, 2.0);
      } else if (context_->getDimensions() == 3u) {
        auto vertices = getVertices3D(displayIteration_);
        drawPoints(vertices, blue, 2.0);
      }
    } else if (optionDrawEdges) {
      if (context_->getDimensions() == 2u) {
        auto edges = getEdges2D(displayIteration_);
        drawLines(edges, 1.0, gray);
      } else if (context_->getDimensions() == 3u) {
        auto edges = getEdges3D(displayIteration_);
        drawLines(edges, 1.0, gray, 0.8);
      }
    }

    // Draw the solution.
    if (optionDrawSolution) {
      if (context_->getDimensions() == 2u) {
        auto path = getPath2D(displayIteration_);
        drawPath(path, 3.0, purple);
      } else if (context_->getDimensions() == 3u) {
        auto path = getPath3D(displayIteration_);
        drawPath(path, 3.0, purple, 1.0);
      }
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

void InteractiveVisualizer::visit(const CentreSquare& context) const {
  // Draw the start states.
  drawPoints(context.getStartStates(), red, 5.0);
  // Draw the goal states.
  drawPoints(context.getGoalStates(), green, 5.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const DividingWalls& context) const {
  // Draw the start states.
  drawPoints(context.getStartStates(), red, 5.0);
  // Draw the goal states.
  drawPoints(context.getGoalStates(), green, 5.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const DoubleEnclosure& context) const {
  // Draw the start states.
  drawPoints(context.getStartStates(), red, 5.0);
  // Draw the goal states.
  drawPoints(context.getGoalStates(), green, 5.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const FlankingGap& context) const {
  // Draw the start states.
  drawPoints(context.getStartStates(), red, 5.0);
  // Draw the goal states.
  drawPoints(context.getGoalStates(), green, 5.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const GoalEnclosure& context) const {
  // Draw the start states.
  drawPoints(context.getStartStates(), red, 5.0);
  // Draw the goal states.
  drawPoints(context.getGoalStates(), green, 5.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const ObstacleFree& context) const {
  // Draw the start states.
  drawPoints(context.getStartStates(), red, 5.0);
  // Draw the goal states.
  drawPoints(context.getGoalStates(), green, 5.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const RandomRectangles& context) const {
  // Draw the start states.
  drawPoints(context.getStartStates(), red, 5.0);
  // Draw the goal states.
  drawPoints(context.getGoalStates(), green, 5.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const RandomRectanglesMultiStartGoal& context) const {
  // Draw the start states.
  drawPoints(context.getStartStates(), red, 5.0);
  // Draw the goal states.
  drawPoints(context.getGoalStates(), green, 5.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const RepeatingRectangles& context) const {
  // Draw the start states.
  drawPoints(context.getStartStates(), red, 5.0);
  // Draw the goal states.
  drawPoints(context.getGoalStates(), green, 5.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const StartEnclosure& context) const {
  // Draw the start states.
  drawPoints(context.getStartStates(), red, 5.0);
  // Draw the goal states.
  drawPoints(context.getGoalStates(), green, 5.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::visit(const WallGap& context) const {
  // Draw the start states.
  drawPoints(context.getStartStates(), red, 5.0);
  // Draw the goal states.
  drawPoints(context.getGoalStates(), green, 5.0);
  // Draw the boundaries.
  drawBoundary(context);
}

void InteractiveVisualizer::drawBoundary(const BaseContext& context) const {
  glColor4fv(black);
  glLineWidth(3.0);
  if (context.getDimensions() == 2u) {
    auto bounds = context.getBoundaries();
    pangolin::glDrawRectPerimeter(bounds.at(0).first, bounds.at(1).first, bounds.at(0).second,
                                  bounds.at(1).second);
  } else if (context.getDimensions() == 3u) {
    auto bounds = context.getBoundaries();
    std::vector<Eigen::Vector3d> points{
        {bounds.at(0).first, bounds.at(1).first, bounds.at(2).first},
        {bounds.at(0).first, bounds.at(1).first, bounds.at(2).second},
        {bounds.at(0).first, bounds.at(1).first, bounds.at(2).first},
        {bounds.at(0).first, bounds.at(1).second, bounds.at(2).first},
        {bounds.at(0).first, bounds.at(1).first, bounds.at(2).first},
        {bounds.at(0).second, bounds.at(1).first, bounds.at(2).first},
        {bounds.at(0).second, bounds.at(1).second, bounds.at(2).second},
        {bounds.at(0).second, bounds.at(1).second, bounds.at(2).first},
        {bounds.at(0).second, bounds.at(1).second, bounds.at(2).second},
        {bounds.at(0).second, bounds.at(1).first, bounds.at(2).second},
        {bounds.at(0).second, bounds.at(1).second, bounds.at(2).second},
        {bounds.at(0).first, bounds.at(1).second, bounds.at(2).second},
        {bounds.at(0).first, bounds.at(1).first, bounds.at(2).second},
        {bounds.at(0).first, bounds.at(1).second, bounds.at(2).second},
        {bounds.at(0).first, bounds.at(1).first, bounds.at(2).second},
        {bounds.at(0).second, bounds.at(1).first, bounds.at(2).second},
        {bounds.at(0).first, bounds.at(1).second, bounds.at(2).first},
        {bounds.at(0).first, bounds.at(1).second, bounds.at(2).second},
        {bounds.at(0).first, bounds.at(1).second, bounds.at(2).first},
        {bounds.at(0).second, bounds.at(1).second, bounds.at(2).first},
        {bounds.at(0).second, bounds.at(1).first, bounds.at(2).first},
        {bounds.at(0).second, bounds.at(1).second, bounds.at(2).first},
        {bounds.at(0).second, bounds.at(1).first, bounds.at(2).first},
        {bounds.at(0).second, bounds.at(1).first, bounds.at(2).second},
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

void InteractiveVisualizer::drawPoints(const std::vector<Eigen::Vector2d>& points,
                                       const float* color, float size) const {
  glColor4fv(color);
  glPointSize(size);
  pangolin::glDrawPoints(points);
}

void InteractiveVisualizer::drawPoints(const std::vector<Eigen::Vector3d>& points,
                                       const float* color, float size) const {
  glColor4fv(color);
  glPointSize(size);
  pangolin::glDrawPoints(points);
}

void InteractiveVisualizer::drawPoints(const std::vector<ompl::base::ScopedState<>>& states,
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
  drawRectangle(obstacle.getAnchorCoordinates(), obstacle.getWidths(), black, gray);
}

void InteractiveVisualizer::visit(const Hyperrectangle<BaseAntiObstacle>& antiObstacle) const {
  drawRectangle(antiObstacle.getAnchorCoordinates(), antiObstacle.getWidths(), white, white);
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
