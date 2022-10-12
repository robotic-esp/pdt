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

#include "pdt/visualization/interactive_visualizer.h"

#include <ompl/base/goals/GoalSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/informedtrees/bitstar/Vertex.h>

#include "pdt/visualization/fonts.h"

namespace pdt {

namespace visualization {

InteractiveVisualizer::InteractiveVisualizer(
    const std::shared_ptr<config::Configuration>& config,
    const std::shared_ptr<planning_contexts::BaseContext>& context,
    const std::pair<std::shared_ptr<ompl::base::Planner>, common::PLANNER_TYPE> plannerPair) :
    BaseVisualizer(config, context, plannerPair),
    bounds_(2u),
    tikzVisualizer_(config, context, plannerPair),
    config_(config) {
  if (context_->getStateSpace()->getType() != ompl::base::StateSpaceType::STATE_SPACE_REAL_VECTOR &&
      context_->getStateSpace()->getType() != ompl::base::StateSpaceType::STATE_SPACE_SE2 &&
      context_->getStateSpace()->getType() != ompl::base::StateSpaceType::STATE_SPACE_REEDS_SHEPP) {
    OMPL_ERROR("Visualizer only tested for real vector and SE(2) state spaces.");
    throw std::runtime_error("Visualizer error.");
  }

  // Get the bounds of the real vector part.
  const auto vectorContext =
      std::dynamic_pointer_cast<planning_contexts::RealVectorGeometricContext>(context_);
  const auto se2Context =
      std::dynamic_pointer_cast<planning_contexts::ReedsSheppRandomRectangles>(context_);
  if (vectorContext) {
    bounds_ = vectorContext->getBoundaries();
  } else if (se2Context) {
    bounds_ = se2Context->getBoundaries();
  } else {
    std::runtime_error("Interactive visualizer can only handle real vector and SE2 state spaces.");
  }
}

void InteractiveVisualizer::run() {
  // The default window width and height.
  constexpr std::size_t windowWidth = 800;
  constexpr std::size_t windowHeight = 800;

  costLog_.SetLabels(std::vector<std::string>({planner_->getName()}));

  // Create a window and bind it to the current OpenGL context.
  pangolin::CreateWindowAndBind("PDT Interactive Visualizer", windowWidth, windowHeight);

  // Set up a viewport. A viewport is where OpenGL draws to. We can
  // have multiple viewports for different parts of a window. Any draw
  // command will draw to the active viewport.
  pangolin::View& contextView = pangolin::CreateDisplay();
  pangolin::View& plotView = pangolin::CreateDisplay();

  // We can set the bounds of this viewport. The coordinates are with
  // respect to the window boundaries (0 is left/bottom, 1 is right/top).
  // We can also set an aspect ratio such that even if the window gets
  // rescaled the drawings in the viewport do not get distorted.
  const auto dimension = bounds_.low.size();
  if (dimension == 2u) {
    contextView.SetBounds(
        pangolin::Attach::Pix(205), pangolin::Attach::ReversePix(5), pangolin::Attach::Pix(205),
        pangolin::Attach::ReversePix(5),
        (bounds_.high.at(0) - bounds_.low.at(0)) / (bounds_.high.at(1) - bounds_.low.at(1)));
  } else {
    contextView.SetBounds(
        pangolin::Attach::Pix(205), pangolin::Attach::ReversePix(5), pangolin::Attach::Pix(205),
        pangolin::Attach::ReversePix(5),
        -(bounds_.high.at(0) - bounds_.low.at(0)) / (bounds_.high.at(1) - bounds_.low.at(1)));
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
  if (dimension == 2u) {
    // glShadeModel(GL_FLAT);
    // glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    // glDisable(GL_DEPTH_TEST);
    // glDisable(GL_LIGHTING);
    renderState.SetProjectionMatrix(pangolin::ProjectionMatrixOrthographic(
        bounds_.low.at(0),   // The left boundary of the problem
        bounds_.high.at(0),  // The right boundary of the problem
        bounds_.low.at(1),   // The bottom boundary of the porblem
        bounds_.high.at(1),  // The top boundary of the problem
        -1.0f,               // Shouldn't have to change this in 2D
        1.0f                 // Shouldn't have to change this in 2D
        ));
  } else if (dimension == 3u) {
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
  pangolin::Var<bool> optionDrawStateIds(optionsName + ".State IDs", false, true);
  pangolin::Var<bool> optionTrack(optionsName + ".Track", false, true);
  // Buttons.
  pangolin::Var<bool> optionScreenshot(optionsName + ".Screenshot", false, false);
  pangolin::Var<bool> optionTikzshot(optionsName + ".TikZshot", false, false);
  pangolin::Var<double> optionSlowdown(optionsName + ".Replay Factor", 1, 1e-3, 1e2, true);
  pangolin::Var<bool> optionPlay(optionsName + ".Play", false, false);
  pangolin::Var<bool> optionExport(optionsName + ".Export", false, false);

  // Register some keypresses.
  pangolin::RegisterKeyPressCallback('f', [this]() { incrementIteration(1u); });
  pangolin::RegisterKeyPressCallback('b', [this]() { decrementIteration(1u); });
  pangolin::RegisterKeyPressCallback('F', [this]() {
    incrementIteration(
        static_cast<std::size_t>(std::ceil(0.1 * static_cast<double>(largestIteration_))));
  });
  pangolin::RegisterKeyPressCallback('B', [this]() {
    decrementIteration(
        static_cast<std::size_t>(std::ceil(0.1 * static_cast<double>(largestIteration_))));
  });
  pangolin::RegisterKeyPressCallback(' ', [&optionTrack]() { optionTrack = !optionTrack; });

  // This sets the color used when clearing the screen.
  glClearColor(1.0, 1.0, 1.0, 1.0);

  maxCost_ = 0.1f;
  minCost_ = 0.0f;

  // Give the data thread a head start.
  std::this_thread::sleep_for(std::chrono::milliseconds(10u));

  while (!pangolin::ShouldQuit()) {
    // Compute the time spent at current query by finding the iteration at which we started
    // with the current query, and taking the difference in total times.
    // This could be done much more efficiently.
    double timeAtCurrentQuery = getTotalElapsedDuration(displayIteration_).count();
    const std::size_t queryNumber = getQueryNumber(displayIteration_);

    if (queryNumber > 0u) {
      for (std::size_t i = displayIteration_; i >= 1u; --i) {
        if (BaseVisualizer::getQueryNumber(i) != queryNumber) {
          timeAtCurrentQuery -= getTotalElapsedDuration(i).count();
          break;
        }
      }
    }
    if (displayIteration_ != lastDisplayIteration_) {
      std::cout << "Iteration: " << displayIteration_
                << ", query: " << getQueryNumber(displayIteration_)
                << ", time: " << timeAtCurrentQuery
                << ", time: " << getTotalElapsedDuration(displayIteration_).count()
                << ", cost: " << getSolutionCost(displayIteration_).value() << '\n';
      lastDisplayIteration_ = displayIteration_;
    }
    // Register input.
    if (pangolin::Pushed(optionScreenshot)) {
      pangolin::SaveWindowOnRender(std::to_string(screencaptureId_++) + '_' + context_->getName() +
                                   '_' + planner_->getName());
    }
    if (pangolin::Pushed(optionTikzshot)) {
      if (optionDrawPlannerSpecificData) {
        tikzVisualizer_.render(*getPlannerData(displayIteration_), displayIteration_,
                               getQueryNumber(displayIteration_),
                               getSolutionPath(displayIteration_),
                               getPlannerSpecificData(displayIteration_),
                               getIterationDuration(displayIteration_).count(), timeAtCurrentQuery,
                               getSolutionCost(displayIteration_).value());
      } else {
        tikzVisualizer_.render(*getPlannerData(displayIteration_), displayIteration_,
                               getQueryNumber(displayIteration_),
                               getSolutionPath(displayIteration_), nullptr,
                               getIterationDuration(displayIteration_).count(), timeAtCurrentQuery,
                               getSolutionCost(displayIteration_).value());
      }
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
                                 getQueryNumber(displayIteration_),
                                 getSolutionPath(displayIteration_),
                                 getPlannerSpecificData(displayIteration_),
                                 getIterationDuration(displayIteration_).count(),
                                 timeAtCurrentQuery, getSolutionCost(displayIteration_).value());
        } else {
          tikzVisualizer_.render(*getPlannerData(displayIteration_), displayIteration_,
                                 getQueryNumber(displayIteration_),
                                 getSolutionPath(displayIteration_), nullptr,
                                 getIterationDuration(displayIteration_).count(),
                                 timeAtCurrentQuery, getSolutionCost(displayIteration_).value());
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
      if (auto objective = std::dynamic_pointer_cast<objectives::BaseOptimizationObjective>(
              context_->getObjective())) {
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

    // Draw the start states.
    drawStarts();
    // Draw the goal states.
    drawGoal();
    // Draw the boundaries.
    drawBoundary();

    // Draw the state ids.
    if (optionDrawStateIds) {
      drawStateIds(displayIteration_);
    }

    // Set the correct view of the plot.
    plotter.SetView(pangolin::XYRangef(0, 1.06f * static_cast<float>(largestPlottedIteration_),
                                       0.98f * minCost_, 1.02f * maxCost_));
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
    auto path = getSolutionPath(static_cast<std::size_t>(++largestPlottedIteration_))
                    ->as<ompl::geometric::PathGeometric>();
    if (path != nullptr) {
      float cost = static_cast<float>(
          path->cost(planner_->getProblemDefinition()->getOptimizationObjective()).value());
      if (cost > maxCost_) {
        maxCost_ = cost;
      }
      if (cost < minCost_) {
        minCost_ = cost;
      }
      costLog_.Log(cost);
    } else {
      costLog_.Log(std::numeric_limits<float>::max());
    }
  }
}

void InteractiveVisualizer::drawVerticesAndEdges(std::size_t iteration) {
  if (bounds_.low.size() == 2u) {
    auto [vertices, edges] = getVerticesAndEdges2D(iteration);
    drawPoints(vertices, blue, 2.0f);
    drawLines(edges, 3.0f, gray);
  } else if (bounds_.low.size() == 3u) {
    auto [vertices, edges] = getVerticesAndEdges3D(iteration);
    drawPoints(vertices, blue, 2.0f);
    drawLines(edges, 3.0f, gray, 0.8f);
  }
}

void InteractiveVisualizer::drawVertices(std::size_t iteration) {
  if (bounds_.low.size() == 2u) {
    auto vertices = getVertices2D(iteration);
    drawPoints(vertices, blue, 2.0);
  } else if (bounds_.low.size() == 3u) {
    auto vertices = getVertices3D(iteration);
    drawPoints(vertices, blue, 2.0);
  }
}

void InteractiveVisualizer::drawEdges(std::size_t iteration) {
  if (bounds_.low.size() == 2u) {
    auto edges = getEdges2D(iteration);
    drawLines(edges, 3.0f, gray);
  } else if (bounds_.low.size() == 3u) {
    auto edges = getEdges3D(iteration);
    drawLines(edges, 3.0f, gray, 0.8f);
  }
}

void InteractiveVisualizer::drawSolution(std::size_t iteration) {
  if (context_->getStateSpace()->getType() == ompl::base::StateSpaceType::STATE_SPACE_REAL_VECTOR) {
    if (bounds_.low.size() == 2u) {
      auto path = getPath2D(iteration);
      drawPath(path, 3.0, purple);
    } else if (bounds_.low.size() == 3u) {
      auto path = getPath3D(iteration);
      drawPath(path, 3.0, purple, 1.0);
    }
  } else if (context_->getStateSpace()->getType() ==
             ompl::base::StateSpaceType::STATE_SPACE_REEDS_SHEPP) {
    auto path = getPathSE2(iteration);
    drawCars(path, 1.0, purple);
  } else {
    throw std::runtime_error("Don't know how to draw the solution for given state space.");
  }
}

void InteractiveVisualizer::drawStateIds(std::size_t iteration) {
  // We must enable alpha blending for text rendering to work with pangolin.
  GLboolean glBlendEnabled{false};
  glGetBooleanv(GL_BLEND, &glBlendEnabled);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  const auto& plannerData = getPlannerData(iteration);
  pangolin::GlFont font(Fonts::ROBOTO_REGULAR.string(), 16);
  for (auto i = 0u; i < plannerData->numVertices(); ++i) {
    auto vertex = plannerData->getVertex(i);
    if (vertex != ompl::base::PlannerData::NO_VERTEX) {
      auto vertexState = vertex.getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      glColor3f(1.0f, 0.2f, 0.4f);
      pangolin::GlText txt = font.Text(std::to_string(vertex.getTag()).c_str());
      txt.Draw(static_cast<float>(vertexState->values[0]),
               static_cast<float>(vertexState->values[1]));
    }
  }

  if (!glBlendEnabled) {
    glDisable(GL_BLEND);
  }
}

void InteractiveVisualizer::visit(const planning_contexts::CenterSquare& /* context */) const {
}

void InteractiveVisualizer::visit(const planning_contexts::DividingWalls& /* context */) const {
}

void InteractiveVisualizer::visit(const planning_contexts::DoubleEnclosure& /* context */) const {
}

void InteractiveVisualizer::visit(const planning_contexts::FlankingGap& /* context */) const {
}

void InteractiveVisualizer::visit(const planning_contexts::FourRooms& /* context */) const {
}

void InteractiveVisualizer::visit(const planning_contexts::GoalEnclosure& /* context */) const {
}

void InteractiveVisualizer::visit(const planning_contexts::NarrowPassage& /* context */) const {
}

void InteractiveVisualizer::visit(const planning_contexts::ObstacleFree& /* context */) const {
}

void InteractiveVisualizer::visit(const planning_contexts::RandomRectangles& /* context */) const {
}

void InteractiveVisualizer::visit(
    const planning_contexts::RandomRectanglesMultiStartGoal& /* context */) const {
}

void InteractiveVisualizer::visit(
    const planning_contexts::ReedsSheppRandomRectangles& /* context */) const {
}

void InteractiveVisualizer::visit(
    const planning_contexts::RepeatingRectangles& /* context */) const {
}

void InteractiveVisualizer::visit(const planning_contexts::StartEnclosure& /* context */) const {
}

void InteractiveVisualizer::visit(const planning_contexts::WallGap& /* context */) const {
}

void InteractiveVisualizer::drawStarts() const {
  const auto& starts =
      context_->getNthStartGoalPair(BaseVisualizer::getQueryNumber(displayIteration_)).start;

  for (const auto& start : starts) {
    drawPoint(start, green, 4.0);
  }
}

void InteractiveVisualizer::drawGoal() const {
  const auto& goal =
      context_->getNthStartGoalPair(BaseVisualizer::getQueryNumber(displayIteration_)).goal;

  switch (goal->getType()) {
    case ompl::base::GoalType::GOAL_STATE: {
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goalState(
          context_->getStateSpace(), goal->as<ompl::base::GoalState>()->getState());
      drawPoint(goalState, red, 4.0);
      break;
    }
    case ompl::base::GoalType::GOAL_STATES: {
      for (auto i = 0u; i < goal->as<ompl::base::GoalStates>()->getStateCount(); ++i) {
        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goalState(
            context_->getStateSpace(), goal->as<ompl::base::GoalStates>()->getState(i));
        drawPoint(goalState, red, 5.0);
      }
      break;
    }
#ifdef PDT_EXTRA_GOAL_SPACE
    case ompl::base::GoalType::GOAL_SPACE: {
      float goalColor[4] = {0.808f, 0.243f, 0.082f, 0.3f};
      auto goalSpace = goal->as<ompl::base::GoalSpace>()->getSpace();
      auto bounds = goalSpace->as<ompl::base::RealVectorStateSpace>()->getBounds();
      std::vector<float> widths;
      std::vector<float> anchor;
      for (auto i = 0u; i < goalSpace->getDimension(); ++i) {
        widths.emplace_back(bounds.high[i] - bounds.low[i]);
        anchor.emplace_back((bounds.high[i] + bounds.low[i]) / 2.0f);
      }
      drawRectangle(anchor, widths, goalColor, goalColor);
      break;
    }
#endif  // #ifdef PDT_EXTRA_GOAL_SPACE
    default: { throw std::runtime_error("Can not visualize goal type."); }
  }
}

void InteractiveVisualizer::drawBoundary() const {
  glColor4fv(black);
  const auto dimension = bounds_.low.size();
  if (dimension == 2u) {
    pangolin::glDrawRectPerimeter(
        static_cast<float>(bounds_.low.at(0)), static_cast<float>(bounds_.low.at(1)),
        static_cast<float>(bounds_.high.at(0)), static_cast<float>(bounds_.high.at(1)));
  } else if (dimension == 3u) {
    std::vector<Eigen::Vector3d> points{
        {bounds_.low.at(0), bounds_.low.at(1), bounds_.low.at(2)},
        {bounds_.low.at(0), bounds_.low.at(1), bounds_.high.at(2)},
        {bounds_.low.at(0), bounds_.low.at(1), bounds_.low.at(2)},
        {bounds_.low.at(0), bounds_.high.at(1), bounds_.low.at(2)},
        {bounds_.low.at(0), bounds_.low.at(1), bounds_.low.at(2)},
        {bounds_.high.at(0), bounds_.low.at(1), bounds_.low.at(2)},
        {bounds_.high.at(0), bounds_.high.at(1), bounds_.high.at(2)},
        {bounds_.high.at(0), bounds_.high.at(1), bounds_.low.at(2)},
        {bounds_.high.at(0), bounds_.high.at(1), bounds_.high.at(2)},
        {bounds_.high.at(0), bounds_.low.at(1), bounds_.high.at(2)},
        {bounds_.high.at(0), bounds_.high.at(1), bounds_.high.at(2)},
        {bounds_.low.at(0), bounds_.high.at(1), bounds_.high.at(2)},
        {bounds_.low.at(0), bounds_.low.at(1), bounds_.high.at(2)},
        {bounds_.low.at(0), bounds_.high.at(1), bounds_.high.at(2)},
        {bounds_.low.at(0), bounds_.low.at(1), bounds_.high.at(2)},
        {bounds_.high.at(0), bounds_.low.at(1), bounds_.high.at(2)},
        {bounds_.low.at(0), bounds_.high.at(1), bounds_.low.at(2)},
        {bounds_.low.at(0), bounds_.high.at(1), bounds_.high.at(2)},
        {bounds_.low.at(0), bounds_.high.at(1), bounds_.low.at(2)},
        {bounds_.high.at(0), bounds_.high.at(1), bounds_.low.at(2)},
        {bounds_.high.at(0), bounds_.low.at(1), bounds_.low.at(2)},
        {bounds_.high.at(0), bounds_.high.at(1), bounds_.low.at(2)},
        {bounds_.high.at(0), bounds_.low.at(1), bounds_.low.at(2)},
        {bounds_.high.at(0), bounds_.low.at(1), bounds_.high.at(2)},
    };
    pangolin::glDrawLines(points);
  } else {
    OMPL_ERROR("Interactive visualizer can only visualize 2D or 3D contexts.");
    throw std::runtime_error("Visualization error.");
  }
}

void InteractiveVisualizer::drawRectangle(const std::vector<float>& midpoint,
                                          const std::vector<float>& widths, const float* faceColor,
                                          const float* edgeColor) const {
  if (widths.size() == 2u) {
    drawRectangle2D(midpoint, widths, faceColor, edgeColor);
  } else if (widths.size() == 3u) {
    drawRectangle3D(midpoint, widths, faceColor, edgeColor);
  } else {
    OMPL_ERROR("Interactive visualizer can only visualize 2D or 3D contexts.");
    throw std::runtime_error("Visualization error.");
  }
}

void InteractiveVisualizer::drawRectangle2D(const std::vector<float>& midpoint,
                                            const std::vector<float>& widths,
                                            const float* faceColor, const float* edgeColor) const {
  glColor4fv(faceColor);
  pangolin::glDrawRect(midpoint.at(0) - widths.at(0) / 2.0f, midpoint.at(1) - widths.at(1) / 2.0f,
                       midpoint.at(0) + widths.at(0) / 2.0f, midpoint.at(1) + widths.at(1) / 2.0f);
  glColor4fv(edgeColor);
  pangolin::glDrawRectPerimeter(
      midpoint.at(0) - widths.at(0) / 2.0f, midpoint.at(1) - widths.at(1) / 2.0f,
      midpoint.at(0) + widths.at(0) / 2.0f, midpoint.at(1) + widths.at(1) / 2.0f);
}

void InteractiveVisualizer::drawRectangle3D(const std::vector<float>& midpoint,
                                            const std::vector<float>& widths,
                                            const float* faceColor, const float* edgeColor) const {
  float xhalf = widths[0] / 2.0f;
  float yhalf = widths[1] / 2.0f;
  float zhalf = widths[2] / 2.0f;
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

void InteractiveVisualizer::drawPoint(const Eigen::Vector2f& point, const float* color,
                                      float size) const {
  glColor4fv(color);
  pangolin::glDrawCircle(point[0], point[1], size / 1000.0f);
}

void InteractiveVisualizer::drawPoint(const Eigen::Vector3f& point, const float* color,
                                      float size) const {
  std::vector<Eigen::Vector3f> points;
  points.push_back(point);
  drawPoints(points, color, size);
}

void InteractiveVisualizer::drawPoint(
    const ompl::base::ScopedState<ompl::base::RealVectorStateSpace>& state, const float* color,
    float size) const {
  if (state.getSpace()->getDimension() == 2u) {
    drawPoint(Eigen::Vector2f(static_cast<float>(state[0u]), static_cast<float>(state[1u])), color,
              size);
  } else if (state.getSpace()->getDimension() == 3u) {
    drawPoint(Eigen::Vector3f(static_cast<float>(state[0u]), static_cast<float>(state[1u]),
                              static_cast<float>(state[2u])),
              color, size);
  } else {
    throw std::runtime_error("Can only visualize 2d and 3d states.");
  }
}

void InteractiveVisualizer::drawPoints(const std::vector<Eigen::Vector2f>& points,
                                       const float* color, float size) const {
  glColor4fv(color);
  for (const auto& point : points) {
    pangolin::glDrawCircle(point[0], point[1], size / 1000.0f);
  }
}

void InteractiveVisualizer::drawPoints(const std::vector<Eigen::Vector3f>& points,
                                       const float* color, float size) const {
  glColor4fv(color);
  glPointSize(size);
  pangolin::glDrawPoints(points);
}

void InteractiveVisualizer::drawPoints(const std::vector<ompl::base::ScopedState<>>& states,
                                       const float* color, float size) const {
  if (states.front().getSpace()->getDimension() == 2u) {
    std::vector<Eigen::Vector2f> points;
    for (const auto& state : states) {
      points.emplace_back(state[0], state[1]);
    }
    drawPoints(points, color, size);
  } else if (states.front().getSpace()->getDimension() == 3u) {
    std::vector<Eigen::Vector3f> points;
    for (const auto& state : states) {
      points.emplace_back(state[0], state[1], state[2]);
    }
    drawPoints(points, color, size);
  }
}

void InteractiveVisualizer::drawLines(const std::vector<Eigen::Vector2f>& points, float width,
                                      const float* color, float alpha) const {
  assert(points.size() % 2 == 0u);
  glColor4f(color[0], color[1], color[2], alpha);
  glLineWidth(width);
  pangolin::glDrawLines(points);
}

void InteractiveVisualizer::drawLines(const std::vector<Eigen::Vector3f>& points, float width,
                                      const float* color, float alpha) const {
  assert(points.size() % 2 == 0u);
  glColor4f(color[0], color[1], color[2], alpha);
  glLineWidth(width);
  pangolin::glDrawLines(points);
}

void InteractiveVisualizer::drawPath(const std::vector<Eigen::Vector2f>& points, float width,
                                     const float* color, float alpha) const {
  glColor4f(color[0], color[1], color[2], alpha);
  glLineWidth(width);
  pangolin::glDrawLineStrip(points);
}

void InteractiveVisualizer::drawPath(const std::vector<Eigen::Vector3f>& points, float width,
                                     const float* color, float alpha) const {
  glColor4f(color[0], color[1], color[2], alpha);
  glLineWidth(width);
  pangolin::glDrawLineStrip(points);
}

void InteractiveVisualizer::drawCars(const std::vector<Eigen::Vector3f>& states, float width,
                                     const float* color, float alpha) const {
  glColor4f(color[0], color[1], color[2], alpha);
  const auto carWidth = 0.02;
  const auto carLength = 0.04;

  // Compute the corners of the car in car coordinates and same order as above:
  // upper left, upper right, lower right, lower left.
  const std::vector<Eigen::Vector2f> carCornersCar{{-carLength / 2.0, carWidth / 2.0},
                                                   {carLength / 2.0, carWidth / 2.0},
                                                   {carLength / 2.0, -carWidth / 2.0},
                                                   {-carLength / 2.0, -carWidth / 2.0}};

  glLineWidth(width);

  // Draw all cars.
  std::vector<Eigen::Vector2f> points;
  points.reserve(states.size());
  for (const auto& state : states) {
    // Remember the position of the car.
    points.emplace_back(state[0u], state[1u]);

    // Compute the corners of the car in world coordinates.
    std::vector<Eigen::Vector2f> carCornersWorld{{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
    for (auto i = 0u; i < 4u; ++i) {
      carCornersWorld.at(i)[0u] = carCornersCar[i][0u] * std::cos(state[2u]) -
                                  carCornersCar[i][1u] * std::sin(state[2u]) + state[0u];
      carCornersWorld.at(i)[1u] = carCornersCar[i][0u] * std::sin(state[2u]) +
                                  carCornersCar[i][1u] * std::cos(state[2u]) + state[1u];
    }

    // Draw the car.
    pangolin::glDrawLine(carCornersWorld[0u][0u], carCornersWorld[0u][1u], carCornersWorld[1u][0u],
                         carCornersWorld[1u][1u]);
    pangolin::glDrawLine(carCornersWorld[1u][0u], carCornersWorld[1u][1u], carCornersWorld[2u][0u],
                         carCornersWorld[2u][1u]);
    pangolin::glDrawLine(carCornersWorld[2u][0u], carCornersWorld[2u][1u], carCornersWorld[3u][0u],
                         carCornersWorld[3u][1u]);
    pangolin::glDrawLine(carCornersWorld[3u][0u], carCornersWorld[3u][1u], carCornersWorld[0u][0u],
                         carCornersWorld[0u][1u]);
  }

  // Draw the path as well.
  drawPath(points, width, color, alpha);
}

void InteractiveVisualizer::visit(
    const obstacles::Hyperrectangle<obstacles::BaseObstacle>& obstacle) const {
  const auto anchorCoords = obstacle.getAnchorCoordinates();
  std::vector<float> anchor(anchorCoords.begin(), anchorCoords.end());
  const auto obstacleWidth = obstacle.getWidths();
  std::vector<float> widths(obstacleWidth.begin(), obstacleWidth.end());
  drawRectangle(anchor, widths, black, black);
}

void InteractiveVisualizer::visit(
    const obstacles::Hyperrectangle<obstacles::BaseAntiObstacle>& antiObstacle) const {
  const auto anchorCoords = antiObstacle.getAnchorCoordinates();
  std::vector<float> anchor(anchorCoords.begin(), anchorCoords.end());
  const auto antiObstacleWidth = antiObstacle.getWidths();
  std::vector<float> widths(antiObstacleWidth.begin(), antiObstacleWidth.end());
  drawRectangle(anchor, widths, white, white);
}

void InteractiveVisualizer::visit(
    const objectives::PotentialFieldOptimizationObjective& objective) const {
  if (bounds_.low.size() == 2u) {
    const auto vectorContext =
        std::dynamic_pointer_cast<planning_contexts::RealVectorGeometricContext>(context_);
    const auto se2Context =
        std::dynamic_pointer_cast<planning_contexts::ReedsSheppRandomRectangles>(context_);
    auto boundaries = ompl::base::RealVectorBounds(2u);

    if (vectorContext) {
      boundaries = vectorContext->getBoundaries();
    } else if (se2Context) {
      boundaries = se2Context->getBoundaries();
    }

    // Get the widths of the context.
    const auto minX = static_cast<float>(boundaries.low.at(0u));
    const auto maxX = static_cast<float>(boundaries.high.at(0u));
    const auto minY = static_cast<float>(boundaries.low.at(1u));
    const auto maxY = static_cast<float>(boundaries.high.at(1u));
    const auto widthX = maxX - minX;
    const auto widthY = maxY - minY;

    // Define the number of points per axis.
    constexpr unsigned numPointsPerAxis{40u};

    // Compute the resulting step sizes.
    const auto stepX = widthX / static_cast<float>(numPointsPerAxis);
    const auto stepY = widthY / static_cast<float>(numPointsPerAxis);

    // Prepare a state to probe the cost at the discretized locations.
    auto costState =
        context_->getStateSpace()->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();

    // Determine the min and max costs if not already determined.
    if (minOptimizationCost_ == std::numeric_limits<float>::max()) {
      for (auto i = 0u; i < numPointsPerAxis; ++i) {
        costState->operator[](0u) = minX + stepX / 2.0f + static_cast<float>(i) * stepX;
        for (auto j = 0u; j < numPointsPerAxis; ++j) {
          costState->operator[](1u) = minY + stepY / 2.0f + static_cast<float>(j) * stepY;

          // Compute the cost at this state.
          auto cost = objective.stateCost(costState);
          if (objective.isCostBetterThan(cost, ompl::base::Cost(minOptimizationCost_))) {
            minOptimizationCost_ = static_cast<float>(cost.value());
          }
          if (objective.isCostBetterThan(ompl::base::Cost(maxOptimizationCost_), cost)) {
            maxOptimizationCost_ = static_cast<float>(cost.value());
          }
        }
      }
    }

    // Iterate over the grid to visualize the resulting cost.
    for (auto i = 0u; i < numPointsPerAxis; ++i) {
      float x = minX + stepX / 2.0f + static_cast<float>(i) * stepX;
      costState->operator[](0u) = x;
      for (std::size_t j = 0u; j < numPointsPerAxis; ++j) {
        float y = minY + stepY / 2.0f + static_cast<float>(j) * stepY;
        costState->operator[](1u) = y;

        // Compute the cost at this state.
        auto cost = static_cast<float>(objective.stateCost(costState).value());

        // Compute the color for this cost.
        auto color = interpolateColors(
            green, red,
            (cost - minOptimizationCost_) / (maxOptimizationCost_ - minOptimizationCost_));

        // Draw the rectangle.
        drawRectangle2D(std::vector<float>{x, y}, std::vector<float>{stepX, stepY}, color.data(),
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
    const objectives::MaxMinClearanceOptimizationObjective& /* objective */) const {
}

void InteractiveVisualizer::visit(
    const objectives::ReciprocalClearanceOptimizationObjective& /* objective */) const {
}

std::array<float, 4u> InteractiveVisualizer::interpolateColors(const float* color1,
                                                               const float* color2,
                                                               const float step) const {
  std::array<float, 4u> color;
  for (std::size_t i = 0u; i < 4u; ++i) {
    color[i] = color1[i] + (color2[i] - color1[i]) * std::pow(step, 0.25f);
  }
  return color;
}

void InteractiveVisualizer::drawPlannerSpecificVisualizations(const std::size_t iteration) const {
  switch (plannerType_) {
    case common::PLANNER_TYPE::BITSTAR:
    case common::PLANNER_TYPE::ABITSTAR: {
      drawBITstarSpecificVisualizations(iteration);
      return;
    }
    case common::PLANNER_TYPE::AITSTAR: {
      drawAITstarSpecificVisualizations(iteration);
      return;
    }
#ifdef PDT_EXTRA_EITSTAR_PR
    case common::PLANNER_TYPE::EIRMSTAR:
    case common::PLANNER_TYPE::EITSTAR: {
      drawEITstarSpecificVisualizations(iteration);
      return;
    }
#endif  // #ifdef PDT_EXTRA_EITSTAR_PR
    case common::PLANNER_TYPE::LAZYPRMSTAR: {
      drawLazyPRMstarSpecificVisualizations(iteration);
      return;
    }
    default:
      return;
  }
}

void InteractiveVisualizer::drawBITstarSpecificVisualizations(const std::size_t iteration) const {
  // Get the BIT* specific data.
  auto bitstarData =
      std::dynamic_pointer_cast<const BITstarData>(getPlannerSpecificData(iteration));
  if (bounds_.low.size() == 2u) {
    // Get the edge queue.
    auto edgeQueue = bitstarData->getEdgeQueue();
    std::vector<Eigen::Vector2f> edges{};
    for (const auto& edge : edgeQueue) {
      auto sourceState = edge.first->state()->as<ompl::base::RealVectorStateSpace::StateType>();
      edges.push_back(Eigen::Vector2f((*sourceState)[0u], (*sourceState)[1u]));
      auto targetState = edge.second->state()->as<ompl::base::RealVectorStateSpace::StateType>();
      edges.push_back(Eigen::Vector2f((*targetState)[0u], (*targetState)[1u]));
    }

    // Draw the edge queue.
    drawLines(edges, 1.5, lightblue);

    // Get the next edge in the queue.
    auto nextEdgeStates = bitstarData->getNextEdge();

    // If there are no more edges in the queue, this will return nullptrs.
    if (nextEdgeStates.first == nullptr || nextEdgeStates.second == nullptr) {
      return;
    }
    auto sourceState = nextEdgeStates.first->as<ompl::base::RealVectorStateSpace::StateType>();
    auto targetState = nextEdgeStates.second->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<Eigen::Vector2f> nextEdge{Eigen::Vector2f((*sourceState)[0u], (*sourceState)[1u]),
                                          Eigen::Vector2f((*targetState)[0u], (*targetState)[1u])};

    // Draw the next edge.
    drawLines(nextEdge, 3.0, red);
  } else if (bounds_.low.size() == 3u) {
    // Get the edge queue.
    auto edgeQueue = bitstarData->getEdgeQueue();
    std::vector<Eigen::Vector3f> edges{};
    for (const auto& edge : edgeQueue) {
      auto sourceState = edge.first->state()->as<ompl::base::RealVectorStateSpace::StateType>();
      edges.push_back(Eigen::Vector3f(static_cast<float>((*sourceState)[0u]),
                                      static_cast<float>((*sourceState)[1u]),
                                      static_cast<float>((*sourceState)[2u])));
      auto targetState = edge.second->state()->as<ompl::base::RealVectorStateSpace::StateType>();
      edges.push_back(Eigen::Vector3f(static_cast<float>((*targetState)[0u]),
                                      static_cast<float>((*targetState)[1u]),
                                      static_cast<float>((*targetState)[2u])));
    }

    // Draw the edge queue.
    drawLines(edges, 1.5, lightblue);

    // Get the next edge in the queue.
    auto nextEdgeStates = bitstarData->getNextEdge();

    // If there are no more edges in the queue, this will return nullptrs.
    if (nextEdgeStates.first == nullptr || nextEdgeStates.second == nullptr) {
      return;
    }
    auto sourceState = nextEdgeStates.first->as<ompl::base::RealVectorStateSpace::StateType>();
    auto targetState = nextEdgeStates.second->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<Eigen::Vector3f> nextEdge{Eigen::Vector3f(static_cast<float>((*sourceState)[0u]),
                                                          static_cast<float>((*sourceState)[1u]),
                                                          static_cast<float>((*sourceState)[2u])),
                                          Eigen::Vector3f(static_cast<float>((*targetState)[0u]),
                                                          static_cast<float>((*targetState)[1u]),
                                                          static_cast<float>((*targetState)[2u]))};

    // Draw the next edge.
    drawLines(nextEdge, 3.0, red);
  } else {
    throw std::runtime_error(
        "BITstar specific visualizations only implemented for 2d or 3d contexts.");
  }
}

void InteractiveVisualizer::drawAITstarSpecificVisualizations(const std::size_t iteration) const {
  // Get the TBD* specific data.
  auto aitstarData =
      std::dynamic_pointer_cast<const AITstarData>(getPlannerSpecificData(iteration));
  if (bounds_.low.size() == 2u) {
    // Get the forward edge queue.
    auto forwardQueue = aitstarData->getForwardQueue();
    std::vector<Eigen::Vector2f> forwardQueueEdges;
    forwardQueueEdges.reserve(2u * forwardQueue.size());
    for (const auto& edge : forwardQueue) {
      auto sourceState =
          edge.getParent()->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      forwardQueueEdges.emplace_back(static_cast<float>((*sourceState)[0u]),
                                     static_cast<float>((*sourceState)[1u]));
      auto targetState =
          edge.getChild()->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      forwardQueueEdges.emplace_back(static_cast<float>((*targetState)[0u]),
                                     static_cast<float>((*targetState)[1u]));
    }

    drawLines(forwardQueueEdges, 1.5, lightblue);

    // Get the vertex queue.
    auto backwardQueue = aitstarData->getBackwardQueue();
    std::vector<Eigen::Vector2f> backwardQueueVertices{};
    for (const auto& vertex : backwardQueue) {
      auto state = vertex->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      backwardQueueVertices.emplace_back(static_cast<float>((*state)[0u]),
                                         static_cast<float>((*state)[1u]));
    }

    drawPoints(backwardQueueVertices, yellow, 5.0f);

    // Get the next vertex in the queue.
    auto nextVertex = aitstarData->getNextVertex();
    if (nextVertex) {
      auto state = nextVertex->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      drawPoints(std::vector<Eigen::Vector2f>{Eigen::Vector2f(static_cast<float>((*state)[0u]),
                                                              static_cast<float>((*state)[1u]))},
                 red, 10.0f);
    }

    // Draw the backward search tree.
    auto backwardSearchTree = aitstarData->getVerticesInBackwardSearchTree();
    std::vector<Eigen::Vector2f> backwardSearchTreeEdges;
    for (const auto& vertex : backwardSearchTree) {
      // Add the edge to the parent.
      if (vertex->hasReverseParent()) {
        auto state = vertex->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
        auto parent = vertex->getReverseParent()
                          ->getState()
                          ->as<ompl::base::RealVectorStateSpace::StateType>();
        backwardSearchTreeEdges.emplace_back(static_cast<float>((*state)[0u]),
                                             static_cast<float>((*state)[1u]));
        backwardSearchTreeEdges.emplace_back(static_cast<float>((*parent)[0u]),
                                             static_cast<float>((*parent)[1u]));
      }
    }

    drawLines(backwardSearchTreeEdges, 2.0f, yellow);

    // Get the next edge in the queue.
    auto nextEdgeStates = aitstarData->getNextEdge();

    // If there are no more edges in the queue, this will return an edge with nullptrs.
    if (!nextEdgeStates.first || !nextEdgeStates.second) {
      return;
    }
    auto sourceState = nextEdgeStates.first->as<ompl::base::RealVectorStateSpace::StateType>();
    auto targetState = nextEdgeStates.second->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<Eigen::Vector2f> nextEdge{Eigen::Vector2f(static_cast<float>((*sourceState)[0u]),
                                                          static_cast<float>((*sourceState)[1u])),
                                          Eigen::Vector2f(static_cast<float>((*targetState)[0u]),
                                                          static_cast<float>((*targetState)[1u]))};
    // Draw the next edge.
    drawLines(nextEdge, 3.0f, red);
  } else if (bounds_.low.size() == 3u) {
    // Get the edge queue.
    auto edgeQueue = aitstarData->getForwardQueue();
    std::vector<Eigen::Vector3f> edges{};
    for (const auto& edge : edgeQueue) {
      auto sourceState =
          edge.getParent()->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      edges.push_back(Eigen::Vector3f(static_cast<float>((*sourceState)[0u]),
                                      static_cast<float>((*sourceState)[1u]),
                                      static_cast<float>((*sourceState)[2u])));
      auto targetState =
          edge.getChild()->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      edges.push_back(Eigen::Vector3f(static_cast<float>((*targetState)[0u]),
                                      static_cast<float>((*targetState)[1u]),
                                      static_cast<float>((*targetState)[2u])));
    }

    // Draw the edge queue.
    drawLines(edges, 1.5f, lightblue);

    // Get the next edge in the queue.
    auto nextEdgeStates = aitstarData->getNextEdge();

    // If there are no more edges in the queue, this will return an edge with nullptrs.
    if (!nextEdgeStates.first || !nextEdgeStates.second) {
      return;
    }
    auto sourceState = nextEdgeStates.first->as<ompl::base::RealVectorStateSpace::StateType>();
    auto targetState = nextEdgeStates.second->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<Eigen::Vector3f> nextEdgeVector{
        Eigen::Vector3f(static_cast<float>((*sourceState)[0u]),
                        static_cast<float>((*sourceState)[1u]),
                        static_cast<float>((*sourceState)[2u])),
        Eigen::Vector3f(static_cast<float>((*targetState)[0u]),
                        static_cast<float>((*targetState)[1u]),
                        static_cast<float>((*targetState)[2u]))};
    // Draw the next edge.
    drawLines(nextEdgeVector, 3.0f, red);
  }
}

#ifdef PDT_EXTRA_EITSTAR_PR
void InteractiveVisualizer::drawEITstarSpecificVisualizations(const std::size_t iteration) const {
  // Get the EIT* specific data.
  auto eitstarData =
      std::dynamic_pointer_cast<const EITstarData>(getPlannerSpecificData(iteration));
  if (bounds_.low.size() == 2u) {
    // Get the forward queue.
    auto forwardQueue = eitstarData->getForwardQueue();
    std::vector<Eigen::Vector2f> forwardQueueEdges;
    forwardQueueEdges.reserve(2u * forwardQueue.size());
    for (const auto& edge : forwardQueue) {
      auto parentState = edge.source->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      forwardQueueEdges.emplace_back(static_cast<float>((*parentState)[0u]),
                                     static_cast<float>((*parentState)[1u]));
      auto childState = edge.target->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      forwardQueueEdges.emplace_back(static_cast<float>((*childState)[0u]),
                                     static_cast<float>((*childState)[1u]));
    }

    // Draw the forward queue.
    drawLines(forwardQueueEdges, 1.5f, lightblue);

    // // Get the reverse queue.
    // auto reverseQueue = eitstarData->getReverseQueue();
    // std::vector<Eigen::Vector2f> reverseQueueEdges;
    // reverseQueueEdges.reserve(2u * reverseQueue.size());
    // for (const auto& edge : reverseQueue) {
    //   auto parentState = edge.source->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
    //   reverseQueueEdges.emplace_back(static_cast<float>((*parentState)[0u]),
    //                                  static_cast<float>((*parentState)[1u]));
    //   auto childState = edge.target->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
    //   reverseQueueEdges.emplace_back(static_cast<float>((*childState)[0u]),
    //                                  static_cast<float>((*childState)[1u]));
    // }

    // // Draw the reverse queue.
    // drawLines(reverseQueueEdges, 1.5f, yellow);

    // Get the reverse tree.
    auto reverseTree = eitstarData->getReverseTree();
    std::vector<Eigen::Vector2f> reverseTreeEdges;
    reverseTreeEdges.reserve(2u * reverseTree.size());
    for (const auto& edge : reverseTree) {
      auto parentState = edge.source->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      reverseTreeEdges.emplace_back(static_cast<float>((*parentState)[0u]),
                                    static_cast<float>((*parentState)[1u]));
      auto childState = edge.target->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      reverseTreeEdges.emplace_back(static_cast<float>((*childState)[0u]),
                                    static_cast<float>((*childState)[1u]));
    }

    // Draw the reverse tree.
    drawLines(reverseTreeEdges, 2.0f, blue);

    // Get the next edge in the forward queue.
    auto nextForwardEdge = eitstarData->getNextForwardEdge();

    // If there are no more edges in the queue, this will return an edge with nullptrs.
    if (nextForwardEdge.source && nextForwardEdge.target) {
      std::vector<Eigen::Vector2f> nextEdge;
      auto parentState =
          nextForwardEdge.source->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      auto childState =
          nextForwardEdge.target->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      nextEdge.emplace_back(static_cast<float>((*parentState)[0u]),
                            static_cast<float>((*parentState)[1u]));
      nextEdge.emplace_back(static_cast<float>((*childState)[0u]),
                            static_cast<float>((*childState)[1u]));
      // Draw the next edge.
      drawLines(nextEdge, 3.0f, red);
    }

    // Get the next edge in the reverse queue.
    auto nextReverseEdge = eitstarData->getNextReverseEdge();

    // If there are no more edges in the queue, this will return an edge with nullptrs.
    if (nextReverseEdge.source && nextReverseEdge.target) {
      std::vector<Eigen::Vector2f> nextEdge;
      auto parentState =
          nextReverseEdge.source->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      auto childState =
          nextReverseEdge.target->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      nextEdge.emplace_back(static_cast<float>((*parentState)[0u]),
                            static_cast<float>((*parentState)[1u]));
      nextEdge.emplace_back(static_cast<float>((*childState)[0u]),
                            static_cast<float>((*childState)[1u]));
      // Draw the next edge.
      drawLines(nextEdge, 3.0f, darkred);
    }
  } else if (bounds_.low.size() == 3u) {
    // Get the forward queue.
    auto forwardQueue = eitstarData->getForwardQueue();
    std::vector<Eigen::Vector3f> forwardQueueEdges;
    forwardQueueEdges.reserve(2u * forwardQueue.size());
    for (const auto& edge : forwardQueue) {
      auto parentState = edge.source->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      forwardQueueEdges.emplace_back(static_cast<float>((*parentState)[0u]),
                                     static_cast<float>((*parentState)[1u]),
                                     static_cast<float>((*parentState)[2u]));
      auto childState = edge.target->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      forwardQueueEdges.emplace_back(static_cast<float>((*childState)[0u]),
                                     static_cast<float>((*childState)[1u]),
                                     static_cast<float>((*childState)[2u]));
    }

    // Draw the forward queue.
    drawLines(forwardQueueEdges, 1.5f, lightblue);

    // Get the reverse queue.
    auto reverseQueue = eitstarData->getReverseQueue();
    std::vector<Eigen::Vector3f> reverseQueueEdges;
    reverseQueueEdges.reserve(2u * reverseQueue.size());
    for (const auto& edge : reverseQueue) {
      auto parentState = edge.source->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      reverseQueueEdges.emplace_back(static_cast<float>((*parentState)[0u]),
                                     static_cast<float>((*parentState)[1u]),
                                     static_cast<float>((*parentState)[2u]));
      auto childState = edge.target->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      reverseQueueEdges.emplace_back(static_cast<float>((*childState)[0u]),
                                     static_cast<float>((*childState)[1u]),
                                     static_cast<float>((*childState)[2u]));
    }

    // Draw the reverse queue.
    drawLines(reverseQueueEdges, 1.5f, yellow);

    // Get the reverse tree.
    auto reverseTree = eitstarData->getReverseTree();
    std::vector<Eigen::Vector3f> reverseTreeEdges;
    reverseTreeEdges.reserve(2u * reverseTree.size());
    for (const auto& edge : reverseTree) {
      auto parentState = edge.source->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      reverseTreeEdges.emplace_back(static_cast<float>((*parentState)[0u]),
                                    static_cast<float>((*parentState)[1u]),
                                    static_cast<float>((*parentState)[2u]));
      auto childState = edge.target->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      reverseTreeEdges.emplace_back(static_cast<float>((*childState)[0u]),
                                    static_cast<float>((*childState)[1u]),
                                    static_cast<float>((*childState)[2u]));
    }

    // Draw the reverse tree.
    drawLines(reverseTreeEdges, 2.0f, blue);

    // Get the next edge in the forward queue.
    auto nextForwardEdge = eitstarData->getNextForwardEdge();

    // If there are no more edges in the queue, this will return an edge with nullptrs.
    if (nextForwardEdge.source && nextForwardEdge.target) {
      std::vector<Eigen::Vector3f> nextEdge;
      auto parentState =
          nextForwardEdge.source->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      auto childState =
          nextForwardEdge.target->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      nextEdge.emplace_back(static_cast<float>((*parentState)[0u]),
                            static_cast<float>((*parentState)[1u]),
                            static_cast<float>((*parentState)[2u]));
      nextEdge.emplace_back(static_cast<float>((*childState)[0u]),
                            static_cast<float>((*childState)[1u]),
                            static_cast<float>((*childState)[2u]));
      // Draw the next edge.
      drawLines(nextEdge, 3.0f, red);
    }

    // Get the next edge in the reverse queue.
    auto nextReverseEdge = eitstarData->getNextReverseEdge();

    // If there are no more edges in the queue, this will return an edge with nullptrs.
    if (nextReverseEdge.source && nextReverseEdge.target) {
      std::vector<Eigen::Vector3f> nextEdge;
      auto parentState =
          nextReverseEdge.source->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      auto childState =
          nextReverseEdge.target->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
      nextEdge.emplace_back(static_cast<float>((*parentState)[0u]),
                            static_cast<float>((*parentState)[1u]),
                            static_cast<float>((*parentState)[2u]));
      nextEdge.emplace_back(static_cast<float>((*childState)[0u]),
                            static_cast<float>((*childState)[1u]),
                            static_cast<float>((*childState)[2u]));
      // Draw the next edge.
      drawLines(nextEdge, 3.0f, darkred);
    }
  }
}
#endif  // #ifdef PDT_EXTRA_EITSTAR_PR

void InteractiveVisualizer::drawLazyPRMstarSpecificVisualizations(std::size_t iteration) const {
  // Get the LPRM* specific data.
  auto lPRMstarData =
      std::dynamic_pointer_cast<const LazyPRMstarData>(getPlannerSpecificData(iteration));
  if (context_->getDimension() == 2u) {
    std::vector<Eigen::Vector2f> edges{};
    for (const auto& edge : lPRMstarData->getValidEdges()) {
      auto parentState = edge.first.getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      edges.push_back(Eigen::Vector2f(static_cast<float>((*parentState)[0u]),
                                      static_cast<float>((*parentState)[1u])));
      auto childState = edge.second.getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      edges.push_back(Eigen::Vector2f(static_cast<float>((*childState)[0u]),
                                      static_cast<float>((*childState)[1u])));
    }
    // Draw the edge queue.
    drawLines(edges, 1.5f, lightblue);
  } else {
    throw std::runtime_error(
        "LazyPRMstar specific visualizations only implemented for 2d contexts.");
  }
}

std::pair<std::vector<Eigen::Vector2f>, std::vector<Eigen::Vector2f>>
InteractiveVisualizer::getVerticesAndEdges2D(std::size_t iteration) const {
  const auto& currentPlannerData = getPlannerData(iteration);
  const auto vectorContext =
      std::dynamic_pointer_cast<planning_contexts::RealVectorGeometricContext>(context_);
  const auto se2Context =
      std::dynamic_pointer_cast<planning_contexts::ReedsSheppRandomRectangles>(context_);

  // Get the vertices and edges in the format supported by Panglin.
  std::vector<Eigen::Vector2f> vertices{};
  std::vector<Eigen::Vector2f> edges{};  // Size must be multiple of two.
  for (auto i = 0u; i < currentPlannerData->numVertices(); ++i) {
    auto vertex = currentPlannerData->getVertex(i);

    // Check the vertex is valid.
    if (vertex != ompl::base::PlannerData::NO_VERTEX) {
      auto vertexX = 0.0;
      auto vertexY = 0.0;

      if (vectorContext) {
        vertexX = vertex.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->values[0u];
        vertexY = vertex.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->values[1u];
      } else if (se2Context) {
        vertexX = vertex.getState()->as<ompl::base::SE2StateSpace::StateType>()->getX();
        vertexY = vertex.getState()->as<ompl::base::SE2StateSpace::StateType>()->getY();
      } else {
        std::runtime_error(
            "Interactive visualizer can only handle real vector and SE2 state spaces.");
      }
      vertices.emplace_back(vertexX, vertexY);

      // Get the outgoing edges of this vertex.
      std::vector<unsigned int> outgoingEdges{};
      currentPlannerData->getEdges(i, outgoingEdges);
      for (std::size_t j = 0; j < outgoingEdges.size(); ++j) {
        // Check that the vertex is valid.
        auto child = currentPlannerData->getVertex(outgoingEdges.at(j));
        if (child != ompl::base::PlannerData::NO_VERTEX) {
          auto childX = 0.0;
          auto childY = 0.0;

          if (vectorContext) {
            childX =
                child.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->values[0u];
            childY =
                child.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->values[1u];
          } else if (se2Context) {
            childX = child.getState()->as<ompl::base::SE2StateSpace::StateType>()->getX();
            childY = child.getState()->as<ompl::base::SE2StateSpace::StateType>()->getY();
          } else {
            std::runtime_error(
                "Interactive visualizer can only handle real vector and SE2 state spaces.");
          }

          // Add the parent.
          edges.emplace_back(vertexX, vertexY);

          // Add the child.
          edges.emplace_back(childX, childY);
        }
      }
    }
  }

  return {vertices, edges};
}

std::pair<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>>
InteractiveVisualizer::getVerticesAndEdges3D(std::size_t iteration) const {
  const auto& currentPlannerData = getPlannerData(iteration);
  // Get the vertices and edges in the format supported by Panglin.
  std::vector<Eigen::Vector3f> vertices{};
  std::vector<Eigen::Vector3f> edges{};  // Size must be multiple of two.
  for (auto i = 0u; i < currentPlannerData->numVertices(); ++i) {
    auto vertex = currentPlannerData->getVertex(i);
    // Check the vertex is valid.
    if (vertex != ompl::base::PlannerData::NO_VERTEX) {
      const auto* vertexState =
          static_cast<const ompl::base::RealVectorStateSpace::StateType*>(vertex.getState());
      vertices.emplace_back(vertexState->values[0], vertexState->values[1], vertexState->values[2]);

      // Get the outgoing edges of this vertex.
      std::vector<unsigned int> outgoingEdges{};
      currentPlannerData->getEdges(i, outgoingEdges);
      for (auto j = 0u; j < outgoingEdges.size(); ++j) {
        // Check that the vertex is valid.
        auto child = currentPlannerData->getVertex(outgoingEdges.at(j));
        if (child != ompl::base::PlannerData::NO_VERTEX) {
          // Add the parent.
          edges.emplace_back(vertexState->values[0], vertexState->values[1],
                             vertexState->values[2]);

          // Add the child.
          const auto* targetState =
              static_cast<const ompl::base::RealVectorStateSpace::StateType*>(child.getState());
          edges.emplace_back(targetState->values[0], targetState->values[1],
                             targetState->values[2]);
        }
      }
    }
  }
  return {vertices, edges};
}

std::vector<Eigen::Vector2f> InteractiveVisualizer::getVertices2D(std::size_t iteration) const {
  const auto& currentPlannerData = getPlannerData(iteration);
  // Get the vertices in a format supported by Pangolin.
  std::vector<Eigen::Vector2f> vertices{};
  for (auto i = 0u; i < currentPlannerData->numVertices(); ++i) {
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

std::vector<Eigen::Vector3f> InteractiveVisualizer::getVertices3D(std::size_t iteration) const {
  const auto& currentPlannerData = getPlannerData(iteration);
  // Get the vertices in a format supported by Pangolin.
  std::vector<Eigen::Vector3f> vertices{};
  for (auto i = 0u; i < currentPlannerData->numVertices(); ++i) {
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

std::vector<Eigen::Vector2f> InteractiveVisualizer::getEdges2D(std::size_t iteration) const {
  const auto& currentPlannerData = getPlannerData(iteration);
  std::vector<Eigen::Vector2f> edges{};  // Size must be multiple of two.
  for (auto i = 0u; i < currentPlannerData->numVertices(); ++i) {
    auto parent = currentPlannerData->getVertex(i);
    // Check the vertex is valid.
    if (parent != ompl::base::PlannerData::NO_VERTEX) {
      const auto* sourceState =
          static_cast<const ompl::base::RealVectorStateSpace::StateType*>(parent.getState());

      // Get the outgoing edges of this vertex.
      std::vector<unsigned int> outgoingEdges{};
      currentPlannerData->getEdges(i, outgoingEdges);
      for (std::size_t j = 0; j < outgoingEdges.size(); ++j) {
        // Check that the vertex is valid.
        auto child = currentPlannerData->getVertex(outgoingEdges.at(j));
        if (child != ompl::base::PlannerData::NO_VERTEX) {
          // Add the parent.
          edges.emplace_back(sourceState->values[0], sourceState->values[1]);

          // Add the child.
          const auto* targetState =
              static_cast<const ompl::base::RealVectorStateSpace::StateType*>(child.getState());
          edges.emplace_back(targetState->values[0], targetState->values[1]);
        }
      }
    }
  }
  return edges;
}

std::vector<Eigen::Vector3f> InteractiveVisualizer::getEdges3D(std::size_t iteration) const {
  const auto& currentPlannerData = getPlannerData(iteration);
  std::vector<Eigen::Vector3f> edges{};  // Size must be multiple of two.
  for (auto i = 0u; i < currentPlannerData->numVertices(); ++i) {
    auto parent = currentPlannerData->getVertex(i);
    // Check the vertex is valid.
    if (parent != ompl::base::PlannerData::NO_VERTEX) {
      const auto* sourceState =
          static_cast<const ompl::base::RealVectorStateSpace::StateType*>(parent.getState());

      // Get the outgoing edges of this vertex.
      std::vector<unsigned int> outgoingEdges{};
      currentPlannerData->getEdges(i, outgoingEdges);
      for (std::size_t j = 0; j < outgoingEdges.size(); ++j) {
        // Check that the vertex is valid.
        auto child = currentPlannerData->getVertex(outgoingEdges.at(j));
        if (child != ompl::base::PlannerData::NO_VERTEX) {
          // Add the parent.
          edges.emplace_back(sourceState->values[0], sourceState->values[1],
                             sourceState->values[2]);

          // Add the child.
          const auto* targetState =
              static_cast<const ompl::base::RealVectorStateSpace::StateType*>(child.getState());
          edges.emplace_back(targetState->values[0], targetState->values[1],
                             targetState->values[2]);
        }
      }
    }
  }
  return edges;
}

std::vector<Eigen::Vector2f> InteractiveVisualizer::getPath2D(const std::size_t iteration) const {
  std::vector<Eigen::Vector2f> points{};
  auto solution = getSolutionPath(iteration);
  if (solution != nullptr) {
    auto path = solution->as<ompl::geometric::PathGeometric>()->getStates();
    points.reserve(path.size());
    for (const auto state : path) {
      const auto* rstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
      points.emplace_back(rstate->values[0], rstate->values[1]);
    }
  }
  return points;
}

std::vector<Eigen::Vector3f> InteractiveVisualizer::getPath3D(const std::size_t iteration) const {
  std::vector<Eigen::Vector3f> points{};
  auto solution = getSolutionPath(iteration);
  if (solution != nullptr) {
    auto path = solution->as<ompl::geometric::PathGeometric>()->getStates();
    points.reserve(path.size());
    for (const auto state : path) {
      const auto* rstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
      points.emplace_back(rstate->values[0], rstate->values[1], rstate->values[2]);
    }
  }
  return points;
}

std::vector<Eigen::Vector3f> InteractiveVisualizer::getPathSE2(const std::size_t iteration) const {
  std::vector<Eigen::Vector3f> states{};
  auto solution = getSolutionPath(iteration);
  if (solution != nullptr) {
    auto path = solution->as<ompl::geometric::PathGeometric>();
    path->interpolate(200);
    auto se2states = path->getStates();
    states.reserve(se2states.size());
    for (const auto state : se2states) {
      const auto* se2state = state->as<ompl::base::SE2StateSpace::StateType>();
      states.emplace_back(se2state->getX(), se2state->getY(), se2state->getYaw());
    }
  }
  return states;
}

}  // namespace visualization

}  // namespace pdt
