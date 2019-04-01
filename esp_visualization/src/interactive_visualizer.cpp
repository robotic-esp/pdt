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

namespace esp {

namespace ompltools {

InteractiveVisualizer::InteractiveVisualizer(
    const std::shared_ptr<BaseContext>& context,
    const std::pair<std::shared_ptr<ompl::base::Planner>, PLANNER_TYPE> plannerPair) :
    BaseVisualizer(context, plannerPair) {
}

void InteractiveVisualizer::run() {
  // The default window width and height.
  constexpr std::size_t windowWidth = 800;
  constexpr std::size_t windowHeight = 600;

  // Create a window and bind it to the current OpenGL context.
  pangolin::CreateWindowAndBind("esp ompltools", windowWidth, windowHeight);

  // Create the options panel.
  const std::string optionsName{"options"};
  pangolin::CreatePanel(optionsName).SetBounds(0.0f, 1.0f, 0.0f, pangolin::Attach::Pix(200));
  pangolin::Var<bool> optionDrawEdges(optionsName + ".Draw Edges", true, true);
  pangolin::Var<bool> optionDrawObstacles(optionsName + ".Draw Obstacles", true, true);
  pangolin::Var<bool> optionDrawContext(optionsName + ".Draw Context", true, true);

  // Register some keypresses.
  pangolin::RegisterKeyPressCallback('f', [this]() {
    if (viewedIteration_ < largestIteration_) {
      ++viewedIteration_;
      std::cout << "Viewed iteration: " << viewedIteration_ << '\n';
      std::cout << "Largest iteration: " << largestIteration_ << '\n';
    } else {
      OMPL_WARN("Cannot advance iteration.");
    }
  });
  pangolin::RegisterKeyPressCallback('b', [this]() {
    if (viewedIteration_ > 0u) {
      --viewedIteration_;
    } else {
      OMPL_WARN("Already displaying iteration 0.");
    }
  });
  pangolin::RegisterKeyPressCallback('F', [this]() {
    const std::size_t maxIters = 50u;
    std::size_t iter = 0u;
    while (viewedIteration_ < largestIteration_ && iter < maxIters) {
      ++viewedIteration_;
      ++iter;
    }
    std::cout << "Viewed iteration: " << viewedIteration_ << '\n';
    std::cout << "Largest iteration: " << largestIteration_ << '\n';
  });
  pangolin::RegisterKeyPressCallback('B', [this]() {
    const std::size_t maxIters = 50u;
    std::size_t iter = 0u;
    while (viewedIteration_ > 0u && iter < maxIters) {
      --viewedIteration_;
      ++iter;
    }
    std::cout << "Viewed iteration: " << viewedIteration_ << '\n';
    std::cout << "Largest iteration: " << largestIteration_ << '\n';
  });

  // Set up a viewport. A viewport is where OpenGL draws to. We can
  // have multiple viewports for different parts of a window. Any draw
  // command will draw to the active viewport.
  pangolin::View& canvas = pangolin::CreateDisplay();

  auto bounds = context_->getLimits();

  // We can set the bounds of this viewport. The coordinates are with
  // respect to the window boundaries (0 is left/bottom, 1 is right/top).
  // We can also set an aspect ratio such that even if the window gets
  // rescaled the drawings in the viewport do not get distorted.
  canvas.SetBounds(
      0.0, 1.0, pangolin::Attach::Pix(200), 1.0,
      (bounds.at(0).second - bounds.at(0).first) / (bounds.at(1).second - bounds.at(1).first));

  // Create an OpenGL render state. This controls how coordinates are
  // processed before they are drawn in OpenGL's [-1, 1] x [-1, 1] coordinates.
  pangolin::OpenGlRenderState renderState(pangolin::ProjectionMatrixOrthographic(
      bounds.at(0).first,   // The left boundary of the problem
      bounds.at(0).second,  // The right boundary of the problem
      bounds.at(1).first,   // The bottom boundary of the porblem
      bounds.at(1).second,  // The top boundary of the problem
      -1.0f,                // Shouldn't have to change this in 2D
      1.0f                  // Shouldn't have to change this in 2D
      ));

  // Get the bounds of the context to setup a correct renderstate.
  if (BaseVisualizer::context_->getDimensions() == 2u) {
    glShadeModel(GL_FLAT);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glLineWidth(1.0);
    glPointSize(2.0);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
  } else {
    throw std::runtime_error("Interactive visualizer can currently only visualize 2D-context.");
  }
  // This sets the color used when clearing the screen.
  glClearColor(1.0, 1.0, 1.0, 1.0);

  while (!pangolin::ShouldQuit()) {
    const auto& currentPlannerData = getPlannerData(viewedIteration_);

    // Get the vertices and edges in the format supported by Panglin.
    std::vector<Eigen::Vector2d> vertices{};
    std::vector<Eigen::Vector2d> edges{};  // Size must be multiple of two.
    for (std::size_t i = 0u; i < currentPlannerData->numVertices(); ++i) {
      auto parent = currentPlannerData->getVertex(i);
      // Check the vertex is valid.
      if (parent != ompl::base::PlannerData::NO_VERTEX) {
        // Do I really have to make an extra allocation just to get at the coordinates?
        ompl::base::ScopedState<> scopedParent(context_->getStateSpace(), parent.getState());
        vertices.emplace_back(scopedParent[0], scopedParent[1]);

        // Get the outgoing edges of this vertex.
        std::vector<unsigned int> outgoingEdges{};
        currentPlannerData->getEdges(i, outgoingEdges);
        for (std::size_t j = 0; j < outgoingEdges.size(); ++j) {
          // Check that the vertex is valid.
          auto child = currentPlannerData->getVertex(outgoingEdges.at(j));
          if (child != ompl::base::PlannerData::NO_VERTEX) {
            // Add the parent.
            edges.emplace_back(scopedParent[0], scopedParent[1]);

            // Add the child.
            ompl::base::ScopedState<> scopedChild(context_->getStateSpace(), child.getState());
            edges.emplace_back(scopedChild[0], scopedChild[1]);
          }
        }
      }
    }

    // Activate this render state for the canvas.
    canvas.Activate(renderState);

    // Clear the viewport.
    glClear(GL_COLOR_BUFFER_BIT);

    // Draw the obstacles.
    if (optionDrawObstacles) {
      for (auto obstacle : context_->getObstacles()) {
        obstacle->accept(*this);
      }
      for (auto antiObstacle : context_->getAntiObstacles()) {
        antiObstacle->accept(*this);
      }
    }

    // Draw the vertices.
    glColor3fv(blue);
    pangolin::glDrawPoints(vertices);
    if (optionDrawEdges) {
      glColor3fv(green);
      pangolin::glDrawLines(edges);
    }

    // Draw the context.
    if (optionDrawContext) {
      context_->accept(*this);
    }

    pangolin::FinishFrame();
  }
}

void InteractiveVisualizer::visit(const CentreSquare& centreSquare) const {
  // Draw the start states.
  glColor3fv(green);
  glPointSize(5.0);
  std::vector<Eigen::Vector2d> starts;
  for (const auto& start : centreSquare.getStartStates()) {
    starts.emplace_back(start[0], start[1]);
  }
  pangolin::glDrawPoints(starts);
  // Draw the goal states.
  glColor3fv(red);
  std::vector<Eigen::Vector2d> goals;
  for (const auto& goal : centreSquare.getGoalStates()) {
    goals.emplace_back(goal[0], goal[1]);
  }
  pangolin::glDrawPoints(goals);
  glPointSize(2.0);
}

void InteractiveVisualizer::visit(const Hyperrectangle<BaseObstacle>& obstacle) const {
  glColor3fv(black);
  std::vector<double> widths = obstacle.getWidths();
  std::vector<double> midpoint = obstacle.getAnchorCoordinates();
  if (widths.size() == 2 && midpoint.size() == 2) {
    pangolin::glDrawRect(midpoint.at(0) - widths.at(0) / 2.0, midpoint.at(1) - widths.at(1) / 2.0,
                         midpoint.at(0) + widths.at(0) / 2.0, midpoint.at(1) + widths.at(1) / 2.0);
  }
}

void InteractiveVisualizer::visit(const Hyperrectangle<BaseAntiObstacle>& antiObstacle) const {
  glColor3fv(white);
  std::vector<double> widths = antiObstacle.getWidths();
  std::vector<double> midpoint = antiObstacle.getAnchorCoordinates();
  if (widths.size() == 2 && midpoint.size() == 2) {
    pangolin::glDrawRect(midpoint.at(0) - widths.at(0) / 2.0, midpoint.at(1) - widths.at(1) / 2.0,
                         midpoint.at(0) + widths.at(0) / 2.0, midpoint.at(1) + widths.at(1) / 2.0);
  }
}

}  // namespace ompltools

}  // namespace esp
