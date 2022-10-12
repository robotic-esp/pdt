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

#include "pdt/visualization/tikz_visualizer.h"

#include <ompl/base/goals/GoalSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/informedtrees/bitstar/Vertex.h>

#include "pdt/pgftikz/define_latex_colors.h"
#include "pdt/pgftikz/tikz_draw.h"
#include "pdt/pgftikz/tikz_node.h"

namespace pdt {

namespace visualization {

using namespace std::string_literals;

TikzVisualizer::TikzVisualizer(
    const std::shared_ptr<const config::Configuration>& config,
    const std::shared_ptr<planning_contexts::BaseContext>& context,
    const std::pair<std::shared_ptr<ompl::base::Planner>, common::PLANNER_TYPE>& plannerPair) :
    config_(config),
    context_(context),
    plannerType_(plannerPair.second),
    name_(plannerPair.first->getName()),
    picture_(config) {
  if (context_->getStateSpace()->getType() != ompl::base::StateSpaceType::STATE_SPACE_REAL_VECTOR &&
      context_->getStateSpace()->getType() != ompl::base::StateSpaceType::STATE_SPACE_SE2 &&
      context_->getStateSpace()->getType() != ompl::base::StateSpaceType::STATE_SPACE_REEDS_SHEPP) {
    OMPL_ERROR("Tikz visualizer only tested for real vector and SE2 state spaces.");
    throw std::runtime_error("Visualizer error.");
  }
}

void TikzVisualizer::render(const ompl::base::PlannerData& plannerData, const std::size_t iteration,
                            const std::size_t queryNumber, const ompl::base::PathPtr path,
                            const std::shared_ptr<const PlannerSpecificData>& plannerSpecificData,
                            const double iterationTime, const double totalTime,
                            const double solutionCost) {
  if (context_->getStateSpace()->getType() != ompl::base::StateSpaceType::STATE_SPACE_REAL_VECTOR &&
      context_->getStateSpace()->getType() != ompl::base::StateSpaceType::STATE_SPACE_SE2) {
    OMPL_ERROR("Tikz visualizer can only visualize 2d real vector or se2 contexts.");
    throw std::runtime_error("Visualizer error.");
  }

  // Draw the obstacles.
  for (const auto& obstacle : context_->getObstacles()) {
    obstacle->accept(*this);
  }

  // Draw the antiobstacles.
  for (const auto& antiObstacle : context_->getAntiObstacles()) {
    antiObstacle->accept(*this);
  }

  // Draw the context.
  context_->accept(*this);

  // Draw the planner specific visualizations.
  drawPlannerSpecificVisualizations(plannerSpecificData);

  // Draw the vertices and edges.
  for (auto i = 0u; i < plannerData.numVertices(); ++i) {
    // Get the vertex.
    auto vertex = plannerData.getVertex(i);

    // Draw if it is valid.
    if (vertex != ompl::base::PlannerData::NO_VERTEX /* && vertex.getTag() != 0u &&
                                                        vertex.getTag() != 1u */) {
      drawVertex(vertex.getState()->as<ompl::base::RealVectorStateSpace::StateType>());

      // Get all outgoing edges of this vertex.
      std::vector<unsigned int> outgoingEdges{};
      plannerData.getEdges(i, outgoingEdges);

      // Loop over the edges to draw them if they're connecting valid vertices.
      for (std::size_t j = 0u; j < outgoingEdges.size(); ++j) {
        // Get the child.
        auto child = plannerData.getVertex(outgoingEdges.at(j));

        // Draw the edge if the child is valid.
        if (child != ompl::base::PlannerData::NO_VERTEX) {
          drawEdge(vertex.getState()->as<ompl::base::RealVectorStateSpace::StateType>(),
                   child.getState()->as<ompl::base::RealVectorStateSpace::StateType>(),
                   pgftikz::zlevels::EDGE, "edge");
        }
      }
    }
  }

  const auto startGoalPair = context_->getNthStartGoalPair(queryNumber);

  // Draw the start states.
  drawStartStates(startGoalPair.start);

  // Draw the goal states.
  drawGoal(startGoalPair.goal);

  // Draw the solution path.
  drawSolution(path);

  // Clip the picture to the boundaries.
  const auto vectorContext =
      std::dynamic_pointer_cast<planning_contexts::RealVectorGeometricContext>(context_);
  const auto se2Context =
      std::dynamic_pointer_cast<planning_contexts::RealVectorGeometricContext>(context_);
  auto boundaries = ompl::base::RealVectorBounds(2u);
  if (vectorContext) {
    boundaries = vectorContext->getBoundaries();
  } else if (se2Context) {
    boundaries = se2Context->getBoundaries();
  } else {
    std::runtime_error("Tikz visualizer can only handle real vector and SE2 state spaces.");
  }
  auto minX = boundaries.low.at(0);
  auto maxX = boundaries.high.at(0);
  auto minY = boundaries.low.at(1);
  auto maxY = boundaries.high.at(1);
  picture_.setClipCommand("\\clip ("s + std::to_string(minX) + ", "s + std::to_string(minY) +
                          ") rectangle (" + std::to_string(maxX) + ", "s + std::to_string(maxY) +
                          ");");
  picture_.setOptions(pgftikz::TikzPictureOptions({10, 10}));

  // Export to a file.
  std::stringstream texFilename;
  texFilename << config_->get<std::string>("experiment/context") << '_'
              << config_->get<std::string>("experiment/planner") << '_'
              << std::to_string(config_->get<std::size_t>("experiment/seed")) << "/tikzpictures/"
              << std::setfill('0') << std::setw(6) << iteration << ".tex";
  std::experimental::filesystem::path texPath(texFilename.str());
  picture_.write(texPath);

  // Compile the exported file.
  auto pngPath = compile(texPath, solutionCost, totalTime, queryNumber);

  // Log the duration to the frame times file.
  logToFrameTimes(pngPath, iterationTime);

  // Export the config if it doesn't already exist.
  auto configPath = texPath.parent_path().parent_path() / "config.json";
  if (!std::experimental::filesystem::exists(configPath)) {
    config_->dumpAccessed(configPath.string());
  }

  // Clear the picture.
  picture_.clear();
}

std::experimental::filesystem::path TikzVisualizer::compile(
    const std::experimental::filesystem::path& texPath, const double cost, const double time,
    const std::size_t queryNumber) {
  // Get the current path.
  auto currentPath = std::experimental::filesystem::current_path();

  // Create the png folder if it does not yet exist.
  std::experimental::filesystem::create_directories(texPath.parent_path().parent_path() / "png");

  // Create the standalone latex file.
  std::experimental::filesystem::path standalonePath =
      (texPath.parent_path().parent_path() / "png" / texPath.stem()).string() + ".tex"s;
  std::ofstream standalone;
  standalone.open(standalonePath.string());
  standalone.precision(5);
  standalone << std::fixed;

  // Load the required packages.
  standalone << "\\RequirePackage{shellesc}\n"
             << "\\RequirePackage{pdftexcmds}\n"
             << "\\makeatletter\n"
             << "\\let\\pdfshellescape\\pdf@shellescape\n"
             << "\\makeatother\n"
             << "\\RequirePackage{luatex85}\n"
             << "\\documentclass[convert={density=300,outext=.png}]{"
                "standalone}\n"
             << "\\usepackage{xcolor}\n"
             << "\\usepackage{fontspec}\n"
             << "\\setmainfont{Roboto}"
             << "\\usepackage{tikz}\n\n";

  // Define the colors.
  standalone << pgftikz::defineLatexColors(config_);

  // Set the styles for the tikz elements.
  standalone << "\n\\tikzset{\n"
             << "  start/.style={fill = pdtgreen, circle, inner sep = 0pt, minimum width = 4pt},\n"
             << "  goal/.style={fill = pdtred, circle, inner sep = 0pt, minimum width = 4pt},\n"
             << "  goal region/.style={fill = pdtred, inner sep = 0pt, opacity = 0.8},\n"
             << "  vertex/.style={fill = pdtblue, circle, inner sep = 0pt, minimum width = 2pt},\n"
             << "  edge/.style={pdtblue, thick},\n"
             << "  solution/.style={pdtyellow, line width = 2.0pt},\n"
             << "  boundary/.style={draw = black, thick, fill = none},\n"
             << "  obstacle/.style={draw = none, fill = black},\n"
             << "  antiobstacle/.style={draw = white, fill = white}\n"
             << "}\n";

  // Create the document.
  standalone.precision(3);
  standalone << "\n\\begin{document}\n"
             << "\\pagecolor{white}\n"
             << "\\begin{minipage}{10cm}\n"
             << "\n\\noindent\\Huge\\vphantom{pP}\\textbf{"
             << config_->get<std::string>("planner/" + name_ + "/report/name")
             << "}\\vphantom{pP}\\\\\\LARGE Query: " << queryNumber << ", Time: " << time
             << "s, Cost: ";
  if (std::isfinite(cost)) {
    standalone << cost;
  } else {
    standalone << "$\\infty$";
  }
  standalone << "\\\\\n\\resizebox*{10cm}{10cm}{%\n"
             << "\\noindent\\input{" << (currentPath / texPath).string() << "}%\n"
             << "}\n\\end{minipage}\n"
             << "\\end{document}";

  standalone.close();

  // Compiling with lualatex is slower than pdflatex but has dynamic memory allocation. Since
  // these plots can be quite large, pdflatex has run into memory issues. Lualatex should be
  // available with all major tex distributions.
  auto cmd = "cd \""s + standalonePath.parent_path().string() +
             "\" && lualatex --interaction=nonstopmode --shell-escape > /dev/null \""s +
             // "\" && lualatex --interaction=nonstopmode --shell-escape \""s +
             (currentPath / standalonePath).string() + "\" && cd \""s + currentPath.string() + '\"';
  int retval = std::system(cmd.c_str());
  retval = std::system(cmd.c_str());
  (void)retval;

  return (currentPath / standalonePath).replace_extension(".png");
}

void TikzVisualizer::logToFrameTimes(const std::experimental::filesystem::path& pngPath,
                                     double iterationTime) {
  if (!std::experimental::filesystem::exists(pngPath.parent_path() / "frame_times.txt")) {
    frameTimes_.open((pngPath.parent_path() / "frame_times.txt").string());
    if (!frameTimes_.is_open()) {
      OMPL_ERROR("Could not open file.");
    }
    frameTimes_ << "ffconcat version 1.0\n";
    frameTimes_ << "# duration times are x 1000.\n";
    frameTimes_.close();
  }
  frameTimes_.open((pngPath.parent_path() / "frame_times.txt").string(),
                   std::fstream::in | std::fstream::out | std::fstream::app);
  if (!frameTimes_.is_open()) {
    OMPL_ERROR("Could not open file.");
  }
  frameTimes_.precision(6);
  frameTimes_ << std::fixed;
  frameTimes_ << "file " << pngPath.filename().string() << '\n'
              << "duration " << 1000.0 * iterationTime << '\n';
  frameTimes_.close();
}

void TikzVisualizer::visit(const planning_contexts::CenterSquare& context) const {
  // Draw the boundary.
  drawBoundary(context);
}

void TikzVisualizer::visit(const planning_contexts::DividingWalls& context) const {
  // Draw the boundary.
  drawBoundary(context);
}

void TikzVisualizer::visit(const planning_contexts::DoubleEnclosure& context) const {
  // Draw the boundary.
  drawBoundary(context);
}

void TikzVisualizer::visit(const planning_contexts::FlankingGap& context) const {
  // Draw the boundary.
  drawBoundary(context);
}

void TikzVisualizer::visit(const planning_contexts::FourRooms& context) const {
  // Draw the boundary.
  drawBoundary(context);
}

void TikzVisualizer::visit(const planning_contexts::GoalEnclosure& context) const {
  // Draw the boundary.
  drawBoundary(context);
}

void TikzVisualizer::visit(const planning_contexts::NarrowPassage& context) const {
  // Draw the boundary.
  drawBoundary(context);
}

void TikzVisualizer::visit(const planning_contexts::ObstacleFree& context) const {
  // Draw the boundary.
  drawBoundary(context);
}

void TikzVisualizer::visit(const planning_contexts::RandomRectangles& context) const {
  // Draw the boundary.
  drawBoundary(context);
}

void TikzVisualizer::visit(const planning_contexts::RandomRectanglesMultiStartGoal& context) const {
  // Draw the boundary.
  drawBoundary(context);
}

void TikzVisualizer::visit(const planning_contexts::ReedsSheppRandomRectangles& context) const {
  // Draw the boundary.
  drawBoundary(context);
}

void TikzVisualizer::visit(const planning_contexts::RepeatingRectangles& context) const {
  // Draw the boundary.
  drawBoundary(context);
}

void TikzVisualizer::visit(const planning_contexts::StartEnclosure& context) const {
  // Draw the boundary.
  drawBoundary(context);
}

void TikzVisualizer::visit(const planning_contexts::WallGap& context) const {
  // Draw the boundary.
  drawBoundary(context);
}

void TikzVisualizer::visit(
    const obstacles::Hyperrectangle<obstacles::BaseObstacle>& obstacle) const {
  drawRectangle(obstacle.getAnchorCoordinates().at(0), obstacle.getAnchorCoordinates().at(1),
                obstacle.getWidths().at(0), obstacle.getWidths().at(1), pgftikz::zlevels::OBSTACLE,
                "obstacle");
}

void TikzVisualizer::visit(
    const obstacles::Hyperrectangle<obstacles::BaseAntiObstacle>& antiObstacle) const {
  drawRectangle(antiObstacle.getAnchorCoordinates().at(0),
                antiObstacle.getAnchorCoordinates().at(1), antiObstacle.getWidths().at(0) + 1e-2,
                antiObstacle.getWidths().at(1) + 1e-2, pgftikz::zlevels::ANTIOBSTACLE,
                "antiobstacle");
}

void TikzVisualizer::drawBoundary(
    const planning_contexts::RealVectorGeometricContext& context) const {
  const auto boundaries = context.getBoundaries();
  double midX = (boundaries.low.at(0u) + boundaries.high.at(0u)) / 2.0;
  double midY = (boundaries.low.at(1u) + boundaries.high.at(1u)) / 2.0;
  double widthX = boundaries.high.at(0u) - boundaries.low.at(0u);
  double widthY = boundaries.high.at(1u) - boundaries.low.at(1u);
  drawRectangle(midX, midY, widthX, widthY, pgftikz::zlevels::BOUNDARY, "boundary");
}

void TikzVisualizer::drawBoundary(
    const planning_contexts::ReedsSheppRandomRectangles& context) const {
  const auto boundaries = context.getBoundaries();
  double midX = (boundaries.low.at(0u) + boundaries.high.at(0u)) / 2.0;
  double midY = (boundaries.low.at(1u) + boundaries.high.at(1u)) / 2.0;
  double widthX = boundaries.high.at(0u) - boundaries.low.at(0u);
  double widthY = boundaries.high.at(1u) - boundaries.low.at(1u);
  drawRectangle(midX, midY, widthX, widthY, pgftikz::zlevels::BOUNDARY, "boundary");
}

void TikzVisualizer::drawGoal(const std::shared_ptr<ompl::base::Goal>& goal) const {
  switch (goal->getType()) {
    case ompl::base::GoalType::GOAL_STATE: {
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goalState(
          context_->getStateSpace(), goal->as<ompl::base::GoalState>()->getState());
      auto goal = std::make_shared<pgftikz::TikzNode>();
      goal->setPosition(goalState[0], goalState[1]);
      goal->setZLevel(pgftikz::zlevels::GOAL);
      goal->setOptions("goal");
      goal->setName("goal");
      picture_.addNode(goal);
      break;
    }
    case ompl::base::GoalType::GOAL_STATES: {
      for (auto i = 0u; i < goal->as<ompl::base::GoalStates>()->getStateCount(); ++i) {
        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goalState(
            context_->getStateSpace(), goal->as<ompl::base::GoalStates>()->getState(i));
        auto goal = std::make_shared<pgftikz::TikzNode>();
        goal->setPosition(goalState[0], goalState[1]);
        goal->setZLevel(pgftikz::zlevels::GOAL);
        goal->setOptions("goal");
        goal->setName("goal");
        picture_.addNode(goal);
      }
      break;
    }
#ifdef PDT_EXTRA_GOAL_SPACE
    case ompl::base::GoalType::GOAL_SPACE: {
      auto goalSpace = goal->as<ompl::base::GoalSpace>()->getSpace();
      auto bounds = goalSpace->as<ompl::base::RealVectorStateSpace>()->getBounds();
      std::vector<double> widths;
      std::vector<double> anchor;
      for (auto i = 0u; i < goalSpace->getDimension(); ++i) {
        widths.push_back(bounds.high[i] - bounds.low[i]);
        anchor.push_back((bounds.high[i] + bounds.low[i]) / 2.0);
      }
      drawRectangle(anchor[0], anchor[1], widths[0], widths[1], pgftikz::zlevels::GOAL,
                    "goal region");
      break;
    }
#endif  // #ifdef PDT_EXTRA_GOAL_SPACE
    default: { throw std::runtime_error("Can not visualize goal type."); }
  }
}

void TikzVisualizer::drawStartVertex(const ompl::base::PlannerDataVertex& vertex) const {
  auto start = std::make_shared<pgftikz::TikzNode>();
  start->setOptions("start");
  double x = vertex.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->operator[](0u);
  double y = vertex.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->operator[](1u);
  start->setPosition(x, y);
  start->setZLevel(pgftikz::zlevels::START);
  start->setName("vertex" + std::to_string(vertex.getTag()));
  picture_.addNode(start);
}

void TikzVisualizer::drawStartState(
    const ompl::base::ScopedState<ompl::base::RealVectorStateSpace>& state) const {
  auto start = std::make_shared<pgftikz::TikzNode>();
  start->setOptions("start");
  start->setPosition(state[0], state[1]);
  start->setZLevel(pgftikz::zlevels::START);
  start->setName("start" + std::to_string(state[0]) + std::to_string(state[1]));
  picture_.addNode(start);
}

void TikzVisualizer::drawStartStates(const std::vector<ompl::base::ScopedState<>>& states) const {
  for (const auto& state : states) {
    drawStartState(state);
  }
}

void TikzVisualizer::drawGoalVertex(const ompl::base::PlannerDataVertex& vertex) const {
  auto goal = std::make_shared<pgftikz::TikzNode>();
  goal->setOptions("goal");
  double x = vertex.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->operator[](0u);
  double y = vertex.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->operator[](1u);
  goal->setPosition(x, y);
  goal->setZLevel(pgftikz::zlevels::GOAL);
  goal->setName("vertex" + std::to_string(vertex.getTag()));
  picture_.addNode(goal);
}

void TikzVisualizer::drawGoalState(
    const ompl::base::ScopedState<ompl::base::RealVectorStateSpace>& state) const {
  auto goal = std::make_shared<pgftikz::TikzNode>();
  goal->setOptions("goal");
  goal->setPosition(state[0], state[1]);
  goal->setZLevel(pgftikz::zlevels::GOAL);
  goal->setName("goal" + std::to_string(state[0]) + std::to_string(state[1]));
  picture_.addNode(goal);
}

void TikzVisualizer::drawGoalStates(
    const std::vector<ompl::base::ScopedState<ompl::base::RealVectorStateSpace>>& states) const {
  for (const auto& state : states) {
    drawGoalState(state);
  }
}

void TikzVisualizer::drawRectangle(double midX, double midY, double widthX, double widthY,
                                   std::size_t zLevel, const std::string& options) const {
  auto rectangle = std::make_shared<pgftikz::TikzDraw>();
  rectangle->setFromPosition(midX - widthX / 2.0, midY - widthY / 2.0);
  rectangle->setToPosition(midX + widthX / 2.0, midY + widthY / 2.0);
  rectangle->setConnection("rectangle");
  rectangle->setZLevel(zLevel);
  rectangle->setOptions(options);

  picture_.addDraw(rectangle);
}

void TikzVisualizer::drawPlannerSpecificVisualizations(
    const std::shared_ptr<const PlannerSpecificData>& plannerSpecificData) const {
  if (plannerSpecificData == nullptr) {
    return;
  }
  switch (plannerType_) {
    case common::PLANNER_TYPE::BITSTAR:
    case common::PLANNER_TYPE::ABITSTAR: {
      drawBITstarSpecificVisualizations(
          std::dynamic_pointer_cast<const BITstarData>(plannerSpecificData));
      break;
    }
    case common::PLANNER_TYPE::AITSTAR: {
      drawAITstarSpecificVisualizations(
          std::dynamic_pointer_cast<const AITstarData>(plannerSpecificData));
      break;
    }
#ifdef PDT_EXTRA_EITSTAR_PR
    case common::PLANNER_TYPE::EIRMSTAR:
    case common::PLANNER_TYPE::EITSTAR: {
      drawEITstarSpecificVisualizations(
          std::dynamic_pointer_cast<const EITstarData>(plannerSpecificData));
      break;
    }
#endif  // #ifdef PDT_EXTRA_EITSTAR_PR
    case common::PLANNER_TYPE::LAZYPRMSTAR: {
      drawLazyPRMstarSpecificVisualizations(
          std::dynamic_pointer_cast<const LazyPRMstarData>(plannerSpecificData));
      break;
    }
    default: { return; }
  }
}

void TikzVisualizer::drawBITstarSpecificVisualizations(
    const std::shared_ptr<const BITstarData>& bitstarData) const {
  // // Draw the edge queue.
  // auto edgeQueue = bitstarData->getEdgeQueue();
  // for (const auto& edge : edgeQueue) {
  //   auto parent = edge.first->state()->as<ompl::base::RealVectorStateSpace::StateType>();
  //   auto child = edge.second->state()->as<ompl::base::RealVectorStateSpace::StateType>();
  //   drawEdge(parent, child, "pdtlightblue, dash pattern=on 0.02mm off 0.03mm, line width =
  //   0.02mm");
  // }

  // Draw the next edge.
  auto nextEdgeStates = bitstarData->getNextEdge();

  // If there are no more edges in the queue, this will return nullptrs.
  if (nextEdgeStates.first != nullptr && nextEdgeStates.second != nullptr) {
    auto parent = nextEdgeStates.first->as<ompl::base::RealVectorStateSpace::StateType>();
    auto child = nextEdgeStates.second->as<ompl::base::RealVectorStateSpace::StateType>();
    drawEdge(parent, child, pgftikz::zlevels::EDGE_HIGHLIGHT, "edge, pdtred");
  }

  // Draw the ellipse.
  auto nextEdgeQueueValue = bitstarData->getNextEdgeValueInQueue();
  if (!std::isnan(nextEdgeQueueValue.value()) && !std::isinf(nextEdgeQueueValue.value())) {
    drawEllipse(nextEdgeQueueValue.value());
  }
}

void TikzVisualizer::drawAITstarSpecificVisualizations(
    const std::shared_ptr<const AITstarData>& aitstarData) const {
  // // Draw the forward queue.
  // for (const auto& edge : aitstarData->getForwardQueue()) {
  //   drawEdge(edge.getParent()->getState()->as<ompl::base::RealVectorStateSpace::StateType>(),
  //            edge.getChild()->getState()->as<ompl::base::RealVectorStateSpace::StateType>(),
  //            "pdtlightblue, dash pattern=on 0.02mm off 0.03mm, line width = 0.02mm");
  // }

  // // Draw the backward queue.
  // for (const auto& vertex : aitstarData->getBackwardQueue()) {
  //   drawVertex(vertex->getState()->as<ompl::base::RealVectorStateSpace::StateType>(),
  //              "fill = pdtgray, inner sep = 0mm, circle, minimum size = 0.2mm");
  // }

  // Draw the backward search tree.
  for (const auto& vertex : aitstarData->getVerticesInBackwardSearchTree()) {
    // Add the edge to the parent.
    if (vertex->hasReverseParent()) {
      auto state = vertex->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      auto parent =
          vertex->getReverseParent()->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      drawEdge(parent, state, pgftikz::zlevels::EDGE, "edge, pdtlightblue");
    }
  }

  // Draw the top edge in the queue.
  auto nextEdge = aitstarData->getNextEdge();
  if (nextEdge.first && nextEdge.second) {
    drawEdge(nextEdge.first->as<ompl::base::RealVectorStateSpace::StateType>(),
             nextEdge.second->as<ompl::base::RealVectorStateSpace::StateType>(),
             pgftikz::zlevels::EDGE_HIGHLIGHT, "edge, pdtred");
  }

  // // Draw the next vertex in the queue.
  // auto nextVertex = aitstarData->getNextVertex();
  // if (nextVertex) {
  //   drawVertex(nextVertex->getState()->as<ompl::base::RealVectorStateSpace::StateType>(),
  //              "fill = pdtred, inner sep = 0mm, circle, minimum size = 0.3mm");
  // }
}

#ifdef PDT_EXTRA_EITSTAR_PR
void TikzVisualizer::drawEITstarSpecificVisualizations(
    const std::shared_ptr<const EITstarData>& eitstarData) const {
  // // Draw the backward search tree.
  // for (const auto& edge : eitstarData->getReverseTree()) {
  //   const auto source = edge.source->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
  //   const auto target = edge.target->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
  //   drawEdge(source, target, "edge, pdtlightblue");
  // }

  // Draw the top edge in the queue.
  auto nextEdge = eitstarData->getNextForwardEdge();
  if (nextEdge.source && nextEdge.target) {
    drawEdge(nextEdge.source->raw()->as<ompl::base::RealVectorStateSpace::StateType>(),
             nextEdge.target->raw()->as<ompl::base::RealVectorStateSpace::StateType>(),
             pgftikz::zlevels::EDGE_HIGHLIGHT, "edge, pdtred");
  }

  // // Draw the top edge in the queue.
  // auto nextEdge = eitstarData->getNextForwardEdge();
  // if (nextEdge.source && nextEdge.target) {
  //   drawEdge(nextEdge.source->raw()->as<ompl::base::RealVectorStateSpace::StateType>(),
  //            nextEdge.target->raw()->as<ompl::base::RealVectorStateSpace::StateType>(),
  //            "edge, pdtred");
  // }
}
#endif  // #ifdef PDT_EXTRA_EITSTAR_PR

void TikzVisualizer::drawLazyPRMstarSpecificVisualizations(
    const std::shared_ptr<const LazyPRMstarData>& lPRMstarData) const {
  if (context_->getDimension() == 2u) {
    // draw all valid edges
    const auto& edges = lPRMstarData->getValidEdges();
    for (const auto& edge : edges) {
      auto source = edge.first.getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      auto target = edge.second.getState()->as<ompl::base::RealVectorStateSpace::StateType>();

      drawEdge(source, target, pgftikz::zlevels::EDGE_LOWLIGHT, "edge, pdtgray");
    }

    // draw all new edges
    for (const auto& idx : lPRMstarData->getNewEdgeIndices()) {
      auto source = edges[idx].first.getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      auto target = edges[idx].second.getState()->as<ompl::base::RealVectorStateSpace::StateType>();

      drawEdge(source, target, pgftikz::zlevels::EDGE_HIGHLIGHT, "edge, pdtred");
    }
  }
}

void TikzVisualizer::drawVertex(const ompl::base::PlannerDataVertex& vertex) const {
  auto node = std::make_shared<pgftikz::TikzNode>();
  node->setOptions("vertex");
  double x = vertex.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->operator[](0u);
  double y = vertex.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->operator[](1u);
  node->setPosition(x, y);
  node->setZLevel(pgftikz::zlevels::VERTEX);
  node->setName("vertex"s + std::to_string(vertex.getTag()));
  picture_.addNode(node);
}

void TikzVisualizer::drawVertex(const ompl::base::RealVectorStateSpace::StateType* state,
                                const std::string& options) const {
  auto node = std::make_shared<pgftikz::TikzNode>();
  node->setOptions(options);
  double x = state->operator[](0u);
  double y = state->operator[](1u);
  node->setPosition(x, y);
  node->setZLevel(pgftikz::zlevels::VERTEX);
  picture_.addNode(node);
}

void TikzVisualizer::drawEdge(const ompl::base::PlannerDataVertex& parent,
                              const ompl::base::PlannerDataVertex& child, std::size_t zLevel,
                              const std::string& options) const {
  auto draw = std::make_shared<pgftikz::TikzDraw>();
  draw->setFromPosition("vertex" + std::to_string(parent.getTag()) + ".center");
  draw->setToPosition("vertex" + std::to_string(child.getTag()) + ".center");
  draw->setConnection("--");
  draw->setZLevel(zLevel);
  draw->setOptions(options);
  picture_.addDraw(draw);
}

void TikzVisualizer::drawEdge(const ompl::base::RealVectorStateSpace::StateType* parent,
                              const ompl::base::RealVectorStateSpace::StateType* child,
                              std::size_t zLevel, const std::string& options) const {
  auto draw = std::make_shared<pgftikz::TikzDraw>();
  draw->setFromPosition(parent->operator[](0u), parent->operator[](1u));
  draw->setToPosition(child->operator[](0u), child->operator[](1u));
  draw->setConnection("--");
  draw->setZLevel(zLevel);
  draw->setOptions(options);
  picture_.addDraw(draw);
}

void TikzVisualizer::drawSolution(const ompl::base::PathPtr path) const {
  if (path != nullptr) {
    auto states = path->as<ompl::geometric::PathGeometric>()->getStates();
    if (states.size() < 2u) {
      throw std::runtime_error("Encountered solution path with less than two states.");
    }
    for (std::size_t i = 1u; i < states.size(); ++i) {
      auto parent = states.at(i - 1u)->as<ompl::base::RealVectorStateSpace::StateType>();
      auto child = states.at(i)->as<ompl::base::RealVectorStateSpace::StateType>();
      drawEdge(parent, child, pgftikz::zlevels::SOLUTION, "solution");
    }
  }
}

void TikzVisualizer::drawEllipse(double cost) const {
  // Get the context name.
  auto contextName = config_->get<std::string>("experiment/context");
  auto start = config_->get<std::vector<double>>("context/"s + contextName + "/start");
  auto goal = config_->get<std::vector<double>>("context/"s + contextName + "/goal");
  auto length =
      std::sqrt(std::pow(start.at(0u) - goal.at(0u), 2) + std::pow(start.at(1u) - goal.at(1u), 2));
  auto angle = std::atan2(goal.at(1u) - start.at(1u), goal.at(0u) - start.at(0u));

  std::vector<double> center{(start.at(0u) + goal.at(0u)) / 2.0,
                             (start.at(1u) + goal.at(1u)) / 2.0};

  // Compute the semi major-axis.
  auto majorAxisLength = cost / 2.0;
  auto minorAxisLength = std::sqrt(cost * cost - length * length) / 2.0;

  constexpr auto piHalf = 3.1415926535897 / 2.0;
  std::vector<double> majorAxis{majorAxisLength * std::cos(angle),
                                majorAxisLength * std::sin(angle)};
  std::vector<double> minorAxis{minorAxisLength * std::cos(angle + piHalf),
                                minorAxisLength * std::sin(angle + piHalf)};

  std::ostringstream ellipseCommand{};

  ellipseCommand << "\\color{gray}\n\\pgfpathellipse"
                 // << "{\\pgfpointxy{" << start.at(0u) << "}{" << start.at(1u) << "}}"
                 // << "{\\pgfpointxy{" << cost << "}{" << 0 << "}}"
                 // << "{\\pgfpointxy{" << 0 << "}{" << cost << "}}"
                 << "{\\pgfpointxy{" << center.at(0u) << "}{" << center.at(1u) << "}}"
                 << "{\\pgfpointxy{" << majorAxis.at(0u) << "}{" << majorAxis.at(1u) << "}}"
                 << "{\\pgfpointxy{" << minorAxis.at(0u) << "}{" << minorAxis.at(1u) << "}}\n"
                 << "\\pgfsetdash{{1.2mm}{0.8mm}}{0cm}\n"
                 << "\\pgfsetlinewidth{0.8mm}"
                 << "\\pgfusepath{draw}\n";
  picture_.addText(ellipseCommand.str());
}

}  // namespace visualization

}  // namespace pdt
