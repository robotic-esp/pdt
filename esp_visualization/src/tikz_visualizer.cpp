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

#include "esp_visualization/tikz_visualizer.h"

#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/bitstar/datastructures/Vertex.h>

#include "esp_tikz/tikz_draw.h"
#include "esp_tikz/tikz_node.h"

namespace esp {

namespace ompltools {

using namespace std::string_literals;

TikzVisualizer::TikzVisualizer(
    const std::shared_ptr<const Configuration>& config,
    const std::shared_ptr<RealVectorGeometricContext>& context,
    const std::pair<std::shared_ptr<ompl::base::Planner>, PLANNER_TYPE>& plannerPair) :
    config_(config),
    context_(context),
    plannerType_(plannerPair.second),
    name_(plannerPair.first->getName()),
    picture_(config) {
  if (context_->getStateSpace()->getType() != ompl::base::StateSpaceType::STATE_SPACE_REAL_VECTOR) {
    OMPL_ERROR("Tikz visualizer only tested for real vector state spaces.");
    throw std::runtime_error("Visualizer error.");
  }
  // Load colors from config.
  espColors_.emplace("espblack", config_->get<std::array<int, 3>>("colors/espblack"));
  espColors_.emplace("espwhite", config_->get<std::array<int, 3>>("colors/espwhite"));
  espColors_.emplace("espgray", config_->get<std::array<int, 3>>("colors/espgray"));
  espColors_.emplace("espblue", config_->get<std::array<int, 3>>("colors/espblue"));
  espColors_.emplace("espred", config_->get<std::array<int, 3>>("colors/espred"));
  espColors_.emplace("espyellow", config_->get<std::array<int, 3>>("colors/espyellow"));
  espColors_.emplace("espgreen", config_->get<std::array<int, 3>>("colors/espgreen"));
  espColors_.emplace("esppurple", config_->get<std::array<int, 3>>("colors/esppurple"));
  espColors_.emplace("esplightblue", config_->get<std::array<int, 3>>("colors/esplightblue"));
  espColors_.emplace("espdarkred", config_->get<std::array<int, 3>>("colors/espdarkred"));
}

void TikzVisualizer::render(const ompl::base::PlannerData& plannerData, std::size_t iteration,
                            const ompl::base::PathPtr path,
                            const std::shared_ptr<const PlannerSpecificData>& plannerSpecificData,
                            double iterationTime, double totalTime, double solutionCost) {
  if (context_->getDimension() != 2u) {
    OMPL_ERROR("Tikz visualizer can only handle two dimensional contexts.");
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

  // Draw the vertices and edges.
  for (std::size_t i = 0u; i < plannerData.numVertices(); ++i) {
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
                   "edge");
        }
      }
    }
  }

  // Draw the planner specific visualizations.
  drawPlannerSpecificVisualizations(plannerSpecificData);

  // Draw the solution path.
  drawSolution(path);

  // Clip the picture to the boundaries.
  auto boundaries = context_->getBoundaries();
  auto minX = boundaries.low.at(0);
  auto maxX = boundaries.high.at(0);
  auto minY = boundaries.low.at(1);
  auto maxY = boundaries.high.at(1);
  picture_.setClipCommand("\\clip ("s + std::to_string(minX) + ", "s + std::to_string(minY) +
                          ") rectangle (" + std::to_string(maxX) + ", "s + std::to_string(maxY) +
                          ");");
  picture_.setOptions(TikzPictureOptions({10, 10}));

  // Export to a file.
  std::stringstream texFilename;
  texFilename << config_->get<std::string>("experiment/context") << '_'
              << config_->get<std::string>("experiment/planner") << '_'
              << std::to_string(config_->get<std::size_t>("experiment/seed")) << "/tikzpictures/"
              << std::setfill('0') << std::setw(6) << iteration << ".tex";
  std::experimental::filesystem::path texPath(texFilename.str());
  picture_.write(texPath);

  // Compile the exported file.
  auto pngPath = compile(texPath, solutionCost, totalTime);

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
    const std::experimental::filesystem::path& texPath, double cost, double time) {
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
             << "\\makeatother "
             << "\\RequirePackage{luatex85}\n"
             << "\\documentclass[convert={density=300,outext=.png}]{"
                "standalone}\n"
             << "\\usepackage{xcolor}\n"
             << "\\usepackage{fontspec}\n"
             << "\\usepackage{tikz}\n\n";

  // Define the colors.
  for (const auto& [name, values] : espColors_) {
    standalone << "\\definecolor{" << name << "}{RGB}{" << values[0u] << ',' << values[1u] << ','
               << values[2u] << "}\n";
  }

  // Set the styles for the tikz elements.
  standalone << "\n\\tikzset{\n"
             << "  start/.style={fill = espgreen, circle, inner sep = 0pt, minimum width = 4pt},\n"
             << "  goal/.style={fill = espred, circle, inner sep = 0pt, minimum width = 4pt},\n"
             << "  vertex/.style={fill = espblue, circle, inner sep = 0pt, minimum width = 2pt},\n"
             << "  edge/.style={espblue, thick},\n"
             << "  solution/.style={espyellow, line width = 2.0pt},\n"
             << "  boundary/.style={draw = black, thick, fill = none},\n"
             << "  obstacle/.style={draw = none, fill = black},\n"
             << "  antiobstacle/.style={draw = white, fill = white}\n"
             << "}\n";

  // Create the document.
  standalone << "\n\\begin{document}\n"
             << "\\pagecolor{white}\n"
             << "\\begin{minipage}{10cm}\n"
             << "\n\\noindent\\Large\\vphantom{pP}"
             << config_->get<std::string>("planner/" + name_ + "/report/name")
             << "\\vphantom{pP}\\quad\\small Cost: ";
  if (std::isfinite(cost)) {
    standalone << cost;
  } else {
    standalone << "$\\infty$";
  }
  standalone << ", Time: " << time << "s\\\\\n\\resizebox*{10cm}{10cm}{%\n"
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

void TikzVisualizer::visit(const CentreSquare& context) const {
  // Draw the boundary.
  drawBoundary(context);

  // Draw the start states.
  drawStartState(context.getStartState());

  // Draw the goal states.
  drawGoalState(context.getGoalState());
}

void TikzVisualizer::visit(const DividingWalls& context) const {
  // Draw the boundary.
  drawBoundary(context);

  // Draw the start states.
  drawStartState(context.getStartState());

  // Draw the goal states.
  drawGoalState(context.getGoalState());
}

void TikzVisualizer::visit(const DoubleEnclosure& context) const {
  // Draw the boundary.
  drawBoundary(context);

  // Draw the start states.
  drawStartState(context.getStartState());

  // Draw the goal states.
  drawGoalState(context.getGoalState());
}

void TikzVisualizer::visit(const FlankingGap& context) const {
  // Draw the boundary.
  drawBoundary(context);

  // Draw the start states.
  drawStartState(context.getStartState());

  // Draw the goal states.
  drawGoalState(context.getGoalState());
}

void TikzVisualizer::visit(const FourRooms& context) const {
  // Draw the boundary.
  drawBoundary(context);

  // Draw the start states.
  drawStartState(context.getStartState());

  // Draw the goal states.
  drawGoalState(context.getGoalState());
}

void TikzVisualizer::visit(const GoalEnclosure& context) const {
  // Draw the boundary.
  drawBoundary(context);

  // Draw the start states.
  drawStartState(context.getStartState());

  // Draw the goal states.
  drawGoalState(context.getGoalState());
}

void TikzVisualizer::visit(const NarrowPassage& context) const {
  // Draw the boundary.
  drawBoundary(context);

  // Draw the start states.
  drawStartState(context.getStartState());

  // Draw the goal states.
  drawGoalState(context.getGoalState());
}

void TikzVisualizer::visit(const ObstacleFree& context) const {
  // Draw the boundary.
  drawBoundary(context);

  // Draw the start states.
  drawStartState(context.getStartState());

  // Draw the goal states.
  drawGoalState(context.getGoalState());
}

void TikzVisualizer::visit(const RandomRectangles& context) const {
  // Draw the boundary.
  drawBoundary(context);

  // Draw the start states.
  drawStartState(context.getStartState());

  // Draw the goal states.
  drawGoalState(context.getGoalState());
}

void TikzVisualizer::visit(const RandomRectanglesMultiStartGoal& context) const {
  // Draw the boundary.
  drawBoundary(context);

  // Draw the start states.
  drawStartStates(context.getStartStates());

  // Draw the goal states.
  drawGoalStates(context.getGoalStates());
}

void TikzVisualizer::visit(const RepeatingRectangles& context) const {
  // Draw the boundary.
  drawBoundary(context);

  // Draw the start states.
  drawStartState(context.getStartState());

  // Draw the goal states.
  drawGoalState(context.getGoalState());
}

void TikzVisualizer::visit(const StartEnclosure& context) const {
  // Draw the boundary.
  drawBoundary(context);

  // Draw the start states.
  drawStartState(context.getStartState());

  // Draw the goal states.
  drawGoalState(context.getGoalState());
}

void TikzVisualizer::visit(const WallGap& context) const {
  // Draw the boundary.
  drawBoundary(context);

  // Draw the start states.
  drawStartState(context.getStartState());

  // Draw the goal states.
  drawGoalState(context.getGoalState());
}

void TikzVisualizer::visit(const Hyperrectangle<BaseObstacle>& obstacle) const {
  drawRectangle(obstacle.getAnchorCoordinates().at(0), obstacle.getAnchorCoordinates().at(1),
                obstacle.getWidths().at(0), obstacle.getWidths().at(1),
                "obstacle");
}

void TikzVisualizer::visit(const Hyperrectangle<BaseAntiObstacle>& antiObstacle) const {
  drawRectangle(antiObstacle.getAnchorCoordinates().at(0),
                antiObstacle.getAnchorCoordinates().at(1), antiObstacle.getWidths().at(0) + 1e-2,
                antiObstacle.getWidths().at(1) + 1e-2, "antiobstacle");
}

void TikzVisualizer::drawBoundary(const RealVectorGeometricContext& context) const {
  const auto boundaries = context.getBoundaries();
  double midX = (boundaries.low.at(0u) + boundaries.high.at(0u)) / 2.0;
  double midY = (boundaries.low.at(1u) + boundaries.high.at(1u)) / 2.0;
  double widthX = boundaries.high.at(0u) - boundaries.low.at(0u);
  double widthY = boundaries.high.at(1u) - boundaries.low.at(1u);
  drawRectangle(midX, midY, widthX, widthY, "boundary");
}

void TikzVisualizer::drawStartVertex(const ompl::base::PlannerDataVertex& vertex) const {
  auto start = std::make_shared<TikzNode>();
  start->setOptions("start");
  double x = vertex.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->operator[](0u);
  double y = vertex.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->operator[](1u);
  start->setPosition(x, y);
  start->setName("vertex" + std::to_string(vertex.getTag()));
  picture_.addNode(start);
}

void TikzVisualizer::drawStartState(
    const ompl::base::ScopedState<ompl::base::RealVectorStateSpace>& state) const {
  auto start = std::make_shared<TikzNode>();
  start->setOptions("start");
  start->setPosition(state[0], state[1]);
  start->setName("start" + std::to_string(state[0]) + std::to_string(state[1]));
  picture_.addNode(start);
}

void TikzVisualizer::drawStartStates(
    const std::vector<ompl::base::ScopedState<ompl::base::RealVectorStateSpace>>& states) const {
  for (const auto& state : states) {
    drawStartState(state);
  }
}

void TikzVisualizer::drawGoalVertex(const ompl::base::PlannerDataVertex& vertex) const {
  auto goal = std::make_shared<TikzNode>();
  goal->setOptions("goal");
  double x = vertex.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->operator[](0u);
  double y = vertex.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->operator[](1u);
  goal->setPosition(x, y);
  goal->setName("vertex" + std::to_string(vertex.getTag()));
  picture_.addNode(goal);
}

void TikzVisualizer::drawGoalState(
    const ompl::base::ScopedState<ompl::base::RealVectorStateSpace>& state) const {
  auto goal = std::make_shared<TikzNode>();
  goal->setOptions("goal");
  goal->setPosition(state[0], state[1]);
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
                                   const std::string& options) const {
  auto rectangle = std::make_shared<TikzDraw>();
  rectangle->setFromPosition(midX - widthX / 2.0, midY - widthY / 2.0);
  rectangle->setToPosition(midX + widthX / 2.0, midY + widthY / 2.0);
  rectangle->setConnection("rectangle");
  rectangle->setOptions(options);

  picture_.addDraw(rectangle);
}

void TikzVisualizer::drawPlannerSpecificVisualizations(
    const std::shared_ptr<const PlannerSpecificData>& plannerSpecificData) const {
  if (plannerSpecificData == nullptr) {
    return;
  }
  switch (plannerType_) {
    case PLANNER_TYPE::BITSTAR:
    case PLANNER_TYPE::ABITSTAR: {
      drawBITstarSpecificVisualizations(
          std::dynamic_pointer_cast<const BITstarData>(plannerSpecificData));
      break;
    }
    case PLANNER_TYPE::AITSTAR: {
      drawAITstarSpecificVisualizations(
          std::dynamic_pointer_cast<const AITstarData>(plannerSpecificData));
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
  //   drawEdge(parent, child, "esplightblue, dash pattern=on 0.02mm off 0.03mm, line width =
  //   0.02mm");
  // }

  // Draw the next edge.
  auto nextEdgeStates = bitstarData->getNextEdge();

  // If there are no more edges in the queue, this will return nullptrs.
  if (nextEdgeStates.first != nullptr && nextEdgeStates.second != nullptr) {
    auto parent = nextEdgeStates.first->as<ompl::base::RealVectorStateSpace::StateType>();
    auto child = nextEdgeStates.second->as<ompl::base::RealVectorStateSpace::StateType>();
    drawEdge(parent, child, "edge, espred");
  }

  // Draw the ellipse.
  // auto nextEdgeQueueValue = bitstarData->getNextEdgeValueInQueue();
  // if (!std::isnan(nextEdgeQueueValue.value()) && !std::isinf(nextEdgeQueueValue.value())) {
  //   drawEllipse(nextEdgeQueueValue.value());
  // }
}

void TikzVisualizer::drawAITstarSpecificVisualizations(
    const std::shared_ptr<const AITstarData>& aitstarData) const {
  // // Draw the forward queue.
  // for (const auto& edge : aitstarData->getForwardQueue()) {
  //   drawEdge(edge.getParent()->getState()->as<ompl::base::RealVectorStateSpace::StateType>(),
  //            edge.getChild()->getState()->as<ompl::base::RealVectorStateSpace::StateType>(),
  //            "esplightblue, dash pattern=on 0.02mm off 0.03mm, line width = 0.02mm");
  // }

  // // Draw the top edge in the queue.
  // auto nextEdge = aitstarData->getNextEdge();
  // if (nextEdge.first && nextEdge.second) {
  //   drawEdge(nextEdge.first->as<ompl::base::RealVectorStateSpace::StateType>(),
  //            nextEdge.second->as<ompl::base::RealVectorStateSpace::StateType>(),
  //            "espred, line width = 0.1mm");
  // }

  // // Draw the backward queue.
  // for (const auto& vertex : aitstarData->getBackwardQueue()) {
  //   drawVertex(vertex->getState()->as<ompl::base::RealVectorStateSpace::StateType>(),
  //              "fill = espgray, inner sep = 0mm, circle, minimum size = 0.2mm");
  // }

  // Draw the backward search tree.
  for (const auto& vertex : aitstarData->getVerticesInBackwardSearchTree()) {
    // Add the edge to the parent.
    if (vertex->hasBackwardParent()) {
      auto state = vertex->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      auto parent = vertex->getBackwardParent()
                        ->getState()
                        ->as<ompl::base::RealVectorStateSpace::StateType>();
      drawEdge(parent, state, "espgray!50, line width = 0.02mm");
    }
  }

  // // Draw the next vertex in the queue.
  // auto nextVertex = aitstarData->getNextVertex();
  // if (nextVertex) {
  //   drawVertex(nextVertex->getState()->as<ompl::base::RealVectorStateSpace::StateType>(),
  //              "fill = espred, inner sep = 0mm, circle, minimum size = 0.3mm");
  // }
}

void TikzVisualizer::drawVertex(const ompl::base::PlannerDataVertex& vertex) const {
  auto node = std::make_shared<TikzNode>();
  node->setOptions("vertex");
  double x = vertex.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->operator[](0u);
  double y = vertex.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->operator[](1u);
  node->setPosition(x, y);
  node->setName("vertex"s + std::to_string(vertex.getTag()));
  picture_.addNode(node);
}

void TikzVisualizer::drawVertex(const ompl::base::RealVectorStateSpace::StateType* state,
                                const std::string& options) const {
  auto node = std::make_shared<TikzNode>();
  node->setOptions(options);
  double x = state->operator[](0u);
  double y = state->operator[](1u);
  node->setPosition(x, y);
  picture_.addNode(node);
}

void TikzVisualizer::drawEdge(const ompl::base::PlannerDataVertex& parent,
                              const ompl::base::PlannerDataVertex& child,
                              const std::string& options) const {
  auto draw = std::make_shared<TikzDraw>();
  draw->setFromPosition("vertex" + std::to_string(parent.getTag()) + ".center");
  draw->setToPosition("vertex" + std::to_string(child.getTag()) + ".center");
  draw->setConnection("--");
  draw->setOptions(options);
  picture_.addDraw(draw);
}

void TikzVisualizer::drawEdge(const ompl::base::RealVectorStateSpace::StateType* parent,
                              const ompl::base::RealVectorStateSpace::StateType* child,
                              const std::string& options) const {
  auto draw = std::make_shared<TikzDraw>();
  draw->setFromPosition(parent->operator[](0u), parent->operator[](1u));
  draw->setToPosition(child->operator[](0u), child->operator[](1u));
  draw->setConnection("--");
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
      drawEdge(parent, child, "solution");
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

  // Compute the semi major-axis.
  auto semiMajorAxis = cost / 2.0;
  auto semiMinorAxis = std::sqrt(std::pow((cost / 2.0), 2) - std::pow((length / 2.0), 2));

  // Get midpoint of start and goal.
  auto draw = std::make_shared<TikzDraw>();
  draw->setFromPosition((start.at(0u) + goal.at(0u)) / 2.0, (start.at(1u) + goal.at(1u)) / 2.0);
  draw->setToPosition(std::to_string(semiMajorAxis) + " and "s + std::to_string(semiMinorAxis));
  draw->setConnection("ellipse");
  draw->setOptions("densely dashed, thin, espgray");
  picture_.addDraw(draw);
}

}  // namespace ompltools

}  // namespace esp
