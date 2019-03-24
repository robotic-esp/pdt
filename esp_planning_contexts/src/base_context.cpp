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

/* Authors: Jonathan Gammell */

#include "esp_planning_contexts/base_context.h"

namespace esp {

namespace ompltools {

BaseContext::BaseContext(const unsigned int dim,
                         const std::vector<std::pair<double, double>> limits,
                         const double runSeconds, std::string name) :
    name_(name),
    dim_(dim),
    limits_(limits),
    targetDuration_(asrl::time::seconds(runSeconds)) {
}

ompl::base::SpaceInformationPtr BaseContext::getSpaceInformation() const {
  return si_;
}

ompl::base::ProblemDefinitionPtr BaseContext::newProblemDefinition() const {
  // Variables
  // The problem definition
  ompl::base::ProblemDefinitionPtr pdef;
  // Allocate
  pdef = std::make_shared<ompl::base::ProblemDefinition>(si_);

  // Store the optimization objective
  pdef->setOptimizationObjective(opt_);

  // Add the start state(s)
  for (unsigned int i = 0u; i < startStates_.size(); ++i) {
    pdef->addStartState(startStates_.at(i));
  }

  // Store
  pdef->setGoal(goalPtr_);

  // Return
  return pdef;
}

ompl::base::OptimizationObjectivePtr BaseContext::getOptimizationObjective() const {
  return opt_;
}

asrl::time::duration BaseContext::getTargetDuration() const {
  return targetDuration_;
}

ompl::base::GoalPtr BaseContext::getGoalPtr() const {
  return goalPtr_;
}

std::vector<ompl::base::ScopedState<>> BaseContext::getStartStates() const {
  return startStates_;
}

std::vector<ompl::base::ScopedState<>> BaseContext::getGoalStates() const {
  return goalStates_;
}

std::string BaseContext::getName() const {
  return name_;
}

std::vector<std::pair<double, double>> BaseContext::getLimits() const {
  return limits_;
}

unsigned int BaseContext::getDimensions() const {
  return dim_;
}

ompl::base::Cost BaseContext::getMinimum() const {
  // Return the minimum of each start to the goal
  ompl::base::Cost minCost = opt_->infiniteCost();

  // Iterate over the list of starts:
  for (unsigned int i = 0u; i < startStates_.size(); ++i) {
    // Store the best cost to go from this start
    minCost = opt_->betterCost(
        minCost, ompl::base::goalRegionCostToGo(startStates_.at(i).get(), goalPtr_.get()));
  }

  return minCost;
}

void BaseContext::print(const bool verbose /* == false */) const {
  std::cout << name_ << " in R^" << dim_ << ". runtime: " << asrl::time::seconds(targetDuration_)
            << ". target: " << opt_->getCostThreshold();
  if (this->knowsOptimum()) {
    std::cout << " (opt: " << this->getOptimum() << ")";
  }
  // No else
  std::cout << ". map: (" << limits_.at(0u).first << ", " << limits_.at(0u).second << "). "
            << this->lineInfo() << std::endl;

  if (verbose == true) {
    std::cout << "starts =" << std::endl;
    for (unsigned int i = 0u; i < startStates_.size(); ++i) {
      std::cout << "    [" << startStates_.at(i)[0u] << ", " << startStates_.at(i)[1u] << ", ..."
                << startStates_.at(i)[dim_ - 1u] << "]" << std::endl;
    }

    std::cout << "goals =" << std::endl;
    for (unsigned int i = 0u; i < goalStates_.size(); ++i) {
      std::cout << "    [" << goalStates_.at(i)[0u] << ", " << goalStates_.at(i)[1u] << ", ..."
                << goalStates_.at(i)[dim_ - 1u] << "]" << std::endl;
    }

    std::cout << this->paraInfo();
  }
}

std::string BaseContext::mfileHeader(bool monochrome) const {
  std::stringstream rval;

  rval << "%%%%%% Pre config %%%%%%" << std::endl;
  rval << "cla;" << std::endl;
  rval << "hold on;" << std::endl;
  rval << "% colours = permute(get(gca, 'colororder'), [1 3 2]);" << std::endl;
  rval << "% colours_resize = imresize(colours, 50.0, 'nearest');" << std::endl;
  rval << "% imshow(colours_resize);" << std::endl;
  rval << "% get(gca, 'colororder')" << std::endl;
  rval << "y = [0.9290 0.6940 0.1250]; %3" << std::endl;
  rval << "m = [0.4940 0.1840 0.5560]; %4" << std::endl;
  rval << "c = [0.3010 0.7450 0.9330]; %6" << std::endl;
  rval << "r = [0.6350 0.0780 0.1840]; %7" << std::endl;
  rval << "g = [0.4660 0.6740 0.1880]; %5" << std::endl;
  rval << "b = [0 0.4470 0.7410]; %1" << std::endl;
  rval << "o = [0.8500 0.3250 0.0980]; %2" << std::endl;
  rval << "w = [1 1 1];" << std::endl;
  rval << "k = [0 0 0];" << std::endl;
  rval << "gray30 = [0.7 0.7 0.7];" << std::endl;
  rval << "gray50 = [0.5 0.5 0.5];" << std::endl;
  rval << "gray70 = [0.3 0.3 0.3];" << std::endl;
  if (monochrome == false) {
    rval << "startColour = g;" << std::endl;
    rval << "goalColour = r;" << std::endl;
    rval << "vertexColour = [0.0 139/255 139/255];" << std::endl;
    //        rval << "vertexColour(vertexColour > 1) = 1;" << std::endl;
    rval << "startEdgeColour = b;" << std::endl;
    rval << "goalEdgeColour = c;" << std::endl;
    //        rval << "startEdgeColour(startEdgeColour > 1) = 1;" << std::endl;
    //        rval << "goalEdgeColour(goalEdgeColour > 1) = 1;" << std::endl;
    rval << "solnColour = m;" << std::endl;
    rval << "nextEdgeColour = o;" << std::endl;
    rval << "queueColour = y;" << std::endl;
    //        rval << "queueColour(queueColour > 1) = 1;" << std::endl;

    rval << "vertexSize = 1;" << std::endl;
    rval << "startVertexSize = 5;" << std::endl;
    rval << "goalVertexSize = startVertexSize ;" << std::endl;
    rval << "edgeWeight = 1;" << std::endl;
    rval << "solnWeight = 2*edgeWeight;" << std::endl;
  } else {
    rval << "startColour = k;" << std::endl;
    rval << "goalColour = k;" << std::endl;
    rval << "vertexColour = gray;" << std::endl;
    rval << "startEdgeColour = gray;" << std::endl;
    rval << "goalEdgeColour = gray;" << std::endl;
    rval << "solnColour = k;" << std::endl;
    rval << "nextEdgeColour = gray;" << std::endl;
    rval << "queueColour = gray;" << std::endl;

    rval << "vertexSize = 1;" << std::endl;
    rval << "startVertexSize = 5;" << std::endl;
    rval << "goalVertexSize = startVertexSize ;" << std::endl;
    rval << "edgeWeight = 1;" << std::endl;
    rval << "solnWeight = 3*edgeWeight;" << std::endl;
  }
  rval << "queueEllipseColour = gray30;" << std::endl;
  rval << "worldEllipseColour = gray70;" << std::endl;

  rval << "edgeStyle = '-';" << std::endl;
  rval << "solnStyle = '-';" << std::endl;
  rval << "nextEdgeWeight = 2*edgeWeight;" << std::endl;
  rval << "nextEdgeStyle = '-';" << std::endl;
  rval << "heuristicEdgeStyle = ':';" << std::endl;
  rval << "queueWeight = edgeWeight;" << std::endl;
  rval << "queueStyle = '-.';" << std::endl;
  rval << "queueEllipseStyle = '--';" << std::endl;
  rval << "worldEllipseStyle = '-.';" << std::endl;

  // The problem limits:
  rval << "%%%%%% Problem info %%%%%%" << std::endl;
  rval << "minX = " << limits_.at(0u).first << ";" << std::endl;
  rval << "maxX = " << limits_.at(0u).second << ";" << std::endl;
  rval << "minY = " << limits_.at(1u).first << ";" << std::endl;
  rval << "maxY = " << limits_.at(1u).second << ";" << std::endl;

  // Write the starts to mfile:
  rval << "%%%%%% Starts and Goals %%%%%%" << std::endl;
  rval << "xstarts = [";
  for (unsigned int i = 0u; i < startStates_.size(); ++i) {
    rval << " [" << startStates_.at(i)[0u] << "; " << startStates_.at(i)[1u] << "] ";
  }
  rval << "];" << std::endl;

  // And the goals to mfile:
  rval << "xgoals = [";
  for (unsigned int i = 0u; i < goalStates_.size(); ++i) {
    rval << " [" << goalStates_.at(i)[0u] << "; " << goalStates_.at(i)[1u] << "] ";
  }
  rval << "];" << std::endl;

  // Write the obstacles
  rval << "%%%%%% Obstacles %%%%%%" << std::endl;
  rval << obs_->mfile();

  // Return
  return rval.str();
}

std::string BaseContext::mfileFooter() const {
  std::stringstream rval;

  // Plot the start and goal vertices:
  rval << "for i = 1:size(xstarts,2)" << std::endl;
  rval << "    plot(xstarts(1,i), xstarts(2,i), 'o', 'Color', startColour, 'MarkerFaceColor', "
          "startColour, 'MarkerSize', startVertexSize);"
       << std::endl;
  rval << "end" << std::endl;
  rval << "for i = 1:size(xgoals,2)" << std::endl;
  rval << "    plot(xgoals(1,i), xgoals(2,i), 'o', 'Color', goalColour, 'MarkerFaceColor', "
          "goalColour, 'MarkerSize', goalVertexSize);"
       << std::endl;
  rval << "end" << std::endl;

  // Write some post config:
  rval << "%%%%%% Post config %%%%%%" << std::endl;
  rval << "axis equal;" << std::endl;
  rval << "xlim([minX maxX]);" << std::endl;
  rval << "ylim([minY maxY]);" << std::endl;
  rval << "box on;" << std::endl;
  rval << "grid off;" << std::endl;
  rval << "set(gca, 'XTickLabel', []);" << std::endl;
  rval << "set(gca, 'YTickLabel', []);" << std::endl;
  rval << "set(gca, 'XTick', get(gca, 'XTick'));" << std::endl;
  rval << "set(gca, 'YTick', get(gca, 'XTick'));" << std::endl;

  return rval.str();
}

}  // namespace ompltools

}  // namespace esp
