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

// Authors: Jonathan Gammell, Marlin Strub

#pragma once

// GCC complains about variadic macros with no arguments in pangolin. This should be allowed in
// c++20 (see http://www.open-std.org/jtc1/sc22/wg14/www/docs/n2034.htm). I haven't found a way to
// ignore just this specific warning with a #pragma, so I include this as a system header. Not sure
// of all the implications of this though.
#pragma GCC system_header
#include <pangolin/pangolin.h>

#include "esp_common/context_type.h"
#include "esp_common/planner_type.h"
#include "esp_obstacles/obstacle_visitor.h"
#include "esp_planning_contexts/all_contexts.h"
#include "esp_planning_contexts/context_visitor.h"
#include "esp_visualization/base_visualizer.h"

namespace esp {

namespace ompltools {

class InteractiveVisualizer : public BaseVisualizer, public ContextVisitor, public ObstacleVisitor {
 public:
  InteractiveVisualizer(
      const std::shared_ptr<BaseContext>& context,
      const std::pair<std::shared_ptr<ompl::base::Planner>, PLANNER_TYPE> plannerPair);
  ~InteractiveVisualizer() = default;

  void run();

 private:
  // Helper methods to draw.
  void drawRectangle2D(const std::vector<double>& midpoint, const std::vector<double>& widths,
                       const float* color) const;
  void drawStates(const std::vector<ompl::base::ScopedState<>>& states, const float* color,
                  float size) const;

  // Implement visualizations of contexts.
  void visit(const CentreSquare& context) const override;
  void visit(const DividingWalls& context) const override;
  void visit(const DoubleEnclosure& context) const override;
  void visit(const RandomRectangles& context) const override;
  void visit(const StartEnclosure& context) const override;

  // Implement visualizations of obstacles.
  void visit(const Hyperrectangle<BaseObstacle>& obstacle) const override;
  void visit(const Hyperrectangle<BaseAntiObstacle>& antiObstacle) const override;
};

}  // namespace ompltools

}  // namespace esp
