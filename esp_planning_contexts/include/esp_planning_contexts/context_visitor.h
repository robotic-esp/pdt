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

namespace esp {

namespace ompltools {

// Forward declarations.
class CentreSquare;
class DividingWall;
class DoubleEnclosure;
class FlankingGap;
class GoalEnclosure;
class MultiStartGoal;
class ObstacleFree;
class RandomRectangles;
class RegularRectangles;
class Spiral;
class StartEnclosure;
class WallGap;

class ContextVisitor {
 public:
  ContextVisitor() = default;
  virtual ~ContextVisitor() = default;

  // Any context visitor must implement its actions on all contexts.
  virtual void visit(const CentreSquare &context) const = 0;
  // virtual void visit(const DividingWall &context) const = 0;
  // virtual void visit(const DoubleEnclosure &context) const = 0;
  // virtual void visit(const FlankingGap &context) const = 0;
  // virtual void visit(const GoalEnclosure &context) const = 0;
  // virtual void visit(const MultiStartGoal &context) const = 0;
  // virtual void visit(const ObstacleFree &context) const = 0;
  // virtual void visit(const RandomRectangles &context) const = 0;
  // virtual void visit(const RegularRectangles &context) const = 0;
  // virtual void visit(const Spiral &context) const = 0;
  // virtual void visit(const StartEnclosure &context) const = 0;
  // virtual void visit(const WallGap &context) const = 0;

  // This is only needed until all other contexts are implemented.
  // ADL should resolve to the above methods.
  template <typename C>
  void visit(const C& /* context */) const {}
};

}  // namespace ompltools

}  // namespace esp
