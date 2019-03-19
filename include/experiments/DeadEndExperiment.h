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

#ifndef EXPERIMENTS_DEAD_END_EXPERIMENT
#define EXPERIMENTS_DEAD_END_EXPERIMENT

#include "experiments/BaseExperiment.h"

// An Obstacle-World
#include "obstacles/HyperrectangleObstacles.h"

/** \brief An experiment with heuristic breaking deadend "C" */
class DeadEndExperiment : public BaseExperiment {
 public:
  /** \brief Constructor */
  DeadEndExperiment(const double distFraction, const double runSeconds,
                    const double checkResolution);

  /** \brief Whether the problem has an exact expression for the optimum */
  virtual bool knowsOptimum() const;

  /** \brief This problem knows its optimum */
  virtual ompl::base::Cost getOptimum() const;

  /** \brief Set the target cost. How this is defined depends on the specific experiment. */
  virtual void setTarget(double targetSpecifier);

  /** \brief Derived class specific information to include in the title line. */
  virtual std::string lineInfo() const;

  /** \brief Derived class specific information to include at the end. */
  virtual std::string paraInfo() const;

 protected:
  // Variables
  /** \brief The obstacle world */
  std::shared_ptr<HyperrectangleObstacles> rectObs_;
  /** \brief The lower-left corners of the obstacles*/
  std::shared_ptr<ompl::base::ScopedState<> > topHorizontal_;
  std::shared_ptr<ompl::base::ScopedState<> > sideVertical_;
  std::shared_ptr<ompl::base::ScopedState<> > bottomHorizontal_;
  /** The widths of the obstacles */
  std::vector<double> topHorizontalWidths_;
  std::vector<double> sideVerticalWidths_;
  std::vector<double> bottomHorizontalWidths_;

  // Constant Parameters
  /** \brief The basic thickness of the obstacle. */
  double obsThickness_;
};

typedef std::shared_ptr<DeadEndExperiment> DeadEndExperimentPtr;

#endif  // EXPERIMENTS_DEAD_END_EXPERIMENT
