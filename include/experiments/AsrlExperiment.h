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

#ifndef EXPERIMENTS_ASRL_EXPERIMENT
#define EXPERIMENTS_ASRL_EXPERIMENT

#include "experiments/BaseExperiment.h"

// An Obstacle-World
#include "obstacles/CsvObstacle.h"

/** \brief An experiment around the ASRL logo */
class AsrlExperiment : public BaseExperiment {
 public:
  /** \brief Constructor */
  AsrlExperiment(const bool plotCsv, const bool plotOverlay, const double runSeconds,
                 const double checkResolution);

  /** \brief This problem does not know its optimum. */
  virtual bool knowsOptimum() const;

  /** \brief Throw as the optimum is unknown */
  virtual ompl::base::Cost getOptimum() const;

  /** \brief Set the target cost as the specified cost. */
  virtual void setTarget(double targetSpecifier);

  /** \brief Derived class specific information to include in the title line. */
  virtual std::string lineInfo() const;

  /** \brief Derived class specific information to include at the end. */
  virtual std::string paraInfo() const;

 protected:
  // Variables
  /** \brief The obstacle world */
  std::shared_ptr<CsvObstacle> asrl_;

  // Constant Parameters
  /** \brief The start position */
  double startPosX_;
  double startPosY_;
  /** \brief The goal position */
  double goalPosX_;
  double goalPosY_;
};

typedef std::shared_ptr<AsrlExperiment> AsrlExperimentPtr;

#endif  // EXPERIMENTS_ASRL_EXPERIMENT
