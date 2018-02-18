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

#ifndef EXPERIMENTS_OBSTACLE_FREE_EXPERIMENT
#define EXPERIMENTS_OBSTACLE_FREE_EXPERIMENT

#include "experiments/BaseExperiment.h"

#include "ompl/base/ScopedState.h"
#include "ompl/base/Goal.h"


/** \brief An obstacle-free multigoal/multistart experiment */
class ObstacleFreeExperiment : public BaseExperiment
{
public:
    /** \brief Constructor */
    ObstacleFreeExperiment(const unsigned int dim, const unsigned int maxNumStarts, const unsigned int maxNumGoals, const double runSeconds);

    /** \brief This problem knows the optimum */
    virtual bool knowsOptimum() const;

    /** \brief Returns the optimum cost, the minimum distance from start to goal. */
    virtual ompl::base::Cost getOptimum() const;

    /** \brief Set the target cost as the specified multiplier of the optimum. */
    virtual void setTarget(double targetSpecifier);

    /** \brief Derived class specific information to include in the title line. */
    virtual std::string lineInfo() const;

    /** \brief Derived class specific information to include at the end. */
    virtual std::string paraInfo() const;

protected:
    // Constant Parameters
    /** \brief The definition of the "centre" and "outside" positions */
    double centrePos_;
    double outsidePos_;
};

typedef std::shared_ptr<ObstacleFreeExperiment> ObstacleFreeExperimentPtr;

#endif //EXPERIMENTS_OBSTACLE_FREE_EXPERIMENT
