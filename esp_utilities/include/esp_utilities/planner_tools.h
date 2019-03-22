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

// Authors: Jonathan Gammell

#pragma once

#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>

#define BITSTAR_REGRESSION 1

#ifdef BITSTAR_REGRESSION
#include <ompl/geometric/planners/bitstar_regression/BITstarRegression.h>
#endif  // BITSTAR_REGRESSION

// Compile hooks for a planner called BITstarRegression:
//#define BITSTAR_REGRESSION

/*
How to make a copy of BIT* for regression testing:
cd ompl/geometric/planners/
cp -r bitstar bitstar_regression
cd bitstar_regression
mv BITstar.h BITstarRegression.h
mv src/BITstar.cpp src/BITstarRegression.cpp
sed -i.bak -e 's|_BITSTAR|_BITSTARREGRESSION|g' -e 's|/bitstar|/bitstar_regression|g' -e
's|BITstar|BITstarRegression|g' BITstarRegression.h sed -i.bak -e 's|_BITSTAR|_BITSTARREGRESSION|g'
-e 's|/bitstar|/bitstar_regression|g' -e 's|BITstar|BITstarRegression|g' src/BITstarRegression.cpp
cd datastructures
sed -i.bak -e 's|_BITSTAR|_BITSTARREGRESSION|g' -e 's|/bitstar|/bitstar_regression|g' -e
's|BITstar|BITstarRegression|g' *.h cd src sed -i.bak -e 's|_BITSTAR|_BITSTARREGRESSION|g' -e
's|/bitstar|/bitstar_regression|g' -e 's|BITstar|BITstarRegression|g' *.cpp
*/

namespace esp {

namespace ompltools {

enum PlannerType {
  PLANNER_NOPLANNER,
  PLANNER_RRT,
  PLANNER_RRTCONNECT,
  PLANNER_RRTSTAR,
  PLANNER_RRTSTAR_INFORMED,
  PLANNER_RRTSHARP,
  PLANNER_RRTSHARP_INFORMED,
  PLANNER_LBTRRT,
  PLANNER_SORRTSTAR,
  PLANNER_RRTSTAR_PRUNE,
  PLANNER_RRTSTAR_NEW_REJECT,
  PLANNER_RRTSTAR_SAMPLE_REJECT,
  PLANNER_RRTSTAR_TRIO,
  PLANNER_FMTSTAR,
  PLANNER_BITSTAR,
  PLANNER_SBITSTAR,
  PLANNER_RRTSTAR_SEED,
  PLANNER_BITSTAR_SEED,
#ifdef BITSTAR_REGRESSION
  PLANNER_BITSTAR_REGRESSION
#endif
};

/** \brief Allocation function for RRT */
std::shared_ptr<ompl::geometric::RRT> allocateRrt(const ompl::base::SpaceInformationPtr &si,
                                                  const double steerEta, const double goalBias);

/** \brief Allocation function for RRTConnect */
std::shared_ptr<ompl::geometric::RRTConnect> allocateRrtConnect(
    const ompl::base::SpaceInformationPtr &si, const double steerEta);

/** \brief Allocation function for RRT* */
std::shared_ptr<ompl::geometric::RRTstar> allocateRrtStar(const ompl::base::SpaceInformationPtr &si,
                                                          const double steerEta,
                                                          const double goalBias,
                                                          const bool kNearest,
                                                          const double rewireScale);

/** \brief Allocation function for RRT# */
std::shared_ptr<ompl::geometric::RRTsharp> allocateRrtSharp(
    const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias,
    const bool kNearest, const double rewireScale, const bool reject, const bool informed,
    const unsigned int variant);

/** \brief Allocation function for LBTRRT */
std::shared_ptr<ompl::geometric::LBTRRT> allocateLbtRrt(const ompl::base::SpaceInformationPtr &si,
                                                        const double steerEta,
                                                        const double goalBias,
                                                        const double epsilon);

/** \brief Allocation function for Informed RRT* */
std::shared_ptr<ompl::geometric::InformedRRTstar> allocateInformedRrtStar(
    const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias,
    const bool kNearest, const double rewireScale, const double pruneFraction);

/** \brief Allocation function for SORRT* */
std::shared_ptr<ompl::geometric::SORRTstar> allocateSorrtStar(
    const ompl::base::SpaceInformationPtr &si, const double steerEta, const double goalBias,
    const bool kNearest, const double rewireScale, const double pruneFraction,
    const unsigned int numSamples);

/** \brief Allocation function for FMT* */
std::shared_ptr<ompl::geometric::FMT> allocateFmtStar(const ompl::base::SpaceInformationPtr &si,
                                                      const bool kNearest, const double rewireScale,
                                                      const unsigned int numSamples,
                                                      const bool cacheCC, const bool useHeuristics);

/** \brief Allocation function for BIT* */
std::shared_ptr<ompl::geometric::BITstar> allocateBitStar(
    const ompl::base::SpaceInformationPtr &si, const bool kNearest, const double rewireScale,
    const unsigned int numSamples, const bool enablePruning, const double pruneFraction,
    const bool jit, const bool refreshBatches, const double initialInflationFactor,
    const double inflationFactorParameter, const double truncationFactorParameter);

#ifdef BITSTAR_REGRESSION
/** \brief Allocation function for a regression testing BIT* */
std::shared_ptr<ompl::geometric::BITstarRegression> allocateBitStarRegression(
    const ompl::base::SpaceInformationPtr &si, const bool kNearest, const double rewireScale,
    const unsigned int numSamples, const double pruneFraction, const bool strictQueue,
    const bool delayRewire, const bool jit, const bool refreshBatches);
#endif  // BITSTAR_REGRESSION

/** \brief Return true if the planner is any of the RRT types */
bool isRrt(PlannerType plnrType);

/** \brief Return true if the planner is any of the RRTstar types */
bool isRrtStar(PlannerType plnrType);

/** \brief Return true if the planner is any of the RRTsharp types */
bool isRrtSharp(PlannerType plnrType);

/** \brief Return true if the planner is any of the LBTRRT types */
bool isLbtRrt(PlannerType plnrType);

/** \brief Return true if the planner is any of the BITstar types */
bool isBitStar(PlannerType plnrType);

/** \brief Convert a planner enum into a planner name */
std::string plannerName(PlannerType plnrType);

}  // namespace ompltools

}  // namespace esp
