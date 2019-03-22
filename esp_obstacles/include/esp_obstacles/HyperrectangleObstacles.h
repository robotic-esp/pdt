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

#ifndef OBSTACLES_HYPERRECTANGLE_OBSTACLES
#define OBSTACLES_HYPERRECTANGLE_OBSTACLES

#include <memory>
#include <vector>

#include "obstacles/BaseObstacle.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/StateValidityChecker.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/util/RandomNumbers.h"

/** \brief A world consisting of random hyperrectangular obstacles.*/
class HyperrectangleObstacles : public BaseObstacle {
 public:
  /** \brief a pair of lower-left corner and widths for a hyperrectangle obstacle*/
  typedef std::pair<const ompl::base::State*, std::vector<double> > obstacle_corner_widths_t;

  /** \brief Constructor. */
  HyperrectangleObstacles(ompl::base::SpaceInformation* si, bool separateObstacles);
  /** \brief Constructor. */
  HyperrectangleObstacles(const ompl::base::SpaceInformationPtr& si, bool separateObstacles);
  /** \brief Destructor */
  ~HyperrectangleObstacles();

  /** \brief Clear the obstacle space */
  virtual void clear();

  /** \brief The number of obstacles in the random world */
  unsigned int size() const;

  /** \brief Check for state validity */
  virtual bool isValid(const ompl::base::State* state) const;

  /** \brief Get the vector of obstacles in the obstacle space */
  std::vector<obstacle_corner_widths_t> getObstacles() const;

  /** \brief Add an obstacle to the obstacle space */
  void addObstacle(const obstacle_corner_widths_t& newObstacle);

  /** \brief Add multiple obstacles to the obstacle space */
  void addObstacles(const std::vector<obstacle_corner_widths_t>& newObstacles);

  /** \brief Create random obstacles such that the ratio of the space bounds covered by obstacles is
   * no more than the given parameter */
  void randomize(double minObsSize, double maxObsSize, double obsRatio);

  /** \brief Create random obstacles such that the ratio of the space bounds covered by obstacles is
   * no more than the given parameter and that the states given are not inside*/
  void randomize(double minObsSize, double maxObsSize, double obsRatio,
                 const std::vector<ompl::base::ScopedState<> >& existingStates);

  /** \brief Create random obstacles such that the ratio of the space bounds covered by obstacles is
   * no more than the given parameter and that the states given are not inside*/
  void randomize(double minObsSize, double maxObsSize, double obsRatio,
                 const std::vector<const ompl::base::State*>& existingStates);

  /** \brief The volume an n-dimensional sphere */
  static double rectangleVolume(const std::vector<double>& widths);

  /** \brief The obstacle map as a series of Matlab plot functions. Discard the space-colour
   * argument. */
  virtual std::string mfile(const std::string& obsColour, const std::string& /*spaceColour*/) const;

 protected:
 private:
  /** \brief The individual obstacles in a nearest neighbours structure */
  std::shared_ptr<ompl::NearestNeighbors<obstacle_corner_widths_t> > nnObstacles_ { };
  /** \brief A vector of states that I allocated (and therefore must free) */
  std::vector<ompl::base::State*> statesToFree_ { };
  /** \brief The largest obstacle, used to find a small set of obstacles to check */
  double maxWidth_ { 0.0 };
  /** \brief The measure of obstacles. This is really an "upper bound", as we don't check for
   * overlapping obstacle regions when calculating. */
  double obsMeasure_ { 0.0 };
  /** \brief Whether to disallow obstacle-obstacle collision */
  bool separateObstacles_ { false };
  /** \brief The state sampler */
  ompl::base::StateSamplerPtr stateSampler_ { };
  /** \brief The random number generator */
  ompl::RNG rng_ { };

  /** \brief A common construtor */
  void construct();

  /** \brief Check whether a state is valid w.r.t to a specific obstacle. I.e., isValid is
   * calculated by and'ing the following function over all obstacles:*/
  bool verifyStateObstaclePair(const ompl::base::State* state,
                               const obstacle_corner_widths_t& obstacle) const;

  /** \brief Check whether two obstacles are in collision */
  bool verifyObstacle(const obstacle_corner_widths_t& obstacle) const;

  /** \brief The distance function for finding the nearest neighbour obstacles */
  double distanceFunction(const obstacle_corner_widths_t& a,
                          const obstacle_corner_widths_t& b) const;
};

#endif  // OBSTACLES_HYPERRECTANGLE_OBSTACLES
