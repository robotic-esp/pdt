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

#include "esp_obstacles/hyperrectangle.h"

#include <functional>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/util/Exception.h>

Hyperrectangle::Hyperrectangle(ompl::base::SpaceInformation* si,
                                                 bool separateObstacles)
    : BaseObstacle(si), separateObstacles_(separateObstacles) {
  this->construct();
}

Hyperrectangle::Hyperrectangle(const ompl::base::SpaceInformationPtr& si,
                                                 bool separateObstacles)
    : BaseObstacle(si), maxWidth_(0.0), obsMeasure_(0.0), separateObstacles_(separateObstacles) {
  this->construct();
}
void Hyperrectangle::construct() {
  // Setup the NN structure
  nnObstacles_.reset(new ompl::NearestNeighborsGNAT<obstacle_corner_widths_t>());
  nnObstacles_->setDistanceFunction(std::bind(&Hyperrectangle::distanceFunction, this,
                                              std::placeholders::_1, std::placeholders::_2));

  // Allocate a sampler
  stateSampler_ = StateValidityChecker::si_->allocStateSampler();
}
Hyperrectangle::~Hyperrectangle() { this->clear(); }

void Hyperrectangle::clear() {
  // Free the memory:
  StateValidityChecker::si_->freeStates(statesToFree_);

  // Forget:
  statesToFree_.clear();

  // Clear the NN struct
  nnObstacles_->clear();

  // Reset things:
  maxWidth_ = 0.0;
  obsMeasure_ = 0.0;
}

unsigned int Hyperrectangle::size() const { return nnObstacles_->size(); }

bool Hyperrectangle::isValid(const ompl::base::State* state) const {
  // Variable
  // The return value
  bool validState;

  // Check if the state satisfies the bounds
  validState = StateValidityChecker::si_->satisfiesBounds(state);

  if (validState == true) {
    // If it does, get the obstacles from the nearest-neighbour struture that are within maxWidth of
    // the state Variables The nearby obstacles
    std::vector<obstacle_corner_widths_t> neighbours;

    // Get the obstacles that I could be in collision with. Pretend I myself am an obstacle
    // nnObstacles_->nearestR(std::make_pair(state,0.0), maxWidth_, neighbours);
    nnObstacles_->nearestR(
        std::make_pair(state, std::vector<double>(0.0)),
        std::sqrt(static_cast<double>(StateValidityChecker::si_->getStateDimension()) *
                  std::pow(maxWidth_, 2.0)),
        neighbours);

    // Check each obstacle
    for (unsigned int j = 0u; j < neighbours.size() && validState == true; ++j) {
      validState = this->verifyStateObstaclePair(state, neighbours.at(j));
    }
  }
  // No else, we're done

  return validState;
}

void Hyperrectangle::addObstacle(const obstacle_corner_widths_t& newObstacle) {
  //    //Print the obstacle out to the terminal:
  //    std::cout << "ll = [";
  //    for (unsigned int i = 0u; i < newObstacle.second.size(); ++i)
  //    {
  //        std::cout <<
  //        newObstacle.first->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]; if (i
  //        != (newObstacle.second.size() - 1u))
  //        {
  //            std::cout << ", ";
  //        }
  //    }
  //    std::cout << "], ur = [";
  //    for (unsigned int i = 0u; i < newObstacle.second.size(); ++i)
  //    {
  //        std::cout <<
  //        newObstacle.first->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] +
  //        newObstacle.second.at(i); if (i != (newObstacle.second.size() - 1u))
  //        {
  //            std::cout << ", ";
  //        }
  //    }
  //    std::cout << "]" << std::endl;

  // Add the obstacle
  nnObstacles_->add(newObstacle);

  // Update the volume
  obsMeasure_ = obsMeasure_ + this->rectangleVolume(newObstacle.second);

  // Update the largest obstacle size:
  for (unsigned int i = 0u; i < newObstacle.second.size(); ++i) {
    // Update the largest obstacle size:
    maxWidth_ = std::max(maxWidth_, newObstacle.second.at(i));
  }
}

void Hyperrectangle::addObstacles(
    const std::vector<obstacle_corner_widths_t>& newObstacles) {
  for (unsigned int i = 0u; i < newObstacles.size(); ++i) {
    this->addObstacle(newObstacles.at(i));
  }
}

std::vector<Hyperrectangle::obstacle_corner_widths_t>
Hyperrectangle::getObstacles() const {
  // Create a return value
  std::vector<obstacle_corner_widths_t> obsVector;

  // Get
  nnObstacles_->list(obsVector);

  // Return it to the user
  return obsVector;
}

void Hyperrectangle::randomize(double minObsSize, double maxObsSize, double obsRatio) {
  this->randomize(minObsSize, maxObsSize, obsRatio, std::vector<const ompl::base::State*>());
}

void Hyperrectangle::randomize(
    double minObsSize, double maxObsSize, double obsRatio,
    const std::vector<ompl::base::ScopedState<> >& existingStates) {
  // Make a vector
  std::vector<const ompl::base::State*> tVec(existingStates.size(), NULL);

  // Populate
  for (unsigned int i = 0u; i < existingStates.size(); ++i) {
    tVec.at(i) = existingStates.at(i).get();
  }

  // Call
  this->randomize(minObsSize, maxObsSize, obsRatio, tVec);
}

void Hyperrectangle::randomize(
    double minObsSize, double maxObsSize, double obsRatio,
    const std::vector<const ompl::base::State*>& existingStates) {
  while (obsMeasure_ / StateValidityChecker::si_->getSpaceMeasure() < obsRatio) {
    // Variables:
    // The newly created state
    ompl::base::State* obsState;
    // The random widths
    std::vector<double> obsWidths(StateValidityChecker::si_->getStateDimension(), 0.0);
    // Whether the obstacle is valid
    bool validObstacle;

    // Allocate a state
    obsState = StateValidityChecker::si_->allocState();

    // Randomize the state and widths
    stateSampler_->sampleUniform(obsState);
    for (unsigned int i = 0u; i < obsWidths.size(); ++i) {
      obsWidths.at(i) = rng_.uniformReal(minObsSize, maxObsSize);
    }

    // Verify that the obstacle does not collide with others:
    if (separateObstacles_ == true) {
      validObstacle = this->verifyObstacle(std::make_pair(obsState, obsWidths));
    } else {
      validObstacle = true;
    }

    // Iterate over the states in the vector and check that all of them remain valid with this new
    // obstacle:
    for (unsigned int i = 0u; i < existingStates.size() && validObstacle == true; ++i) {
      validObstacle =
          verifyStateObstaclePair(existingStates.at(i), std::make_pair(obsState, obsWidths));
    }

    if (validObstacle == true)
    // The newly created obstacle does not invalidated any of the existing states, keep:
    {
      // Add it to the list of states to free on destruction:
      statesToFree_.push_back(obsState);

      // Add it to the list of obstacles
      this->addObstacle(std::make_pair(obsState, obsWidths));
    } else
    // The newly created obstacle is in collision with one of the given states, free it
    {
      // Free the state:
      StateValidityChecker::si_->freeState(obsState);
    }
  }
}

std::string Hyperrectangle::mfile(const std::string& obsColour,
                                           const std::string& /*spaceColour*/) const {
  // Variables
  // The string stream:
  std::stringstream rval;
  // The vector of obstacles:
  std::vector<obstacle_corner_widths_t> obsVect;

  // get the obstacles as a vector
  nnObstacles_->list(obsVect);

  for (unsigned int i = 0u; i < obsVect.size(); ++i) {
    rval << "fill(["
         << obsVect.at(i).first->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]
         << ", "
         << obsVect.at(i).first->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] +
                obsVect.at(i).second.at(0u)
         << ", "
         << obsVect.at(i).first->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] +
                obsVect.at(i).second.at(0u)
         << ", "
         << obsVect.at(i).first->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]
         << "],";
    rval << " ["
         << obsVect.at(i).first->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]
         << ", "
         << obsVect.at(i).first->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]
         << ", "
         << obsVect.at(i).first->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] +
                obsVect.at(i).second.at(1u)
         << ", "
         << obsVect.at(i).first->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] +
                obsVect.at(i).second.at(1u)
         << "], " << obsColour << ", 'LineStyle', 'none');\n";
  }

  return rval.str();
}

bool Hyperrectangle::verifyStateObstaclePair(
    const ompl::base::State* state, const obstacle_corner_widths_t& obstacle) const {
  // Variables:
  // Whether the state is valid
  bool validState;

  // Assume the state is invalid, and continue until we find a dimension in which that's not true
  validState = false;
  for (unsigned int j = 0u; j < obstacle.second.size() && validState == false; ++j) {
    // Variable
    // The j-th component of the distance from the obstacle to the state
    double dj_toState_fromObs;

    // Calculate state - obstacle
    dj_toState_fromObs =
        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] -
        obstacle.first->as<ompl::base::RealVectorStateSpace::StateType>()->values[j];

    if (dj_toState_fromObs >= 0.0) {
      // If that distance is positive, than the state is to the right of the obstacle and we need to
      // check against the widths
      validState = dj_toState_fromObs > obstacle.second.at(j);
    } else {
      validState = true;
    }
    // No else, if the state is to the "left" of the "left" corner, it's not in collision
  }

  return validState;
}

bool Hyperrectangle::verifyObstacle(const obstacle_corner_widths_t& obstacle) const {
  throw ompl::Exception("This may not work properly. Try it in R2 and plot to be sure");
  // Variables:
  // Whether the obstacle is valid
  bool validObs;
  // The max width of the obstacle being checked
  double maxNewWidth;
  // The vector of near obstacles:
  std::vector<obstacle_corner_widths_t> neighbours;

  // Find the max width of the obs being checked
  maxNewWidth = 0.0;
  for (unsigned int i = 0u; i < obstacle.second.size(); ++i) {
    maxNewWidth = std::max(maxNewWidth, obstacle.second.at(i));
  }

  // Get the obstacles within maxWidth
  // nnObstacles_->nearestR(obstacle, std::max(maxWidth_, maxNewWidth), neighbours);
  nnObstacles_->nearestR(
      obstacle,
      std::sqrt(static_cast<double>(StateValidityChecker::si_->getStateDimension()) *
                std::pow(std::max(maxWidth_, maxNewWidth), 2.0)),
      neighbours);

  // Assume the obstacle is good to enter the loop
  validObs = true;
  for (unsigned int i = 0u; i < neighbours.size() && validObs == true; ++i) {
    // Check each component of the state space
    // Assume the obstacle is false, and continue until proven to be true
    validObs = false;
    for (unsigned int j = 0u; j < obstacle.second.size() && validObs == false; ++j) {
      // Variable
      // The j-th component of the distance from this corner to the i-th nearest neighbour corner
      double dj_toNear_fromNew;

      // Calculate neighbour - me
      dj_toNear_fromNew =
          neighbours.at(i).first->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] -
          obstacle.first->as<ompl::base::RealVectorStateSpace::StateType>()->values[j];

      if (dj_toNear_fromNew < 0) {
        // If that distance is negative, than the I am to the "right" of my neighbour and the check
        // is against its widths
        validObs = dj_toNear_fromNew > neighbours.at(i).second.at(j);
      } else if (dj_toNear_fromNew > 0) {
        // If that distance is positive, than the I am to the "left" of my neighbour and the check
        // is against my widths
        validObs = dj_toNear_fromNew > obstacle.second.at(j);
      } else {
        validObs = false;
      }
    }
  }

  // Return
  return validObs;
}

double Hyperrectangle::rectangleVolume(const std::vector<double>& widths) {
  // Variable
  // The measure of this rectangle
  double measure;

  // Calculate the lebesgue measure by compound multiplication
  measure = 1.0;
  for (unsigned int i = 0u; i < widths.size(); ++i) {
    measure = measure * widths.at(i);
  }
  return measure;
}

double Hyperrectangle::distanceFunction(const obstacle_corner_widths_t& a,
                                                 const obstacle_corner_widths_t& b) const {
  return StateValidityChecker::si_->distance(a.first, b.first);

  // I think this *should* be std::max, but that clearly misses obstacles for reasons I don't
  // understand, so I don't trust it and instead call nearest neighbours with the radius of the
  // bounding circle...
  /*
  //Variable
  //The distance
  double dist;

  //The distance is the largest component of the absolute dist between the states
  dist = 0.0;
  for (unsigned int i = 0u; i < a.second.size(); ++i)
  {
      dist = std::min(dist,
  std::abs(a.first->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] -
  b.first->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]) );
  }

  return dist;
  */
}
