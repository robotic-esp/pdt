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

// For std::cout
#include <iostream>
// For std::ifstream and std::ofstream
#include <fstream>
// For std::setprecision
#include <iomanip>
// For std::stringstream
#include <sstream>
// For std::shared_ptr, etc.
#include <memory>
// For std::bind
#include <functional>
// For boost program options
#include <boost/program_options.hpp>

// OMPL:
// For OMPL_INFORM et al.
#include "ompl/util/Console.h"
// For exceptions
#include "ompl/util/Exception.h"
//"random" numbers
#include "ompl/util/RandomNumbers.h"

// NN structs:
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h"

// The general helper functions
#include "tools/general_tools.h"

const unsigned int N = 2u;

class TestElement {
 public:
  TestElement(unsigned int id) : id_(id), data_(std::vector<double>(N, 0.0)), locked_(false){};

  ~TestElement(){
      //        std::cout << " -> " << id_ << std::endl;
  };

  void lock() { locked_ = true; };

  void unlock() { locked_ = false; };

  unsigned int id_;
  std::vector<double> data_;
  bool locked_;
};

typedef std::shared_ptr<TestElement> test_element_ptr_t;

// L2 norm
double distanceFunction(const test_element_ptr_t& a, const test_element_ptr_t& b) {
  if (a->locked_ == true) {
    std::cout << std::endl
              << std::endl
              << "In call to distanceFunction(id# " << a->id_ << ", id# " << b->id_ << "), id# "
              << a->id_ << " is already locked." << std::endl;
    throw ompl::Exception("distanceFunction accessed a locked element.");
  }

  if (b->locked_ == true) {
    std::cout << std::endl
              << std::endl
              << "In call to distanceFunction(id# " << a->id_ << ", id# " << b->id_ << "), id# "
              << b->id_ << " is already locked." << std::endl;
    throw ompl::Exception("distanceFunction accessed a locked element.");
  }

  double dist = 0.0;
  for (unsigned int i = 0u; i < a->data_.size(); ++i) {
    dist = dist + std::pow(a->data_.at(i) - b->data_.at(i), 2.0);
  }

  return std::sqrt(dist);
}

// Convenience data
class NnTestingData {
 public:
  typedef std::shared_ptr<ompl::NearestNeighbors<test_element_ptr_t> > nn_ptr_t;

  NnTestingData(std::string name, nn_ptr_t ptr)
      : name_(name), insertTime_(0), removeTime_(0), nn_(ptr) {
    // Set the distance function
    nn_->setDistanceFunction(
        std::bind(&distanceFunction, std::placeholders::_1, std::placeholders::_2));
  };

  NnTestingData(){};

  std::string name_;
  asrl::time::duration insertTime_;
  asrl::time::duration removeTime_;
  nn_ptr_t nn_;
};

bool argParse(int argc, char** argv, std::uint_fast32_t* seedPtr) {
  // Declare the supported options.
  boost::program_options::options_description desc("Allowed options");
  desc.add_options()("help,h", "produce help message")(
      "seed,s", boost::program_options::value<std::uint_fast32_t>()->default_value(11u),
      "The seed for the number sequence.");
  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return false;
  }

  if (vm.count("seed")) {
    *seedPtr = vm["seed"].as<std::uint_fast32_t>();
  } else {
    std::cout << "Seed not specified." << std::endl << std::endl << desc << std::endl;
  }

  return true;
}

int main(int argc, char** argv) {
  typedef std::shared_ptr<TestElement> test_element_ptr_t;
  // Configuration
  // The seed
  std::uint_fast32_t masterSeed;

  // Get the command line arguments
  if (argParse(argc, argv, &masterSeed) == false) {
    return 1;
  }

  // Set the seed:
  ompl::RNG::setSeed(masterSeed);

  // Variables
  // The number sequence
  ompl::RNG rng;
  // The id counter
  unsigned int id = 0u;
  // The number of elements to insert each pass:
  unsigned int numToCreate;
  // The number of elements to remove each pass:
  unsigned int numToRemove;
  // The number of passes to perform
  unsigned int numPasses;
  // A vector of NN structs to test, as a helper testing data class:
  std::vector<NnTestingData> nnToTest;
  nnToTest.push_back(
      NnTestingData("GNAT", std::make_shared<ompl::NearestNeighborsGNAT<test_element_ptr_t> >()));
  nnToTest.push_back(
      NnTestingData("GNAT(4, 2, 6, 50, 50, false)",
                    std::make_shared<ompl::NearestNeighborsGNAT<test_element_ptr_t> >(
                        4, 2, 6, 50, 50, false)));  // Old GNAT settings.
  //    nnToTest.push_back( NnTestingData("GNAT(3, 2, 3, 4, 50, false)",
  //    std::make_shared<ompl::NearestNeighborsGNAT<test_element_ptr_t> >(3, 2, 3, 4, 50, false)) );
  //    //Old GNATNoThreadSafety settings. nnToTest.push_back( NnTestingData("GNATNoThreadSafety",
  //    std::make_shared<ompl::NearestNeighborsGNATNoThreadSafety<test_element_ptr_t> >()) );

  // Pick the number of elements to insert/remove per pass, and the number of passes
  numToCreate = rng.uniformInt(100u, 1000u);
  numToRemove = rng.uniformInt(10u, 0.75 * numToCreate);
  numPasses = rng.uniformInt(30u, 100u);

  std::cout << "Seed: " << masterSeed << " -> Performing " << numPasses << " passes, adding "
            << numToCreate << " and then removing " << numToRemove << " elements in each pass."
            << std::endl;

  // Perform numPasses
  for (unsigned int i = 0u; i < numPasses; ++i) {
    // Variables
    // The elements created in this pass
    std::vector<test_element_ptr_t> passElements;

    // Insert into each struct
    std::cout << i << ": Adding " << numToCreate << " elements." << std::flush;
    for (unsigned int j = 0u; j < nnToTest.size(); ++j) {
      // The timing at the start of this pass:
      asrl::time::duration oldTime(nnToTest.at(j).insertTime_);

      // Insert numToCreate elements:
      for (unsigned int k = 0u; k < numToCreate; ++k) {
        // we need to create an element if this is the first NN struct tested?
        if (j == 0u) {
          // Allocate the element
          passElements.push_back(std::make_shared<TestElement>(++id));

          // Put data in the element
          rng.uniformNormalVector(passElements.back()->data_);
        }
        // No else, allocated earlier

        // Store the time:
        asrl::time::point startTime = asrl::time::now();

        // Add
        nnToTest.at(j).nn_->add(passElements.at(k));

        // Store the change in time:
        nnToTest.at(j).insertTime_ = nnToTest.at(j).insertTime_ + (asrl::time::now() - startTime);
      }

      // Results
      std::cout << " " << nnToTest.at(j).name_ << ": " << nnToTest.at(j).insertTime_ - oldTime
                << std::flush;
    }

    // Remove from each struct:
    std::cout << "    Removing " << numToRemove << " elements." << std::flush;
    for (unsigned int j = 0u; j < nnToTest.size(); ++j) {
      // The timing at the start of this pass:
      asrl::time::duration oldTime(nnToTest.at(j).removeTime_);

      // Unlock all the elements for this NN struct
      for (unsigned int k = 0u; k < passElements.size(); ++k) {
        passElements.at(k)->unlock();
      }

      // Remove numToRemove elements:
      for (unsigned int k = 0u; k < numToRemove; ++k) {
        // Get the element that is "closest"
        std::vector<test_element_ptr_t> nbh;
        nnToTest.at(j).nn_->nearestK(passElements.at(k), 1, nbh);

        // Test if it's the same:
        if (passElements.at(k)->id_ != nbh.front()->id_) {
          std::cout << std::endl << std::endl;
          std::cout << "Searching with element " << passElements.at(k)->id_ << " [" << std::flush;
          for (unsigned int q = 0u; q < N; ++q) {
            std::cout << " " << passElements.at(k)->data_.at(q) << std::flush;
          }
          std::cout << " ] finds " << nbh.front()->id_ << " as the nearest element [" << std::flush;
          for (unsigned int q = 0u; q < N; ++q) {
            std::cout << " " << nbh.front()->data_.at(q) << std::flush;
          }
          std::cout << " ]" << std::endl;
          std::cout << "Expect the wrong element to be removed and problems to follow."
                    << std::endl;
        }

        // Store the time
        asrl::time::point startTime = asrl::time::now();

        // Remove
        nnToTest.at(j).nn_->remove(passElements.at(k));

        // Store the change in time:
        nnToTest.at(j).removeTime_ = nnToTest.at(j).removeTime_ + (asrl::time::now() - startTime);

        // Mark the element as deleted by locking it for the remainder of this struct's run:
        passElements.at(k)->lock();
      }

      // Results
      std::cout << " " << nnToTest.at(j).name_ << ": " << nnToTest.at(j).removeTime_ - oldTime
                << std::flush;
    }
    std::cout << std::endl;
  }

  std::cout << std::endl;
  std::cout << "Total time. Insert:" << std::flush;
  for (unsigned int i = 0u; i < nnToTest.size(); ++i) {
    std::cout << " " << nnToTest.at(i).name_ << ": " << nnToTest.at(i).insertTime_ << std::flush;
  }
  std::cout << "    Remove:" << std::flush;
  for (unsigned int i = 0u; i < nnToTest.size(); ++i) {
    std::cout << " " << nnToTest.at(i).name_ << ": " << nnToTest.at(i).removeTime_ << std::flush;
  }
  std::cout << std::endl;

  return 0;
}
