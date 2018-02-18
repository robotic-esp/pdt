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

//For std::cout
#include <iostream>
//For std::ifstream and std::ofstream
#include <fstream>
//For std::setprecision
#include <iomanip>
//For std::stringstream
#include <sstream>
//For std::shared_ptr, etc.
#include <memory>
//For std::bind
#include <functional>
//For boost program options
#include <boost/program_options.hpp>

//OMPL:
//For OMPL_INFORM et al.
#include "ompl/util/Console.h"
//For exceptions
#include "ompl/util/Exception.h"
//"random" numbers
#include "ompl/util/RandomNumbers.h"

//Binary heap:
#include "ompl/datastructures/BinaryHeap.h"

//The general helper functions
#include "tools/general_tools.h"

const unsigned int N = 2u;

bool argParse(int argc, char** argv, std::uint_fast32_t* seedPtr)
{
    // Declare the supported options.
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("seed,s", boost::program_options::value<std::uint_fast32_t>()->default_value(11u), "The seed for the number sequence.");
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return false;
    }

    if (vm.count("seed"))
    {
        *seedPtr = vm["seed"].as<std::uint_fast32_t>();
    }
    else
    {
        std::cout << "Seed not specified." << std::endl << std::endl << desc << std::endl;
    }

    return true;
}

typedef double value_t;
typedef std::string element_t;
//typedef std::shared_ptr<element_t> element_ptr_t;
typedef std::pair<value_t, element_t> heap_element_t;

typedef std::function<bool(const heap_element_t &, const heap_element_t &)> comparison_fnc_t;
typedef ompl::BinaryHeap<heap_element_t, comparison_fnc_t> bin_heap_t;


bool ElementComparison (const heap_element_t& a, const heap_element_t& b)
{
    return (a.first < b.first);
};

std::ostream &operator<<(std::ostream &stream, heap_element_t a)
{
    stream << "(" << a.first << ", " << a.second << ")";
    return stream;
}

int main(int argc, char **argv)
{
    //Configuration
    //The seed
    std::uint_fast32_t masterSeed;

    //Get the command line arguments
    if (argParse(argc, argv, &masterSeed) == false)
    {
        return 1;
    }

    // Set the seed:
    ompl::RNG::setSeed(masterSeed);

    // Create the heap
    std::shared_ptr<bin_heap_t> binHeap;
    binHeap = std::make_shared<bin_heap_t>(std::bind(&ElementComparison, std::placeholders::_1, std::placeholders::_2));
//    ompl::BinaryHeap<heap_element_t, ComparisonObject> binHeap;
    bin_heap_t::Element* updateElem;

    std::cout << "Made an" << (binHeap->empty() ? " empty" : "") << " binary heap." << std::endl;

    std::cout << "Inserting first elements" << std::endl;
    binHeap->insert(std::make_pair(3.0, "1st element"));
    binHeap->insert(std::make_pair(1.0, "2nd element"));
    updateElem = binHeap->insert(std::make_pair(5.0, "3rd element"));

    std::cout << "Top:" << binHeap->top()->data << std::endl;

    std::cout << "Inserting more elements" << std::endl;
    binHeap->insert(std::make_pair(0.0, "4th element"));
    binHeap->insert(std::make_pair(5.0, "5th element"));

    std::cout << "Top:" << binHeap->top()->data << std::endl;

    std::cout << "Updating an element" << std::endl;
    updateElem->data.first = -1.0;
    binHeap->update(updateElem);

    std::cout << "Top:" << binHeap->top()->data << std::endl;
}
