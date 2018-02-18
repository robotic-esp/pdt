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

#ifndef RECORDING_TOOLS
#define RECORDING_TOOLS

//For std::ifstream and std::ofstream
#include <fstream>

//For vector and tuple
#include <vector>
#include <tuple>

//For ompl exceptions
#include "ompl/util/Exception.h"

//For some general time helpers
#include "tools/general_tools.h"

/** \brief A helper function to create directories using boost filesystem */
void createDirectories(std::string fileName);

//******* The different pieces of data to be recorded*******//
/** \brief A vector of time & cost */
class TimeCostHistory
{
    public:
        /** \brief Data type */
        typedef std::pair<asrl::time::duration, double> data_t;

        /** \brief Constructors */
        TimeCostHistory(double runTimeSeconds, unsigned int recordPeriodMicrosecond);
        TimeCostHistory(const asrl::time::duration& runTime, unsigned int recordPeriodMicrosecond);

        /** \brief Output the data with the appropriate label*/
        std::string output(const std::string& labelPrefix);

        /** \brief std::vector pass-throughs */
        bool empty() const
        {
            return data_.empty();
        }
        void push_back(const data_t& newData)
        {
            data_.push_back(newData);
        };
        data_t back()
        {
            return data_.back();
        };

    private:
        /** \brief Raw data */
        std::vector<data_t> data_;

        /** \brief Preallocated size */
        unsigned int allocSize_;

};

/** \brief A vector of time & iteration & cost */
class TimeIterationCostHistory
{
    public:
        /** \brief Data type */
        typedef std::tuple<asrl::time::duration, unsigned int, double> data_t;

        /** \brief Constructors */
        TimeIterationCostHistory(double runTimeSeconds, unsigned int recordPeriodMicrosecond);
        TimeIterationCostHistory(const asrl::time::duration& runTime, unsigned int recordPeriodMicrosecond);

        /** \brief Output the data with the appropriate label*/
        std::string output(const std::string& labelPrefix);

        /** \brief std::vector pass-throughs */
        bool empty() const
        {
            return data_.empty();
        }
        void push_back(const data_t& newData)
        {
            data_.push_back(newData);
        };
        data_t back()
        {
            return data_.back();
        };

    private:
        /** \brief Raw data */
        std::vector<data_t> data_;

        /** \brief Preallocated size */
        unsigned int allocSize_;
};

/** \brief A vector of iteration & cost */
class IterationCostHistory
{
    public:
        /** \brief Data type */
        typedef std::pair<unsigned int, double> data_t;

        /** \brief Constructors */
        IterationCostHistory(unsigned int numIterations);
        IterationCostHistory(double runTimeSeconds, unsigned int recordPeriodMicrosecond);
        IterationCostHistory(const asrl::time::duration& runTime, unsigned int recordPeriodMicrosecond);

        /** \brief Output the data with the appropriate label*/
        std::string output(const std::string& labelPrefix);

        /** \brief std::vector pass-throughs */
        bool empty() const
        {
            return data_.empty();
        }
        bool empty()
        {
            return data_.empty();
        }
        void push_back(const data_t& newData)
        {
            data_.push_back(newData);
        };
        data_t back()
        {
            return data_.back();
        };

    private:
        /** \brief Raw data */
        std::vector<data_t> data_;

        /** \brief Preallocated size */
        unsigned int allocSize_;
};

/** \brief A vector of "target" & time */
class TargetTimeResults
{
    public:
        /** \brief Data type */
        typedef std::pair<double, asrl::time::duration> data_t;

        /** \brief Constructors */
        TargetTimeResults(unsigned int numTargets);

        /** \brief Output the data with the appropriate label*/
        std::string output(const std::string& labelPrefix);

        /** \brief std::vector pass-throughs */
        bool empty() const
        {
            return data_.empty();
        }
        void push_back(const data_t& newData)
        {
            data_.push_back(newData);
        };
        data_t back()
        {
            return data_.back();
        };

    private:
        /** \brief Raw data */
        std::vector<data_t> data_;

        /** \brief Preallocated size */
        unsigned int allocSize_;
};


//******* The file that writes the data to disk*******//
/** \brief A class to write results to disk. */
template <class DATA_T>
class ResultsFile
{
    public:
        ResultsFile(const std::string& fullFileName)
            : filename_(fullFileName)
        {
            createDirectories(filename_);
        };

        void addResult(const std::string& plannerName, DATA_T& data)
        {
            //Variable:
            //The output file stream
            std::ofstream mfile;

            //Open the file:
            mfile.open(filename_.c_str(), std::ofstream::out | std::ofstream::app);

            //Check on the failbit:
            if (mfile.fail() == true)
            {
                throw ompl::Exception("Could not open file.");
            }

            //Write the data
            mfile << data.output(plannerName);

            //Flush the file:
            mfile.flush();

            //Close the file:
            mfile.close();
        };

    private:
        std::string filename_;
};

#endif //RECORDING_TOOLS
