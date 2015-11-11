#include "recording_tools.h"

//For std::setfill and std::setw and std::setprecision
#include <iomanip>
//For boost::filesystem
#include <boost/filesystem.hpp>


void createDirectories(std::string fileName)
{
  ///Variables
  //The boost::path representation of the string
  boost::filesystem::path fullPath;

  //Create a boost::path from the provided string
  fullPath = fileName.c_str();

  //Decompose the path into the parent directories and check if they exist
  if (fullPath.parent_path().empty() == false)
  {
    if (boost::filesystem::exists(fullPath.parent_path()) == false)
    {
      //If they don't exist, make them
      boost::filesystem::create_directories(fullPath.parent_path());

    //    std::cout << "Created: " << boost::filesystem::absolute(fullPath.parent_path()) << "\n";
    }
  }
  //Else, do nothing
}





//******* The different pieces of data to be recorded*******//
TimeCostHistory::TimeCostHistory(double runTimeSeconds, double recordPeriodMillisecond)
{
    data_.reserve(1.1*runTimeSeconds/(recordPeriodMillisecond/1000.0));
}
TimeCostHistory::TimeCostHistory(const ompl::time::duration& runTime, double recordPeriodMillisecond)
{
    data_.reserve(1.1*ompl::time::seconds(runTime)/(recordPeriodMillisecond/1000.0));
}
std::string TimeCostHistory::output(const std::string& labelPrefix)
{
    //Variable
    //The return value
    std::stringstream rval;

    //Write the time first:
    rval << labelPrefix << ", ";
    for (unsigned int i = 0u; i < data_.size(); ++i)
    {
        rval << std::setprecision(21) << data_.at(i).first.total_nanoseconds()/1e9;
        if (i != data_.size() - 1u)
        {
            rval << ", ";
        }
    }
    rval << std::endl;

    //then costs:
    rval << labelPrefix << ", ";
    for (unsigned int i = 0u; i < data_.size(); ++i)
    {
        rval << std::setprecision(21) << data_.at(i).second;
        if (i != data_.size() - 1u)
        {
            rval << ", ";
        }
    }
    rval << std::endl;

    //Return
    return rval.str();
}

TimeIterationCostHistory::TimeIterationCostHistory(double runTimeSeconds, double recordPeriodMillisecond)
{
    data_.reserve(1.1*runTimeSeconds/(recordPeriodMillisecond/1000.0));
}
TimeIterationCostHistory::TimeIterationCostHistory(const ompl::time::duration& runTime, double recordPeriodMillisecond)
{
    data_.reserve(1.1*ompl::time::seconds(runTime)/(recordPeriodMillisecond/1000.0));
}
std::string TimeIterationCostHistory::output(const std::string& labelPrefix)
{
    //Variable
    //The return value
    std::stringstream rval;

    //Write the time first:
    rval << labelPrefix << ", ";
    for (unsigned int i = 0u; i < data_.size(); ++i)
    {
        rval << std::setprecision(21) << data_.at(i).get<0u>().total_nanoseconds()/1e9;
        if (i != data_.size() - 1u)
        {
            rval << ", ";
        }
    }
    rval << std::endl;

    //then the iterations:
    rval << labelPrefix << ", ";
    for (unsigned int i = 0u; i < data_.size(); ++i)
    {
        rval << std::setprecision(21) << data_.at(i).get<1u>();
        if (i != data_.size() - 1u)
        {
            rval << ", ";
        }
    }
    rval << std::endl;

    //then the costs:
    rval << labelPrefix << ", ";
    for (unsigned int i = 0u; i < data_.size(); ++i)
    {
        rval << std::setprecision(21) << data_.at(i).get<2u>();
        if (i != data_.size() - 1u)
        {
            rval << ", ";
        }
    }
    rval << std::endl;

    //Return
    return rval.str();
}

IterationCostHistory::IterationCostHistory(unsigned int numIterations)
{
    data_.reserve(numIterations+5u);
}
IterationCostHistory::IterationCostHistory(double runTimeSeconds, double recordPeriodMillisecond)
{
    data_.reserve(1.1*runTimeSeconds/(recordPeriodMillisecond/1000.0));
}
IterationCostHistory::IterationCostHistory(const ompl::time::duration& runTime, double recordPeriodMillisecond)
{
    data_.reserve(1.1*ompl::time::seconds(runTime)/(recordPeriodMillisecond/1000.0));
}
std::string IterationCostHistory::output(const std::string& labelPrefix)
{
    //Variable
    //The return value
    std::stringstream rval;

    //Write the iteration first:
    rval << labelPrefix << ", ";
    for (unsigned int i = 0u; i < data_.size(); ++i)
    {
        rval << data_.at(i).first;
        if (i != data_.size() - 1u)
        {
            rval << ", ";
        }
    }
    rval << std::endl;

    //then the cost
    rval << labelPrefix << ", ";
    for (unsigned int i = 0u; i < data_.size(); ++i)
    {
        rval << std::setprecision(21) << data_.at(i).second;
        if (i != data_.size() - 1u)
        {
            rval << ", ";
        }
    }
    rval << std::endl;

    //Return
    return rval.str();
}

TargetTimeResults::TargetTimeResults(unsigned int numTargets)
{
    data_.reserve(numTargets+2u);
}
std::string TargetTimeResults::output(const std::string& labelPrefix)
{
    //Variable
    //The return value
    std::stringstream rval;

    //Write the targets first:
    rval << labelPrefix << ", ";
    for (unsigned int i = 0u; i < data_.size(); ++i)
    {
        rval << std::setprecision(21) << data_.at(i).first;
        if (i != data_.size() - 1u)
        {
            rval << ", ";
        }
    }
    rval << std::endl;

    //the time
    rval << labelPrefix << ", ";
    for (unsigned int i = 0u; i < data_.size(); ++i)
    {
        rval << std::setprecision(21) << data_.at(i).second.total_nanoseconds()/1e9;
        if (i != data_.size() - 1u)
        {
            rval << ", ";
        }
    }
    rval << std::endl;

    //Return
    return rval.str();
}
