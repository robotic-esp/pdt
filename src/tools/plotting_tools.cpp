//Me
#include "tools/plotting_tools.h"

//For std::ifstream and std::ofstream
#include <fstream>
//For std::setfill and std::setw and std::setprecision
#include <iomanip>
//For boost::filesystem
#include <boost/filesystem.hpp>
//boost::tuple (pre-C++11 std::tuple)
#include <boost/tuple/tuple.hpp>

//R^n
#include <ompl/base/spaces/RealVectorStateSpace.h>
//For geometric::path
#include <ompl/geometric/PathGeometric.h>
//For ompl::base::GoalStates:
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/goals/GoalStates.h"
#include "ompl/base/OptimizationObjective.h"
#include <ompl/geometric/planners/bitstar/BITstar.h>
//#include <ompl/geometric/planners/bitstar/HybridBITstar.h>
//#include <ompl/geometric/planners/bitstar/BITstarOld.h>
//#include <ompl/geometric/planners/bitstar/ICRA16.h>
#include <ompl/util/Exception.h>

std::string plotVertex(BaseExperimentPtr experiment, const ompl::base::State* vertex, std::string vertexColour, std::string vertexSize)
{
    //A scoped state
    ompl::base::ScopedState<> scopedVertex(experiment->getSpaceInformation()->getStateSpace(), vertex);

    std::stringstream rval;
    rval << "plot(" <<  scopedVertex[0] << ", " << scopedVertex[1] << ", '.', 'Color', " << vertexColour << ", 'MarkerSize', " << vertexSize << ");" << std::endl;
    return rval.str();
}

std::string plotEdge(BaseExperimentPtr experiment, const ompl::base::State* parent, const ompl::base::State* child, std::string edgeColour, std::string lineStyle, std::string edgeWeight)
{
    //Scoped states
    ompl::base::ScopedState<> scopedParent(experiment->getSpaceInformation()->getStateSpace(), parent);
    ompl::base::ScopedState<> scopedChild(experiment->getSpaceInformation()->getStateSpace(), child);

    std::stringstream rval;
    rval << "plot([" << scopedParent[0] << ", " << scopedChild[0] << "], [" << scopedParent[1] << ", " << scopedChild[1] << "], '-', 'Color', " << edgeColour << ", 'LineStyle', " << lineStyle << ", 'LineWidth', " << edgeWeight << ");" << std::endl;
    return rval.str();
}

std::string matlabExtraHeader(std::string plannerName, bool informedWorldEllipse, bool bitStarQueueEllipse, bool bitStarNextEdge, bool bitStarFullQueue)
{
    std::stringstream rval;

    rval << "%%%%%% Extra Config %%%%%%" << std::endl;
    rval << "title('" << plannerName << "');" << std::endl;
    rval << "plotInformedEllipse = " << (informedWorldEllipse ? "true" : "false") << ";" << std::endl; //<condition> ? <true-case-code> : <false-case-code>;
    rval << "plotBitStarQueueEllipse = " << (bitStarQueueEllipse ? "true" : "false") << ";" << std::endl; //<condition> ? <true-case-code> : <false-case-code>;
    rval << "plotBitStarNextEdge = " << (bitStarNextEdge ? "true" : "false") << ";" << std::endl; //<condition> ? <true-case-code> : <false-case-code>;
    rval << "plotBitStarFullQueue = " << (bitStarFullQueue ? "true" : "false") << ";" << std::endl; //<condition> ? <true-case-code> : <false-case-code>;

    return rval.str();
}


void createMatlabHelpers(std::string path)
{
    //Variable:
    //The filestream
    std::ofstream mfile;
    //The string stream of the filename
    std::stringstream fileName;

    fileName << path << "filledCircle.m";

    if (boost::filesystem::exists( fileName.str() ) == false)
    {
        createDirectories(path);

        //std::cout << "Creating m-file function: filledCircle.m" << std::endl;

        mfile.open(fileName.str().c_str());
        mfile << "%From: http://www.mathworks.com/matlabcentral/fileexchange/27703-draw-a-filled-circle/content/filledCircle.m" << std::endl;
        mfile << "function h = filledCircle(center,r,N,color)" << std::endl;
        mfile << "    THETA=linspace(0,2*pi,N);" << std::endl;
        mfile << "    RHO=ones(1,N)*r;" << std::endl;
        mfile << "    [X,Y] = pol2cart(THETA,RHO);" << std::endl;
        mfile << "    X=X+center(1);" << std::endl;
        mfile << "    Y=Y+center(2);" << std::endl;
        mfile << "    h=fill(X,Y,color);" << std::endl;
        mfile << "end" << std::endl;
        mfile.close();
    }
    //No else
}


void writeMatlabMap(BaseExperimentPtr experiment, PlannerType plannerType, ompl::base::PlannerPtr planner, unsigned int worldSeed, bool informedWorldEllipse, bool bitStarQueueEllipse, bool bitStarNextEdge, bool bitStarFullQueue, std::string path /*= "plots/"*/, std::string postFix /*= std::string()*/, bool monochrome /*= false*/)
{
    //If we're 2D, plot:
    if (planner->getSpaceInformation()->getStateDimension() == 2u)
    {
        //Variable:
        //The filestream
        std::ofstream mfile;
        //The file name as a string stream
        std::stringstream fileName;
        //The space information
        ompl::base::SpaceInformationPtr si = planner->getSpaceInformation();
        //The problem definition
        ompl::base::ProblemDefinitionPtr pdef;
        //The optimization objective
        ompl::base::OptimizationObjectivePtr opt;
        //The start and goal as vectors of pairs:
        std::vector<std::pair<double, double> > startPairs;
        std::vector<std::pair<double, double> > goalPairs;

        //And the problem definition
        pdef = planner->getProblemDefinition();

        //and the optimization objective
        opt = pdef->getOptimizationObjective();

        //Make the file name:
        fileName << path << planner->getName() << "S" << worldSeed << postFix << ".m";

        //Create the directories
        createDirectories(fileName.str());

        //Create the helpers
        createMatlabHelpers(path);

        //Announce:
//        std::cout << "Creating m-file plot: " << fileName.str() << std::endl;

        //Open the file with the name of the planner
        mfile.open(fileName.str().c_str());

        //Create the matlab header:
//        if (plannerType == PLANNER_HYBRID_BITSTAR)
//        {
//            if (bool(planner->as<ompl::geometric::HybridBITstar>()->getNextShortcut().get<0u>()) && bitStarNextEdge == true)
//            {
//                mfile << matlabGlobalHeader(planner->getName(), 0.80);
//            }
//            else
//            {
//                mfile << matlabGlobalHeader(planner->getName());
//            }
//        }
//        else
//        {

        //Write the experiment header as well as the extra information
        mfile << experiment->mfileHeader(monochrome);
        mfile << matlabExtraHeader(planner->getName(), informedWorldEllipse, bitStarQueueEllipse, bitStarNextEdge, bitStarFullQueue);

        if (postFix != "I000000")
        {
            mfile << std::endl;
            mfile << "%%%%%%PLANNER%%%%%%" << std::endl;
            mfile << std::endl;
            //If the planner is a BIT*, do some special stuff
            if (plannerType == PLANNER_BITSTAR || plannerType == PLANNER_BITSTAR_SEED)
            {
                //Full queue
                //Variables
                //The vector of edges in the queue
                std::vector<std::pair<ompl::geometric::BITstar::VertexConstPtr, ompl::geometric::BITstar::VertexConstPtr> > queueEdges;

                //Get the queue edges
                planner->as<ompl::geometric::BITstar>()->getEdgeQueue(&queueEdges);

                //Annotate:
                mfile << "%%%%%% Queue edges %%%%%%" << std::endl;
                mfile << "if plotBitStarFullQueue" << std::endl;

                //Iterate over the list of edges, calling the edge-plot function:
                for (unsigned int i = 0u; i < queueEdges.size(); ++i)
                {
                    mfile << "    " << plotEdge(experiment, queueEdges.at(i).first->stateConst(), queueEdges.at(i).second->stateConst(), "queueColour", "queueStyle", "queueWeight");
                }
                mfile << "end" << std::endl;
            }
//            else if (plannerType == PLANNER_HYBRID_BITSTAR)
//            {
//                //Full queue
//                //Variables
//                //The vector of edges in the queue
//                std::vector<std::pair<ompl::geometric::HybridBITstar::VertexConstPtr, ompl::geometric::HybridBITstar::VertexConstPtr> > queueEdges;
//                //The vector of shortcuts in the queue:
//                std::vector<boost::tuple<ompl::geometric::HybridBITstar::VertexConstPtr, ompl::geometric::HybridBITstar::VertexConstPtr, ompl::geometric::HybridBITstar::VertexConstPtr, ompl::geometric::HybridBITstar::VertexConstPtr> > queueShortcuts;
//
//                //Get the queue edges
//                planner->as<ompl::geometric::HybridBITstar>()->getEdgeQueue(&queueEdges);
//
//                //Annotate:
//                mfile << "%%%%%% Queue edges %%%%%%" << std::endl;
//                mfile << "if plotBitStarFullQueue" << std::endl;
//
//                //Iterate over the list of edges, calling the edge-plot function:
//                for (unsigned int i = 0u; i < queueEdges.size(); ++i)
//                {
//                    mfile << "    " << plotEdge(experiment, queueEdges.at(i).first->stateConst(), queueEdges.at(i).second->stateConst(), "queueColour", "queueStyle", "queueWeight");
//                }
//                mfile << "end" << std::endl;
//
//                //Get the shortcut edges
//                planner->as<ompl::geometric::HybridBITstar>()->getShortcutQueue(&queueShortcuts);
//
//                //Annotate:
//                mfile << "%%%%%% Shortcut edges %%%%%%" << std::endl;
//                mfile << "if plotBitStarFullQueue" << std::endl;
//
//                //Iterate over the list of shortcuts, calling the edge-plot function 3 times:
//                for (unsigned int i = 0u; i < queueShortcuts.size(); ++i)
//                {
//                    mfile << "    "  << plotEdge(experiment, queueShortcuts.at(i).get<0u>()->stateConst(), queueShortcuts.at(i).get<1u>()->stateConst(), "queueColour", "queueStyle", "queueWeight");
//                    mfile << "    "  << plotEdge(experiment, queueShortcuts.at(i).get<1u>()->stateConst(), queueShortcuts.at(i).get<2u>()->stateConst(), "queueColour", "queueStyle", "queueWeight");
//                    mfile << "    "  << plotEdge(experiment, queueShortcuts.at(i).get<2u>()->stateConst(), queueShortcuts.at(i).get<3u>()->stateConst(), "queueColour", "queueStyle", "queueWeight");
//                }
//                mfile << "end" << std::endl;
//            }
//            else if (plannerType == PLANNER_DUALTREE_BITSTAR)
//            {
//                //Full queue
//                //Variables
//                //The vector of edges in the queue
//                std::vector<std::pair<ompl::geometric::ICRA16::MultigraphVertexConstPtr, ompl::geometric::ICRA16::MultigraphVertexConstPtr> > queueEdges;
//
//                //Get the queue edges
//                for (unsigned int i = 0u; i < 2u; ++i)
//                {
//                    queueEdges.clear();
//
//                    planner->as<ompl::geometric::ICRA16>()->getSubEdgeQueue(i, &queueEdges);
//
//                    //Annotate:
//                    mfile << "%%%%%% Queue " << i << " edges %%%%%%" << std::endl;
//                    mfile << "if plotBitStarFullQueue" << std::endl;
//
//                    //Iterate over the list of edges, calling the edge-plot function:
//                    for (unsigned int i = 0u; i < queueEdges.size(); ++i)
//                    {
//                        mfile << "    " << plotEdge(experiment, queueEdges.at(i).first->stateConst(), queueEdges.at(i).second->stateConst(), "queueColour", "queueStyle", "queueWeight");
//                    }
//                    mfile << "end" << std::endl;
//                }
//            }

            //Annotate the mfile:
            mfile << "%%%%%% The graph %%%%%%" << std::endl;

//            if (plannerType == PLANNER_DUALTREE_BITSTAR)
//            {
//                //Get the start tree and then the goal tree
//
//                //Variables:
//                //The planner data
//                ompl::base::PlannerData startData(si);
//                ompl::base::PlannerData goalData(si);
//
//
//                //The goal tree:
//                //Get the data to output the results:
//                planner->as<ompl::geometric::ICRA16>()->getSubPlannerData(1u, goalData);
//
//                mfile << "%%%%%% Reverse tree %%%%%%" << std::endl;
//
//                //Write the planner data to file:
//                for (unsigned int i = 0u; i < goalData.numVertices(); ++i)
//                {
//                    //The vertex being processed:
//                    ompl::base::PlannerDataVertex vertex = goalData.getVertex(i);
//                    //The vector of parent ids
//                    std::vector<unsigned int> parentIds;
//
//                    //Not all indexes exist I think?
//                    if (vertex != ompl::base::PlannerData::NO_VERTEX)
//                    {
//                        //Print it's coordinates to file:
//                        mfile << plotVertex(experiment, vertex.getState(), "vertexColour", "vertexSize");
//
//                        //Get it's incoming edge, if it has one:
//                        if (goalData.getIncomingEdges(i, parentIds) > 0u)
//                        {
//                            //The parent of the vertex:
//                            ompl::base::PlannerDataVertex parent = goalData.getVertex(parentIds.front());
//
//                             //Plot the edge:
//                             mfile << plotEdge(experiment, parent.getState(), vertex.getState(), "goalEdgeColour", "edgeStyle", "edgeWeight");
//                        }
//                    }
//                }
//
//                //The start tree:
//                //Get the data to output the results:
//                planner->as<ompl::geometric::ICRA16>()->getSubPlannerData(0u, startData);
//
//                mfile << "%%%%%% Forward tree %%%%%%" << std::endl;
//
//                //Write the planner data to file:
//                for (unsigned int i = 0u; i < startData.numVertices(); ++i)
//                {
//                    //The vertex being processed:
//                    ompl::base::PlannerDataVertex vertex = startData.getVertex(i);
//                    //The vector of parent ids
//                    std::vector<unsigned int> parentIds;
//
//                    //Not all indexes exist I think?
//                    if (vertex != ompl::base::PlannerData::NO_VERTEX)
//                    {
//                        //Print it's coordinates to file:
//                        mfile << plotVertex(experiment, vertex.getState(), "vertexColour", "vertexSize");
//
//                        //Get it's incoming edge, if it has one:
//                        if (startData.getIncomingEdges(i, parentIds) > 0u)
//                        {
//                            //The parent of the vertex:
//                            ompl::base::PlannerDataVertex parent = startData.getVertex(parentIds.front());
//
//                             //Plot the edge:
//                             mfile << plotEdge(experiment, parent.getState(), vertex.getState(), "startEdgeColour", "edgeStyle", "edgeWeight");
//                        }
//                    }
//                }
//            }
//            else
//            {
                //The planner data
                ompl::base::PlannerData pdata(si);
                //Get the data to output the results:
                planner->getPlannerData(pdata);

                //Write the planner data to file:
                for (unsigned int i = 0u; i < pdata.numVertices(); ++i)
                {
                    //The vertex being processed:
                    ompl::base::PlannerDataVertex vertex = pdata.getVertex(i);
                    //The vector of parent ids
                    std::vector<unsigned int> parentIds;

                    //Not all indexes exist I think?
                    if (vertex != ompl::base::PlannerData::NO_VERTEX)
                    {
                        //Print it's coordinates to file:
                        mfile << plotVertex(experiment, vertex.getState(), "vertexColour", "vertexSize");

                        //Get it's incoming edge, if it has one:
                        if (pdata.getIncomingEdges(i, parentIds) > 0u)
                        {
                            //The parent of the vertex:
                            ompl::base::PlannerDataVertex parent = pdata.getVertex(parentIds.front());

                             //Plot the edge:
                             mfile << plotEdge(experiment, parent.getState(), vertex.getState(), "startEdgeColour", "edgeStyle", "edgeWeight");
                        }
                    }
                }
//            }

            //If the planner is a BIT*, do some more special stuff
            if (plannerType == PLANNER_BITSTAR || plannerType == PLANNER_BITSTAR_SEED)
            {
                //Value of best edge as an ellipse:
                if (std::isfinite(planner->as<ompl::geometric::BITstar>()->getNextEdgeValueInQueue().value()))
                {
                    //Annotate
                    mfile << "%%%%%% Queue ellipses %%%%%%" << std::endl;

                    mfile << "if plotBitStarQueueEllipse" << std::endl;
                    mfile << "    for  i = 1:size(xstarts,2)" << std::endl;
                    mfile << "        for  j = 1:size(xgoals,2)" << std::endl;
                    mfile << "            S = ellipseMatrix(xstarts(:,i), xgoals(:,j)," << planner->as<ompl::geometric::BITstar>()->getNextEdgeValueInQueue().value() << ", true);" << std::endl;
                    mfile << "            plotEllipseMatrix(S, 0.5.*(xstarts(:,i) + xgoals(:,j)), 100);" << std::endl;
                    mfile << "        end" << std::endl;
                    mfile << "    end" << std::endl;
                    mfile << "end" << std::endl;
                }


                //Best edge in queue
                std::pair<ompl::base::State const*, ompl::base::State const*> edge;

                edge = planner->as<ompl::geometric::BITstar>()->getNextEdgeInQueue();

                if (bool(edge.first) && bool(edge.second))
                {
                    //Annotate:
                    mfile << "%%%%%% Next edge in the queue %%%%%%" << std::endl;
                    mfile << "if plotBitStarNextEdge" << std::endl;

                    mfile << "    " << plotEdge(experiment, edge.first, edge.second, "nextEdgeColour", "nextEdgeStyle", "nextEdgeWeight");

                    mfile << "end" << std::endl;
                }

                //Append the queue value to the xlabel if plotting ellipse or edge
                if (std::isfinite(planner->as<ompl::geometric::BITstar>()->getNextEdgeValueInQueue().value()))
                {
                    mfile << "xlabel(['q=' num2str(" << planner->as<ompl::geometric::BITstar>()->getNextEdgeValueInQueue().value() << ", '%.6f') ', ' get(get(gca, 'xlabel'), 'String')]);" << std::endl;
                }
            }
//            else if (plannerType == PLANNER_HYBRID_BITSTAR)
//            {
//                //Value of best edge as an ellipse:
//                if (std::isfinite(planner->as<ompl::geometric::HybridBITstar>()->getNextEdgeValueInQueue().value()))
//                {
//                    //Annotate
//                    mfile << "%%%%%% Queue ellipses %%%%%%" << std::endl;
//
//                    mfile << "if plotBitStarQueueEllipse" << std::endl;
//                    mfile << "    for  i = 1:size(xstarts,2)" << std::endl;
//                    mfile << "        for  j = 1:size(xgoals,2)" << std::endl;
//                    mfile << "            S = ellipseMatrix(xstarts(:,i), xgoals(:,j)," << planner->as<ompl::geometric::HybridBITstar>()->getNextEdgeValueInQueue().value() << ");" << std::endl;
//                    mfile << "            plotEllipseMatrix(S, 0.5.*(xstarts(:,i) + xgoals(:,j)), 100);" << std::endl;
//                    mfile << "        end" << std::endl;
//                    mfile << "    end" << std::endl;
//                    mfile << "end" << std::endl;
//                }
//
//
//                //Best edge in queue
//                std::pair<ompl::base::State const*, ompl::base::State const*> edge;
//                boost::tuple<ompl::base::State const*, ompl::base::State const*, ompl::base::State const*, ompl::base::State const*> shortcut;
//
//                edge = planner->as<ompl::geometric::HybridBITstar>()->getNextEdgeInQueue();
//
//                if (bool(edge.first) && bool(edge.second))
//                {
//                    //Annotate:
//                    mfile << "%%%%%% Next edge in the queue %%%%%%" << std::endl;
//                    mfile << "if plotBitStarNextEdge" << std::endl;
//
//                    mfile << "    " << plotEdge(experiment, edge.first, edge.second, "nextEdgeColour", "nextEdgeStyle", "nextEdgeWeight");
//
//                    mfile << "end" << std::endl;
//                }
//
//                shortcut = planner->as<ompl::geometric::HybridBITstar>()->getNextShortcut();
//
//                if (bool(shortcut.get<0u>()) && bool(shortcut.get<1u>()) && bool(shortcut.get<2u>()) && bool(shortcut.get<3u>()))
//                {
//                    //Annotate:
//                    mfile << "%%%%%% Next shortcut in the queue %%%%%%" << std::endl;
//                    mfile << "if plotBitStarNextEdge" << std::endl;
//
//                    mfile << "    " << plotEdge(experiment, shortcut.get<0u>(), shortcut.get<1u>(), "nextEdgeColour", "nextEdgeStyle", "nextEdgeWeight");
//                    mfile << "    " << plotEdge(experiment, shortcut.get<1u>(), shortcut.get<2u>(), "nextEdgeColour", "nextEdgeStyle", "nextEdgeWeight");
//                    mfile << "    " << plotEdge(experiment, shortcut.get<2u>(), shortcut.get<3u>(), "nextEdgeColour", "nextEdgeStyle", "nextEdgeWeight");
//
//                    mfile << "end" << std::endl;
//                }
//
//                //Append the queue value to the xlabel if plotting ellipse or edge
//                if (std::isfinite(planner->as<ompl::geometric::HybridBITstar>()->getNextEdgeValueInQueue().value()))
//                {
//                    mfile << "xlabel(['q=' num2str(" << planner->as<ompl::geometric::HybridBITstar>()->getNextEdgeValueInQueue().value() << ", '%.6f') ', ' get(get(gca, 'xlabel'), 'String')]);" << std::endl;
//                }
//            }
//            else if (plannerType == PLANNER_DUALTREE_BITSTAR)
//            {
//                //Value of best edge as an ellipse:
//                if (std::isfinite(planner->as<ompl::geometric::ICRA16>()->getNextEdgeValueInQueue().value()))
//                {
//                    //Annotate
//                    mfile << "%%%%%% Queue ellipses %%%%%%" << std::endl;
//
//                    mfile << "if plotBitStarQueueEllipse" << std::endl;
//                    mfile << "    for  i = 1:size(xstarts,2)" << std::endl;
//                    mfile << "        for  j = 1:size(xgoals,2)" << std::endl;
//                    mfile << "            S = ellipseMatrix(xstarts(:,i), xgoals(:,j)," << planner->as<ompl::geometric::ICRA16>()->getNextEdgeValueInQueue().value() << ");" << std::endl;
//                    mfile << "            plotEllipseMatrix(S, 0.5.*(xstarts(:,i) + xgoals(:,j)), 100);" << std::endl;
//                    mfile << "        end" << std::endl;
//                    mfile << "    end" << std::endl;
//                    mfile << "end" << std::endl;
//                }
//
//
//                //Best edge in queue
//                std::pair<ompl::base::State const*, ompl::base::State const*> edge;
//
//                edge = planner->as<ompl::geometric::ICRA16>()->getNextEdgeInQueue();
//
//                if (bool(edge.first) && bool(edge.second))
//                {
//                    //Annotate:
//                    mfile << "%%%%%% Next edge in the queue %%%%%%" << std::endl;
//
//                    mfile << "if plotBitStarNextEdge" << std::endl;
//                    mfile << "    " << plotEdge(experiment, edge.first, edge.second, "nextEdgeColour", "nextEdgeStyle", "nextEdgeWeight");
//
//                    mfile << "end" << std::endl;
//                }
//
//                //Append the queue value to the xlabel if plotting ellipse or edge
//                if (std::isfinite(planner->as<ompl::geometric::ICRA16>()->getNextEdgeValueInQueue().value()))
//                {
//                    mfile << "xlabel(['q=' num2str(" << planner->as<ompl::geometric::ICRA16>()->getNextEdgeValueInQueue().value() << ", '%.6f') ', ' get(get(gca, 'xlabel'), 'String')]);" << std::endl;
//                }
//            }

            if (plannerType == PLANNER_RRTSTAR_INFORMED || plannerType == PLANNER_BITSTAR || plannerType == PLANNER_BITSTAR_SEED || plannerType == PLANNER_HYBRID_BITSTAR || plannerType == PLANNER_DUALTREE_BITSTAR)
            {
                if (pdef->hasExactSolution() == true)
                {
                    //Annotate:
                    mfile << "%%%%%% World ellipses %%%%%%" << std::endl;

                    mfile << "if plotInformedEllipse" << std::endl;
                    mfile << "    for  i = 1:size(xstarts,2)" << std::endl;
                    mfile << "        for  j = 1:size(xgoals,2)" << std::endl;
                    mfile << "            S = ellipseMatrix(xstarts(:,i), xgoals(:,j)," << pdef->getSolutionPath()->cost(opt).value() << ", true);" << std::endl;
                    mfile << "            plotEllipseMatrix(S, 0.5.*(xstarts(:,i) + xgoals(:,j)), 100, '-.', 3, k);" << std::endl;
                    mfile << "        end" << std::endl;
                    mfile << "    end" << std::endl;
                    mfile << "end" << std::endl;
                }
            }

            //Plot the solution (if one exists)
            if (pdef->hasExactSolution() == true)
            {
                std::vector<ompl::base::State*> path;

                //Annotate:
                mfile << "%%%%%% The solution %%%%%%" << std::endl;

                path = pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>()->getStates();

                for (unsigned int i = 1u; i < path.size(); ++i)
                {
                    mfile << plotEdge(experiment, path.at(i - 1u), path.at(i), "solnColour", "solnStyle", "solnWeight");
                }
                mfile << "xlabel(['c=' num2str(" << pdef->getSolutionPath()->cost(opt).value() << ", '%.6f')]);" << std::endl;
            }
            else
            {
                mfile << "xlabel('');" << std::endl;
            }
        }

        mfile << experiment->mfileFooter();

        mfile.close();
    }
}

ompl::time::duration createAnimation(BaseExperimentPtr experiment, PlannerType plannerType, ompl::base::PlannerPtr planner, unsigned int worldSeed, ompl::time::duration timeToRun, bool informedWorldEllipse, bool bitStarEllipse, bool bitStarEdge, bool bitStarQueue, unsigned int initialIterNumber /*= 0u*/, bool monochrome /*= false*/)
{
    //Variables:
    //A 1-iteration class:
    ompl::base::IterationTerminationCondition iterationPtc(1u);
    //Met the optimization objective
    bool optimized;
    //The start time
    ompl::time::point startTime;
    //The total run time of the algorithm:
    ompl::time::duration runTime(0,0,0,0);
    //The path
    std::stringstream pathStream;
    //The number of iterations
    unsigned int iter = initialIterNumber;
    //The filestream for the animation file
    std::ofstream mfile;
    //The animation name stream
    std::stringstream animationStream;
    //The planner status
    ompl::base::PlannerStatus plannerStatus = ompl::base::PlannerStatus::UNKNOWN;

    pathStream << "frames/" << planner->getName() << "S" << worldSeed << "/";

    //Run until the real PTC is met
    optimized = false;

    //Write initial map:
    if (iter == 0u)
    {
        writeMatlabMap(experiment, plannerType, planner, worldSeed, informedWorldEllipse, bitStarEllipse, bitStarEdge, bitStarQueue, pathStream.str(), "I000000", monochrome);
        writeMatlabMap(experiment, plannerType, planner, worldSeed, informedWorldEllipse, bitStarEllipse, bitStarEdge, bitStarQueue, pathStream.str(), "I000001", monochrome);
        ++iter;
    }

    while (runTime < timeToRun && optimized == false && (plannerType != PLANNER_FMTSTAR || (plannerStatus != ompl::base::PlannerStatus::EXACT_SOLUTION && plannerStatus != ompl::base::PlannerStatus::CRASH)))
    {
        //The iteration as a string
        std::stringstream iterStream;

        ++iter;

        iterationPtc.reset();

        startTime = ompl::time::now();
        plannerStatus = planner->solve(iterationPtc);
        runTime = runTime + (ompl::time::now() - startTime);

        //Make frame:
        iterStream << "I" << std::setfill('0') << std::setw(6) << iter;

        writeMatlabMap(experiment, plannerType, planner, worldSeed, informedWorldEllipse, bitStarEllipse, bitStarEdge, bitStarQueue, pathStream.str(), iterStream.str(), monochrome);

        if (planner->getProblemDefinition()->hasExactSolution() == true)
        {
            if (planner->getProblemDefinition()->getSolutionPath())
            {
//                std::cout << "Optimized? " << planner->getProblemDefinition()->getSolutionPath()->cost(planner->getProblemDefinition()->getOptimizationObjective()).value() << " < " << planner->getProblemDefinition()->getOptimizationObjective()->getCostThreshold().value() << std::endl;
                optimized = planner->getProblemDefinition()->getOptimizationObjective()->isSatisfied( planner->getProblemDefinition()->getSolutionPath()->cost(planner->getProblemDefinition()->getOptimizationObjective()) );
            }
        }
    }

    //Now make the master animation file:

    animationStream << "frames/" << planner->getName() << "S" << worldSeed << ".m";

    createDirectories(animationStream.str());

    mfile.open(animationStream.str().c_str());

    mfile << "startFrame = 0;" << std::endl;
    mfile << "endFrame = " << iter << ";" << std::endl;
    mfile << std::endl;


    mfile << "warning('off', 'ASRL:Ellipses')" << std::endl;
    mfile << "figure;" << std::endl;
    mfile << "cd " << planner->getName() << "S" << worldSeed << "/" << ";" << std::endl;
    mfile << "for i = startFrame:endFrame" << std::endl;
    mfile << "    iterString = sprintf('%06d',i);" << std::endl;
    mfile << "    eval(['" << planner->getName() << "S" << worldSeed << "I' iterString]);" << std::endl;
    mfile << "    print('-dpng', [iterString '.png']);" << std::endl;
    mfile << "end" << std::endl;
    mfile << "close;" << std::endl;
    mfile << "warning('on', 'ASRL:Ellipses')" << std::endl;
    mfile << std::endl;

    mfile << "system('LD_LIBRARY_PATH="" && mencoder mf://./*.png -mf w=1200:h=900:fps=30:type=png -ovc lavc -lavcopts vcodec=mpeg4 -o " << planner->getName() << "S" << worldSeed <<".avi');" << std::endl;
    mfile << "system('mkdir -p src');" << std::endl;
    mfile << "system('mv *.m ./src/');" << std::endl;
    mfile << "system('mkdir -p png');" << std::endl;
    mfile << "system('mv *.png ./png/');" << std::endl;
    mfile << "cd ../;" << std::endl;
    mfile.close();

    return runTime;
}
