//Me
#include "tools/plotting_tools.h"

//For std::ifstream and std::ofstream
#include <fstream>
//For std::setfill and std::setw and std::setprecision
#include <iomanip>
//C++11 Tuple
#include <tuple>
//For boost::filesystem
#include <boost/filesystem.hpp>

//R^n
#include <ompl/base/spaces/RealVectorStateSpace.h>
//For geometric::path
#include <ompl/geometric/PathGeometric.h>
//For ompl::base::GoalStates:
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/goals/GoalStates.h"
#include "ompl/base/OptimizationObjective.h"
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include "ompl/geometric/planners/bitstar/datastructures/Vertex.h"
#include <ompl/util/Exception.h>

#ifdef BITSTAR_REGRESSION
#include <ompl/geometric/planners/bitstar_regression/BITstarRegression.h>
#include "ompl/geometric/planners/bitstar_regression/datastructures/Vertex.h"
#endif // BITSTAR_REGRESSION

std::string plotVertex(BaseExperimentPtr experiment, const ompl::base::State* vertex, std::string vertexColour, std::string vertexSize)
{
    //A scoped state
    ompl::base::ScopedState<> scopedVertex(experiment->getSpaceInformation()->getStateSpace(), vertex);

    std::stringstream rval;
    rval << "plot(" <<  scopedVertex[0] << ", " << scopedVertex[1] << ", 'o', 'Color', " << vertexColour << ", 'MarkerFaceColor', " << vertexColour << ", 'MarkerSize', " << vertexSize << ");" << '\n';
    return rval.str();
}

std::string plotEdge(BaseExperimentPtr experiment, const ompl::base::State* parent, const ompl::base::State* child, std::string edgeColour, std::string lineStyle, std::string edgeWeight)
{
    //Scoped states
    ompl::base::ScopedState<> scopedParent(experiment->getSpaceInformation()->getStateSpace(), parent);
    ompl::base::ScopedState<> scopedChild(experiment->getSpaceInformation()->getStateSpace(), child);

    std::stringstream rval;
    rval << "plot([" << scopedParent[0] << ", " << scopedChild[0] << "], [" << scopedParent[1] << ", " << scopedChild[1] << "], '-', 'Color', " << edgeColour << ", 'LineStyle', " << lineStyle << ", 'LineWidth', " << edgeWeight << ");" << '\n';
    return rval.str();
}

std::string matlabExtraHeader(std::string plannerName, bool plotVertices, bool plotIndices, bool informedWorldEllipse, bool bitStarQueueEllipse, bool bitStarNextEdge, bool bitStarFullQueue)
{
    std::stringstream rval;

    rval << "%%%%%% Extra Config %%%%%%" << '\n';
    rval << "titleText = '" << plannerName << "';" << '\n';
    rval << "xLabelText = [];" << '\n';
    rval << "plotVertices = " << (plotVertices ? "true" : "false") << ";" << '\n'; //<condition> ? <true-case-code> : <false-case-code>;
    rval << "plotIndices = " << (plotIndices ? "true" : "false") << ";" << '\n'; //<condition> ? <true-case-code> : <false-case-code>;
    rval << "plotInformedEllipse = " << (informedWorldEllipse ? "true" : "false") << ";" << '\n'; //<condition> ? <true-case-code> : <false-case-code>;
    rval << "plotBitStarQueueEllipse = " << (bitStarQueueEllipse ? "true" : "false") << ";" << '\n'; //<condition> ? <true-case-code> : <false-case-code>;
    rval << "plotBitStarNextEdge = " << (bitStarNextEdge ? "true" : "false") << ";" << '\n'; //<condition> ? <true-case-code> : <false-case-code>;
    rval << "plotBitStarFullQueue = " << (bitStarFullQueue ? "true" : "false") << ";" << '\n'; //<condition> ? <true-case-code> : <false-case-code>;

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

        //std::cout << "Creating m-file function: filledCircle.m" << '\n';

        mfile.open(fileName.str().c_str());
        mfile << "%From: http://www.mathworks.com/matlabcentral/fileexchange/27703-draw-a-filled-circle/content/filledCircle.m" << '\n';
        mfile << "function h = filledCircle(center,r,N,color)" << '\n';
        mfile << "    THETA=linspace(0,2*pi,N);" << '\n';
        mfile << "    RHO=ones(1,N)*r;" << '\n';
        mfile << "    [X,Y] = pol2cart(THETA,RHO);" << '\n';
        mfile << "    X=X+center(1);" << '\n';
        mfile << "    Y=Y+center(2);" << '\n';
        mfile << "    h=fill(X,Y,color);" << '\n';
        mfile << "end" << '\n';
        mfile.flush();
        mfile.close();
    }
    //No else
}


void writeMatlabMap(BaseExperimentPtr experiment, PlannerType plannerType, ompl::base::PlannerPtr planner, unsigned int worldSeed, bool plotVertices, bool plotIndices, bool informedWorldEllipse, bool bitStarQueueEllipse, bool bitStarNextEdge, bool bitStarFullQueue, std::string path /*= "plots/"*/, std::string postFix /*= std::string()*/, bool monochrome /*= false*/)
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

        //Open the file with the name of the planner
        mfile.open(fileName.str().c_str());

        //Write the experiment header as well as the extra information
        mfile << experiment->mfileHeader(monochrome);
        mfile << matlabExtraHeader(planner->getName(), plotVertices, plotIndices, informedWorldEllipse, bitStarQueueEllipse, bitStarNextEdge, bitStarFullQueue);

        if (postFix != "I000000")
        {
            mfile << '\n';
            mfile << "%%%%%%PLANNER%%%%%%" << '\n';
            mfile << '\n';
            //If the planner is a BIT*, do some special stuff
            if (plannerType == PLANNER_BITSTAR || plannerType == PLANNER_BITSTAR_SEED || plannerType == PLANNER_ABITSTAR)
            {
                //Get the queue edges
                std::vector<std::pair<ompl::geometric::BITstar::VertexConstPtr, ompl::geometric::BITstar::VertexConstPtr> > queueEdges;
                planner->as<ompl::geometric::BITstar>()->getEdgeQueue(&queueEdges);

                std::vector<double> queueXCoordsChildren { };
                std::vector<double> queueYCoordsChildren { };
                std::vector<double> queueXCoordsParents { };
                std::vector<double> queueYCoordsParents { };

                //Iterate over the list of edges, calling the edge-plot function:
                for (const auto &edge : queueEdges)
                {
                     ompl::base::ScopedState<> scopedChild(experiment->getSpaceInformation()->getStateSpace(), edge.first->state());
                     queueXCoordsChildren.emplace_back(scopedChild[0]);
                     queueYCoordsChildren.emplace_back(scopedChild[1]);

                     ompl::base::ScopedState<> scopedParent(experiment->getSpaceInformation()->getStateSpace(), edge.second->state());
                     queueXCoordsParents.emplace_back(scopedParent[0]);
                     queueYCoordsParents.emplace_back(scopedParent[1]);
                }
                //Annotate:
                mfile << "%%%%%% Queue edges %%%%%%" << '\n';
                mfile << "if plotBitStarFullQueue" << '\n';

                mfile << "    queueXCoordsChildren = [ ";
                for (const auto x : queueXCoordsChildren)
                {
                    mfile << x << ' ';
                }
                mfile << "];\n";
                mfile << "    queueYCoordsChildren = [ ";
                for (const auto y : queueYCoordsChildren)
                {
                    mfile << y << ' ';
                }
                mfile << "];\n";
                mfile << "    queueXCoordsParents = [ ";
                for (const auto x : queueXCoordsParents)
                {
                    mfile << x << ' ';
                }
                mfile << "];\n";
                mfile << "    queueYCoordsParents = [ ";
                for (const auto y : queueYCoordsParents)
                {
                    mfile << y << ' ';
                }
                mfile << "];\n";

                mfile << "    plot([ queueXCoordsChildren; queueXCoordsParents ], [ queueYCoordsChildren; queueYCoordsParents ], '-', 'Color', queueColour, 'LineStyle', queueStyle, 'LineWidth', queueWeight);" << '\n';
                mfile << "end" << '\n';
            }

            if (plannerType == PLANNER_BITSTAR_REGRESSION)
            {
                //Full queue
                //Variables
                //The vector of edges in the queue
                std::vector<std::pair<ompl::geometric::BITstarRegression::VertexConstPtr, ompl::geometric::BITstarRegression::VertexConstPtr> > queueEdges;

                //Get the queue edges
                planner->as<ompl::geometric::BITstarRegression>()->getEdgeQueue(&queueEdges);

                std::vector<double> queueXCoordsChildren { };
                std::vector<double> queueYCoordsChildren { };
                std::vector<double> queueXCoordsParents { };
                std::vector<double> queueYCoordsParents { };

                //Iterate over the list of edges, calling the edge-plot function:
                for (const auto &edge : queueEdges)
                {
                     ompl::base::ScopedState<> scopedChild(experiment->getSpaceInformation()->getStateSpace(), edge.first->stateConst());
                     queueXCoordsChildren.emplace_back(scopedChild[0]);
                     queueYCoordsChildren.emplace_back(scopedChild[1]);

                     ompl::base::ScopedState<> scopedParent(experiment->getSpaceInformation()->getStateSpace(), edge.second->stateConst());
                     queueXCoordsParents.emplace_back(scopedParent[0]);
                     queueYCoordsParents.emplace_back(scopedParent[1]);
                }
                //Annotate:
                mfile << "%%%%%% Queue edges %%%%%%" << '\n';
                mfile << "if plotBitStarFullQueue" << '\n';

                mfile << "    queueXCoordsChildren = [ ";
                for (const auto x : queueXCoordsChildren)
                {
                    mfile << x << ' ';
                }
                mfile << "];\n";
                mfile << "    queueYCoordsChildren = [ ";
                for (const auto y : queueYCoordsChildren)
                {
                    mfile << y << ' ';
                }
                mfile << "];\n";
                mfile << "    queueXCoordsParents = [ ";
                for (const auto x : queueXCoordsParents)
                {
                    mfile << x << ' ';
                }
                mfile << "];\n";
                mfile << "    queueYCoordsParents = [ ";
                for (const auto y : queueYCoordsParents)
                {
                    mfile << y << ' ';
                }
                mfile << "];\n";

                mfile << "    plot([ queueXCoordsChildren; queueXCoordsParents ], [ queueYCoordsChildren; queueYCoordsParents ], '-', 'Color', queueColour, 'LineStyle', queueStyle, 'LineWidth', queueWeight);" << '\n';
                mfile << "end" << '\n';
            }
            //Annotate the mfile:
            mfile << "%%%%%% The graph %%%%%%" << '\n';

            //The planner data
            ompl::base::PlannerData pdata(si);
            //Get the data to output the results:
            planner->getPlannerData(pdata);

            //Write the planner data to file. Vertices and edges separately:
            //Vertices only
            mfile << "%%%%%% Vertices %%%%%%" << '\n';
            std::vector<double> vertexXCoords { };
            std::vector<double> vertexYCoords { };
            std::vector<unsigned int> vertexIds { };
            for (unsigned int i = 0u; i < pdata.numVertices(); ++i)
            {
                //The vertex being processed:
                ompl::base::PlannerDataVertex vertex = pdata.getVertex(i);
                ompl::base::ScopedState<> scopedVertex(experiment->getSpaceInformation()->getStateSpace(), vertex.getState());

                //Not all indexes exist I think?
                if (vertex != ompl::base::PlannerData::NO_VERTEX)
                {
                    vertexXCoords.emplace_back(scopedVertex[0]);
                    vertexYCoords.emplace_back(scopedVertex[1]);
                    vertexIds.emplace_back(vertex.getTag());
                }
            }
            mfile << "if plotVertices" << '\n';
            mfile << "    vertexXCoords = [ ";
            for (const auto x : vertexXCoords)
            {
                mfile << x << ' ';
            }
            mfile << "];\n";
            mfile << "    vertexYCoords = [ ";
            for (const auto y : vertexYCoords)
            {
                mfile << y << ' ';
            }
            mfile << "];\n";
            mfile << "    plot(vertexXCoords, vertexYCoords, 'o', 'Color', vertexColour, 'MarkerFaceColor', vertexColour, 'MarkerSize', vertexSize);" << '\n';
            mfile << "    if plotIndices" << '\n';
            mfile << "        vertexIds = cellstr(num2str([ ";
            for (const auto id : vertexIds)
            {
                mfile << id << "; ";
            }
            mfile << "]));\n";
            mfile << "        text(vertexXCoords, vertexYCoords, vertexIds, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');" << '\n';
            mfile << "    end" << '\n';
            mfile << "end" << '\n';

            //Edges only
            mfile << "%%%%%% Edges %%%%%%" << '\n';
            std::vector<double> edgeXCoordsChildren { };
            std::vector<double> edgeYCoordsChildren { };
            std::vector<double> edgeXCoordsParents { };
            std::vector<double> edgeYCoordsParents { };
            for (unsigned int i = 0u; i < pdata.numVertices(); ++i)
            {
                //The vertex being processed:
                ompl::base::PlannerDataVertex vertex = pdata.getVertex(i);
                //The vector of parent ids
                std::vector<unsigned int> parentIds;

                //Not all indexes exist I think?
                if (vertex != ompl::base::PlannerData::NO_VERTEX)
                {
                    //Get it's incoming edge, if it has one:
                    if (pdata.getIncomingEdges(i, parentIds) > 0u)
                    {
                         //The parent of the vertex:
                         ompl::base::PlannerDataVertex parent = pdata.getVertex(parentIds.front());

                         ompl::base::ScopedState<> scopedChild(experiment->getSpaceInformation()->getStateSpace(), vertex.getState());
                         edgeXCoordsChildren.emplace_back(scopedChild[0]);
                         edgeYCoordsChildren.emplace_back(scopedChild[1]);

                         ompl::base::ScopedState<> scopedParent(experiment->getSpaceInformation()->getStateSpace(), parent.getState());
                         edgeXCoordsParents.emplace_back(scopedParent[0]);
                         edgeYCoordsParents.emplace_back(scopedParent[1]);
                    }
                }
            }
            mfile << "edgeXCoordsChildren = [ ";
            for (const auto x : edgeXCoordsChildren)
            {
                mfile << x << ' ';
            }
            mfile << "];\n";
            mfile << "edgeYCoordsChildren = [ ";
            for (const auto y : edgeYCoordsChildren)
            {
                mfile << y << ' ';
            }
            mfile << "];\n";
            mfile << "edgeXCoordsParents = [ ";
            for (const auto x : edgeXCoordsParents)
            {
                mfile << x << ' ';
            }
            mfile << "];\n";
            mfile << "edgeYCoordsParents = [ ";
            for (const auto y : edgeYCoordsParents)
            {
                mfile << y << ' ';
            }
            mfile << "];\n";

            mfile << "plot([ edgeXCoordsChildren; edgeXCoordsParents ], [ edgeYCoordsChildren; edgeYCoordsParents ], '-', 'Color', startEdgeColour, 'LineStyle', edgeStyle, 'LineWidth', edgeWeight);" << '\n';

            //If the planner is a BIT*, do some more special stuff
            if (plannerType == PLANNER_BITSTAR || plannerType == PLANNER_BITSTAR_SEED || plannerType == PLANNER_ABITSTAR)
            {
                //Value of best edge as an ellipse:
                if (std::isfinite(planner->as<ompl::geometric::BITstar>()->getNextEdgeValueInQueue().value()))
                {
                    //Annotate
                    mfile << "%%%%%% Queue ellipses %%%%%%" << '\n';

                    mfile << "queueValue = " << planner->as<ompl::geometric::BITstar>()->getNextEdgeValueInQueue().value() << ";" << '\n';
                    mfile << "if plotBitStarQueueEllipse" << '\n';
                    mfile << "    for  i = 1:size(xstarts,2)" << '\n';
                    mfile << "        for  j = 1:size(xgoals,2)" << '\n';
                    mfile << "            S = ellipseMatrix(xstarts(:,i), xgoals(:,j), queueValue, true);" << '\n';
                    mfile << "            plotEllipseMatrix(S, 0.5.*(xstarts(:,i) + xgoals(:,j)), 100, queueEllipseStyle, 3, queueEllipseColour);" << '\n';
                    mfile << "        end" << '\n';
                    mfile << "    end" << '\n';
                    mfile << "end" << '\n';
                }


                //Best edge in queue
                std::pair<ompl::base::State const*, ompl::base::State const*> edge;

                edge = planner->as<ompl::geometric::BITstar>()->getNextEdgeInQueue();

                if (bool(edge.first) && bool(edge.second))
                {
                    //Annotate:
                    mfile << "%%%%%% Next edge in the queue %%%%%%" << '\n';
                    mfile << "if plotBitStarNextEdge" << '\n';

                    mfile << "    " << plotEdge(experiment, edge.first, edge.second, "nextEdgeColour", "nextEdgeStyle", "nextEdgeWeight");

                    mfile << "end" << '\n';
                }

                //Append the queue value to the xlabel if plotting ellipse or edge
                if (std::isfinite(planner->as<ompl::geometric::BITstar>()->getNextEdgeValueInQueue().value()))
                {
                    mfile << "xLabelText = [xLabelText ' q=' num2str(queueValue, '%.6f')];" << '\n';
                }
            }

            if (plannerType == PLANNER_BITSTAR_REGRESSION)
            {
                //Value of best edge as an ellipse:
                if (std::isfinite(planner->as<ompl::geometric::BITstarRegression>()->getNextEdgeValueInQueue().value()))
                {
                    //Annotate
                    mfile << "%%%%%% Queue ellipses %%%%%%" << '\n';

                    mfile << "queueValue = " << planner->as<ompl::geometric::BITstarRegression>()->getNextEdgeValueInQueue().value() << ";" << '\n';
                    mfile << "if plotBitStarQueueEllipse" << '\n';
                    mfile << "    for  i = 1:size(xstarts,2)" << '\n';
                    mfile << "        for  j = 1:size(xgoals,2)" << '\n';
                    mfile << "            S = ellipseMatrix(xstarts(:,i), xgoals(:,j), queueValue, true);" << '\n';
                    mfile << "            plotEllipseMatrix(S, 0.5.*(xstarts(:,i) + xgoals(:,j)), 100, queueEllipseStyle, 3, queueEllipseColour);" << '\n';
                    mfile << "        end" << '\n';
                    mfile << "    end" << '\n';
                    mfile << "end" << '\n';
                }


                //Best edge in queue
                std::pair<ompl::base::State const*, ompl::base::State const*> edge;

                edge = planner->as<ompl::geometric::BITstarRegression>()->getNextEdgeInQueue();

                if (bool(edge.first) && bool(edge.second))
                {
                    //Annotate:
                    mfile << "%%%%%% Next edge in the queue %%%%%%" << '\n';
                    mfile << "if plotBitStarNextEdge" << '\n';

                    mfile << "    " << plotEdge(experiment, edge.first, edge.second, "nextEdgeColour", "nextEdgeStyle", "nextEdgeWeight");

                    mfile << "end" << '\n';
                }

                //Append the queue value to the xlabel if plotting ellipse or edge
                if (std::isfinite(planner->as<ompl::geometric::BITstarRegression>()->getNextEdgeValueInQueue().value()))
                {
                    mfile << "xLabelText = [xLabelText ' q=' num2str(queueValue, '%.6f')];" << '\n';
                }
            }

            //Info about the solution
            mfile << "%%%%%% Solution %%%%%%" << '\n';
            if (pdef->hasExactSolution() == true)
            {
                mfile << "solnCost = " << pdef->getSolutionPath()->cost(opt).value() << ";" << '\n';
            }
            else
            {
                mfile << "solnCost = inf;" << '\n';
            }
            mfile << "xLabelText = [xLabelText ' c=' num2str(solnCost, '%.6f')];" << '\n';

            // Info about the inflation and truncation factor
            mfile << "%%%%%% Inflation %%%%%%" << '\n';
            mfile << "inflationFactor = " << planner->as<ompl::geometric::BITstar>()->getCurrentInflationFactor() << ";" << '\n';
            mfile << "xLabelText = [ xLabelText ' inflation = ' num2str(inflationFactor) ', iter: ' iterString ];" << '\n';

            if (plannerType == PLANNER_RRTSTAR_INFORMED || plannerType == PLANNER_SORRTSTAR || plannerType == PLANNER_BITSTAR || plannerType == PLANNER_BITSTAR_SEED || plannerType == PLANNER_ABITSTAR || plannerType == PLANNER_BITSTAR_REGRESSION)
            {
                if (pdef->hasExactSolution() == true)
                {
                    //Annotate:
                    mfile << "%%%%%% World ellipses %%%%%%" << '\n';

                    mfile << "if plotInformedEllipse" << '\n';
                    mfile << "    for  i = 1:size(xstarts,2)" << '\n';
                    mfile << "        for  j = 1:size(xgoals,2)" << '\n';
                    mfile << "            S = ellipseMatrix(xstarts(:,i), xgoals(:,j), solnCost, true);" << '\n';
                    mfile << "            plotEllipseMatrix(S, 0.5.*(xstarts(:,i) + xgoals(:,j)), 100, worldEllipseStyle, 3, worldEllipseColour);" << '\n';
                    mfile << "        end" << '\n';
                    mfile << "    end" << '\n';
                    mfile << "end" << '\n';
                }
            }

            //Plot the solution (if one exists)
            if (pdef->hasExactSolution() == true)
            {
                std::vector<ompl::base::State*> path;

                //Annotate:
                mfile << "%%%%%% The solution path %%%%%%" << '\n';

                path = pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>()->getStates();

                for (unsigned int i = 1u; i < path.size(); ++i)
                {
                    mfile << plotEdge(experiment, path.at(i - 1u), path.at(i), "solnColour", "solnStyle", "solnWeight");
                }
            }
        }

        mfile << "%%%%%% Plot config %%%%%%" << '\n';
        mfile << "xlabel(xLabelText);" << '\n';
        mfile << "title(titleText);" << '\n';

        mfile << experiment->mfileFooter();

        mfile.flush();
        mfile.close();
    }
}

asrl::time::duration createAnimation(BaseExperimentPtr experiment, PlannerType plannerType, ompl::base::PlannerPtr planner, unsigned int worldSeed, asrl::time::duration timeToRun, bool plotVertices, bool plotIndices, bool informedWorldEllipse, bool bitStarEllipse, bool bitStarEdge, bool bitStarQueue, unsigned int initialIterNumber /*= 0u*/, bool monochrome /*= false*/)
{
    //Variables:
    //A 1-iteration class:
    ompl::base::IterationTerminationCondition iterationPtc(1u);
    //Met the optimization objective
    bool optimized;
    //The start time
    asrl::time::point startTime;
    //The total run time of the algorithm:
    asrl::time::duration runTime(0);
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
        writeMatlabMap(experiment, plannerType, planner, worldSeed, plotVertices, plotIndices, informedWorldEllipse, bitStarEllipse, bitStarEdge, bitStarQueue, pathStream.str(), "I000000", monochrome);
        writeMatlabMap(experiment, plannerType, planner, worldSeed, plotVertices, plotIndices, informedWorldEllipse, bitStarEllipse, bitStarEdge, bitStarQueue, pathStream.str(), "I000001", monochrome);
        ++iter;
    }

    while (runTime < timeToRun && optimized == false && (plannerType != PLANNER_FMTSTAR || (plannerStatus != ompl::base::PlannerStatus::EXACT_SOLUTION && plannerStatus != ompl::base::PlannerStatus::CRASH)))
    {
        //The iteration as a string
        std::stringstream iterStream;

        ++iter;

        iterationPtc.reset();

        startTime = asrl::time::now();
        plannerStatus = planner->solve(iterationPtc);
        runTime = runTime + (asrl::time::now() - startTime);

        //Make frame:
        iterStream << "I" << std::setfill('0') << std::setw(6) << iter;

        writeMatlabMap(experiment, plannerType, planner, worldSeed, plotVertices, plotIndices, informedWorldEllipse, bitStarEllipse, bitStarEdge, bitStarQueue, pathStream.str(), iterStream.str(), monochrome);

        if (planner->getProblemDefinition()->hasExactSolution() == true)
        {
            if (planner->getProblemDefinition()->getSolutionPath())
            {
//                std::cout << "Optimized? " << planner->getProblemDefinition()->getSolutionPath()->cost(planner->getProblemDefinition()->getOptimizationObjective()).value() << " < " << planner->getProblemDefinition()->getOptimizationObjective()->getCostThreshold().value() << '\n';
                optimized = planner->getProblemDefinition()->getOptimizationObjective()->isSatisfied( planner->getProblemDefinition()->getSolutionPath()->cost(planner->getProblemDefinition()->getOptimizationObjective()) );
            }
        }
    }

    //Now make the master animation file:

    animationStream << "frames/" << planner->getName() << "S" << worldSeed << ".m";

    createDirectories(animationStream.str());

    mfile.open(animationStream.str().c_str());

    mfile << "startFrame = 0;" << '\n';
    mfile << "endFrame = " << iter << ";" << '\n';
    mfile << '\n';


    mfile << "warning('off', 'ASRL:Ellipses')" << '\n';
    mfile << "figure;" << '\n';
    mfile << "cd " << planner->getName() << "S" << worldSeed << "/" << ";" << '\n';
    mfile << "for i = startFrame:endFrame" << '\n';
    mfile << "    iterString = sprintf('%06d',i);" << '\n';
    mfile << "    eval(['" << planner->getName() << "S" << worldSeed << "I' iterString]);" << '\n';
    mfile << "    print('-dpng', [iterString '.png']);" << '\n';
    mfile << "end" << '\n';
    mfile << "close;" << '\n';
    mfile << "warning('on', 'ASRL:Ellipses')" << '\n';
    mfile << '\n';

    mfile << "system('ffmpeg -r 30 -i %06d.png " << planner->getName() << "S" << worldSeed <<".mp4');" << '\n';
    mfile << "system('mkdir -p src');" << '\n';
    mfile << "system('mv *.m ./src/');" << '\n';
    mfile << "system('mkdir -p png');" << '\n';
    mfile << "system('mv *.png ./png/');" << '\n';
    mfile << "cd ../;" << '\n';
    mfile.flush();
    mfile.close();

    return runTime;
}
