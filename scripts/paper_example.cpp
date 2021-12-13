/* Author: Constantinos Chamzas */

// ROS
#include <moveit_msgs/MoveItErrorCodes.h>
#include <ros/ros.h>

// Robowflex dataset
#include <motion_bench_maker/setup.h>
#include <motion_bench_maker/parser.h>

// Robowflex library
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/io.h>

// Robowflex ompl
#include <robowflex_ompl/ompl_interface.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    // Enables multiple instances running with the same name
    ros::init(argc, argv, "benchmark");
    ros::NodeHandle node("~");

    // Benchmarker
    Profiler::Options options;
    options.metrics = Profiler::WAYPOINTS | Profiler::CORRECT | Profiler::LENGTH | Profiler::SMOOTHNESS;

    // Load the dataset given a meta-data file
    auto setup = std::make_shared<Setup>("package://motion_bench_maker/problems/box_fetch/config.yaml");
    auto robot = setup->getRobot();
    auto planner = setup->createPlanner("planner");
    Experiment experiment("exp", Profiler::Options());

    for (int i = 1; i <= setup->getNumSamples(); i++)
    {
        // Load the ith scene in the dataset
        auto scene = std::make_shared<Scene>(robot);
        setup->loadGeometricScene(i, scene);

        for (auto planner_name : {"PRM", "BiEST"})
        {
            // Load the start and goal configuration
            auto request = setup->createRequest();
            setup->loadRequest(i, request);

            // Set planner.e.g., PRM, BiEST
            request->setConfig(planner_name);
            experiment.addQuery(  //
                planner_name, scene, planner, request);
        }
    }

    auto data = experiment.benchmark();
    OMPLPlanDataSetOutputter output("results.log");
    output.dump(*data);

    return 0;
}
