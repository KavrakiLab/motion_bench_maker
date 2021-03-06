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

#include <ompl/tools/thunder/Thunder.h>
#include <ompl/tools/lightning/Lightning.h>

using namespace robowflex;

void prerunCallback(PlanningQuery &query)
{
    const auto planner = dynamic_cast<OMPL::OMPLInterfacePlanner *>(query.planner.get());
    if (planner)
    {
        // Get the name of the planner.
        const auto &planner_name = query.request.planner_id;
        if (planner_name == "thunder")
        {
            const auto &ss = dynamic_cast<ompl::tools::Thunder *>(planner->getLastSimpleSetup().get());
            if (ss)
            {
                ss->setFilePath("thunder.db");
                ss->setup();
            }
            else
                ROS_ERROR("Failed to typecast Thunder simple Setup!");
        }
        else if (planner_name == "lightning")
        {
            const auto &ss = dynamic_cast<ompl::tools::Lightning *>(planner->getLastSimpleSetup().get());
            if (ss)
            {
                ss->setFilePath("lightning.db");
                ss->setup();
            }
            else
                ROS_ERROR("Failed to typecast Thunder simple Setup!");
        }
        else
        {
            ROS_ERROR("Underlying planners is not lightning or thunder");
        }
    }
}

int main(int argc, char **argv)
{
    // Enables multiple instances running with the same name
    ros::init(argc, argv, "benchmark");
    ros::NodeHandle node("~");

    std::string dataset, results, exp_name;
    std::vector<std::string> planners;
    double time, runs;
    bool sensed;

    int start, end;

    std::string exec_name = "benchmark";

    // Load ROS params
    size_t error = 0;
    // Parsing the parametes from the param server
    error += !parser::get(exec_name, node, "dataset", dataset);
    error += !parser::get(exec_name, node, "results", results);
    error += !parser::get(exec_name, node, "exp_name", exp_name);
    error += !parser::get(exec_name, node, "time", time);
    error += !parser::get(exec_name, node, "planners", planners);
    error += !parser::get(exec_name, node, "start", start);
    error += !parser::get(exec_name, node, "end", end);
    error += !parser::get(exec_name, node, "runs", runs);
    error += !parser::get(exec_name, node, "sensed", sensed);

    parser::shutdownIfError(exec_name, error);

    auto setup = std::make_shared<Setup>(dataset);
    auto robot = setup->getRobot();

    // Benchmarker
    Profiler::Options options;
    options.metrics = Profiler::WAYPOINTS | Profiler::CORRECT | Profiler::LENGTH | Profiler::SMOOTHNESS;

    Experiment experiment(exp_name, options, time, runs);
    auto settings = OMPL::Settings();
    settings.simplify_solutions = false;
    settings.interpolate_solutions = false;

    auto planner = setup->createPlanner("planner_name", settings);

    int numSamples = (end > 1) ? end : setup->getNumSamples();
    for (int i = start; i <= numSamples; i++)
    {
        const auto &scene = std::make_shared<Scene>(robot);
        if (sensed)
            setup->loadSensedScene(i, scene);
        else
            setup->loadGeometricScene(i, scene);

        for (const auto &planner_name : planners)
        {
            const auto &request = std::make_shared<MotionRequestBuilder>(planner, setup->getGroup());
            setup->loadRequest(i, request);
            request->setAllowedPlanningTime(time);
            request->setNumPlanningAttempts(1);

            // Set the name of the planner
            if (!request->setConfig(planner_name))
                ROS_ERROR("Did not find planner %s", planner_name.c_str());

            experiment.setPreRunCallback(prerunCallback)
                experiment.addQuery(planner_name, scene, planner, request);
        }
    }

    auto results_data = experiment.benchmark(1);

    OMPLPlanDataSetOutputter output(IO::resolveParent(results));
    output.dump(*results_data);

    return 0;
}
