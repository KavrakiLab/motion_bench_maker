/* Author: Constantinos Chamzas */

// ROS
#include <moveit_msgs/MoveItErrorCodes.h>
#include <ros/ros.h>

// Robowflex dataset
#include <motion_bench_maker/setup.h>
#include <motion_bench_maker/parser.h>

// Robowflex library
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>

// Robowflex ompl
#include <robowflex_ompl/ompl_interface.h>

using namespace robowflex;
int main(int argc, char **argv)
{
    // Enables multiple instances running with the same name
    ros::init(argc, argv, "visualize");
    ros::NodeHandle node("~");

    std::string dataset, config;
    bool geometric, sensed, solve, pcd;

    std::string exec_name = "visualize";

    // Load ROS params
    size_t error = 0;
    // Parsing the parametes from the param server
    error += !parser::get(exec_name, node, "dataset", dataset);
    error += !parser::get(exec_name, node, "geometric", geometric);
    error += !parser::get(exec_name, node, "sensed", sensed);
    error += !parser::get(exec_name, node, "solve", solve);
    error += !parser::get(exec_name, node, "pcd", pcd);
    parser::shutdownIfError(exec_name, error);

    auto setup = std::make_shared<Setup>(dataset);
    auto robot = setup->getRobot();
    auto settings = OMPL::Settings();
    settings.hybridize_solutions = false;
    settings.interpolate_solutions = true;

    auto planner = setup->createPlanner("planner", settings);

    auto rviz = std::make_shared<IO::RVIZHelper>(robot);

    for (int i = 1; i <= setup->getNumSamples(); i++)
    {
        auto scene_geom = std::make_shared<Scene>(robot);
        auto scene_sensed = std::make_shared<Scene>(robot);
        auto scene_pcd = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        if (geometric)
        {
            setup->loadGeometricScene(i, scene_geom);
            rviz->updateScene(scene_geom);
        }
        if (sensed)
        {
            setup->loadSensedScene(i, scene_sensed);
            rviz->updateScene(scene_sensed);
        }
        // if (pcd)
        //{
        //    // setup->loadPCDScene(i, scene_pcd);
        //    // for (const auto &p : *scene_pcd)
        //    //    std::cout << p.x << p.y << p.z;

        //    //#TODO not implemented yet
        //    // rviz->updateScene(scene_pcd);
        //}

        auto request = std::make_shared<MotionRequestBuilder>(robot, setup->getGroup());
        setup->loadRequest(i, request);

        auto start = request->getStartConfiguration();
        rviz->visualizeState(start);

        parser::waitForUser("Displaying start:" + std::to_string(i));
        if (geometric)  // Trick to visualize start/goal together
        {
            scene_geom->getCurrentState() = *start;
            rviz->updateScene(scene_geom);
        }

        auto goal = request->getGoalConfiguration();
        rviz->visualizeState(goal);

        parser::waitForUser("Displaying Goal:" + std::to_string(i));

        auto trajectory = std::make_shared<Trajectory>(robot, setup->getGroup());
        if (solve)
        {
            const auto &res = planner->plan(scene_geom, request->getRequestConst());
            if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
                trajectory->getTrajectory() = res.trajectory_;
            else
                continue;
        }

        else
            setup->loadTrajectory(i, *start, trajectory);

        trajectory->interpolate(100);

        rviz->updateTrajectory(trajectory->getTrajectoryConst());

        parser::waitForUser("Displaying computed/loaded trajectory:" + std::to_string(i));
    }
    return 0;
}
