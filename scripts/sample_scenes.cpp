/* Author: Constantinos Chamzas */

// ROS
#include <moveit_msgs/MoveItErrorCodes.h>
#include <ros/ros.h>

// Robowflex dataset
#include <motion_bench_maker/parser.h>
#include <motion_bench_maker/scene_sampler.h>

// Robowflex library
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/robot.h>

// Robowflex ompl
using namespace robowflex;
int main(int argc, char **argv)
{
    // Enables multiple instances running with the same name
    ros::init(argc, argv, "sample_scenes");
    ros::NodeHandle node("~");

    std::string robot_file, scene_file, var_file;

    std::string exec_name = "sample_scenes";

    // Load ROS params
    size_t error = 0;
    // Parsing the parametes from the param server
    error += !parser::get(exec_name, node, "robot", robot_file);
    error += !parser::get(exec_name, node, "scene", scene_file);
    error += !parser::get(exec_name, node, "variation", var_file);
    parser::shutdownIfError(exec_name, error);

    auto robot = std::make_shared<Robot>("RoboCop");
    robot->initialize(robot_file);
    auto rviz = std::make_shared<IO::RVIZHelper>(robot);

    auto scene = std::make_shared<Scene>(robot);
    scene->fromYAMLFile(scene_file);

    auto scene_sampler = std::make_shared<SceneSampler>(var_file);

    while (true)
    {
        auto sampled_scene = scene_sampler->sample(scene);
        rviz->updateScene(sampled_scene);
        parser::waitForUser("Displaying sampled scene!");
    }

    return 0;
}
