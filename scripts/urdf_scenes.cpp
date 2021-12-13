/*Constantinos Chamzas */

#include <motion_bench_maker/parser.h>

// Robowflex library
#include <robowflex_library/util.h>
#include <robowflex_library/yaml.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/io.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/builder.h>

// Robowflex ompl
#include <robowflex_ompl/ompl_interface.h>

static const std::string GROUP = "arm_with_torso";

using namespace robowflex;
int main(int argc, char **argv)
{
    ROS ros(argc, argv, "urdf_scenes");
    ros::NodeHandle node("~");

    // Create scene
    auto scene_robot = std::make_shared<Robot>("robot");
    // scene_robot->initialize("package://fetch_description/robots/fetch.urdf");
    scene_robot->initialize("package://motion_bench_maker/configs/scenes/kitchen/kitchen.urdf");
    scene_robot->getScratchState()->setToRandomPositions();
    scene_robot->dumpToScene(IO::resolvePackage("package://motion_bench_maker/"
                                                "configs/scenes/kitchen/"
                                                "kitchen_urdf.yaml"));

    auto robot = std::make_shared<robowflex::FetchRobot>();
    robot->initialize();

    auto scene = std::make_shared<robowflex::Scene>(robot);
    // Create RVIZ helper.
    auto rviz = std::make_shared<IO::RVIZHelper>(robot);

    // Create the default planner for the Fetch.
    auto planner = std::make_shared<OMPL::FetchOMPLPipelinePlanner>(robot, "default");
    planner->initialize();

    // Create a motion planning request with a pose goal.
    auto request = std::make_shared<MotionRequestBuilder>(planner, GROUP);
    robot->setGroupState(GROUP, {0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0});  // Stow
    request->setStartConfiguration(robot->getScratchState());

    robot->setGroupState(GROUP, {0.15, 0, 0, 0, 0, 0, 0, 0});

    request->setGoalConfiguration(robot->getScratchState());

    request->setConfig("RRTConnect");

    int attempts = 0;
    while (1)
    {
        scene_robot->getScratchState()->setToRandomPositions();
        scene_robot->dumpToScene(IO::resolvePackage("package://motion_bench_maker/"
                                                    "configs/scenes/kitchen/"
                                                    "kitchen_urdf.yaml"));

        ROS_INFO("Visualizing scene %d ....", attempts);
        attempts += 1;
        scene->fromYAMLFile(IO::resolvePackage("package://motion_bench_maker/"
                                               "configs/scenes/kitchen/"
                                               "kitchen_urdf.yaml"));

        rviz->updateScene(scene);
        rviz->updateScene(scene);
        // Visualize start state.
        rviz->visualizeState(request->getStartConfiguration());
        parser::waitForUser("Displaying initial state!");

        // Visualize goal state.
        rviz->visualizeState(request->getGoalConfiguration());
        parser::waitForUser("Displaying goal state!");

        auto res = planner->plan(scene, request->getRequestConst());
        auto trajectory = std::make_shared<Trajectory>(*res.trajectory_);

        if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            // Visualize the trajectory.
            rviz->updateTrajectory(trajectory->getTrajectory());
            parser::waitForUser("Displaying The trajectory!");
        }
    }
}
