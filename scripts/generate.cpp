/* Author: Carlos Quintero, Constantinos Chamzas */

// Robowflex dataset
#include <motion_bench_maker/octomap_generator.h>
#include <motion_bench_maker/parser.h>
#include <motion_bench_maker/problem_generator.h>
#include <motion_bench_maker/scene_sampler.h>
#include <motion_bench_maker/setup.h>

// Robowflex library
#include <robowflex_library/io.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>
#include <robowflex_library/yaml.h>

// Robowflex ompl
#include <robowflex_ompl/ompl_interface.h>

using namespace robowflex;
int main(int argc, char **argv)
{
    ROS ros(argc, argv, "generate");
    ros::NodeHandle node("~");

    std::string dataset, config, planner_name;
    bool solve, visualize, sensed, pointcloud, visualize_sensed, tuning_mode;

    int start, end;

    std::string exec_name = "generate";

    // Load ROS params
    size_t error = 0;
    // Parsing the parametes from the param server
    error += !parser::get(exec_name, node, "dataset", dataset);
    error += !parser::get(exec_name, node, "config", config);
    error += !parser::get(exec_name, node, "sensed", sensed);
    error += !parser::get(exec_name, node, "planner_name", planner_name);
    error += !parser::get(exec_name, node, "pointcloud", pointcloud);
    error += !parser::get(exec_name, node, "visualize", visualize);
    error += !parser::get(exec_name, node, "visualize_sensed", visualize_sensed);
    error += !parser::get(exec_name, node, "tuning_mode", tuning_mode);
    error += !parser::get(exec_name, node, "start", start);
    error += !parser::get(exec_name, node, "end", end);
    parser::shutdownIfError(exec_name, error);

    auto setup = std::make_shared<Setup>(config, dataset);
    auto robot = setup->getRobot();
    auto group = setup->getGroup();

    // Disable hybridization
    auto settings = OMPL::Settings();
    settings.hybridize_solutions = false;
    settings.interpolate_solutions = false;

    // Create planner
    auto planner = setup->createPlanner("planner", settings);
    auto gp = setup->getGenParameters();

    // Nominal scene.
    auto nominal_yaml = std::make_shared<Scene>(robot);
    auto nominal_urdf = std::make_shared<Robot>("scene_robot");

    if (setup->isSceneYaml())
        setup->loadSceneYaml(nominal_yaml);
    else
        setup->loadSceneUrdf(nominal_urdf);

    // Create problem generator.
    auto problem_generator = std::make_shared<ProblemGenerator>(gp->queries);

    // If single tip
    if (gp->tips.empty())
        problem_generator->setParameters(robot, group, gp->ee_offset[0]);
    else
        problem_generator->setParameters(robot, group, gp->ee_offset, gp->tips, gp->ee_dependency);

    // Create an octomap generator.
    OctomapGeneratorPtr octomap_generator = nullptr;
    if (sensed)
        octomap_generator = std::make_shared<OctomapGenerator>(gp->sensors);

    // Create scene_sampler
    auto scene_sampler = std::make_shared<SceneSampler>(gp->variation, gp->base_offset);

    // Create default start_state
    auto start_state = std::make_shared<robot_state::RobotState>(*robot->getScratchStateConst());

    // Create RVIZ helper.
    auto rviz = std::make_shared<IO::RVIZHelper>(setup->getRobot());

    int index = start;
    int numSamples = (end > 1) ? end : setup->getNumSamples();
    while (index <= numSamples)
    {
        ROS_INFO("Attempting [%d/%d] ....", index, numSamples);

        auto scene_geom = std::make_shared<Scene>(robot);

        // check if the scene is of yaml format, and load it appropriately.
        if (setup->isSceneYaml())
            scene_geom = scene_sampler->sample(nominal_yaml);
        else
            scene_geom = scene_sampler->sample(nominal_urdf, robot);

        auto scene = scene_geom->deepCopy();
        if (sensed and octomap_generator)
            octomap_generator->geomToSensed(scene_geom, scene, visualize_sensed ? rviz : nullptr);

        // Update the scene in the problem_generator
        problem_generator->updateScene(scene);

        // If a start query is not provided use the default start_state.
        auto result = problem_generator->getNumberOfStartObjectQueries() == 0 ?
                          problem_generator->createRandomRequestWithStartState(start_state) :
                          problem_generator->createRandomRequest();

        if (result.second)
        {
            // Try to solve with a planner to verify feasibility
            const auto &request = result.first;
            // Add the planner so we know that the correct config exists
            request->setPlanner(planner);
            request->setAllowedPlanningTime(60);
            request->setNumPlanningAttempts(2);

            if (!request->setConfig(planner_name))
                ROS_ERROR("Did not find planner %s", planner_name.c_str());

            const auto &res = planner->plan(scene, request->getRequestConst());

            if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
            {
                auto trajectory = std::make_shared<Trajectory>(*res.trajectory_);

                if (visualize)
                {
                    rviz->updateScene(scene);
                    // Visualize start state.
                    rviz->visualizeState(request->getStartConfiguration());
                    parser::waitForUser("Displaying initial state!");

                    // Visualize goal state.
                    rviz->visualizeState(request->getGoalConfiguration());
                    parser::waitForUser("Displaying goal state!");

                    // Visualize the trajectory.
                    rviz->updateTrajectory(trajectory->getTrajectory());
                    parser::waitForUser("Displaying The trajectory!");
                }

                // save request, scene,  trajectory
                setup->saveRequest(index, request);
                setup->saveTrajectory(index, trajectory);
                setup->saveGeometricScene(index, scene_geom);
                // This saveas attached correctly but does not work with the octomap
                // setup->saveGeometricScene(index, scene);

                // Remove all the geometric objects from the scene.
                if (sensed)
                {
                    for (const auto &obj : scene->getCollisionObjects())
                        if (obj != "<octomap>")
                            scene->removeCollisionObject(obj);
                    setup->saveSensedScene(index, scene);
                    // Save the pointcloud as well
                    if (pointcloud)
                        setup->savePCDScene(index, octomap_generator->getLastPointCloud());
                }

                index++;
            }
        }
        if (tuning_mode)
        {
            rviz->updateScene(scene);
            rviz->visualizeState(robot->getScratchStateConst());
            rviz->removeAllMarkers();
            for (int i = 0; i < problem_generator->getLastQueryPose().size(); i++)
                rviz->addTransformMarker("start" + std::to_string(i), "map",
                                         problem_generator->getLastQueryPose()[i]);
            rviz->updateMarkers();
            parser::waitForUser("Displaying the last queried IK pose !");
        }
    }

    setup->saveConfigToDataset();
    return 0;
}
