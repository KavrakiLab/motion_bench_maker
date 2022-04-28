/* Author: Constantinos Chamzas */

// Robowflex dataset
#include <motion_bench_maker/setup.h>
#include <motion_bench_maker/parser.h>

// Robowflex library
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/util.h>
#include <robowflex_library/io.h>
#include <robowflex_library/yaml.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/detail/fetch.h>

// Robowflex ompl
#include <robowflex_ompl/ompl_interface.h>

// Boost
#include <boost/filesystem.hpp>  // for filesystem paths

using namespace robowflex;
namespace
{
    const std::string SENSED_POSTFIX_{"_sensed"};  //<<< sensed scene postfix.
    const std::string PCD_POSTFIX_{"_pcd"};        //<<< pointcloud scene postfix.

    // is lhs a suffix of rhs?
    bool isSuffix(const std::string &lhs, const std::string &rhs)
    {
        return std::equal(lhs.rbegin(), lhs.rbegin() + std::min(lhs.size(), rhs.size()), rhs.rbegin());
    }

}  // namespace

// Constructor
Setup::Setup(const std::string &config, const std::string &dataset)
  : dataset_(dataset), config_(config), robot_(std::make_shared<Robot>("robot"))
{
    auto yaml = IO::loadFileToYAML(config);
    if (!yaml.first)
    {
        ROS_ERROR("Failed to load YAML file `%s`.", config.c_str());
        return;
    }

    loadGenerateParams(yaml.second);

    loadMainParams(yaml.second);

    dwidth_ = int(log10(getNumSamples())) > 3 ? int(log10(getNumSamples())) + 1 : 4;

    robot_->initializeFromYAML(mparams_->robot_description);
    robot_->loadKinematics(mparams_->planning_group);
}

Setup::Setup(const std::string &dataset) : Setup(dataset + "/config.yaml", dataset)
{
}

void Setup::loadGenerateParams(const YAML::Node &node)
{
    gparams_ = std::make_shared<GenParameters>();

    if (IO::isNode(node["scene"]))
        gparams_->scene = node["scene"].as<std::string>();
    else
        throw Exception(1, "No scene entry!");

    if (not isSceneUrdf() and not isSceneYaml())
        throw Exception(1, "Scene suffix must be either .yaml or .urdf!");

    if (IO::isNode(node["base_offset"]))
        gparams_->base_offset = TF::poseMsgToEigen(IO::poseFromNode(node["base_offset"]));
    else
        throw Exception(1, "No base_offset transform specified!");

    if (node["ee_offset"].IsSequence())
    {
        for (const auto &ee_node : node["ee_offset"])
        {
            gparams_->ee_offset.emplace_back(TF::poseMsgToEigen(IO::poseFromNode(ee_node)));
            if (IO::isNode(ee_node["ee_tip"]))
                gparams_->tips.emplace_back(ee_node["ee_tip"].as<std::string>());
            else
                throw Exception(1, "ee_tip not specified while while ee_offset is a list");
        }
        if (IO::isNode(node["ee_dependency"]))
            gparams_->ee_dependency = node["ee_dependency"].as<bool>();
        else
            throw Exception(1, "ee_dependency not specified while ee_offset is a list");
    }
    else if (IO::isNode(node["ee_offset"]))
        gparams_->ee_offset.emplace_back(TF::poseMsgToEigen(IO::poseFromNode(node["ee_offset"])));
    else
        throw Exception(1, "ee_offset not specified or is not a sequence!");

    if (IO::isNode(node["queries"]))
        gparams_->queries = node["queries"].as<std::string>();
    else
        throw Exception(1, "No queries entry!");

    if (IO::isNode(node["variation"]))
        gparams_->variation = node["variation"].as<std::string>();
    else
        throw Exception(1, "No variation entry!");

    if (IO::isNode(node["sensors"]))
        gparams_->sensors = node["sensors"].as<std::string>();
    else
        throw Exception(1, "No sensors entry!");
}

void Setup::loadMainParams(const YAML::Node &node)

{
    mparams_ = std::make_shared<MainParameters>();

    if (IO::isNode(node["robot_description"]))
        mparams_->robot_description = node["robot_description"].as<std::string>();
    else
        throw Exception(1, "No robot_description entry!");

    if (IO::isNode(node["ompl_config"]))
        mparams_->ompl_config = node["ompl_config"].as<std::string>();
    else
        throw Exception(1, "No ompl_config entry!");

    if (IO::isNode(node["planning_group"]))
        mparams_->planning_group = node["planning_group"].as<std::string>();
    else
        throw Exception(1, "No planning group entry!");

    if (IO::isNode(node["samples"]))
        mparams_->samples = node["samples"].as<int>();
    else
        throw Exception(1, "No samples entry!");
}

bool Setup::isSceneYaml()
{
    return isSuffix(".yaml", gparams_->scene);
}

bool Setup::isSceneUrdf()
{
    return isSuffix(".urdf", gparams_->scene);
}

const RobotPtr Setup::getRobot() const
{
    return robot_;
}

const std::string Setup::getGroup() const
{
    return mparams_->planning_group;
}
int Setup::getNumSamples() const
{
    return mparams_->samples;
}
const Setup::MainParametersConstPtr Setup::getMainParameters() const
{
    return mparams_;
}
const Setup::GenParametersConstPtr Setup::getGenParameters() const
{
    return gparams_;
}

bool Setup::loadSceneYaml(const ScenePtr &scene) const
{
    return scene->fromYAMLFile(gparams_->scene);
}

bool Setup::loadSceneUrdf(const RobotPtr &robot) const
{
    return robot->initialize(gparams_->scene);
}

OMPL::OMPLInterfacePlannerPtr Setup::createPlanner(const std::string &name,
                                                   const OMPL::Settings &settings) const
{
    auto planner = std::make_shared<robowflex::OMPL::OMPLInterfacePlanner>(robot_, name);
    planner->initialize(mparams_->ompl_config, settings);
    return planner;
}

MotionRequestBuilderPtr Setup::createRequest() const
{
    return std::make_shared<MotionRequestBuilder>(robot_);
}

bool Setup::loadGeometricScene(const int &index, const ScenePtr &scene) const
{
    auto scene_file = dataset_ + "/scene" + parser::toString(index, dwidth_) + ".yaml";
    if (!scene->fromYAMLFile(scene_file))
    {
        ROS_ERROR("Failed to read file: %s for scene", scene_file.c_str());
        return false;
    }
    return true;
}

bool Setup::loadSensedScene(const int &index, const ScenePtr &scene) const
{
    auto scene_file = dataset_ + "/scene" + SENSED_POSTFIX_ + parser::toString(index, dwidth_) + ".yaml";
    if (!scene->fromYAMLFile(scene_file))
    {
        ROS_ERROR("Failed to read file: %s for scene", scene_file.c_str());
        return false;
    }
    return true;
}

void Setup::loadPCDScene(const int &index, const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &pcd) const
{
    auto scene_file =
        IO::resolvePackage(dataset_ + "/scene" + PCD_POSTFIX_ + parser::toString(index, dwidth_) + ".pcd");

    int code = pcl::io::loadPCDFile(scene_file, *pcd);
    if (code)
        ROS_ERROR("Failed to read file: %s for pointcloud scene, error code: %d", scene_file.c_str(), code);
}

bool Setup::loadTrajectory(const int &index, const robot_state::RobotState &ref_state,
                           const TrajectoryPtr &traj) const
{
    auto traj_file = dataset_ + "/path" + parser::toString(index, dwidth_) + ".yaml";
    if (!traj->fromYAMLFile(ref_state, traj_file))
    {
        ROS_ERROR("Failed to read file: %s for trajectory", traj_file.c_str());
        return false;
    }
    return true;
}

bool Setup::loadRequest(const int &index, const MotionRequestBuilderPtr &request) const
{
    auto request_file = dataset_ + "/request" + parser::toString(index, dwidth_) + ".yaml";
    if (!request->fromYAMLFile(request_file))
    {
        ROS_ERROR("Failed to read file: %s for request", request_file.c_str());
        return false;
    }
    return true;
}
void Setup::saveGeometricScene(const int &index, const SceneConstPtr &scene) const
{
    auto scene_file = IO::resolvePackage(dataset_ + "/scene" + parser::toString(index, dwidth_) + ".yaml");
    // if scene has octomap remove it here.
    for (const auto &obj : scene->getCollisionObjects())
        if (obj == "<octomap>")
            ROS_WARN("Geometric Scene %u has an octomap!", index);

    if (!scene->toYAMLFile(scene_file))
        ROS_ERROR("Failed to save file: %s for scene", scene_file.c_str());
}

void Setup::saveSensedScene(const int &index, const SceneConstPtr &scene) const
{
    auto scene_file = IO::resolvePackage(dataset_ + "/scene" + SENSED_POSTFIX_ +
                                         parser::toString(index, dwidth_) + ".yaml");

    for (const auto &obj : scene->getCollisionObjects())
        if (obj != "<octomap>")
            ROS_WARN("Sensed Scene %u has more Objects than just an octomap %s", index, obj.c_str());

    if (!scene->toYAMLFile(scene_file))
        ROS_ERROR("Failed to save file: %s for scene", scene_file.c_str());
}

void Setup::savePCDScene(const int &index, const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &pcd) const
{
    auto scene_file =
        IO::resolvePackage(dataset_ + "/scene" + PCD_POSTFIX_ + parser::toString(index, dwidth_) + ".pcd");

    int code = pcl::io::savePCDFileBinaryCompressed(scene_file, *pcd);
    if (code)
        ROS_ERROR("Failed to save file: %s for pointcloud scene, error code: %d", scene_file.c_str(), code);
}

void Setup::saveTrajectory(const int &index, const TrajectoryConstPtr &traj) const
{
    auto traj_file = IO::resolvePackage(dataset_ + "/path" + parser::toString(index, dwidth_) + ".yaml");

    if (!traj->toYAMLFile(traj_file))
        ROS_ERROR("Failed to save file: %s for trajectory", traj_file.c_str());
}

void Setup::saveRequest(const int &index, const MotionRequestBuilderConstPtr &request) const
{
    auto request_file =
        IO::resolvePackage(dataset_ + "/request" + parser::toString(index, dwidth_) + ".yaml");
    if (!request->toYAMLFile(request_file))
        ROS_ERROR("Failed to save file: %s for request", request_file.c_str());
}

void Setup::saveConfigToDataset() const
{
    // TODO move to IO?
    boost::filesystem::copy_file(IO::resolvePackage(config_), IO::resolvePackage(dataset_ + "/config.yaml"),
                                 boost::filesystem::copy_option::overwrite_if_exists);
}
