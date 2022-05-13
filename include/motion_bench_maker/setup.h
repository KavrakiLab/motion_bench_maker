#ifndef ROBOWFLEX_DATASET_SETUP_
#define ROBOWFLEX_DATASET_SETUP_

// C++
#include <string.h>

// Robowflex
#include <robowflex_library/class_forward.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/planning.h>
#include <moveit/robot_state/robot_state.h>
#include <pcl_ros/point_cloud.h>

// Yaml
#include <yaml-cpp/yaml.h>

namespace robowflex
{
    ROBOWFLEX_CLASS_FORWARD(Scene);
    ROBOWFLEX_CLASS_FORWARD(MotionRequestBuilder);
    ROBOWFLEX_CLASS_FORWARD(Trajectory);

    ROBOWFLEX_CLASS_FORWARD(Robot);
    namespace OMPL
    {
        ROBOWFLEX_CLASS_FORWARD(OMPLInterfacePlanner);
    }

    ROBOWFLEX_CLASS_FORWARD(Setup);
    class Setup
    {
    public:
        ROBOWFLEX_CLASS_FORWARD(GenParameters);
        ROBOWFLEX_CLASS_FORWARD(MainParameters);
        struct MainParameters
        {
            std::string robot_description;
            std::string planning_group;
            std::string ompl_config;
            int samples;
        };

        struct GenParameters
        {
            std::string scene;
            std::string queries;
            std::string variation;
            std::string sensors;
            RobotPose base_offset;
            RobotPoseVector ee_offset;
            std::vector<std::string> tips;  //<< If not specified default tip will be used.
            bool ee_dependency;             //<< Determines if ees are dependent.
        };

        // Constructor

        Setup(const std::string &config, const std::string &dataset);
        // Convenience constructor for just loading data
        Setup(const std::string &dataset);

        void loadMainParams(const YAML::Node &node);
        void loadGenerateParams(const YAML::Node &node);

        bool isSceneYaml();
        bool isSceneUrdf();

        const RobotPtr getRobot() const;
        const std::string getGroup() const;

        int getNumSamples() const;
        const MainParametersConstPtr getMainParameters() const;
        const GenParametersConstPtr getGenParameters() const;
        OMPL::OMPLInterfacePlannerPtr createPlanner(const std::string &name,
                                                    const OMPL::Settings &settings = OMPL::Settings()) const;
        MotionRequestBuilderPtr createRequest() const;

        // Load functions from yaml files
        bool loadSceneYaml(const ScenePtr &scene) const;
        bool loadSceneUrdf(const RobotPtr &robot) const;
        void loadPCDScene(const int &index, const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &pcd) const;
        bool loadGeometricScene(const int &index, const ScenePtr &scene) const;
        bool loadSensedScene(const int &index, const ScenePtr &scene) const;
        bool loadRequest(const int &index, const MotionRequestBuilderPtr &request) const;
        bool loadTrajectory(const int &index, const robot_state::RobotState &ref_state,
                            const TrajectoryPtr &traj) const;

        // Store functions from yaml files
        void saveGeometricScene(const int &index, const SceneConstPtr &scene) const;
        void saveSensedScene(const int &index, const SceneConstPtr &scene) const;
        void savePCDScene(const int &index, const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &pcd) const;
        void saveRequest(const int &index, const MotionRequestBuilderConstPtr &request) const;
        void saveTrajectory(const int &index, const TrajectoryConstPtr &traj) const;
        void saveConfigToDataset() const;

    protected:
        RobotPtr robot_;
        MainParametersPtr mparams_;  //<<< main parameters used to load/visualize dataset.
        int dwidth_;                 //<<< the width of the string needed to store data.
        GenParametersPtr gparams_;   //<<< generation parameters used to generate the dataset.
        std::string dataset_{""};
        std::string config_{""};
    };  // namespace robowflex
}  // namespace robowflex

#endif
