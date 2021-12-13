#ifndef ROBOWFLEX_DATASET_SCENE_SAMPLER_
#define ROBOWFLEX_DATASET_SCENE_SAMPLER_

// C++
#include <string.h>

// Eigen
#include <Eigen/Core>

// Robowflex
#include <robowflex_library/class_forward.h>
#include <robowflex_library/tf.h>

// Yaml
#include <yaml-cpp/yaml.h>

namespace robowflex
{
    ROBOWFLEX_CLASS_FORWARD(Scene);
    ROBOWFLEX_CLASS_FORWARD(Robot);
    ROBOWFLEX_CLASS_FORWARD(SceneSampler);
    class SceneSampler
    {
    public:
        ROBOWFLEX_CLASS_FORWARD(Variation);
        struct Variation
        {
            std::vector<std::string> names;
            Eigen::Vector3d position;
            Eigen::Vector3d orientation;
            std::string type;

            RobotPose sample() const;
        };

        // Constructor
        SceneSampler(const std::string &config, const RobotPose &offset = RobotPose::Identity());
        SceneSampler(const Variation &var, const RobotPose &offset = RobotPose::Identity());
        ScenePtr sample(const SceneConstPtr &nominal) const;
        ScenePtr sample(const RobotPtr &nominal, const RobotConstPtr &robot) const;

    private:
        // RobowFlex
        std::vector<VariationPtr> object_variations_;
        VariationPtr world_variation_{nullptr};
        RobotPose offset_;
    };
}  // namespace robowflex

#endif
