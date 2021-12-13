/* Author: Constantinos Chamzas */

// Robowflex dataset
#include <motion_bench_maker/scene_sampler.h>
#include <motion_bench_maker/yaml.h>

// Robowflex library
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/io.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/robot.h>

#include <algorithm>

using namespace robowflex;

RobotPose SceneSampler::Variation::sample() const
{
    if (type.compare("uniform"))
        return TF::samplePoseUniform(position, orientation);
    else if (type.compare("gaussian"))
        return TF::samplePoseGaussian(position, orientation);
    else
        ROS_ERROR("%s does not exist, choose from [uniform, gaussian]", type.c_str());

    return RobotPose::Identity();
};

SceneSampler::SceneSampler(const std::string &config, const RobotPose &offset) : offset_(offset)
{
    std::vector<Variation> vars;
    if (!IO::loadVariations(config, vars))
        ROS_ERROR("Reading Scene Sampler config failed!");

    // Split in world/object variations
    for (const auto &v : vars)
        if (std::find(v.names.begin(), v.names.end(), "World") != v.names.end())
            world_variation_ = std::make_shared<Variation>(v);
        else
            object_variations_.emplace_back(std::make_shared<Variation>(v));
}

SceneSampler::SceneSampler(const Variation &var, const RobotPose &offset) : offset_(offset)
{
    if (std::find(var.names.begin(), var.names.end(), "World") != var.names.end())
        world_variation_ = std::make_shared<Variation>(var);
    else
        object_variations_.emplace_back(std::make_shared<Variation>(var));
}

ScenePtr SceneSampler::sample(const SceneConstPtr &nominal) const
{
    // copy the nominal scene
    const auto &scene = nominal->deepCopy();

    // apply scene offset
    scene->moveAllObjectsGlobal(offset_);

    // apply world Variation
    if (world_variation_)
    {
        const auto &tf = world_variation_->sample();
        scene->moveAllObjectsGlobal(tf);
    }

    // Apply object variations
    for (const auto &object_var : object_variations_)
        for (const auto &name : object_var->names)
        {
            if (not scene->hasObject(name))
            {
                ROS_WARN("Object %s does not exist in the nominal scene !", name.c_str());
                continue;
            }

            const auto &tf = object_var->sample();
            scene->moveObjectLocal(name, tf);
        }

    return scene;
}

ScenePtr SceneSampler::sample(const RobotPtr &nominal, const RobotConstPtr &robot) const
{
    const auto &scene = std::make_shared<Scene>(robot);
    nominal->getScratchState()->setToRandomPositions();

    std::ofstream fout;
    // create a temporary file to store the scene
    const auto &temp_file = IO::createTempFile(fout);
    nominal->dumpToScene(temp_file);
    scene->fromYAMLFile(temp_file);

    // delete the temporary file.
    IO::deleteFile(temp_file);

    // apply scene offset
    scene->moveAllObjectsGlobal(offset_);

    // apply world Variation
    if (world_variation_)
    {
        const auto &tf = world_variation_->sample();
        scene->moveAllObjectsGlobal(tf);
    }

    return scene;
}
