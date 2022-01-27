/* Author: Constantinos Chamzas  */

// Robowflex
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/io.h>
#include <robowflex_library/util.h>
#include <robowflex_library/tf.h>

#include <motion_bench_maker/scene_sampler.h>
#include <motion_bench_maker/yaml.h>

// ROS
#include <ros/console.h>

// C++
#include <fstream>

using namespace robowflex;

namespace
{
    static const std::string boolToString(bool b)
    {
        return b ? "true" : "false";
    }
    static bool nodeToBool(const YAML::Node &n)
    {
        std::string s = n.as<std::string>();
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);
        return s == "true";
    }

    Eigen::VectorXd YAMLToEigen(const YAML::Node &node)
    {
        if (!node.IsSequence())
            ROS_ERROR("Node is not a sequence");

        Eigen::VectorXd x(node.size());
        for (unsigned int i = 0; i < node.size(); ++i)
            x[i] = node[i].as<double>();

        return x;
    }
    std::vector<double> YAMLToVector(const YAML::Node &node)
    {
        if (!node.IsSequence())
            ROS_ERROR("Node is not a sequence");

        std::vector<double> x;
        for (unsigned int i = 0; i < node.size(); ++i)
            x.push_back(node[i].as<double>());

        return x;
    }

    robowflex::RobotPose YAMLToPose(const YAML::Node &node)
    {
        Eigen::Vector3d position{0., 0., 0.};
        if (IO::isNode(node["position"]))
        {
            if (node["position"].size() != 3)
                ROS_ERROR(" \"position\" must be three-dimensional");

            position = YAMLToEigen(node["position"]);
        }
        else
            ROS_ERROR("No \"position\" entry in pose in YAML");

        Eigen::Vector4d ornt{0., 0., 0., 0.};
        if (IO::isNode(node["orientation"]))
        {
            if (node["orientation"].size() != 4)
                ROS_ERROR("orientation must be four-dimensional");

            ornt = YAMLToEigen(node["orientation"]);
        }
        else
            ROS_ERROR("No \"orientation\"in pose in YAML");

        robowflex::RobotPose pose;
        pose.translation() = position;
        pose.linear() = Eigen::Quaterniond(ornt).toRotationMatrix();
        return pose;
    }

    // TODO convert to encode/decode?.
    SceneSampler::Variation YAMLToVariation(const YAML::Node &node)
    {
        auto var = SceneSampler::Variation();

        if (IO::isNode(node["names"]))
            var.names = node["names"].as<std::vector<std::string>>();
        else
            ROS_ERROR("No \"names\"in YAML");

        if (IO::isNode(node["type"]))
            var.type = node["type"].as<std::string>();
        else
            ROS_ERROR("No \"type\"in YAML");

        if (IO::isNode(node["position"]))
        {
            if (node["position"].size() != 3)
                ROS_ERROR(" \"position\" must be three-dimensional");

            var.position = YAMLToEigen(node["position"]);
        }
        else
            ROS_ERROR("No \"position\"in YAML");

        if (IO::isNode(node["orientation"]))
        {
            if (node["orientation"].size() != 3)
                ROS_ERROR(" \"orientation\" must be three-dimensional");

            var.orientation = YAMLToEigen(node["orientation"]);
        }
        else
            ROS_ERROR("No \"orientation\"in YAML");

        return var;
    }

    std::vector<ProblemGenerator::ObjectQuery> YAMLToQueries(const YAML::Node &node)
    {
        std::vector<ProblemGenerator::ObjectQuery> queries;
        ProblemGenerator::ObjectQuery query;
        if (node.size() != 4)
            ROS_ERROR("Wrong number of parameters (%lu) in queries file. Expecting 4.", node.size());

        if (IO::isNode(node["tag"]))
            query.tag = node["tag"].as<std::string>();
        else
            ROS_ERROR("No \" tag \"in queries YAML file.");

        if (IO::isNode(node["attached"]))
            query.attached = nodeToBool(node["attached"]);
        else
            ROS_ERROR("No \" attached \"in queries YAML file.");

        if (IO::isNode(node["offset"]))
        {
            if (node["offset"].size() == 4)
            {
                const auto &off = node["offset"];
                query.offset_pose = YAMLToPose(off);
                query.pos_tol = YAMLToEigen(off["position_tol"]);
                query.ornt_tol = YAMLToEigen(off["orientation_tol"]);
            }
            else
                ROS_ERROR("Wrong number of parameters (%lu) in queries - offset. Expecting 4.",
                          node["offset"].size());
        }
        else
            ROS_ERROR("No \" offset \"in queries YAML file.");

        if (IO::isNode(node["objects"]))
        {
            auto objects = node["objects"].as<std::vector<std::string>>();
            if (objects.empty())
                ROS_ERROR("List of objects is empty in queries YAML file.");

            for (const auto &obj : objects)
            {
                // add the object to the query
                query.object = obj;
                // Copy the query and add it in the queries
                queries.emplace_back(query);
            }
        }
        else
            ROS_ERROR("No \"objects \" in queries YAML file.");

        return queries;
    }

    YAML::Node EigenToYAML(const Eigen::Ref<const Eigen::VectorXd> &vec)
    {
        YAML::Node node;
        for (unsigned int i = 0; i < vec.size(); ++i)
            node.push_back(vec[i]);

        return node;
    }

    YAML::Node QuaternionToYAML(const Eigen::Quaternionf &q)
    {
        YAML::Node node;
        node.push_back(q.x());
        node.push_back(q.y());
        node.push_back(q.z());
        node.push_back(q.w());

        return node;
    }

    YAML::Node VectorToYAML(const std::vector<double> &vec)
    {
        YAML::Node node;
        for (unsigned int i = 0; i < vec.size(); ++i)
            node.push_back(vec[i]);

        return node;
    }

    const YAML::Node PoseToYAML(const robowflex::RobotPose &pose)
    {
        YAML::Node node;
        node["position"] = EigenToYAML(pose.translation());
        node["orientation"] = QuaternionToYAML(Eigen::Quaternionf(pose.linear().cast<float>()));
        return node;
    }

}  // namespace

bool IO::loadVariations(const std::string &config, std::vector<SceneSampler::Variation> &vars)
{
    auto result = IO::loadFileToYAML(config);
    if (!result.first)
    {
        ROS_ERROR("Failed to load YAML file `%s`.", config.c_str());
        return false;
    }

    try
    {
        auto &node = result.second;
        for (unsigned int i = 0; i < node.size(); ++i)
            vars.emplace_back(YAMLToVariation(node[i]));
    }
    catch (std::exception &e)
    {
        ROS_ERROR("In YAML File %s %s", config.c_str(), e.what());
        return false;
    }

    return true;
}

bool IO::loadQueries(const std::string &config, ProblemGenerator::ObjectQueryMap &start_queries,
                     ProblemGenerator::ObjectQueryMap &goal_queries)
{
    auto result = IO::loadFileToYAML(config);
    if (!result.first)
    {
        ROS_ERROR("Failed to load YAML file `%s`.", config.c_str());
        return false;
    }

    try
    {
        auto &node = result.second;
        if (!node.IsSequence())
        {
            if (!IO::isNode(node["start_queries"]))
                ROS_WARN("YAML file does not contain start queries.");
            else
            {
                const auto &sq_node = node["start_queries"];
                if (sq_node.size() > 0)
                    for (unsigned int i = 0; i < sq_node.size(); ++i)
                        for (const auto &sq : YAMLToQueries(sq_node[i]))
                            start_queries[sq.getName()] = sq;
                else
                    ROS_ERROR("Start queries is empty in YAML file.");
            }

            if (!IO::isNode(node["goal_queries"]))
                ROS_ERROR("YAML file does not contain goal queries.");
            else
            {
                const auto &gq_node = node["goal_queries"];
                if (gq_node.size() > 0)
                    for (unsigned int i = 0; i < gq_node.size(); ++i)
                        for (const auto &gq : YAMLToQueries(gq_node[i]))
                            goal_queries[gq.getName()] = gq;
                else
                    ROS_ERROR("Goal queries is empty in YAML file.");
            }
        }

        else
            ROS_ERROR("Queries node is not a sequence!");
    }
    catch (std::exception &e)
    {
        ROS_ERROR("In YAML File %s %s", config.c_str(), e.what());
        return false;
    }
    return true;
}
