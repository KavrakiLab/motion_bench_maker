/* Author: Constantinos Chamzas */
#ifndef ROBOWFLEX_DATASET_PARSER_
#define ROBOWFLEX_DATASET_PARSER_

// C++
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>

// Eigen
#include <Eigen/Geometry>

// This file looks could be a substitute for this, or at least draw ideas:
// https://github.com/PickNikRobotics/rosparam_shortcuts

namespace robowflex
{
    namespace parser
    {
        template <typename Type>
        std::string toString(Type val, int width, char fill = '0')
        {
            std::stringstream ss;
            // Preappends 'fill' to val to make it of length at least width
            ss << std::setw(width) << std::setfill(fill) << val;
            return ss.str();
        }

        template <typename Type>
        std::string vecToString(const std::vector<Type> vec)
        {
            std::stringstream ss;
            // Preappends 'fill' to val to make it of length at least width
            ss << "[";
            for (const auto &val : vec)
                ss << val << ",";
            ss << "]";

            return ss.str();
        }

        template <typename Type>
        bool get(const std::string &parent_name, ros::NodeHandle &node, const std::string &param_name,
                 Type &param)
        {
            if (!node.hasParam(param_name))
            {
                ROS_ERROR_STREAM(parent_name << ":Missing " << node.getNamespace() << "/" << param_name);
                return false;
            }

            node.getParam(param_name, param);
            return true;
        }

        template <typename Type>
        bool get(const std::string &parent_name, ros::NodeHandle &node, const std::string &param_name,
                 std::vector<Type> &param_vector)
        {
            if (!node.hasParam(param_name))
            {
                ROS_ERROR_STREAM(parent_name << ": Missing " << node.getNamespace() << "/" << param_name);
                return false;
            }

            node.getParam(param_name, param_vector);

            if (param_vector.empty())
                ROS_WARN_STREAM(parent_name << ": Empty vector for " << node.getNamespace() << "/"
                                            << param_name);

            return true;
        }

        inline bool get(const std::string &parent_name, ros::NodeHandle &node, const std::string &param_name,
                        Eigen::Vector3d &param)
        {
            if (!node.hasParam(param_name))
            {
                ROS_ERROR_STREAM(parent_name << ":Missing " << node.getNamespace() << "/" << param_name);
                return false;
            }

            std::vector<double> values;
            node.getParam(param_name, values);

            if (values.empty())
                ROS_WARN_STREAM_NAMED(parent_name,
                                      "Empty vector for " << node.getNamespace() << "/" << param_name);

            // Copy to the Eigen::Vector
            param[0] = values[0];
            param[1] = values[1];
            param[2] = values[2];
            return true;
        }

        inline void shutdownIfError(const std::string &parent_name, std::size_t error_count)
        {
            if (!error_count)
                return;

            ROS_ERROR_STREAM_NAMED(parent_name, "Missing " << error_count
                                                           << " ros parameters that are required. "
                                                              "Shutting down "
                                                              "to prevent undefined behaviors");
            ros::shutdown();
            exit(0);
        }

        inline void waitForUser(const std::string &task)
        {
            std::cout << "Robot is ";
            if (isatty(STDOUT_FILENO))
                std::cout << "\033[1;32m<" << task << ">\033[0m. ";
            else
                std::cout << "<" << task << ">. ";

            std::cout << "Press enter when ready." << std::endl;
            std::cin.ignore();
        }

    }  // namespace parser

}  // namespace robowflex

#endif
