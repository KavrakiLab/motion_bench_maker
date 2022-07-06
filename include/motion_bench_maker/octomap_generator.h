/* Author: Constantinos Chamzas */

#ifndef ROBOWFLEX_DATASET_OCTOMAP_GENERATOR_
#define ROBOWFLEX_DATASET_OCTOMAP_GENERATOR_

// C++
#include <string.h>
// Robowflex
#include <robowflex_library/class_forward.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/macros.h>

// GL Depth Simulator
#include "gl_depth_sim/sim_depth_camera.h"
#include "gl_depth_sim/mesh_loader.h"

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#if ROBOWFLEX_AT_LEAST_NOETIC

#include <moveit/collision_detection/occupancy_map.h>
namespace occupancy_map_monitor = collision_detection;

#else

#include <moveit/occupancy_map_monitor/occupancy_map.h>

#endif

namespace gds = gl_depth_sim;

namespace robowflex
{
    ROBOWFLEX_CLASS_FORWARD(Scene);
    ROBOWFLEX_CLASS_FORWARD(Geometry);
    ROBOWFLEX_CLASS_FORWARD(OctomapGenerator);
    namespace IO
    {
        ROBOWFLEX_CLASS_FORWARD(RVIZHelper);
    }

    typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;
    typedef std::shared_ptr<CloudXYZ> CloudXYZPtr;

    class OctomapGenerator
    {
    public:
        struct Sensors
        {
            std::string camera_config;
            std::vector<std::pair<std::string, Eigen::Vector3d>> look_objects;
            std::vector<Eigen::Vector3d> cam_points;  // this is the direction of z
            double resolution;
        };
        // Constructor
        OctomapGenerator(const std::string &config);
        // Given the origin position and the point to look at return the pose of the camera in space.
        RobotPose lookat(const Eigen::Vector3d &eye, const Eigen::Vector3d &origin);

        // @brief loads a scene to the

        void loadScene(const SceneConstPtr &scene);
        // Creates a pointcloud with points at max range. This can help create an octomap
        // with correct free cells.

        void setClipAtMaxRange(bool clip_at_max_range);
        void clear();

        // pose The position of the camera in ROS standard coordinates (+Z down camera LoS)
        CloudXYZPtr generateCloud(const RobotPose &cam_pose);
        // Generate the octomap from a PointCloud.
        bool updateOctoMap(const CloudXYZPtr &cloud, const RobotPose &cam_pose);

        bool geomToSensed(const ScenePtr &geometric, const ScenePtr &sensed,
                          const IO::RVIZHelperPtr &rviz = nullptr);

        CloudXYZPtr getLastPointCloud();
        occupancy_map_monitor::OccMapTreePtr getLastOctomapTree();
        const occupancy_map_monitor::OccMapTreeConstPtr getLastOctomapTreeConst();

    private:
        gds::Mesh geomToMesh(const GeometryPtr &geom, const std::string &name);
        void parseCameraProporties(const std::string &config);
        gds::CameraProperties props_;
        Sensors sensors_;
        bool clip_at_max_range_;

        ScenePtr cloudToOctomap(const RobotPose &cam_pose, pcl::PointCloud<pcl::PointXYZ>);
        std::shared_ptr<gds::SimDepthCamera> sim_;
        occupancy_map_monitor::OccMapTreePtr tree_;
        CloudXYZPtr fullCloud_;
    };
}  // namespace robowflex

#endif
