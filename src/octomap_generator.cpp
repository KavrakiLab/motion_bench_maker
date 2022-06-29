/* Author: Constantinos Chamzas */

// Robowflex dataset
#include "geometric_shapes/mesh_operations.h"
#include <motion_bench_maker/octomap_generator.h>
#include <motion_bench_maker/yaml.h>

// Robowflex library
#include <robowflex_library/scene.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/scene.h>

#include <geometric_shapes/shape_operations.h>

#include "gl_depth_sim/interfaces/pcl_interface.h"
#include "gl_depth_sim/mesh.h"

#include <algorithm>

using namespace robowflex;
namespace gds = gl_depth_sim;

OctomapGenerator::OctomapGenerator(const std::string &config)
{
    // Load The octo-generation parameters
    IO::loadSensors(config, sensors_);

    // Load the camera properties for rendering
    IO::loadCameraProperties(sensors_.camera_config, props_);

    // The center point lies in the midle of height/width.
    props_.cx = props_.width / 2.0;
    props_.cy = props_.height / 2.0;

    tree_ = std::make_shared<occupancy_map_monitor::OccMapTree>(sensors_.resolution);
    sim_ = std::make_shared<gds::SimDepthCamera>(props_);
    fullCloud_ = std::make_shared<CloudXYZ>();
};

RobotPose OctomapGenerator::lookat(const Eigen::Vector3d &eye, const Eigen::Vector3d &origin)
{
    const auto up = Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d z = (eye - origin).normalized();
    Eigen::Vector3d x = z.cross(up).normalized();
    Eigen::Vector3d y = z.cross(x).normalized();

    RobotPose p = RobotPose::Identity();
    p.translation() = origin;
    p.matrix().col(0).head<3>() = x;
    p.matrix().col(1).head<3>() = y;
    p.matrix().col(2).head<3>() = z;
    return p;
}

gds::Mesh OctomapGenerator::geomToMesh(const GeometryPtr &obj, const std::string &name)
{
    gl_depth_sim::EigenAlignedVec<Eigen::Vector3f> vertices;
    std::vector<unsigned int> indices;
    auto obj_mesh = obj->isMesh() ? dynamic_cast<shapes::Mesh *>(obj->getShape().get()) :
                                    shapes::createMeshFromShape(obj->getShape().get());

    for (unsigned int i = 0; i < obj_mesh->triangle_count; ++i)
    {
        long unsigned int i3 = i * 3;
        indices.push_back(obj_mesh->triangles[i3]);
        indices.push_back(obj_mesh->triangles[i3 + 1]);
        indices.push_back(obj_mesh->triangles[i3 + 2]);
    }

    for (unsigned int i = 0; i < obj_mesh->vertex_count; ++i)
    {
        long unsigned int i3 = i * 3;
        vertices.push_back({(float)obj_mesh->vertices[i3], (float)obj_mesh->vertices[i3 + 1],
                            (float)obj_mesh->vertices[i3 + 2]});
    }
    auto mesh = gl_depth_sim::Mesh(vertices, indices);
    return mesh;
}

void OctomapGenerator::loadScene(const SceneConstPtr &scene)
{
    for (const auto &name : scene->getCollisionObjects())
    {
        const auto &obj = scene->getObjectGeometry(name);
        const auto &mesh = geomToMesh(obj, name);
        sim_->add(name, mesh, toMatrix<Eigen::Isometry3d>(scene->getObjectPose(name)));
    }
}

CloudXYZPtr OctomapGenerator::generateCloud(const RobotPose &cam_pose)
{
    CloudXYZ cloud;
    const auto &depth_img = sim_->render(toMatrix<Eigen::Isometry3d>(cam_pose));
    gds::toPointCloudXYZ(props_, depth_img, cloud);

    auto tr = cam_pose.translation();
    auto qr = cam_pose.rotation();
    // cloud.sensor_origin_ = tr;
    cloud.sensor_origin_.x() = tr.x();
    cloud.sensor_origin_.y() = tr.y();
    cloud.sensor_origin_.z() = tr.z();

    cloud.sensor_orientation_ = qr.cast<float>();

    return std::make_shared<CloudXYZ>(cloud);
}

bool OctomapGenerator::geomToSensed(const ScenePtr &geometric, const ScenePtr &sensed,
                                    const IO::RVIZHelperPtr &rviz)
{
    tree_->clear();
    fullCloud_->clear();
    auto start = ros::WallTime::now();

    loadScene(geometric);

    for (const auto &el : sensors_.look_objects)
    {
        if (not geometric->hasObject(el.first))
            ROS_ERROR("Object %s does not exist", el.first.c_str());

        const auto eye = geometric->getObjectPose(el.first) * el.second;
        for (const auto &origin : sensors_.cam_points)
        {
            const auto cam_pose = lookat(eye, origin);
            if (rviz)
            {
                rviz->addTransformMarker("camera", "map", cam_pose);
                rviz->addMarker(origin, "origin");
                rviz->addMarker(eye, "eye");
                rviz->updateMarkers();
                std::cout << "Visualizing press enter to continue" << std::endl;
                std::cin.ignore();
            }
            const auto &cloud = generateCloud(cam_pose);

            if (not updateOctoMap(cloud, cam_pose))
                return false;
        }
    }
    sensed->getScene()->processOctomapPtr(tree_, RobotPose::Identity());

    ROS_INFO("Generated Octomap cloud in %lf ms", (ros::WallTime::now() - start).toSec() * 1000.0);
    return true;
}

// Adapted from
// docs.ros.org/melodic/api/moveit_ros_perception/html/classoccupancy__map__monitor_1_1PointCloudOctomapUpdater.html#aa364d681282ab9b68eb2e94e99fefe4c

bool OctomapGenerator::updateOctoMap(const CloudXYZPtr &cloud, const RobotPose &cam_pose)
{
    const auto cso = cloud->sensor_origin_;
    // I need the transform to map coordinates

    // sensor origin is the cloud origin
    octomap::point3d sensor_origin(cso.x(), cso.y(), cso.z());
    octomap::KeySet free_cells, occupied_cells;

    tree_->lockRead();

    try
    {
        /* do ray tracing to find which cells this point cloud indicates should be free, and which it
         * indicates should be occupied */
        for (const auto &p : *cloud)
        {
            /* check for NaN */
            if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
            {
                // transform to camera frame
                auto point = cam_pose * Eigen::Vector3d{p.x, p.y, p.z};
                occupied_cells.insert(tree_->coordToKey(point.x(), point.y(), point.z()));
                fullCloud_->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
            }
        }

        // TODO: Maybe I should remove this completelely
        /* compute the free cells along each ray that ends at an occupied cell */
        for (auto it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
            if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_))
                free_cells.insert(key_ray_.begin(), key_ray_.end());

        /* compute the free cells along each ray that ends at a clipped cell */
        // for (octomap::KeySet::iterator it = clip_cells.begin(), end = clip_cells.end(); it != end;
        // ++it)
        //    if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_))
        //        free_cells.insert(key_ray_.begin(), key_ray_.end());
    }

    catch (...)
    {
        tree_->unlockRead();
        return false;
    }

    tree_->unlockRead();

    /* occupied cells are not free */
    for (auto it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
        free_cells.erase(*it);

    tree_->lockWrite();

    try
    {
        /* mark free cells only if not seen occupied in this cloud */
        for (auto it = free_cells.begin(), end = free_cells.end(); it != end; ++it)
            tree_->updateNode(*it, false);

        /* now mark all occupied cells */
        for (auto it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
            tree_->updateNode(*it, true);
    }
    catch (...)
    {
        ROS_ERROR("Internal error while updating octree");
    }
    tree_->unlockWrite();

    tree_->triggerUpdateCallback();
    return true;
}

CloudXYZPtr OctomapGenerator::getLastPointCloud()
{
    return fullCloud_;
}

occupancy_map_monitor::OccMapTreePtr OctomapGenerator::getLastOctomapTree()
{
    return tree_;
}
