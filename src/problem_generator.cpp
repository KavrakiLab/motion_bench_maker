/* Author: Carlos Quintero, Chamzas Constantinos */

// Robowflex
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/util.h>
#include <robowflex_library/log.h>
#include <robowflex_library/random.h>

// Robowflex dataset
#include <motion_bench_maker/yaml.h>
#include <motion_bench_maker/problem_generator.h>

using namespace robowflex;

std::string ProblemGenerator::ObjectQuery::getName() const
{
    return object + "_" + tag;
}

bool ProblemGenerator::ObjectQuery::operator==(const ObjectQuery &o) const
{
    return o.getName() == getName();
}

std::string ProblemGenerator::Query::getName() const
{
    std::string name = "";
    for (const auto &oq : object_queries)
        name = name + oq.getName().c_str();

    return name;
}

bool ProblemGenerator::Query::operator==(const Query &q) const
{
    return q.getName() == getName();
}

ProblemGenerator::ProblemGenerator(const std::string &config)
{
    if (!IO::loadQueries(config, start_object_queries_, goal_object_queries_))
        throw Exception(1, "Invalid YAML queries file.");
}

void ProblemGenerator::setParameters(const RobotPtr &robot, const std::string &group, const ScenePtr &scene)
{
    updateScene(scene);
    setParameters(robot, group);
}

void ProblemGenerator::setParameters(const RobotPtr &robot, const std::string &group)
{
    robot_ = robot;
    group_ = group;

    // if tips_ not initialized use the robot end_effector(s)
    if (tips_.empty())
        tips_ = robot->getSolverTipFrames(group);

    // if tips_ not initialized use the robots end_effector
    if (ee_offset_.empty())
        ee_offset_.emplace_back(RobotPose::Identity());

    processQueries();
}

void ProblemGenerator::setParameters(const RobotPtr &robot, const std::string &group,
                                     const RobotPose &ee_offset)
{
    ee_offset_ = RobotPoseVector{ee_offset};
    setParameters(robot, group);
}

void ProblemGenerator::setParameters(const RobotPtr &robot, const std::string &group,
                                     const RobotPoseVector &ee_offsets, const std::vector<std::string> tips,
                                     const bool &ee_dependency)
{
    ee_offset_ = ee_offsets;
    ee_dependency_ = ee_dependency;
    tips_ = tips;
    setParameters(robot, group);
}

void ProblemGenerator::updateScene(const ScenePtr &scene)
{
    default_scene_ = scene->deepCopy();
    scene_ = scene;
}

void ProblemGenerator::permutations(const ObjectQueryMap &obj_queries, QueryMap &queries, int K)
{
    // If offsets are all for the same object.
    if (ee_dependency_)
    {
        for (const auto &oq : obj_queries)  // [0..N-1] obj_queries
        {
            Query q;
            for (int k = 0; k < K; k++)
                q.object_queries.emplace_back(oq.second);
            queries[q.getName()] = q;
        }
    }
    else
    {
        // vectorize the object_queries
        std::vector<std::string> obj_vec;
        for (const auto &oq : obj_queries)
            obj_vec.push_back(oq.first);

        do
        {
            Query q;
            for (int k = 0; k < K; k++)  // [0..K-1] elements
                q.object_queries.emplace_back(obj_queries.at(obj_vec[k]));

            queries[q.getName()] = q;

        } while (std::next_permutation(obj_vec.begin(), obj_vec.end()));
    }
}

void ProblemGenerator::processQueries()
{
    // number of end_effectors
    int ee_number = tips_.size();

    // Clear the start and goal queries.
    goal_queries_.clear();
    start_queries_.clear();
    sg_queries_.clear();

    // If start_object_queries not provided, use an empty start_query
    if (not start_object_queries_.empty())
        permutations(start_object_queries_, start_queries_, ee_number);

    // Generate goal_queries from object_queries
    permutations(goal_object_queries_, goal_queries_, ee_number);

    if ((goal_object_queries_.size() < ee_number) and (not ee_dependency_))
        throw Exception(1, "Too few goal queries for given end-effectors");

    if ((start_object_queries_.size() < ee_number) and (not start_object_queries_.empty()) and
        (not ee_dependency_))
        throw Exception(1, "Too few start queries for given end-effectors");

    // We allow empty start_object_queries
    if (not start_object_queries_.empty())
    {
        // Precompute all the valid start/goal pairs
        for (const auto &gq : goal_queries_)
            for (const auto &sq : start_queries_)
                if (sq.first != gq.first)
                    sg_queries_.emplace_back(std::pair<Query, Query>(sq.second, gq.second));

        if (sg_queries_.empty())
            throw Exception(1, "No valid start/goal pairs were found");
    }

    RBX_INFO("Queries were proccesed Start Objects Queries: %d, Goal Object queries: %d, "
             "Start Queries: % d, Goal Queries % d, Total Start / Goal queries %d",
             start_object_queries_.size(), goal_object_queries_.size(), start_queries_.size(),
             goal_queries_.size(), sg_queries_.size());
}

ProblemGenerator::QueryResult ProblemGenerator::createRequest(const robot_state::RobotStatePtr &start_state,
                                                              const Query &goal_query)
{
    // Create empty request.
    auto request = std::make_shared<MotionRequestBuilder>(robot_, group_);

    if (scene_->checkCollision(*start_state).collision)
    {
        ROS_WARN("Start state in collision");
        return QueryResult(request, false);
    }

    // set start state
    request->setStartConfiguration(start_state);

    // Map robot state from goal query.
    bool success = setStateFromQuery(goal_query);
    if (success)
        request->setGoalConfiguration(robot_->getScratchStateConst());

    return QueryResult(request, success);
}

ProblemGenerator::QueryResult ProblemGenerator::createRequest(const Query &start_query,
                                                              const Query &goal_query)
{
    // Create request.
    auto request = std::make_shared<MotionRequestBuilder>(robot_, group_);

    // Map robot state from start query.
    bool start_success = setStateFromQuery(start_query);
    if (start_success)
        request->setStartConfiguration(robot_->getScratchStateConst());

    // Map robot state from goal query.
    bool goal_success = setStateFromQuery(goal_query);
    if (goal_success)
        request->setGoalConfiguration(robot_->getScratchStateConst());

    bool success = start_success and goal_success;
    if (not success)
    {
        // revert back to the original scene, otherwise the object stays attached
        scene_ = default_scene_->deepCopy();
        // clear all the attached bodies
        robot_->getScratchState()->clearAttachedBodies();
    }

    return QueryResult(request, success);
}

ProblemGenerator::QueryResult ProblemGenerator::createRandomRequest()
{
    if (sg_queries_.empty())
        throw Exception(1, "No valid start/goal pairs exist!");

    // Chose a valid random start/goal pair.
    const auto &pair = sg_queries_[rand() % sg_queries_.size()];
    return createRequest(pair.first, pair.second);
}

ProblemGenerator::QueryResult ProblemGenerator::createRandomRequestWithGoalQuery(const Query &goal_query)
{
    std::vector<Query> st_queries;
    for (const auto &sq : start_queries_)
        if (not(sq.second == goal_query) and sq.first != "None")
            st_queries.emplace_back(sq.second);

    if (st_queries.empty())
        throw Exception(1, "Not enough start_queries exist different");

    return createRequest(st_queries[rand() % st_queries.size()], goal_query);
}

ProblemGenerator::QueryResult ProblemGenerator::createRandomRequestWithStartQuery(const Query &start_query)
{
    std::vector<Query> gl_queries;
    for (const auto &gq : goal_queries_)
        if (not(gq.second == start_query))
            gl_queries.emplace_back(gq.second);

    if (gl_queries.empty())
        throw Exception(1, "Not enough qoal_queries exist");

    return createRequest(start_query, gl_queries[rand() % gl_queries.size()]);
}

ProblemGenerator::QueryResult
ProblemGenerator::createRandomRequestWithStartState(const robot_state::RobotStatePtr &start_state)
{
    std::vector<Query> gl_queries;
    for (const auto &gq : goal_queries_)
        gl_queries.emplace_back(gq.second);

    if (gl_queries.empty())
        throw Exception(1, "Not enough qoal_queries exist");

    return createRequest(start_state, gl_queries[rand() % gl_queries.size()]);
}

int ProblemGenerator::getNumberOfStartObjectQueries() const
{
    return start_object_queries_.size();
}

int ProblemGenerator::getNumberOfGoalObjectQueries() const
{
    return goal_object_queries_.size();
}

RobotPoseVector ProblemGenerator::getLastQueryPose() const
{
    return last_query_pose_;
}

ProblemGenerator::Query ProblemGenerator::getStartQuery(const std::string &key) const
{
    if (start_queries_.find(key) != start_queries_.end())
        return start_queries_.at(key);

    for (const auto &sq : start_queries_)
        RBX_ERROR("Available start query: %s", sq.first);

    throw Exception(1, "Start query:" + key + "does exist");
}

ProblemGenerator::Query ProblemGenerator::getGoalQuery(const std::string &key) const
{
    if (goal_queries_.find(key) != start_queries_.end())
        return goal_queries_.at(key);

    for (const auto &gq : goal_queries_)
        RBX_ERROR("Available goal query: %s", gq.first);

    throw Exception(1, "Goal query:" + key + "does not exist");
}

void ProblemGenerator::addStartObjectQuery(const ObjectQuery &soq)
{
    if (start_object_queries_.find(soq.getName()) != start_object_queries_.end())
        throw Exception(1, "Start query:" + soq.getName() + "already exists!");

    start_object_queries_[soq.getName()] = soq;

    // Reprocess the queries when we add more queries
    processQueries();
}

void ProblemGenerator::addGoalObjectQuery(const ObjectQuery &goq)
{
    if (goal_object_queries_.find(goq.getName()) != goal_object_queries_.end())
        throw Exception(1, "Goal query:" + goq.getName() + "already exists!");

    goal_object_queries_[goq.getName()] = goq;

    // Reprocess the queries when we add more queries
    processQueries();
}

bool ProblemGenerator::setStateFromQuery(const Query &query)
{
    // safeguard against empty query
    if (query.object_queries.empty())
        throw Exception(1, "Given query is empty!");

    last_query_pose_.clear();

    std::vector<GeometryConstPtr> regions;
    RobotPoseVector poses;
    std::vector<Eigen::Quaterniond> orientations;
    EigenSTL::vector_Vector3d tolerances;
    bool attached = false;

    // Get a root link pose.
    const auto &root_name = robot_->getModelConst()->getRootLinkName();
    const auto &root_pose = robot_->getLinkTF(root_name);

    for (unsigned int i = 0; i < query.object_queries.size(); i++)
    {
        const auto &oq = query.object_queries[i];

        // Get object pose.
        const auto &object_pose = scene_->getObjectPose(oq.object);

        // Transform from root link to object.
        const auto &root_to_query_object = root_pose * object_pose;

        // Get query pose with respect to root link.
        const auto &query_pose = root_to_query_object * oq.offset_pose;

        // Apply any additional ee_offset pose.
        auto ee_query_pose = query_pose * ee_offset_[i].inverse();

        last_query_pose_.emplace_back(ee_query_pose);
        regions.push_back(robowflex::Geometry::makeBox(oq.pos_tol[0], oq.pos_tol[1], oq.pos_tol[2]));
        poses.emplace_back(TF::createPoseXYZ(ee_query_pose.translation()));
        orientations.emplace_back(TF::getPoseRotation(ee_query_pose));
        tolerances.emplace_back(oq.ornt_tol);
    }

    auto ik_query = Robot::IKQuery(group_, tips_, regions, poses, orientations, tolerances, scene_);
    ik_query.attempts = 200;
    ik_query.timeout = 0.01;

    // Need approximate solutions as multi-target BioIK will only return approximate solutions.
    ik_query.options.return_approximate_solution = true;
    ik_query.validate =
        true;  // Need external validation to verify approximate solutions are within tolerance.
    ik_query.valid_distance = 0.05;  // Tuned distance threshold that is appropriate for query.

    // Sample an IK pose for the end effector
    if (!robot_->setFromIK(ik_query))
    {
        ROS_WARN("No IK solution found for (%s) ...", query.getName().c_str());
        return false;
    }

    for (unsigned int i = 0; i < query.object_queries.size(); i++)
    {
        const auto &oq = query.object_queries[i];
        if (oq.attached)
        {
            // attach the state in the scene
            scene_->attachObject(*robot_->getScratchState(), oq.object, tips_[i], {tips_[i]});
            RBX_INFO("Attaching object %s on robot ...", oq.object);
        }
    }

    RBX_INFO("IK solution found for (%s) ...", query.getName());

    return true;
}
