/* Author: Carlos Quintero, Constantinos Chamzas */

#ifndef ROBOWFLEX_DATASET_PROBLEM_GENERATOR_
#define ROBOWFLEX_DATASET_PROBLEM_GENERATOR_

// C++
#include <string.h>

// Robowflex
#include <robowflex_library/class_forward.h>
#include <robowflex_library/adapter.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/builder.h>
#include <unordered_set>

// Yaml
#include <yaml-cpp/yaml.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Scene);
    ROBOWFLEX_CLASS_FORWARD(Robot);
    ROBOWFLEX_CLASS_FORWARD(MotionRequestBuilder);
    namespace OMPL
    {
        ROBOWFLEX_CLASS_FORWARD(OMPLInterfacePlanner);
    }
    /** \endcond */

    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(ProblemGenerator);
    /** \endcond */

    /** \class robowflex::ProblemGeneratorPtr
        \brief A shared pointer wrapper for robowflex::ProblemGenerator. */

    /** \class robowflex::ProblemGeneratorConstPtr
        \brief A const shared pointer wrapper for robowflex::ProblemGenerator. */

    /** \brief Robowflex Problem Generator.
     */
    class ProblemGenerator
    {
    public:
        /** \brief Structure with parameters for all start and goal queries in a planning problem.
         */

        struct ObjectQuery
        {
            std::string object;        ///< Object for the IK pose.
            std::string tag;           ///< Unique tag for this query (e.g., under, above, etc).
            RobotPose offset_pose;     ///< Pose of the query with respect to each object.
            Eigen::Vector3d pos_tol;   ///< Position tolerance for this query.
            Eigen::Vector3d ornt_tol;  ///< Orientation tolerance for this query.
            bool attached;             ///<  Whether the object is attached to the end-effector.

            /** \brief Creates a unique ID for this query.
             *  \return a unique ID representing the string
             */
            std::string getName() const;
            bool operator==(const ObjectQuery &o) const;
        };

        struct Query
        {
            std::vector<ObjectQuery> object_queries;  ///< List of Object queries, multiple are defined for
                                                      ///< multi-tip robots.

            /** \brief Creates a unique ID for this query.
             *  \return a unique ID representing the string
             */
            std::string getName() const;
            bool operator==(const Query &o) const;
        };

        typedef std::map<std::string, Query> QueryMap;
        typedef std::map<std::string, ObjectQuery> ObjectQueryMap;

        /** \brief Result of a query. Composed of two objects, first is the request, second is a boolean. When
         * false, the query was not succcessfully created.
         */
        typedef std::pair<MotionRequestBuilderPtr, bool> QueryResult;

        /** \brief Constructor.
         *  \param[in] config Name of file with queries description.
         */
        ProblemGenerator(const std::string &config);

        /** \brief Map the problem generator parameters.
         *  \param[in] robot Robot to create problem generator for.
         *  \param[in] scene Scene to create problem generator for.
         *  \param[in] group Group to create problem generator for.
         */
        void setParameters(const RobotPtr &robot, const std::string &group, const ScenePtr &scene);

        /** \brief Map the problem generator parameters.
         *  \param[in] robot Robot to create problem generator for.
         *  \param[in] group Group to create problem generator for.
         */
        void setParameters(const RobotPtr &robot, const std::string &group);

        /** \brief Map the problem generator parameters.
         *  \param[in] robot Robot to create problem generator for.
         *  \param[in] group Group to create problem generator for.
         *  \param[in] ee_offset End-effector pose offset.
         */
        void setParameters(const RobotPtr &robot, const std::string &group, const RobotPose &ee_offset);

        /** \brief Map the problem generator parameters.
         *  \param[in] robot Robot to create problem generator for.
         *  \param[in] group Group to create problem generator for.
         *  \param[in] ee_offsets End-effector poses for offset.
         *  \param[in] tips the end_effector link tips.
         *  \param[in] ee_dependency Flag to treat end-effectors as dependent or not.
         */

        void setParameters(const RobotPtr &robot, const std::string &group, const RobotPoseVector &ee_offsets,
                           const std::vector<std::string> tips, const bool &ee_dependency);

        /** \brief Update the scene to create the problem generator for.
         *  \param[in] scene Scene to be updated.
         */
        void updateScene(const ScenePtr &scene);

        /** \brief Create a motion request from the given \a start_query and \a goal_query.
         *  \param[in] start_query Start query.
         *  \param[in] goal_query Goal query.
         *  \return Result of creating the query.
         */
        QueryResult createRequest(const Query &start_query, const Query &goal_query);

        /** \brief Create a motion request from the given \a start_state and \a goal_query.
         *  \param[in] start_state Start state.
         *  \param[in] goal_query Goal query.
         *  \return Result of creating the query.
         */
        QueryResult createRequest(const robot_state::RobotStatePtr &start_state, const Query &goal_query);

        /** \brief Create a motion request from random start and goal queries.
         *  \return Result of creating the query.
         */
        QueryResult createRandomRequest();

        /** \brief Create a motion request from a random start query and the given \a goal_query.
         *  \param[in] goal_query Goal query.
         *  \return Result of creating the query.
         */
        QueryResult createRandomRequestWithGoalQuery(const Query &goal_query);

        /** \brief Create a motion request from a given \a start_query and a random goal query.
         *  \param[in] start_query Goal query.
         *  \return Result of creating the query.
         */
        QueryResult createRandomRequestWithStartQuery(const Query &start_query);

        /** \brief Create a motion request from a given \a start_query and a random goal query.
         *  \param[in] start_state Start state.
         *  \return Result of creating the query.
         */
        QueryResult createRandomRequestWithStartState(const robot_state::RobotStatePtr &start_state);

        /** \brief Create a motion request from a given \a start_query and a random goal query (only from
         * those with \a goal_tag).
         *  \param[in] goal_tag Tag to find a start query from.
         *  \param[in] start_query The start state.
         *  \return Result of creating the query.
         */
        QueryResult createRequestFromRandomGoalQuery(const std::string &goal_tag, const Query &start_query);

        /** \brief Get the number of start Queries.
         *  \return Number of start queries.
         */
        int getNumberOfStartObjectQueries() const;

        /** \brief Get the number of start Queries.
         *  \return Number of start queries.
         */
        int getNumberOfGoalObjectQueries() const;

        /** \brief Get the number of end effectors.
         *  \return Number of endEffectors.
         */
        int getNumberOfEEs() const;

        /** \brief Get the last query pose that was used for IK.
         *  \return Pose last query pose used for ik sampling.
         */
        RobotPoseVector getLastQueryPose() const;

        /** \brief Get Query from start queries.
         *  \param[in] start_query Name of start query to get.
         *  \return Query object identified by name and tag.
         */
        Query getStartQuery(const std::string &query_name) const;

        /** \brief Get Query from goal queries.
         *  \param[in] goal_query Name of goal query to get.
         *  \return Query object identified by name and tag.
         */
        Query getGoalQuery(const std::string &query_name) const;

        /** \brief Add a start object query only if it does not already exist. Throws an Exception if query
         *  already exists.
         *  \param[in] soq The start object query  add.
         */
        void addStartObjectQuery(const ObjectQuery &soq);

        /** \brief Add a goal object query only if it does not already exist. Throws an Exception if query
         *  already exists.
         *  \param[in] goq The goal object query  add.
         */
        void addGoalObjectQuery(const ObjectQuery &goq);

    private:
        /** \brief Permutations of an object query
         */
        void permutations(const ObjectQueryMap &obj_queries, QueryMap &queries, int K);

        /** \brief Proecess the object_queries to full queries
         */
        void processQueries();

        /** \brief Map the scratch state of the robot from a given \a query from the given \a type.
         *  \param[in] query Query to set the state of the robot from.
         *  \param[in] type Type of query (start or goal).
         *  \return True if the IK query was successful and the robot state was set.
         */
        bool setStateFromQuery(const Query &query);

        /** \brief Get a random query of a \a type.
         *  \param[in] type Type of query (start or goal).
         *  \return Name of random query.
         */
        Query getRandomQuery(const std::string &type) const;

        /** \brief Get a random query of a \a tag and a \a type.
         *  \param[in] tag Tag to find a random query from.
         *  \param[in] type Type of query (start or goal).
         *  \return Name of random query.
         */
        Query getRandomQuery(const std::string &tag, const std::string &type) const;

        /** \brief Get a random query from \a type that is different from \a existing_query.
         *  \param[in] type Type of query (start or goal) to find.
         *  \param[in] existing_query Query that must be different to the output random query.
         *  \param[in] num_attempts Number of attempts to randomly sample a query that is different to the
         * given query before reporting that it failed. \return Name of random query.
         */
        Query getDifferentRandomQueryFromType(const std::string &type, Query existing_query) const;

        /** \brief Get a random query from \a type and \a tag that is different from \a existing_query.
         *  \param[in] tag Tag of query to sample.
         *  \param[in] type Type of query (start or goal) to sample.
         *  \param[in] existing_query Query that must be different to the output random query.
         *  \param[in] num_attempts Number of attempts to randomly sample a query that is different to the
         * given query before reporting that it failed. \return Name of random query.
         */
        Query getDifferentRandomQueryFromType(const std::string &tag, const std::string &type,
                                              Query existing_query) const;

        std::string group_;              ///< Name of planning group.
        RobotPtr robot_;                 ///< Robot to plan for.
        ScenePtr scene_;                 ///< Modifiable scene (used for attaching objects).
        ScenePtr default_scene_;         ///< Initial un-modified scene.
        RobotPoseVector ee_offset_{};    ///< Offset of end-effector(s).
        std::vector<std::string> tips_;  ///< Tip(s) of end-effector(s).

        ObjectQueryMap start_object_queries_, goal_object_queries_;  ///< Start and goal object queries.
        QueryMap start_queries_, goal_queries_;                      ///< Start and goal queries.

        std::vector<std::pair<Query, Query>> sg_queries_;  ///< Map of valid start/goal pairs.

        RobotPoseVector last_query_pose_{{RobotPose::Identity()}};  ///< Last IK query pose(s).
        bool ee_dependency_{false};
        bool initialized_{false};
        //< Dependency between end-effectors
    };
}  // namespace robowflex

#endif
