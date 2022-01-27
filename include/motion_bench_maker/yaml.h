/* Author: Constantinos Chamzas  */

#ifndef ROBOWFLEX_DATASET_YAML
#define ROBOWFLEX_DATASET_YAML

#include <motion_bench_maker/scene_sampler.h>
#include <motion_bench_maker/problem_generator.h>
#include <vector>

namespace robowflex
{
    namespace IO
    {
        // bool isNode(const YAML::Node &node);
        bool loadVariations(const std::string &config, std::vector<SceneSampler::Variation> &vars);
        bool loadQueries(const std::string &config, ProblemGenerator::ObjectQueryMap &start_queries,
                         ProblemGenerator::ObjectQueryMap &goal_queries);

    }  // namespace IO
}  // namespace robowflex

#endif
