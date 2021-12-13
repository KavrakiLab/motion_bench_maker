# motion_bench_maker

# Datasets for planning problems
[motion_bench_maker](https://github.com/KavrakiLab/motion_bench_maker) is a ROS package that

1. Contains pre-generated realistic datasets for motion planning.
2. Provides convenient tools to synthetically generate such datasets.

## Dependencies/Instalation
To build this package you need to have in your ros workspace:
1. The main [robowflex](https://github.com/KavrakiLab/robowflex/tree/master/robowflex_library) library .
2. The [robowflex_resources](https://github.com/KavrakiLab/robowflex_resources) package which holds robot config files.
3. The [gl_depth_sim package](https://github.com/Jmeyer1292/gl_depth_sim) package to synthetically generate pointclouds.
4. The [bio_ik](https://github.com/TAMS-Group/bio_ik) kinematics solver for multi-tip robots, e.g. shadowhand or baxter bi-manual manipulations

## Simple usage 
You can generate a simple dataset, visualize it and run some basic benchmarking with the following 3 main scripts.
 
 - Download script: Download the 40 different prefabricated datasets. 
 - Generate script: Using a dataset specification (robot, scene, variation, queries)  generate a distribution of problems. 
 - Visualize script: Loads a problem set, and displays, start, goal, geometric/sensed scene and example path. 
 - Benchmark script: Loads a problem set, and benchmarks for a specific set of planners in geometric or sensed scenes.

```
# To download all 40 prefabricated dataset: 
cd problems
./download.sh all 

# ... Alternatively you can generate new dataset from the configuration files. 
# For a detailed explanation of each the parameters see the launch files in  launch/
# To Generate a dataset with sensed and geometric representations under using the specificiation in configs/problem/box_fetch.yaml 
roslaunch motion_bench_maker generate.launch sensed:=true config:="package://motion_bench_maker/configs/problems/box_fetch.yaml" dataset:="package://motion_bench_maker/problems/box_fetch/"

# Visualize the created problems in Rviz 
roslaunch motion_bench_maker visualize.launch sensed:=true geometric:=true dataset:="package://motion_bench_maker/problems/box_fetch/"

# Benchmark the created problems 
roslaunch motion_bench_maker benchmark.launch dataset:="package://motion_bench_maker/problems/box_fetch/" planners:="RRTConnect,BiEST,BKPIECE"
```
The scripts ``problems/download {robot_name}`` will download the prefabricated datasets per robot  
The scripts ``problems/generate_{robot_name}.sh`` will generate problems (start,goal,scene,path) from all the available datasets for each robot.   
**Note:** each dataset can take several hours to be generated.

## Detailed description
The main functionality lies in the following somewhat indepentent components:
- **Scene Sampler**: Instead of working with fixed scenes, it provides a simple procedure to specify and sample scenes where the objects' poses are perturbations from a nominal scene that can be described using random variables with some probability distribution.
Using the [Scene Sampler](https://github.com/KavrakiLab/motion_bench_maker/blob/master/include/motion_bench_maker/scene_sampler.h), one can "store" only the nominal scene and then solve motion planning problems for any number of scenes that are perturbations of the nominal one. This may provide a convenient way to test a planner's robustness to slight changes in the environment. We also believe that this procedure resembles more closely realistic scenes where one hardly solve the exact same motion planning problem twice (due to the world being uncertain).
- **Problem Generator**: A planning query is typically represented using a start and a goal state. However, this information depends on the robot being used. Additionally, other robot-dependant procedures, such as IK queries, are often required to fully determine a planning query. For many realistic tasks (of our interest), a planning query can be specified with respect to the scene and not the robot, e.g., *start with the end-effector at some offset pose from object A and end with the end-effector at some offset pose from object B*. In this way, an abstract (scene-dependant, but robot agnostic) query can be used for different robots without having to write additional code (e.g., for the Fetch and the UR5).
The [Problem Generator](https://github.com/KavrakiLab/motion_bench_maker/blob/master/include/motion_bench_maker/problem_generator.h) provides tools to create and manage planning queries that are specified in terms of a given scene by grounding them to a robot. A user can define a set of arbitrary start and goal queries for the scene and then programatically choose one of them or even get a random pair creating new motion planning queries. This should reduce the burden to create start and goal queries for different robots.
- **Octomap Generator**: The octomap generator only need the specification of the robot pose the camera will be placed in order to take a snapshot of the geometric scene


## Scene Sampler
The typical pipeline of the Scene Sampler can be summarized as follows:
1. Create a "variations" yaml file specifying the variations for a scene
2. Create a SceneSampler object for the nominal scene
3. Sample a new scene

The following sections show a simple example of using the Scene Sampler on the "Tall shelf" environment, which contains a large bookshelf with 9 cans. The figure below shows the shelf with all the cans at their "nominal" pose, i.e, inside the shelf centered along the $y$ axis.

![SceneSampler1](https://user-images.githubusercontent.com/5930462/145744867-a945cb30-eacb-47db-996e-813e1dec5f74.png)


### Variations definition (yaml file)
A variation is an additive random perturbation of a group of objects' poses from their nominal ones. Each variation can be applied to a set of objects in the scene and it is made from a position, an orientation and a type. See the example below:

![scenesampler11](https://user-images.githubusercontent.com/5930462/145744892-17a783be-d136-4756-a2e1-30387be32774.png)


The example shows two variations, the first one is a special case where the field names has the value "World", which indicates that such variation is applied to all the objects in the scene and it is expressed in the World frame. The second entry is a variation to only the nine cans in the shelf. Each entry in this file has the following fields:

- **names**: A list of objects in the scene that the current variation applies. As mentioned above, a special case is when the value of this field is the word "World". In that case, the variation is expressed in the world frame and it applies to all the objects in the scene. The second variation in the example applies to each can
- **position**: Parameters of the probability distributions for the current variation in its position in $x, y$ and $z$. When applied to local variations, the perturbation is performed with respect to the positions in the nominal scene. The "World" variation in the file has values of 0 which means that there is no shift in the nominal object's position. The second variation describes that the position of each can (1-9) can be perturbed from its nominal position by adding a uniform random variable $\mathcal{U}(0,0)$ in $x$ and $z$ and $\mathcal{U}(-0.15, 0.15)$ in $y$
- **orientation**: Orientations vary only using a uniform distribution. These parameters are the bounds of a symmetric uniform distribution centered in 0 that perturbs the Euler angles of the nominal object's orientation ($x, y$ and $z$ axes). In the example file, the orientation is perturbed using a uniform distribution with bounds equal to $0$ (i.e., aligned with the nominal scene).
- **type**: Name of the probability distribution of the random variable that perturbs the nominal positions of the objects affected by this variation. Currently, it only supports "uniform" and "gaussian". If "uniform" is chosen,the values in the position field correspond to bounds of the symmetric uniform distribution around 0 that perturbs the $x,y,z$ positions. If "gaussian" is selected, the values in the position field correspond to the variances of a zero-mean gaussian distribution that perturb the object's nominal position.

### Scene sampler creation
A Robowflex::SceneSampler object can be created by calling its constructor with the path of the variations file as only argument.
```
auto scene_sampler = std::make_shared<SceneSampler>("some_path_to_var_file");
```

The constructor will parse the yaml variation file and will internally save the variations of all objects in the scene.

> At this point there is no validation on the existance of the objects in the scene, therefore it is the user's responsibility to ensure that the variation file contains only valid objects
{.is-warning}

> There is one additional overloaded version of the constructor that do not take a variation file as input. Instead, it receives a variation object created and filled programatically.
{.is-info}


### Sample a new scene
The method SceneSampler::sample() can be use to sample a new scene. It receives the nominal scene (a Robowflex::Scene) as only argument and it returns the new sampled scene. Make sure the nominal scene correspond to the objects described in the variations file. If this is not the case, the class will print a warning stating the such objects do not exist, but no error is thrown.

An example of calling the sampler inside a loop is shown next:

```
while (true)
{
    auto sampled_scene = scene_sampler->sample(scene);
    rviz->updateScene(sampled_scene);
    parser::waitForUser("Displaying sampled scene!");
}
```

The following figures show one example of executing the previous code:

![SceneSampler5](https://user-images.githubusercontent.com/5930462/145745041-b93ae366-4d7c-40b9-87fe-7c246f44a949.png)


Each of one the 9 cans in the shelf is perturbed from its nominal position by adding the uniform random variable with distribution $\mathcal{U}(-0.15, 0.15)$ in the $y$ dimension.

The returned sampled scene is a standard Robowflex::Scene and can therefore be stored and be used to perform planning.

## Problem Generator
The pipeline to use the Problem Generator can be summarized as follows:
1. Create a "queries" yaml file specifying a list of possible start and goal queries in a scene
2. Create a ProblemGenerator object and set its parameters (robot, scene and planning group)
3. Create a motion planning request from the available queries

The result of following the mentioned steps is a structure that contains the desired MotionRequestBuilderPtr object which can be seamlessly used in any Robowflex (or MoveIt!) planner to do Motion Planning.

The following section shows details of every step using as example a scene with a table, a cube and two cylinders on it:

![table_with_objects](https://user-images.githubusercontent.com/5930462/145745062-4f44da2c-f6ba-4884-9a61-191b32dc2d06.png)


### Start and Goal queries definition
 (yaml file)
Start and Goal queries can be defined as some offset pose from an object in the scene. These can be easily described by writing down a yaml file. See the example below:

![queriesexample](https://user-images.githubusercontent.com/5930462/145745091-f9027de7-1624-40fe-a06f-fdcf95883767.png)


The example shows 2 entries for the start queries and 2 entries for the goal queries that correspond to a total of 4 start queries and 2 goal queries. Each entry has the following fields:

- **objects**: A list of objects in the scene for which the current entry defines a query for. In the example below, the first entry of start_queries has "Can1" and "Can2", which means that there will be a start query for each of those objects.
- **tag**: Name used to identify the query. For example, the first entry tag is "Front" because it defines the pose required by the robot's end-effector to grasp each can from the front. The second entry in the start_queries defines top grasps for the cans.
- **offset**: Pose offset from the objects. It is defined as an offset in position ($x$, $y$, $z$) and orientation (quaternion $x$, $y$, $z$, $w$) with a position tolerance position_tol and an orientation tolerance orientation_tol. For example, in the first entry, the pose has an offset position of $-0.2$ in $x$ ($20$ cm closer to the robot), $0.0$ (aligned) in $y$ and $0.0254$ in $z$ (a few centimeters higher than the can center). The orientation is exactly aligned with the local frame of the can (therefore the quaternion $0,0,0,1$) and has the tolerances shown in the example.
The second entry of the start_queries is aligned in position with the $x$ and $y$ axes but is located $0.22$ in $z$ (higher than the center of the can). In order to achieve a top grasp pose, the original frame needs to be rotated $90$ degrees around the $y$ axis, therefore the quaternion ($0,\sqrt{2}/2,0,\sqrt{2}/2$).

The goal_queries in the example are **1)** an offset from the Can1 object ($-0.6$ in $y$) called **FrontMoved** and **2)** an offset from the Cube with a pose capable of placing objects on top of the cube ($0.38$ in $z$) with a top-down grasping orientation.

### Problem Generator creation and parameters
Add the Problem Generator header file into your script:
```
#include <motion_bench_maker/problem_generator.h>
```

The Problem Generator constructor has as only argument the path of the yaml file described in the previous section. For example:
```
const std::string &queries_file = "somename.yaml";
auto pg = std::make_shared<ProblemGenerator>(queries_file);
```

At this point, the yaml file is parsed with the information of the start and goal queries. The yaml file can not have repeated tags for the start queries and for the goal queries. This could make two queries indistinguishable one from another. Also you can not have two objects with the same name into the same tag. If this is respected and the file is correctly formated, you should see a message info in the terminal showing how many start and goal queries were found.

After creation, the Problem Generator needs three parameters before it can be used: a robot, a scene and a planning group. Assuming that these have been correctly loaded, the parameters can be set as follows for the Fetch robot using the 7 dof arm:
```
pg->setParameters(fetch, scene, "arm");
```

### Create Motion Planning Requests
There are several ways to create a Motion Planning Request using the Problem Generator and the queries specified in the yaml file:

1. **Create a Request from a desired start and goal queries**: Say that the user wants to create a Planning Request with start query **"Can1-Top"** and goal query **"Cube-AboveDownwards"**. First a query name must be created for each desired query. To do this, the static method ProblemGenereator::createQueryName() can be used with the tag and the name of the object of the query as arguments respectively, for our example, this would be:
```
const auto &start = ProblemGenerator::createQueryName("Top", "Can1");
const auto &goal = ProblemGenerator::createQueryName("AboveDownwards", "Cube");
```
Then, the method ProblemGenerator::createRequest() can be used to create the Request:
```
auto request = pg->createRequest(start, goal);
```

This Request can then be used by any Robowflex planner. The Request defines a motion planning problem that should start from a top grasp of the can and would end at a top grasp above the cube. An example of this request being solved can be seen next:

![CreateRequest1](https://user-images.githubusercontent.com/5930462/145745122-ea5b2ebe-d0a9-46b9-8ce9-b0189f1c6f3c.png)


2. **Create a request from random queries**: Instead of creating a request from specific queries, different planning problems can be created by randomly choosing start and goal queries from the config file. This can be easily achived as follows:
```
auto request = pg->createRequestFromRandomQueries();
```

   The following figure shows an example of creating requests from random queries when the method is called three consecutive times. The first time (left) shows a motion planning from **"Can1-Front"** to **"Cube-AboveDownwards"**, the second time (center) shows from **"Can2-Top"** to **"Can1-FrontMoved"** and the third (right) shows from **"Can2-Top"** to **"Cube-AboveDownwards"**.

![CreateRequest2](https://user-images.githubusercontent.com/5930462/145745135-58a70288-14eb-4bf4-8e2b-9e7ad31ed46e.png)


3. **Create a request from random start queries**: A request is created given a goal pose from the list and randomly selecting from the list of start queries. A query name must be created first for the goal query and then the method ProblemGenerator::cretaeRequestFromRandomStartQuery() can be called. For example, to create a request from a random start query with goal query as **"Can1-FrontMoved"**:
```
const auto &goal = ProblemGenerator::createQueryName("FrontMoved", "Can1");
auto request = createRequestFromRandomStartQuery(goal);
```

4. **Create a request from random start queries from a given tag**: Similar to the previous case, a request is created given a goal query randomly selecting the start query. However, in this case, the start query can only be chosen from certain given tag. This might be useful in our example if we consider the goal query as **"Cube-AboveDownwards"**. Since this is a "top-down" goal pose, it would make sense to consider only top-down start queries. In that case, we would define first a query name for goal query and then call the overloaded method ProblemGenerator::creteRequestFromRandomStartQuery() with the name of the tag that should be considered to select random start queries ("Top" in this case):
```
const auto &goal = ProblemGenerator::createQueryName("AboveDownwards", "Cube");
auto request = createRequestFromRandomStartQuery("Top", goal);
```

5. **Create a request from random goal queries**: Similarly, given a start query we can create requests with randomly selected goal queries. For example:
```
const auto &start = ProblemGenerator::createQueryName("Front", "Can1");
auto request = createRequestFromRandomGoalQuery(start);
```

6. **Create a request from random goal queries from a given tag**: Finally, randomly selected goal queries for a specific tag can be selected. In this case, there is only one option of goal query for every tag in the goal_queries entry and therefore, the goal query will always be the same. For example:
```
const auto &start = ProblemGenerator::createQueryName("Front", "Can1");
auto request = createRequestFromRandomGoalQuery("FrontMoved", start);
```

### Result of creating planning queries
When calling any of the functions described above, the result is a structure called ProblemGenerator::QueryResult, which is defined as a pair, where the first field is a MotionRequestBuilderPtr and the second is a boolean indicating whether the request was successfully created or not. A typical use of this structure to save the request to a yaml file would be:
```
auto request = pg->createRequestFromRandomQueries();
if (request.second)
  request.first->toYAMLFile("somename.yaml");
```

Or to solve a motion planning problem using any Robowflex planner
```
auto request = pg->createRequestFromRandomQueries();
if (request.second)
  some_planner->plan(scene, request.first->getRequestConst());
```

## Setup
In order to use or generate a dataset the following high-level config file has to be created. The following yaml file serves as an example.
```
robot_description: package://motion_bench_maker/configs/robots/fetch.yaml
ompl_config: package://motion_bench_maker/configs/ompl/ompl_planning.yaml
scene: package://motion_bench_maker/configs/scenes/bookshelf/scene_small.yaml
queries: package://motion_bench_maker/configs/scenes/bookshelf/queries_small.yaml
variation: package://motion_bench_maker/configs/scenes/bookshelf/variation_small.yaml
sensors: package://motion_bench_maker/configs/scenes/bookshelf/sensors_small.yaml
planning_group: arm_with_torso
samples: 100
base_offset:
    position    : [0, 0, 0]
    orientation : [0, 0, 0, 1]
ee_offset:
    position    : [0, 0, 0]
    orientation : [0, 0, 0, 1]
```
The following information is required for a config file:\
**robot_description**: the description of the robot of interest, including the urdf, srdf, joint limits, kinematics, and a default robot state. \
**ompl_config**: the planner configs for the robot.\
**scene**: the desired scene for the problem.\
**queries**: the start and goal queries in the scene.\
**variation**: the variations applied to the scene.\
**sensors**: the sensor information to generate octomaps.\
**planning_group**: the desired planning group of the robot.\
**samples**: number of planning problems you wish to solve. In each problem, a random perturbation defined in variation file would be applied to the scene and a corresponding planning problem would be solved.\
**base_offset**: the offset applied to the scene relative to its default position. It consists of two parts: position and orientation. For example, if you would like the bookshelf to be placed 0.15 centimeters lower then its original position, you can add a -0.15 offest to the z axis in position field.\
**ee_offset**: the offset applied to the end-effector's query. This can be helpful when robots have different orientations at their end-effector. 

You are also free to create scene and query files on your own. With the problem config file ready, you can invoke ```generate.launch``` to create datasets. 

