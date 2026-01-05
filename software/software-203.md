<!--
author:   Robot Campus Team
email:    hello@robotcampus.dev
version:  2.0.0
language: en
narrator: US English Female

comment:  Motion Planning for Robotic Manipulators: A comprehensive course on algorithmic motion planning, configuration space representations, collision detection, sampling-based planning algorithms, and the MoveIt2 framework for manipulation.

logo:     https://robotcampus.dev/logo.png

mode:     Textbook

link:     https://robotcampus.dev/styles/course-styles.css

-->

# SOFTWARE-203: Motion Planning for Robotic Manipulators

--{{0}}--
This course provides comprehensive education in motion planning theory and practice for robotic manipulation. Students develop understanding of configuration space, collision detection algorithms, sampling-based planning methods, and industrial motion planning frameworks.

**Course Code**: SOFTWARE-203
**Duration**: 6 hours
**Level**: Advanced
**Prerequisites**: SOFTWARE-202 (Distributed Robotics Systems with ROS2)

## Learning Objectives

By completing this course, students will:

1. Understand configuration space representation and its role in motion planning
2. Analyze computational complexity of motion planning in high-dimensional spaces
3. Implement sampling-based planning algorithms including RRT and PRM
4. Apply collision detection techniques for geometric reasoning
5. Configure and deploy MoveIt2 for industrial-grade motion planning
6. Develop motion planning applications using programmatic APIs

---

## Module 1: Motion Planning Theory

--{{0}}--
This module establishes theoretical foundations of robot motion planning.

### 1.1 The Motion Planning Problem

<!-- class="theory-concept" -->
**Problem Formulation**

The fundamental motion planning problem: Given a robot, initial configuration, goal configuration, and obstacles, find a collision-free path from initial to goal.

**Formal definition**:
- **Configuration space** (C-space): Set of all possible robot configurations
- **Free space** (C_free): Configurations without collisions
- **Obstacle space** (C_obs): Configurations in collision
- **Start configuration**: q_start ∈ C_free
- **Goal configuration**: q_goal ∈ C_free

**Find**: Path τ: [0,1] → C_free such that τ(0) = q_start and τ(1) = q_goal

**Variants**:
- **Feasible planning**: Find any collision-free path
- **Optimal planning**: Find shortest or minimum-cost path
- **Anytime planning**: Improve solution quality over time
- **Multi-query planning**: Precompute roadmap for multiple queries

<!-- class="theory-concept" -->
**Configuration Space (C-space)**

Configuration space is the space of all possible robot configurations.

**Degrees of freedom (DOF)**: Minimum number of parameters to fully specify configuration

Examples:
- Point robot in 2D plane: 2 DOF (x, y)
- Rigid body in 2D: 3 DOF (x, y, θ)
- Rigid body in 3D: 6 DOF (x, y, z, roll, pitch, yaw)
- n-link serial manipulator: n DOF (joint angles)

**C-space topology**: May be Euclidean (R^n) or non-Euclidean (e.g., SO(3) for rotations)

**C-space obstacles**:
- Workspace obstacle → C-space obstacle (potentially complex shape)
- Configuration in collision ⇔ Point in C-space obstacle
- Motion planning reduced to path finding in C-space

**Advantages of C-space representation**:
- Robot becomes a point (simplifies geometric reasoning)
- Obstacles mapped to C-space (one-time computation)
- Path planning algorithms operate on point in space

**Disadvantages**:
- Dimensionality: C-space dimension = robot DOF (curse of dimensionality)
- C-space obstacle computation expensive for complex geometries
- Visualization difficult for >3 DOF

<!-- class="theory-concept" -->
**Computational Complexity**

Motion planning is computationally hard:

**Piano Mover's Problem** (Reif, 1979):
- General motion planning is PSPACE-hard
- Even for polygonal robot and obstacles in 2D

**Implications**:
- No efficient general algorithm for arbitrary environments
- Worst-case complexity exponential in DOF
- Practical algorithms use heuristics and approximations

**Completeness classifications**:
- **Complete**: Finds solution if one exists, reports failure otherwise (exponential time)
- **Resolution complete**: Discretize space; complete at given resolution
- **Probabilistically complete**: Probability of finding solution → 1 as time → ∞
- **Practically incomplete**: May fail even when solution exists (fast heuristics)

Modern motion planners trade completeness for efficiency.

<!-- class="historical-note" -->
**Evolution of Motion Planning**

Motion planning research evolved through several paradigms:

- **1970s-1980s**: Exact algorithms (visibility graphs, Voronoi diagrams, cell decomposition)
- **1985**: Configuration space concept formalized (Lozano-Pérez)
- **1990s**: Sampling-based methods emerge (RRT, PRM)
- **2000s**: Optimization-based methods (CHOMP, TrajOpt)
- **2010s**: Machine learning for planning (MPNet, motion policy networks)
- **2020s**: Hybrid approaches combining classical and learning methods

Sampling-based methods currently dominate for high-DOF manipulation.

### 1.2 Collision Detection

<!-- class="theory-concept" -->
**Geometric Collision Checking**

Collision detection determines if configuration is in C_free or C_obs.

**Exact collision detection**:
- **Separating Axis Theorem (SAT)**: For convex polyhedra
- **Gilbert-Johnson-Keerthi (GJK)**: Distance between convex shapes
- **Computation**: O(n) for n vertices/faces

**Bounding volume hierarchies (BVH)**:
Accelerate collision checks using nested bounding volumes:
- **AABB** (Axis-Aligned Bounding Box): Simple, loose bounds
- **OBB** (Oriented Bounding Box): Tighter, more expensive
- **Spheres**: Fastest check, loose bounds
- **k-DOP** (Discrete Oriented Polytope): Balance tightness and speed

**BVH tree**: Hierarchical organization
- Leaf nodes: Primitive geometry (triangles)
- Internal nodes: Bounding volumes
- Collision check: Traverse tree, prune branches if BVs don't collide

**Performance**: O(log n) average case for n primitives

**Continuous collision detection**:
- Discrete: Check collision at configurations along path
- Continuous: Check swept volume for entire motion
- Conservative advancement: Step along path by safe distance

<!-- class="theory-concept" -->
**Distance Computation**

Minimum distance between robot and obstacles informs planning:

**Signed distance**:
- Positive: Separation distance (in free space)
- Negative: Penetration depth (in collision)
- Zero: Contact

**Applications**:
- **Safety margins**: Maintain minimum clearance
- **Gradient-based optimization**: Descend distance gradient
- **Workspace bounds**: Limit reachable space

**Algorithms**:
- GJK: Distance between convex shapes
- Expanding polytope algorithm (EPA): Penetration depth
- Proximity query package (PQP): Efficient library

**Trade-off**: Exact distance expensive; approximations sufficient for many applications

<!-- class="alternative-approach" -->
**Approximate Collision Detection**

For real-time applications, approximate methods trade accuracy for speed:

**Point cloud collision detection**:
- Represent robot/environment as point clouds
- Check if robot points inside obstacle volume
- Fast for complex geometries

**Octree/voxel representations**:
- Discretize space into cells
- Mark occupied/free cells
- Collision check: Query cell occupancy
- Trade-off: Memory vs. resolution

**Learning-based collision checking**:
- Train neural network to predict collision
- Amortize computation in training
- Fast inference at runtime

**Quiz: Planning Theory**

What is the primary advantage of representing motion planning in configuration space?

[(X)] The robot becomes a point, simplifying path finding
[( )] It reduces the number of degrees of freedom
[( )] It eliminates the need for collision detection
[( )] It guarantees an optimal solution
***
<div>
In configuration space, the robot is represented as a point and obstacles are mapped to C-space obstacles. This transforms the problem into finding a collision-free path for a point, which is conceptually and computationally simpler.
</div>
***

Why are sampling-based planners "probabilistically complete" rather than "complete"?

[( )] They always find the optimal solution
[( )] They discretize the configuration space
[(X)] They have probability approaching 1 of finding a solution as time increases
[( )] They use random number generators
***
<div>
Probabilistically complete algorithms don't guarantee finding a solution in finite time, but the probability of success approaches 1 as more samples are taken. This provides practical performance without the exponential complexity of complete methods.
</div>
***

---

## Module 2: Sampling-Based Planning Algorithms

--{{0}}--
This module examines probabilistic sampling-based motion planning methods.

### 2.1 Probabilistic Roadmap (PRM)

<!-- class="theory-concept" -->
**PRM Algorithm**

PRM builds a roadmap of C_free through random sampling, enabling multi-query planning.

**Construction phase**:
```
1. Sample N random configurations from C-space
2. For each sample q:
   - If q ∈ C_free, add to roadmap vertices V
3. For each vertex v ∈ V:
   - Attempt connection to k nearest neighbors
   - If local path collision-free, add edge to E
4. Return roadmap graph G = (V, E)
```

**Query phase**:
```
1. Connect q_start and q_goal to roadmap (add temporary vertices)
2. Search graph for path from q_start to q_goal
   - Use Dijkstra's or A* algorithm
3. Smooth path (post-processing)
4. Remove temporary connections
```

**Properties**:
- **Probabilistically complete**: As N → ∞, probability of finding solution → 1
- **Multi-query**: Roadmap reused for different start/goal pairs
- **Preprocessing cost**: Expensive to build roadmap
- **Query cost**: Fast graph search

**Parameters**:
- **N**: Number of samples (tradeoff: coverage vs. computation)
- **k**: Neighbors per vertex (higher k → better connectivity, more computation)
- **Local planner**: Straight-line, simple motion primitives

### 2.2 Rapidly-Exploring Random Trees (RRT)

<!-- class="theory-concept" -->
**RRT Algorithm**

RRT grows a tree from q_start toward unexplored regions through random sampling.

**Basic RRT**:
```
1. Initialize tree T with q_start
2. For i = 1 to N iterations:
   - Sample random configuration q_rand
   - Find nearest vertex q_near in tree to q_rand
   - Extend from q_near toward q_rand by step size δ
   - If extension q_new is collision-free:
     - Add q_new to tree with edge from q_near
   - If q_new near q_goal, return path
3. Return failure
```

**Key operations**:
- **Nearest neighbor**: Find closest tree vertex to q_rand
  - Naive: O(n) for n vertices
  - k-d tree: O(log n) average
- **Extend**: Move from q_near toward q_rand by distance δ
- **Collision check**: Verify edge is collision-free

**Voronoi bias**: RRT explores large Voronoi regions preferentially, achieving rapid exploration.

**Properties**:
- **Probabilistically complete**
- **Single-query**: Tree built per query
- **No preprocessing**: Suitable for dynamic environments
- **Not optimal**: First solution often far from optimal

### 2.3 RRT Variants and Optimizations

<!-- class="theory-concept" -->
**RRT* (RRT-star)**

RRT* adds optimization to find asymptotically optimal paths.

**Modifications to RRT**:
1. **Rewiring**: After adding q_new, check if nearby vertices benefit from connection through q_new
2. **Cost propagation**: Update costs when rewiring
3. **Radius**: Connection radius r(n) = γ(log n / n)^(1/d) for n vertices, dimension d

**Guarantees**:
- Cost c(path) → c*(optimal) as iterations → ∞
- Asymptotically optimal (probabilistic)

**Trade-off**: Higher computation per iteration, better solution quality

<!-- class="theory-concept" -->
**Bi-directional RRT (RRT-Connect)**

Grow trees from both q_start and q_goal simultaneously.

**Algorithm**:
```
1. Initialize T_start from q_start, T_goal from q_goal
2. Alternate between trees:
   - Sample q_rand
   - Extend active tree toward q_rand → q_new
   - Attempt to connect other tree to q_new
   - If connection succeeds, return path
3. Swap active tree and repeat
```

**Advantages**:
- Faster convergence (two trees explore toward each other)
- Effective for high-dimensional spaces

**Connect heuristic**: Extend aggressively (multiple steps) rather than single step

<!-- class="theory-concept" -->
**Informed RRT***

Use heuristic to focus sampling on promising regions.

**Informed sampling**:
- After finding initial solution with cost c_best
- Sample only from ellipsoid containing q_start, q_goal with cost ≤ c_best
- Refines solution by exploring only potentially better paths

**Performance**: Faster convergence to near-optimal solution

<!-- class="alternative-approach" -->
**Alternative Planning Paradigms**

**Optimization-based planning**:
- CHOMP (Covariant Hamiltonian Optimization for Motion Planning)
- TrajOpt (Trajectory Optimization)
- Formulate as nonlinear optimization problem
- Minimize cost (smoothness, obstacle distance) subject to constraints
- Requires good initial guess (from RRT)

**Search-based planning**:
- A*, Weighted A*, ARA*
- Discretize C-space into grid
- Graph search with heuristic
- Completeness guarantees (at discretization resolution)
- Curse of dimensionality: Grid size exponential in DOF

**Hybrid approaches**:
- Use RRT for global topology, optimize locally
- Multi-resolution planning (coarse global, fine local)
- Combine sampling and optimization

**Quiz: Sampling-Based Planning**

What is the primary advantage of PRM over RRT for motion planning?

[( )] PRM finds optimal paths
[(X)] PRM enables fast multi-query planning after preprocessing
[( )] PRM works in dynamic environments
[( )] PRM requires fewer samples
***
<div>
PRM builds a roadmap once (expensive preprocessing) but enables fast queries for different start-goal pairs by searching the precomputed graph. RRT rebuilds the tree for each query.
</div>
***

What does the "rewiring" step in RRT* accomplish?

[( )] It removes redundant vertices
[(X)] It improves path cost by reconnecting vertices through better paths
[( )] It balances the tree structure
[( )] It speeds up nearest neighbor queries
***
<div>
Rewiring checks if existing vertices can reach the new vertex with lower cost, restructuring the tree to improve path quality and converge toward the optimal solution.
</div>
***

---

## Module 3: MoveIt2 Framework

--{{0}}--
This module covers the MoveIt2 framework for industrial motion planning in ROS2.

### 3.1 MoveIt2 Architecture

<!-- class="theory-concept" -->
**MoveIt2 System Components**

MoveIt2 provides comprehensive motion planning for manipulation:

**Core components**:
- **move_group**: Central node orchestrating planning pipeline
- **Planning scene**: Representation of robot and environment
- **Planning pipeline**: Plugin architecture for planners
- **Controller interface**: Hardware abstraction for execution
- **Perception**: Sensor integration for environment modeling

**Planning plugins**:
- OMPL (Open Motion Planning Library): RRT, PRM, and variants
- Pilz industrial motion: Blending, point-to-point, linear, circular
- STOMP (Stochastic Trajectory Optimization)
- CHOMP (optimization-based)
- Custom planners via plugin interface

**Execution**: Trajectory controllers execute planned paths on hardware

<!-- class="theory-concept" -->
**SRDF (Semantic Robot Description Format)**

SRDF augments URDF with semantic and planning-specific information:

**Planning groups**: Named kinematic chains for planning
```xml
<group name="arm">
  <chain base_link="base_link" tip_link="end_effector" />
</group>
<group name="gripper">
  <link name="left_finger" />
  <link name="right_finger" />
</group>
```

**Group states**: Named configurations
```xml
<group_state name="home" group="arm">
  <joint name="joint1" value="0.0" />
  <joint name="joint2" value="-1.57" />
  ...
</group_state>
```

**Disable collisions**: Pairs never in collision (reduce checks)
```xml
<disable_collisions link1="link1" link2="link2" reason="Adjacent" />
```

**End effectors**: Grasp pose definitions
```xml
<end_effector name="gripper" parent_link="wrist" group="gripper" />
```

### 3.2 MoveIt2 Setup and Configuration

<!-- class="theory-concept" -->
**MoveIt Setup Assistant Workflow**

The Setup Assistant generates MoveIt configuration package:

**Steps**:
1. **Load URDF**: Import robot description
2. **Self-collision matrix**: Compute adjacent/never-colliding link pairs
3. **Planning groups**: Define kinematic chains for planning
4. **Robot poses**: Add named configurations (home, ready, stow)
5. **End effectors**: Define gripper/tool groups
6. **Passive joints**: Mark non-actuated joints (e.g., casters)
7. **Controllers**: Configure hardware interfaces
8. **Perception**: Setup 3D sensors (optional)
9. **Generate**: Create configuration package

**Generated files**:
```
moveit_config/
├── config/
│   ├── moveit.rviz                 # RViz configuration
│   ├── <robot>.srdf                # Semantic description
│   ├── joint_limits.yaml           # Velocity/acceleration limits
│   ├── kinematics.yaml             # IK solver configuration
│   ├── ompl_planning.yaml          # Planner parameters
│   └── controllers.yaml            # Hardware controllers
├── launch/
│   ├── demo.launch.py              # Interactive demo
│   ├── move_group.launch.py        # Planning node
│   └── ...
└── package.xml
```

**Kinematics solvers**:
- **KDL**: Numerical Jacobian-based IK (slow, general)
- **TRAC-IK**: Improved numerical IK (faster, more robust)
- **Analytical**: Robot-specific closed-form IK (fastest, limited applicability)
- **Plugin**: Custom IK solver

### 3.3 MoveIt2 Programmatic Interface

<!-- class="theory-concept" -->
**MoveGroup C++ API**

MoveGroup provides high-level planning and execution interface:

**Basic usage**:
```cpp
#include <moveit/move_group_interface/move_group_interface.h>

// Create planning interface for "arm" group
moveit::planning_interface::MoveGroupInterface move_group(node, "arm");

// Set pose goal
geometry_msgs::msg::Pose target_pose;
target_pose.position.x = 0.28;
target_pose.position.y = 0.0;
target_pose.position.z = 0.5;
target_pose.orientation.w = 1.0;

move_group.setPoseTarget(target_pose);

// Plan
moveit::planning_interface::MoveGroupInterface::Plan plan;
bool success = (move_group.plan(plan) ==
                moveit::core::MoveItErrorCode::SUCCESS);

// Execute
if (success) {
    move_group.execute(plan);
}
```

**Joint space planning**:
```cpp
std::vector<double> joint_values = {0.0, -1.57, 1.57, 0.0, 1.57, 0.0};
move_group.setJointValueTarget(joint_values);
move_group.move();  // Plan and execute
```

**Named targets**:
```cpp
move_group.setNamedTarget("home");
move_group.move();
```

**Planning constraints**:
```cpp
// Orientation constraint
moveit_msgs::msg::OrientationConstraint oc;
oc.link_name = "end_effector";
oc.orientation.w = 1.0;
oc.absolute_x_axis_tolerance = 0.1;
oc.absolute_y_axis_tolerance = 0.1;
oc.absolute_z_axis_tolerance = 0.1;
oc.weight = 1.0;

moveit_msgs::msg::Constraints constraints;
constraints.orientation_constraints.push_back(oc);
move_group.setPathConstraints(constraints);
```

**Collision object management**:
```cpp
#include <moveit/planning_scene_interface/planning_scene_interface.h>

moveit::planning_interface::PlanningSceneInterface planning_scene;

// Add box obstacle
moveit_msgs::msg::CollisionObject collision_object;
collision_object.header.frame_id = "base_link";
collision_object.id = "box";

shape_msgs::msg::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions.resize(3);
primitive.dimensions[0] = 0.5;
primitive.dimensions[1] = 0.1;
primitive.dimensions[2] = 0.5;

geometry_msgs::msg::Pose box_pose;
box_pose.position.x = 0.4;
box_pose.position.y = 0.0;
box_pose.position.z = 0.25;
box_pose.orientation.w = 1.0;

collision_object.primitives.push_back(primitive);
collision_object.primitive_poses.push_back(box_pose);
collision_object.operation = collision_object.ADD;

std::vector<moveit_msgs::msg::CollisionObject> objects;
objects.push_back(collision_object);
planning_scene.applyCollisionObjects(objects);
```

<!-- class="alternative-approach" -->
**MoveIt Task Constructor (MTC)**

For complex, multi-stage manipulation tasks:

**Task hierarchy**:
- **Stages**: Atomic actions (move, grasp, place)
- **Containers**: Sequence, alternatives, parallel composition
- **Task**: Complete manipulation pipeline

**Example: Pick and place**:
```cpp
auto task = std::make_unique<moveit::task_constructor::Task>();
task->stages()->setName("pick_and_place");

// Current state
task->add(std::make_unique<stages::CurrentState>("current"));

// Open gripper
task->add(std::make_unique<stages::MoveRelative>("open_gripper", ...));

// Approach object
task->add(std::make_unique<stages::MoveRelative>("approach", ...));

// Close gripper (grasp)
task->add(std::make_unique<stages::ModifyPlanningScene>("grasp", ...));

// Lift object
task->add(std::make_unique<stages::MoveRelative>("lift", ...));

// Move to place location
task->add(std::make_unique<stages::MoveTo>("place_location", ...));

// Release object
task->add(std::make_unique<stages::ModifyPlanningScene>("release", ...));

// Plan
task->plan();

// Execute
task->execute();
```

MTC provides composable, reusable manipulation primitives.

**Quiz: MoveIt2**

What is the purpose of the SRDF file in MoveIt2?

[( )] It defines the robot's physical geometry
[(X)] It provides semantic information like planning groups and named poses
[( )] It configures the ROS2 communication interfaces
[( )] It stores trajectory data
***
<div>
SRDF (Semantic Robot Description Format) augments URDF with planning-specific information such as kinematic groups, named configurations, collision matrix, and end effector definitions.
</div>
***

---

## Summary and Key Takeaways

This course established motion planning foundations for robotic manipulation:

1. **Planning Theory**: Configuration space representation enables geometric reasoning about collision-free motion

2. **Computational Complexity**: Motion planning is PSPACE-hard; practical algorithms trade completeness for efficiency

3. **Sampling-Based Methods**: PRM and RRT provide probabilistically complete planning for high-dimensional spaces

4. **Collision Detection**: Efficient geometric algorithms enable real-time collision checking

5. **MoveIt2 Framework**: Industry-standard motion planning with extensive algorithmic and integration capabilities

6. **Programmatic Control**: C++ and Python APIs enable application-specific motion planning

These concepts apply to manipulation, mobile navigation, and multi-robot coordination.

---

## Optional Laboratory Exercises

### Laboratory 1: Configuration Space Visualization

**Duration**: 90 minutes

**Objective**: Understand C-space by visualizing 2D and 3D examples.

<!-- class="optional-exercise" -->
**Exercise 1.1: 2-DOF Planar Arm C-Space**

Visualize C-space for 2-link planar arm with obstacles:

Tasks:
1. Implement forward kinematics for 2-link arm
2. Discretize joint space (θ₁, θ₂) into grid
3. For each grid cell, check if configuration collides
4. Plot C-space: free (white) vs. obstacle (black)
5. Find path from start to goal in C-space
6. Visualize corresponding workspace motion

**Exercise 1.2: SE(2) Robot C-Space**

Explore 3-DOF C-space (x, y, θ):

Tasks:
1. Create 2D environment with polygonal obstacles
2. For robot at each (x, y, θ), check collision
3. Visualize 3D C-space (slice at various θ)
4. Implement sampling-based search in C-space

Deliverable: C-space visualizations demonstrating obstacle mapping

---

### Laboratory 2: Implementing RRT

**Duration**: 150 minutes

**Objective**: Implement RRT planner from scratch.

<!-- class="optional-exercise" -->
**Exercise 2.1: Basic RRT**

Tasks:
1. Implement nearest neighbor search (k-d tree optional)
2. Implement extend function with step size δ
3. Implement collision checker (simple geometric primitives)
4. Implement main RRT loop
5. Visualize tree growth in real-time
6. Test on 2D planning problems

**Exercise 2.2: RRT-Connect**

Extend to bi-directional variant:

Tasks:
1. Maintain two trees (start and goal)
2. Implement tree connection attempt
3. Alternate growth between trees
4. Compare convergence to basic RRT

**Exercise 2.3: Performance Analysis**

Tasks:
1. Vary parameters (step size, iterations, environment complexity)
2. Measure: success rate, planning time, path quality
3. Plot performance characteristics
4. Compare RRT vs. RRT-Connect

Deliverable: Working RRT implementation with performance analysis

<!-- class="exercise-advanced" -->
**Advanced challenge**: Implement RRT* with rewiring and cost propagation. Compare solution quality and computational cost to RRT.

---

### Laboratory 3: MoveIt2 Configuration and Planning

**Duration**: 120 minutes

**Objective**: Configure MoveIt2 for custom robot and execute motion plans.

<!-- class="optional-exercise" -->
**Exercise 3.1: MoveIt Setup Assistant**

Tasks:
1. Load URDF for robot arm
2. Generate self-collision matrix
3. Define planning group "arm"
4. Add named poses ("home", "ready", "stow")
5. Configure KDL or TRAC-IK solver
6. Generate configuration package
7. Test in RViz demo mode

**Exercise 3.2: Interactive Planning**

Using RViz MoveIt plugin:

Tasks:
1. Move interactive marker to target pose
2. Plan and execute motion
3. Add collision objects (box, cylinder, mesh)
4. Plan around obstacles
5. Modify planner parameters (RRT, RRT-Connect, PRM)
6. Compare planning time and path quality

**Exercise 3.3: Multi-Goal Planning**

Tasks:
1. Define sequence of waypoint poses
2. Plan and execute motion through waypoints
3. Test cartesian path planning (straight-line end-effector motion)
4. Handle planning failures (unreachable goals)

Deliverable: Configured MoveIt2 package with demonstrated multi-goal planning

---

### Laboratory 4: MoveIt2 C++ Programming

**Duration**: 120 minutes

**Objective**: Develop motion planning applications using MoveIt2 API.

<!-- class="optional-exercise" -->
**Exercise 4.1: Programmatic Goal Setting**

Create C++ node:

Tasks:
1. Initialize MoveGroupInterface
2. Set pose targets programmatically
3. Plan and execute motion
4. Add logging and error handling
5. Implement named target calls

**Exercise 4.2: Collision Object Management**

Tasks:
1. Create PlanningSceneInterface
2. Add primitive collision objects (box, sphere, cylinder)
3. Add mesh collision objects
4. Attach/detach objects to end-effector (grasping simulation)
5. Remove objects from scene

**Exercise 4.3: Draw Square Application**

Implement application that draws square in space:

Tasks:
1. Define four corner poses
2. Add virtual wall obstacle
3. Plan path through corners avoiding wall
4. Execute trajectory
5. Visualize in RViz

Requirements:
- Square corners at specified coordinates
- Wall intersects direct path between some corners
- Successfully plan and execute despite obstacle

Deliverable: C++ application demonstrating programmatic MoveIt2 control with obstacles

---

## Further Study and Resources

### Recommended Reading

1. **LaValle, S.**: *Planning Algorithms* (comprehensive motion planning textbook, free online)
2. **Choset et al.**: *Principles of Robot Motion* (motion planning theory and algorithms)
3. **Sucan, Chitta**: MoveIt documentation and tutorials

### Online Resources

- **OMPL documentation**: Algorithm descriptions and parameters
- **MoveIt2 tutorials**: Step-by-step configuration and programming guides
- **The Construct**: ROS2 and MoveIt2 online courses

### Next Steps

- **SOFTWARE-204**: System integration capstone project
- **Advanced Topics**: Optimal control, trajectory optimization, learning-based planning
- **Specialized Domains**: Mobile manipulation, dual-arm coordination

---

**Course developed by Robot Campus Team**
Version 2.0.0 | Last updated: 2024
