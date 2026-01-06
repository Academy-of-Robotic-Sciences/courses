<!--
author:   Robot Campus Team
email:    contact@academy-of-robotic-sciences.github.io
version:  2.0.0
language: en
narrator: US English Female

comment:  Autonomous System Integration: A capstone project course integrating software architecture, distributed systems, motion planning, and control into a complete autonomous manipulation system. Students design, implement, and deploy pick-and-place applications.


mode:     Textbook

link:     https://raw.githubusercontent.com/Academy-of-Robotic-Sciences/courses/main/course-styles.css

-->

# SOFTWARE-204: Autonomous System Integration

--{{0}}--
This capstone course synthesizes knowledge from the Software Engineering specialization into a complete autonomous robotic system. Students apply software architecture principles, distributed system design, motion planning algorithms, and system integration methodologies to implement, test, and deploy manipulation applications.

**Course Code**: SOFTWARE-204
**Duration**: 8 hours
**Level**: Capstone / Advanced
**Prerequisites**: SOFTWARE-201, SOFTWARE-202, SOFTWARE-203 (complete Software Track)

## Learning Objectives

By completing this course, students will:

1. Design system architectures for autonomous manipulation tasks
2. Implement state machines for multi-stage task execution
3. Integrate perception, planning, and control subsystems
4. Apply software engineering best practices to complex robotic applications
5. Debug and optimize distributed robotic systems
6. Demonstrate autonomous task execution through system validation

---

## Module 1: System Architecture and Design

--{{0}}--
This module addresses architectural design principles for integrated autonomous systems.

### 1.1 System Architecture Principles

<!-- class="theory-concept" -->
**Layered Architecture for Robotic Systems**

Autonomous robots employ hierarchical architectures separating concerns:

**Three-layer architecture** (Gat, 1998):

**Layer 1 - Reactive/Control**:
- Fast feedback loops (1-100 Hz)
- Low-level motor control
- Sensor processing
- Emergency stops and safety monitoring

**Layer 2 - Executive/Sequencing**:
- Task decomposition and sequencing
- State machine execution
- Resource allocation
- Moderate update rate (0.1-10 Hz)

**Layer 3 - Deliberative/Planning**:
- High-level reasoning and planning
- Goal generation
- Learning and adaptation
- Slow update rate (0.01-1 Hz)

**Communication**:
- Upward: Sensor data, task completion status
- Downward: Commands, goals, parameters

**Advantages**:
- Separation of time scales (fast control, slow planning)
- Modular development and testing
- Clear interfaces between layers

<!-- class="theory-concept" -->
**State Machine Design**

State machines formalize task sequencing and control flow:

**Finite State Machine (FSM)** components:
- **States**: Discrete task phases (IDLE, APPROACHING, GRASPING, LIFTING, etc.)
- **Transitions**: Conditions triggering state changes
- **Actions**: Operations executed in each state
- **Guards**: Preconditions for transitions

**Example: Pick-and-place FSM**:
```
States: {IDLE, MOVING_TO_APPROACH, APPROACHING, GRASPING,
         LIFTING, MOVING_TO_PLACE, PLACING, RELEASING, RETURNING}

Transitions:
  IDLE → MOVING_TO_APPROACH: object_detected
  MOVING_TO_APPROACH → APPROACHING: reached_approach_pose
  APPROACHING → GRASPING: reached_grasp_pose
  GRASPING → LIFTING: gripper_closed
  LIFTING → MOVING_TO_PLACE: reached_lift_height
  MOVING_TO_PLACE → PLACING: reached_place_pose
  PLACING → RELEASING: reached_release_height
  RELEASING → RETURNING: gripper_opened
  RETURNING → IDLE: reached_home
  ANY → IDLE: error_occurred (abort)
```

**Implementation patterns**:
- **Switch statement**: Simple, explicit state handling
- **State pattern** (OOP): State objects with enter/execute/exit methods
- **Hierarchical state machines**: Nested states for complex behaviors
- **State chart libraries**: SMACH (ROS), BehaviorTree.CPP

**Design considerations**:
- **Completeness**: Handle all possible transitions
- **Error handling**: Recovery states, timeout transitions
- **Modularity**: Reusable state implementations
- **Observability**: Logging and monitoring of state transitions

<!-- class="theory-concept" -->
**Component Integration Patterns**

**Perception-Planning-Control loop**:

```
Perception → World Model → Task Planning → Motion Planning → Control → Actuation
     ↑                                                                     ↓
     └─────────────────── Feedback (sensors) ──────────────────────────────┘
```

**Data flow**:
1. **Perception**: Sensors → Object detection → 3D poses
2. **World model**: Fuse sensor data, maintain environment representation
3. **Task planning**: Determine action sequence to achieve goal
4. **Motion planning**: Compute collision-free trajectories
5. **Control**: Execute trajectories, handle disturbances
6. **Actuation**: Command motors and effectors

**Integration challenges**:
- **Asynchrony**: Components operate at different rates
- **Latency**: Sensor-to-actuator delays
- **Uncertainty**: Noisy sensors, imperfect models
- **Coordination**: Synchronize multiple subsystems

**Architectural patterns**:
- **Blackboard**: Shared data structure (planning scene in MoveIt)
- **Observer**: Publish-subscribe for data distribution
- **Command**: Encapsulate requests as objects (action goals)

<!-- class="historical-note" -->
**Evolution of Robot Architectures**

Robot architectures evolved from reactive to deliberative to hybrid:

- **1980s**: Reactive architectures (subsumption, Brooks)
  - Fast, robust, but limited reasoning
- **1980s**: Deliberative architectures (Shakey robot)
  - Planning-heavy, slow, brittle
- **1990s**: Hybrid architectures (3T, TCA)
  - Combine reactive and deliberative layers
- **2000s**: Behavior trees and task networks
  - Modular, composable task representation
- **2010s**: Integrated frameworks (ROS + MoveIt + BT)
  - Standardized interfaces and reusable components

Modern systems leverage decades of architectural evolution.

### 1.2 Requirements Analysis and Specification

<!-- class="theory-concept" -->
**Functional Requirements**

Define what the system must accomplish:

**Pick-and-place task specification**:
- **Input**: Object pose from perception system
- **Output**: Object relocated to target zone
- **Constraints**:
  - Collision-free motion
  - Stable grasp
  - Execution time < T_max
  - Success rate > R_min

**Subtask decomposition**:
1. Perceive object (subscribe to camera topic)
2. Plan approach trajectory (call MoveIt)
3. Execute approach (monitor feedback)
4. Close gripper (action call)
5. Verify grasp (force/tactile sensing)
6. Plan transport trajectory
7. Execute transport
8. Open gripper
9. Verify release
10. Return to home

**Edge cases**:
- Object not detected → timeout, retry
- Planning failure → adjust approach angle
- Grasp failure → re-grasp attempt
- Collision detected → emergency stop

<!-- class="theory-concept" -->
**Non-Functional Requirements**

Define system quality attributes:

**Performance**:
- Cycle time: Complete task in < X seconds
- Throughput: Y picks per hour
- Planning time: < Z seconds per plan

**Reliability**:
- Success rate: > 95% under nominal conditions
- Fault tolerance: Graceful degradation on subsystem failure
- Recovery: Automatic retry on recoverable errors

**Safety**:
- Collision avoidance: No contact with obstacles
- Emergency stop: < 100ms response time
- Workspace limits: Constrain robot motion

**Maintainability**:
- Modular architecture: Replaceable components
- Logging: Comprehensive state and error logs
- Configuration: Runtime parameter adjustment

**Real-time constraints**:
- Control loop: 100 Hz (10ms period)
- Planning: Best-effort with timeout
- Perception: 30 Hz camera processing

<!-- class="alternative-approach" -->
**Behavior Trees vs. State Machines**

Alternative to FSM for task execution:

**Behavior Tree (BT)** structure:
- **Leaf nodes**: Actions, conditions
- **Control nodes**: Sequence, fallback, parallel
- **Execution**: Tick-based traversal

**Example pick-and-place BT**:
```
Fallback (retry on failure)
├── Sequence (main task)
│   ├── Condition: object_detected
│   ├── Action: plan_approach
│   ├── Action: execute_approach
│   ├── Action: grasp_object
│   ├── Action: plan_place
│   ├── Action: execute_place
│   └── Action: release_object
└── Action: abort_and_return_home
```

**Advantages over FSM**:
- Hierarchical composition
- Reusable subtrees
- More intuitive for complex logic

**Disadvantages**:
- Less explicit than FSM for simple sequences
- Requires BT execution engine

**Use cases**:
- Complex multi-branch behaviors
- Game AI (originated in gaming)
- Humanoid robot task execution

**Quiz: System Architecture**

In a three-layer robot architecture, which layer is responsible for high-level task planning?

[( )] Reactive/Control layer
[( )] Executive/Sequencing layer
[(X)] Deliberative/Planning layer
[( )] Communication layer
***
<div>
The Deliberative/Planning layer handles high-level reasoning, goal generation, and long-term planning. It operates at slow update rates and generates goals for lower layers.
</div>
***

What is the primary advantage of using a state machine for task sequencing?

[(X)] Explicit representation of task phases and transitions
[( )] Faster execution than other methods
[( )] Requires less code
[( )] Automatically handles all errors
***
<div>
State machines provide explicit, formal representation of discrete task phases and the conditions governing transitions between them, making task logic clear and verifiable.
</div>
***

---

## Module 2: Implementation Strategies

--{{0}}--
This module covers practical implementation techniques for autonomous manipulation systems.

### 2.1 Perception Integration

<!-- class="theory-concept" -->
**Object Pose Estimation**

Perception provides object locations for manipulation:

**Vision pipeline**:
1. **Acquisition**: Camera image (RGB, depth, or RGB-D)
2. **Preprocessing**: Filtering, segmentation
3. **Detection**: Locate objects in image
4. **Pose estimation**: Determine 3D position and orientation
5. **Publishing**: Broadcast poses via ROS2

**ROS2 integration**:
```python
class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception')
        self.camera_sub = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/object_pose', 10)

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        image = self.bridge.imgmsg_to_cv2(msg)

        # Object detection (simplified)
        detected_objects = self.detect_objects(image)

        # Publish poses
        for obj in detected_objects:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = 'camera_optical_frame'
            pose_msg.pose = obj.pose
            self.pose_pub.publish(pose_msg)
```

**Coordinate frame transformations**:
- Object detected in camera frame
- Transform to robot base frame for planning
- Use tf2 for automatic transformation

```python
# Transform pose to base frame
transform = self.tf_buffer.lookup_transform(
    'base_link', 'camera_optical_frame', rclpy.time.Time())
pose_in_base = tf2_geometry_msgs.do_transform_pose(
    detected_pose, transform)
```

**Handling perception uncertainty**:
- Multiple measurements: Average or filter
- Outlier rejection: RANSAC, statistical filters
- Confidence thresholds: Discard low-confidence detections

### 2.2 Grasp Planning and Execution

<!-- class="theory-concept" -->
**Grasp Pose Generation**

Determine gripper pose to securely grasp object:

**Antipodal grasp model** (parallel-jaw gripper):
- **Contact points**: Two points on object surface
- **Force closure**: Grasp resists arbitrary disturbances
- **Approach direction**: Perpendicular to grasp plane

**Grasp pose calculation**:
```cpp
geometry_msgs::msg::Pose calculate_grasp_pose(
    const geometry_msgs::msg::Pose& object_pose,
    double grasp_height_offset,
    double approach_distance)
{
    geometry_msgs::msg::Pose grasp_pose = object_pose;

    // Offset vertically (assume top grasp)
    grasp_pose.position.z += grasp_height_offset;

    // Gripper orientation (downward-facing)
    grasp_pose.orientation = create_quaternion(M_PI, 0, 0); // Roll 180°

    return grasp_pose;
}

geometry_msgs::msg::Pose calculate_pre_grasp_pose(
    const geometry_msgs::msg::Pose& grasp_pose,
    double approach_distance)
{
    geometry_msgs::msg::Pose pre_grasp = grasp_pose;
    pre_grasp.position.z += approach_distance; // Approach from above
    return pre_grasp;
}
```

**Grasp execution sequence**:
1. **Pre-grasp**: Move to approach pose (open gripper)
2. **Approach**: Linear motion to grasp pose
3. **Close gripper**: Activate gripper closing
4. **Lift**: Move vertically to clear object
5. **Verify**: Check grasp success (force sensor, gripper position)

**Grasp failure recovery**:
- **No contact detected**: Retry with pose adjustment
- **Weak grasp**: Increase gripper force
- **Collision during approach**: Replan with different approach angle

### 2.3 Motion Planning Integration

<!-- class="theory-concept" -->
**Multi-Stage Motion Planning**

Pick-and-place requires several motion plans:

**Planning stages**:
1. **Home → Pre-grasp**: Free-space motion
2. **Pre-grasp → Grasp**: Constrained linear approach
3. **Grasp → Lifted**: Vertical retreat
4. **Lifted → Pre-place**: Free-space motion
5. **Pre-place → Place**: Constrained descent
6. **Place → Home**: Free-space return

**Implementation with MoveIt**:
```cpp
class PickAndPlaceExecutor {
public:
    bool execute(const geometry_msgs::msg::Pose& object_pose,
                 const geometry_msgs::msg::Pose& place_pose) {
        // Calculate intermediate poses
        auto grasp_pose = calculate_grasp_pose(object_pose);
        auto pre_grasp_pose = calculate_pre_grasp_pose(grasp_pose);
        auto lift_pose = calculate_lift_pose(grasp_pose);
        auto pre_place_pose = calculate_pre_place_pose(place_pose);

        // Execute sequence
        if (!move_to_pose(pre_grasp_pose)) return false;
        if (!linear_move(grasp_pose)) return false;
        if (!close_gripper()) return false;
        if (!linear_move(lift_pose)) return false;
        if (!move_to_pose(pre_place_pose)) return false;
        if (!linear_move(place_pose)) return false;
        if (!open_gripper()) return false;
        if (!move_to_pose(home_pose_)) return false;

        return true;
    }

private:
    bool move_to_pose(const geometry_msgs::msg::Pose& target) {
        move_group_->setPoseTarget(target);
        auto result = move_group_->move();
        return result == moveit::core::MoveItErrorCode::SUCCESS;
    }

    bool linear_move(const geometry_msgs::msg::Pose& target) {
        std::vector<geometry_msgs::msg::Pose> waypoints = {target};
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group_->computeCartesianPath(
            waypoints, 0.01, 1.4, trajectory);

        if (fraction > 0.95) { // 95% of path planned
            move_group_->execute(trajectory);
            return true;
        }
        return false;
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};
```

**Error handling**:
- **Planning failure**: Retry with random seed, adjust target
- **Execution failure**: Re-plan from current configuration
- **Timeout**: Abort and report failure

<!-- class="alternative-approach" -->
**MoveIt Task Constructor (MTC) Approach**

MTC provides higher-level abstraction for pick-and-place:

**Advantages**:
- Declarative task specification
- Automatic handling of grasp/place logic
- Built-in error recovery

**Trade-off**:
- Steeper learning curve
- Less control over individual steps
- Heavier computational requirements

For learning and prototyping, explicit sequencing offers clearer understanding. For production, MTC provides robustness.

**Quiz: Implementation**

What is the purpose of the "pre-grasp" pose in a pick-and-place task?

[( )] It is where the gripper closes
[(X)] It is an approach pose offset from the grasp pose
[( )] It is the home position of the robot
[( )] It is where the object is placed
***
<div>
The pre-grasp pose is an approach position offset from the actual grasp pose (typically above or to the side). It allows the robot to plan a free-space motion to a safe location, then perform a constrained linear approach to the grasp.
</div>
***

---

## Module 3: System Validation and Debugging

--{{0}}--
This module addresses testing, debugging, and validation strategies for complex robotic systems.

### 3.1 Testing Strategies

<!-- class="theory-concept" -->
**Hierarchical Testing**

Test system at multiple levels:

**Unit testing**: Individual components
- Test perception: Mock camera data → verify pose output
- Test planning: Known configurations → verify path validity
- Test gripper: Command close → verify position feedback
- Use ROS2 test framework (launch_testing)

**Integration testing**: Component interactions
- Perception → Planning: Object pose → motion plan
- Planning → Control: Trajectory → execution success
- State machine: State transitions under various conditions

**System testing**: End-to-end validation
- Full pick-and-place cycle
- Performance under nominal conditions
- Stress testing: Edge cases, failures

**Acceptance testing**: User requirements
- Success rate over N trials
- Cycle time measurements
- Safety validation (no collisions)

**Simulation-based testing**:
- Gazebo: Physics-based simulation
- Mock nodes: Simulate hardware without robot
- Faster iteration, reproducible conditions

### 3.2 Debugging Distributed Systems

<!-- class="theory-concept" -->
**ROS2 Debugging Tools**

**Command-line tools**:
```bash
# Monitor topics
ros2 topic list
ros2 topic echo /object_pose
ros2 topic hz /joint_states  # Measure publication rate

# Inspect nodes
ros2 node list
ros2 node info /perception_node

# Services and actions
ros2 service list
ros2 action list

# TF tree inspection
ros2 run tf2_tools view_frames  # Generate PDF of transform tree
ros2 run tf2_ros tf2_echo base_link end_effector
```

**Visualization tools**:
- **RViz**: 3D visualization of robot, environment, TF frames
- **rqt_graph**: Node and topic connectivity
- **rqt_plot**: Real-time data plotting
- **rqt_console**: Log message filtering and viewing

**Logging best practices**:
```cpp
// Appropriate log levels
RCLCPP_DEBUG(logger, "Entering state: APPROACHING");
RCLCPP_INFO(logger, "Grasp successful");
RCLCPP_WARN(logger, "Planning took longer than expected: %.2fs", time);
RCLCPP_ERROR(logger, "Failed to connect to gripper action server");
RCLCPP_FATAL(logger, "Emergency stop triggered");
```

**Common issues**:
- **No communication**: Check topic names, QoS compatibility
- **Slow performance**: Profile computational bottlenecks
- **Frame errors**: Verify TF tree, check timestamp synchronization
- **Action timeout**: Increase timeout, check server responsiveness

### 3.3 Performance Optimization

<!-- class="theory-concept" -->
**Profiling and Optimization**

Identify and address performance bottlenecks:

**Profiling tools**:
- **ros2_tracing**: Low-overhead tracing of ROS2 execution
- **perf**: Linux performance profiler
- **valgrind/callgrind**: Call graph and cache profiling
- **Custom timers**: Measure specific operations

**Common bottlenecks**:
- **Collision checking**: Use simplified geometry, BVH acceleration
- **Planning**: Set planning time limits, use faster planners
- **Perception**: Reduce image resolution, optimize algorithms
- **Serialization**: Use zero-copy for large messages (intra-process)

**Optimization strategies**:
- **Precomputation**: Roadmaps, IK solutions
- **Caching**: Reuse previous plans if start/goal similar
- **Parallelization**: Multi-threaded executors for concurrent callbacks
- **Algorithmic**: Choose appropriate algorithms (RRT-Connect vs. PRM)

**Example: Planning time optimization**:
```cpp
// Before: Default settings, slow
move_group.setPlanningTime(5.0);  // 5 seconds
move_group.setPlannerId("RRTstar");  // Optimal but slow

// After: Optimized for speed
move_group.setPlanningTime(1.0);  // 1 second limit
move_group.setPlannerId("RRTConnect");  // Fast, probabilistically complete
move_group.setNumPlanningAttempts(3);  // Multiple tries
```

**Quiz: Validation and Debugging**

What ROS2 command-line tool would you use to verify that two topics are communicating at the expected rate?

[( )] ros2 topic echo
[(X)] ros2 topic hz
[( )] ros2 node info
[( )] rqt_graph
***
<div>
`ros2 topic hz <topic_name>` measures the publication rate (frequency) of messages on a topic, allowing verification that communication occurs at the expected rate.
</div>
***

---

## Summary and Key Takeaways

This capstone course synthesized Software Engineering specialization knowledge:

1. **System Architecture**: Layered architectures and state machines organize complex autonomous systems

2. **Integration**: Perception, planning, and control subsystems compose complete applications

3. **Implementation**: Practical strategies for grasp planning, motion execution, and error handling

4. **Validation**: Hierarchical testing and debugging ensure system correctness and performance

5. **Professional Practice**: Logging, profiling, and optimization maintain code quality

6. **Autonomous Execution**: End-to-end systems demonstrate complete task capability

Students completing this specialization possess professional robotics software engineering skills applicable to industrial manipulation, mobile robotics, and autonomous systems.

---

## Capstone Project: Autonomous Pick-and-Place System

**Duration**: 8 hours (self-directed with instructor support)

**Objective**: Design, implement, and demonstrate a complete autonomous pick-and-place application integrating all Software Track knowledge.

<!-- class="optional-exercise" -->
### Project Specification

**System Requirements**:

**Inputs**:
- Object pose published on `/object_pose` topic (geometry_msgs/PoseStamped)
- Placement zone location (parameter or fixed)

**Outputs**:
- Autonomous pick-and-place execution
- Success/failure reporting
- State transition logging

**Constraints**:
- Collision-free motion planning
- Grasp verification before transport
- Execution time < 60 seconds
- Success demonstration (2 successful runs)

### Phase 1: Architecture and Design (60 minutes)

<!-- class="exercise-tip" -->
**Design Activities**:

1. **System architecture diagram**:
   - All nodes (perception simulator, your application, MoveIt, hardware interface)
   - Topics, services, actions used
   - Data flow through system

2. **State machine design**:
   - Enumerate all states
   - Define transitions and guards
   - Identify error states and recovery

3. **Interface specification**:
   - Subscribed topics and message types
   - Action servers used (MoveGroup, Gripper)
   - Parameters for configuration

4. **ROS2 package setup**:
   - Create package with dependencies
   - Configure CMakeLists.txt or setup.py
   - Set up launch files

Deliverable: Architecture document and configured package

### Phase 2: Core Application Implementation (180 minutes)

<!-- class="optional-exercise" -->
**Implementation Tasks**:

**Task 2.1: State Machine Skeleton**

Create C++ node with state enumeration:
```cpp
enum class State {
    IDLE,
    MOVING_TO_PRE_GRASP,
    APPROACHING,
    GRASPING,
    LIFTING,
    MOVING_TO_PLACE,
    PLACING,
    RELEASING,
    RETURNING,
    ERROR
};

class PickAndPlaceNode : public rclcpp::Node {
public:
    PickAndPlaceNode() : Node("pick_and_place"), state_(State::IDLE) {
        // Initialize subscribers, action clients, timers
    }

    void execute() {
        switch (state_) {
            case State::IDLE:
                // Wait for object detection
                break;
            case State::MOVING_TO_PRE_GRASP:
                // Plan and execute to pre-grasp
                break;
            // ... other states
        }
    }

private:
    State state_;
    // ... member variables
};
```

**Task 2.2: Perception Subscriber**

Subscribe to object pose:
```cpp
void object_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (state_ == State::IDLE) {
        target_object_pose_ = *msg;
        state_ = State::MOVING_TO_PRE_GRASP;
        RCLCPP_INFO(get_logger(), "Object detected, starting pick-and-place");
    }
}
```

**Task 2.3: MoveIt Integration**

Initialize MoveGroupInterface:
```cpp
move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "arm");
move_group_->setPlanningTime(2.0);
move_group_->setNumPlanningAttempts(5);
```

Implement motion functions (refer to Module 2.3 examples)

**Task 2.4: Gripper Control**

Create action client for gripper:
```cpp
using GripperAction = control_msgs::action::GripperCommand;

gripper_client_ = rclcpp_action::create_client<GripperAction>(
    this, "/gripper_controller/gripper_cmd");

void close_gripper() {
    auto goal = GripperAction::Goal();
    goal.command.position = 0.0;  // Closed
    goal.command.max_effort = 10.0;
    gripper_client_->async_send_goal(goal);
}
```

Deliverable: Functional application node with state machine implementation

### Phase 3: Integration and Testing (120 minutes)

<!-- class="optional-exercise" -->
**Integration Activities**:

**Task 3.1: Launch File Creation**

Create launch file starting all components:
```python
def generate_launch_description():
    return LaunchDescription([
        # MoveIt
        IncludeLaunchDescription(...),

        # Your application
        Node(
            package='pick_and_place_app',
            executable='pick_and_place_node',
            parameters=[{'place_zone_x': 0.5, 'place_zone_y': 0.0}]
        ),

        # Perception simulator (provided)
        Node(package='perception_sim', executable='object_publisher'),

        # RViz
        Node(package='rviz2', executable='rviz2', arguments=[...]),
    ])
```

**Task 3.2: Unit Testing**

Test individual functions:
```python
def test_grasp_pose_calculation():
    object_pose = Pose(...)
    grasp_pose = calculate_grasp_pose(object_pose, offset=0.1)
    assert grasp_pose.position.z == object_pose.position.z + 0.1
```

**Task 3.3: Integration Testing**

Test with mock object publisher:
- Verify state transitions
- Check motion execution
- Validate error handling

**Task 3.4: Debugging and Refinement**

Common issues to address:
- Planning failures: Adjust pose offsets, increase planning time
- Grasp failures: Tune grasp parameters (height, gripper force)
- Timing issues: Add delays, check action completion
- Frame errors: Verify TF tree, coordinate transformations

Deliverable: Integrated system passing unit and integration tests

### Phase 4: Demonstration and Validation (60 minutes)

<!-- class="optional-exercise" -->
**Demonstration Requirements**:

**Setup**:
1. Launch complete system
2. Load MoveIt configuration in RViz
3. Ensure perception simulator running

**Execution**:
1. Run launch file
2. Perception publishes random object pose
3. System autonomously completes pick-and-place
4. Logs show state transitions
5. RViz visualizes motion

**Success Criteria**:
- 2 successful pick-and-place cycles
- No collisions (verified in RViz)
- Execution time < 60s per cycle
- Proper error handling if planning fails

**Code Review**:
- Architecture clarity: Clear separation of concerns
- Code quality: Proper logging, error handling, documentation
- ROS2 best practices: Appropriate use of topics/services/actions

Deliverable: Video recording of demonstration + code repository

<!-- class="exercise-advanced" -->
**Advanced Extensions**:

1. **Multiple Objects**: Handle sequence of picks
2. **Perception Integration**: Replace mock with real camera + object detection
3. **Grasp Planning**: Implement multiple grasp candidates
4. **Recovery Behaviors**: Retry with pose perturbations on failure
5. **Performance Optimization**: Minimize cycle time through caching, parallel planning

---

## Further Study and Resources

### Recommended Reading

1. **Correll et al.**: *Introduction to Autonomous Robots* (system integration)
2. **Craig, J.**: *Introduction to Robotics: Mechanics and Control* (manipulation fundamentals)
3. **Siciliano, Khatib**: *Springer Handbook of Robotics* (comprehensive reference)

### Online Resources

- **MoveIt Tutorials**: Pick-and-place examples
- **ROS2 documentation**: Best practices and patterns
- **ROS Discourse**: Community Q&A
- **GitHub**: Open-source manipulation projects

### Career Pathways

**Positions**:
- Robotics Software Engineer
- Automation Engineer
- ROS Developer
- Perception Engineer
- Motion Planning Engineer

**Next Steps**:
- **AI Track**: Add machine learning for adaptive grasping and planning
- **Systems Track**: Real-time control and embedded systems
- **Open Source**: Contribute to MoveIt, navigation, manipulation packages
- **Research**: Advanced manipulation, multi-robot systems, human-robot interaction

---

**Course developed by Robot Campus Team**
Version 2.0.0 | Last updated: 2024
