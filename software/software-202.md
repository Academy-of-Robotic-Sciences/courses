<!--
author:   Robot Campus Team
email:    hello@robotcampus.dev
version:  2.0.0
language: en
narrator: US English Female

comment:  Distributed Robotics Systems with ROS2: A comprehensive course on the Robot Operating System 2 framework, covering middleware architecture, communication patterns, distributed system design, and robotic application development.

logo:     https://robotcampus.dev/logo.png

mode:     Textbook

link:     https://robotcampus.dev/styles/course-styles.css

-->

# SOFTWARE-202: Distributed Robotics Systems with ROS2

--{{0}}--
This course provides comprehensive education in distributed systems architecture for robotics using ROS2 (Robot Operating System 2). Students develop understanding of middleware design patterns, inter-process communication, and scalable software architecture for complex robotic applications.

**Course Code**: SOFTWARE-202
**Duration**: 8 hours
**Level**: Intermediate to Advanced
**Prerequisites**: SOFTWARE-201 (Software Architecture for Robot Control)

## Learning Objectives

By completing this course, students will:

1. Understand distributed systems architecture and middleware design patterns for robotics
2. Implement ROS2 nodes, topics, services, and actions for inter-process communication
3. Design modular, scalable robotics software using publish-subscribe and request-response patterns
4. Develop kinematic solvers as distributed services within ROS2 framework
5. Configure and deploy multi-node systems using launch files and parameter servers
6. Apply quality-of-service policies for reliable real-time communication

---

## Module 1: Distributed Systems and Middleware Architecture

--{{0}}--
This module establishes theoretical foundations of distributed computing and middleware design as applied to robotics.

### 1.1 Distributed Systems Fundamentals

<!-- class="theory-concept" -->
**Distributed System Characteristics**

A distributed system consists of multiple autonomous computational entities that cooperate to achieve a common goal. Key characteristics:

**Concurrency**: Multiple processes execute simultaneously
- Parallel sensor processing
- Concurrent control loops at different rates
- Simultaneous planning and execution

**No global clock**: Processes lack shared time reference
- Timestamp synchronization challenges
- Causality tracking across nodes
- Event ordering ambiguities

**Independent failures**: Components fail independently
- Partial system degradation vs. total failure
- Fault detection and isolation
- Graceful degradation strategies

**Message passing**: Communication via explicit messages (no shared memory)
- Network latency and bandwidth constraints
- Message loss and reordering
- Asynchronous communication patterns

<!-- class="theory-concept" -->
**Advantages of Distributed Robotics Architecture**

**Modularity**: Independent development and testing of components
- Separate nodes for perception, planning, control
- Team parallelization of development
- Isolated debugging and profiling

**Scalability**: Add computational resources as needed
- Distribute CPU-intensive tasks across cores/machines
- Horizontal scaling for increased capability
- Load balancing across processors

**Fault tolerance**: System continues despite individual failures
- Redundant nodes for critical functions
- Automatic failover to backup systems
- Isolated failure domains

**Heterogeneity**: Different languages, platforms, and execution rates
- C++ control loops + Python planning
- Real-time and non-real-time processes
- Hardware-specific and portable components

**Reusability**: Standard interfaces enable component reuse
- Community-contributed packages
- Hardware abstraction layers
- Algorithm libraries (SLAM, navigation, manipulation)

<!-- class="theory-concept" -->
**Challenges of Distributed Systems**

**Complexity**: More moving parts increase system complexity
- Debugging across multiple processes
- Dependency management
- Configuration management

**Latency**: Network communication slower than function calls
- Message transmission delays (µs to ms)
- Serialization/deserialization overhead
- Variable latency affects control stability

**Consistency**: Maintaining coherent system state
- Distributed data synchronization
- Coordinate frame transformations
- Sensor fusion from asynchronous sources

**Partial failure**: Some components fail while others continue
- Detecting silent failures (no heartbeat)
- Handling timeout vs. slow response
- Recovery and restart strategies

<!-- class="historical-note" -->
**Evolution of Robotics Middleware**

Robotics middleware evolved to manage distributed system complexity:

- **1990s**: Ad-hoc custom middleware for specific robots
- **2001**: OROCOS (Open Robot Control Software) - component-based real-time framework
- **2007**: ROS (Robot Operating System) - publish-subscribe middleware gains widespread adoption
- **2010**: YARP (Yet Another Robot Platform) - focused on humanoid robotics
- **2017**: ROS2 introduced with DDS middleware and real-time support
- **2020s**: ROS2 becomes industry standard, replacing ROS1

ROS2 addresses ROS1 limitations: no native real-time support, security, or multi-robot systems.

### 1.2 Middleware Design Patterns

<!-- class="theory-concept" -->
**Publish-Subscribe Pattern**

Publishers produce data on topics; subscribers receive data from topics of interest.

**Characteristics**:
- Decoupling: Publishers don't know subscribers (and vice versa)
- Many-to-many: Multiple publishers and subscribers per topic
- Asynchronous: Non-blocking communication
- Bandwidth-efficient: Data sent only when published

**Advantages**:
- Flexible system reconfiguration
- Easy addition of new data consumers
- Implicit multicast to interested parties

**Disadvantages**:
- No guarantee of delivery or reception
- Difficult to detect missing consumers
- No request-response semantics

**Robotics applications**:
- Sensor data streams (joint states, camera images, laser scans)
- Command velocity to mobile base
- State estimation outputs (pose, odometry)

<!-- class="theory-concept" -->
**Request-Response Pattern (Services)**

Client sends request to server; server processes and returns response.

**Characteristics**:
- Synchronous: Client blocks until response received
- One-to-one: Single client per request
- Transient: No persistent state between calls
- Timeout: Client can detect non-responsive server

**Advantages**:
- Clear causality and ordering
- Explicit error handling
- Deterministic completion

**Disadvantages**:
- Blocking can delay other processing
- Single point of failure (server)
- Not suitable for streaming data

**Robotics applications**:
- Inverse kinematics computation
- Path planning requests
- Configuration parameter queries
- Trigger-based operations (calibration, reset)

<!-- class="theory-concept" -->
**Action Pattern**

Extended request-response for long-running, goal-oriented tasks with feedback and cancellation.

**Components**:
- **Goal**: Task specification sent by client
- **Feedback**: Periodic progress updates from server
- **Result**: Final outcome upon completion or failure
- **Cancel**: Client can abort ongoing task

**State machine**: Goals transition through states (accepted, executing, succeeded, aborted, canceled)

**Advantages**:
- Monitor progress of long operations
- Early termination if conditions change
- Suitable for complex task execution

**Disadvantages**:
- More complex than services
- Overhead of state management

**Robotics applications**:
- Navigation to goal pose (feedback: distance to goal)
- Trajectory execution (feedback: current waypoint)
- Grasping (feedback: approach phase, close gripper, lift)
- Multi-stage manipulation tasks

<!-- class="alternative-approach" -->
**Alternative Middleware Architectures**

**Component-based** (OROCOS, SmartSoft):
- Explicit component ports (inputs/outputs)
- Data-flow connections
- Enforced component interfaces
- More rigid but clearer dependencies

**Behavior-based** (Player/Stage):
- Subsumption architecture
- Layered reactive behaviors
- Simpler than complex planning
- Limited for high-DOF manipulation

**Blackboard architecture**:
- Shared knowledge base
- Multiple agents read/write
- Opportunistic problem-solving
- Coordination challenges at scale

ROS2's hybrid approach (topics + services + actions) provides flexibility for various architectural patterns.

**Quiz: Distributed Systems**

What is the primary advantage of the publish-subscribe pattern over direct function calls?

[( )] It is faster
[(X)] It decouples producers from consumers
[( )] It guarantees message delivery
[( )] It requires less code
***
<div>
Publish-subscribe decouples publishers from subscribers: publishers don't need to know who (if anyone) is consuming their data. This enables flexible system reconfiguration and easy addition of new components.
</div>
***

When should you use a ROS2 action instead of a service?

[( )] When you need the fastest possible response
[(X)] For long-running tasks that require progress feedback and cancellation
[( )] For simple computations
[( )] When you want to broadcast to multiple receivers
***
<div>
Actions are designed for long-running, goal-oriented tasks where you need periodic feedback on progress and the ability to cancel the operation before completion.
</div>
***

---

## Module 2: ROS2 Architecture and Core Concepts

--{{0}}--
This module examines ROS2 system architecture, communication infrastructure, and core abstractions.

### 2.1 ROS2 System Architecture

<!-- class="theory-concept" -->
**ROS2 Layered Architecture**

ROS2 employs a layered design separating concerns:

**Layer 1 - DDS (Data Distribution Service)**:
- OMG standard for distributed pub-sub
- Handles network communication and discovery
- Provides QoS (Quality of Service) policies
- Implementations: Fast-DDS (default), Cyclone DDS, Connext DDS

**Layer 2 - RMW (ROS Middleware Interface)**:
- Abstraction layer over different DDS implementations
- Allows swapping DDS vendors
- Standardizes ROS2 API

**Layer 3 - RCL (ROS Client Library)**:
- Core ROS2 functionality in C
- Node management, graph introspection
- Language-agnostic implementation

**Layer 4 - Language Bindings**:
- **rclcpp**: C++ client library (most feature-complete)
- **rclpy**: Python client library (easier rapid prototyping)
- **rclnodejs**, **rclada**, etc.: Other language bindings

**Layer 5 - Application Code**:
- User-written nodes and systems
- Builds on client libraries

<!-- class="theory-concept" -->
**ROS2 Computation Graph**

The computation graph represents runtime system topology:

**Nodes**: Processes that perform computation
- Minimal unit of executable code
- Can publish, subscribe, offer services, call actions
- Single responsibility principle: focused functionality

**Topics**: Named buses for message passing
- Typed data channels (sensor_msgs/Image, geometry_msgs/Twist)
- Many-to-many communication
- Quality of Service policies

**Services**: Synchronous request-response communication
- Typed request and response messages
- One server, many potential clients
- Blocking call semantics

**Actions**: Asynchronous goal-oriented tasks
- Goal, feedback, and result message types
- Non-blocking with cancellation
- State machine for goal lifecycle

**Parameters**: Configuration values for nodes
- Dynamic reconfiguration at runtime
- Type-checked (int, float, string, bool, arrays)
- Per-node namespaces

**Graph visualization**: Tools like `rqt_graph` display runtime topology

### 2.2 ROS2 Communication Infrastructure

<!-- class="theory-concept" -->
**Message Passing and Serialization**

ROS2 messages are strongly-typed data structures:

**Message definition** (.msg files):
```
# geometry_msgs/msg/Pose.msg
Point position      # x, y, z coordinates
Quaternion orientation  # Orientation as quaternion
```

**Generated code**: Message definitions compile to language-specific classes
- C++ struct with constructors, accessors
- Python dataclass-like objects
- Automatic serialization/deserialization

**Serialization formats**:
- **CDR (Common Data Representation)**: Default DDS serialization
- **Zero-copy**: Intra-process communication without serialization overhead
- **Compression**: Optional for bandwidth-limited networks

**Message design considerations**:
- Size vs. update rate tradeoff
- Fixed-size vs. variable-length fields
- Nesting and composition of message types
- Header timestamps for synchronization

<!-- class="theory-concept" -->
**Quality of Service (QoS) Policies**

QoS policies control communication reliability and performance:

**Reliability**:
- **Reliable**: Guarantee delivery (retransmission on loss) - for commands
- **Best effort**: No guarantees (lower latency) - for sensor streams

**Durability**:
- **Volatile**: New subscribers don't receive old messages
- **Transient local**: New subscribers receive last N messages (configurable depth)

**History**:
- **Keep last N**: Store N most recent messages in queue
- **Keep all**: Unlimited queue (until memory exhausted)

**Deadline**: Maximum time between messages (detect stale data)

**Lifespan**: Maximum age of messages (discard old data)

**Liveliness**: Detect dead publishers/subscribers

**Common QoS profiles**:
```cpp
// Sensor data: best effort, volatile
rclcpp::QoS qos_sensor(10);  // depth 10
qos_sensor.best_effort();

// Commands: reliable, volatile
rclcpp::QoS qos_command(10);
qos_command.reliable();

// Configuration: reliable, transient local
rclcpp::QoS qos_config(10);
qos_config.reliable();
qos_config.transient_local();
```

**QoS matching**: Publishers and subscribers must have compatible QoS

<!-- class="theory-concept" -->
**Transform System (tf2)**

tf2 maintains coordinate frame relationships:

**Transform tree**: Directed acyclic graph of coordinate frames
- **map**: Global fixed frame
- **odom**: Local odometry frame (drift over time)
- **base_link**: Robot base frame
- **sensor frames**: Camera, lidar, etc.

**Transform lookup**: Query relationship between frames
```python
# Get transform from source to target frame
transform = tf_buffer.lookup_transform(
    target_frame='map',
    source_frame='base_link',
    time=rclpy.time.Time()  # Latest available
)
```

**Transform broadcasting**: Publish frame relationships
```python
# Broadcast static transform
static_broadcaster.sendTransform(transform_stamped)

# Broadcast dynamic transform (e.g., from odometry)
dynamic_broadcaster.sendTransform(transform_stamped)
```

**Applications**:
- Sensor fusion from multiple coordinate frames
- Motion planning in global coordinates
- Visualization in common frame

### 2.3 ROS2 Workspace and Build System

<!-- class="theory-concept" -->
**Workspace Structure**

ROS2 workspace organizes packages:

```
ros2_ws/
├── src/                    # Source space
│   ├── package_1/
│   │   ├── package.xml     # Package metadata
│   │   ├── CMakeLists.txt  # Build configuration (C++)
│   │   ├── setup.py        # Build configuration (Python)
│   │   ├── include/        # C++ headers
│   │   ├── src/            # C++ source
│   │   └── scripts/        # Python scripts
│   └── package_2/
├── build/                  # Build space (generated)
├── install/                # Install space (generated)
└── log/                    # Build logs (generated)
```

**package.xml**: Package metadata and dependencies
```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_robot_control</name>
  <version>0.1.0</version>
  <description>Robot control package</description>
  <maintainer email="dev@example.com">Developer</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**Build commands**:
```bash
cd ros2_ws
colcon build                # Build all packages
colcon build --packages-select my_pkg  # Build specific package
source install/setup.bash   # Source environment
```

<!-- class="historical-note" -->
**ROS Build System Evolution**

ROS build systems evolved with changing needs:
- **ROS1**: rosbuild (CMake wrapper) → catkin (improved dependency handling)
- **ROS2**: ament (language-agnostic) with colcon (build tool)

ament improvements:
- Support for pure Python packages (no CMake required)
- Isolated build spaces
- Faster incremental builds
- Better Windows support

**Quiz: ROS2 Architecture**

Which layer of the ROS2 architecture handles network communication and discovery?

[( )] RCL (ROS Client Library)
[(X)] DDS (Data Distribution Service)
[( )] rclcpp/rclpy
[( )] Application code
***
<div>
DDS (Data Distribution Service) is the bottom layer that handles actual network communication, node discovery, and provides Quality of Service policies.
</div>
***

What QoS reliability setting should be used for command messages that must be received?

[(X)] Reliable
[( )] Best effort
[( )] Volatile
[( )] Transient local
***
<div>
Reliable QoS guarantees message delivery through retransmission, essential for critical commands. Best effort offers lower latency but no delivery guarantee, suitable for high-frequency sensor data.
</div>
***

---

## Module 3: Implementing ROS2 Nodes

--{{0}}--
This module covers practical implementation of ROS2 nodes in Python and C++.

### 3.1 Python Node Implementation

<!-- class="theory-concept" -->
**Basic ROS2 Python Node**

Minimal publisher node structure:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key components**:
- `Node.__init__()`: Initialize node with name
- `create_publisher()`: Create publisher for topic
- `create_timer()`: Periodic callback execution
- `rclpy.spin()`: Process callbacks until shutdown
- `get_logger()`: Logging interface (debug, info, warn, error)

**Subscriber node**:
```python
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

### 3.2 C++ Node Implementation

<!-- class="theory-concept" -->
**Basic ROS2 C++ Node**

Minimal publisher node:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'",
                    message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

**C++ advantages**:
- Deterministic memory management (no GC pauses)
- Lower latency message passing
- Stronger type checking at compile time
- Better performance for high-rate control loops

**When to use C++**:
- Real-time control (< 1kHz)
- Computationally intensive algorithms
- Hardware drivers with strict timing
- When callback latency critical

**When to use Python**:
- Rapid prototyping
- High-level task planning
- Integration with ML libraries
- System integration and orchestration

### 3.3 Services and Actions

<!-- class="theory-concept" -->
**Service Implementation**

Service definition (AddTwoInts.srv):
```
int64 a
int64 b
---
int64 sum
```

**Service server (C++)**:
```cpp
class AddTwoIntsServer : public rclcpp::Node {
public:
    AddTwoIntsServer() : Node("add_two_ints_server") {
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&AddTwoIntsServer::add, this,
                     std::placeholders::_1, std::placeholders::_2));
    }

private:
    void add(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "%ld + %ld = %ld",
                    request->a, request->b, response->sum);
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};
```

**Service client (Python)**:
```python
class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        return future
```

<!-- class="theory-concept" -->
**Action Implementation**

Action definition (Fibonacci.action):
```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

**Action server** handles goals, provides feedback, returns results:
```python
class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()

            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] +
                feedback_msg.partial_sequence[i-1])
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result
```

**Action client**: Send goals, receive feedback, handle results
```python
action_client = ActionClient(self, Fibonacci, 'fibonacci')
goal_msg = Fibonacci.Goal()
goal_msg.order = 10

send_goal_future = action_client.send_goal_async(
    goal_msg, feedback_callback=self.feedback_callback)
```

<!-- class="alternative-approach" -->
**Alternative Concurrency Models**

ROS2 executors manage callback execution:

**SingleThreadedExecutor**: Default, sequential callback processing
**MultiThreadedExecutor**: Thread pool for concurrent callbacks
**StaticSingleThreadedExecutor**: Pre-allocated memory for determinism

For complex concurrency needs:
- Callback groups (mutually exclusive vs. reentrant)
- Custom executors
- External threading with ROS2 guards

**Quiz: Node Implementation**

What is the purpose of `rclpy.spin()` or `rclcpp::spin()`?

[( )] To create a new node
[(X)] To process callbacks until shutdown
[( )] To publish messages
[( )] To compile the node
***
<div>
Spin functions block and process incoming messages, timers, services, and actions. They dispatch callbacks as events occur, continuing until the node is shutdown.
</div>
***

---

## Module 4: Kinematics in Distributed Systems

--{{0}}--
This module applies ROS2 concepts to implement kinematic solvers as distributed services.

### 4.1 Forward Kinematics as a ROS2 Node

<!-- class="theory-concept" -->
**Continuous Forward Kinematics Publisher**

FK node subscribes to joint states and publishes end-effector pose:

```python
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster

class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics')

        # Subscriber to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10)

        # Publisher for end-effector pose
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/end_effector_pose',
            10)

        # TF broadcaster for visualization
        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot parameters (DH parameters, link lengths, etc.)
        self.setup_robot_model()

    def joint_callback(self, msg):
        # Extract joint angles from message
        joint_angles = np.array(msg.position)

        # Compute forward kinematics
        T = self.compute_fk(joint_angles)

        # Publish as PoseStamped
        pose_msg = self.matrix_to_pose_stamped(T)
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        self.pose_pub.publish(pose_msg)

        # Broadcast transform for tf tree
        self.broadcast_transform(T)

    def compute_fk(self, joint_angles):
        # Implement FK using DH parameters or POE formula
        # Return 4x4 homogeneous transformation matrix
        # (Implementation from RC-102)
        pass

    def matrix_to_pose_stamped(self, T):
        # Convert 4x4 matrix to Pose message
        # Extract translation and convert rotation to quaternion
        pass
```

**Design considerations**:
- Update rate matches joint_states publication rate
- Computational cost per callback (should be < callback period)
- Frame IDs for coordinate system tracking
- Timestamp synchronization

### 4.2 Inverse Kinematics as a ROS2 Service

<!-- class="theory-concept" -->
**IK Service Definition**

Custom service for IK requests:

```
# ComputeIK.srv
geometry_msgs/Pose target_pose
sensor_msgs/JointState seed_state  # Initial guess
---
bool success
sensor_msgs/JointState joint_solution
string message
```

**IK Service Server (C++)**:

```cpp
class IKServiceNode : public rclcpp::Node {
public:
    IKServiceNode() : Node("ik_service") {
        service_ = this->create_service<robot_interfaces::srv::ComputeIK>(
            "compute_ik",
            std::bind(&IKServiceNode::compute_ik_callback, this,
                     std::placeholders::_1, std::placeholders::_2));
    }

private:
    void compute_ik_callback(
        const std::shared_ptr<robot_interfaces::srv::ComputeIK::Request> request,
        std::shared_ptr<robot_interfaces::srv::ComputeIK::Response> response)
    {
        // Extract target pose
        Eigen::Vector3d target_pos(
            request->target_pose.position.x,
            request->target_pose.position.y,
            request->target_pose.position.z);

        // Extract seed joint angles
        Eigen::VectorXd seed = extract_joint_angles(request->seed_state);

        // Solve IK using Jacobian transpose or other numerical method
        Eigen::VectorXd solution;
        bool success = solve_ik(target_pos, seed, solution);

        // Populate response
        response->success = success;
        if (success) {
            response->joint_solution = create_joint_state_msg(solution);
            response->message = "IK solution found";
        } else {
            response->message = "IK failed to converge";
        }
    }

    bool solve_ik(const Eigen::Vector3d& target,
                  const Eigen::VectorXd& seed,
                  Eigen::VectorXd& solution)
    {
        // Jacobian transpose iterative solver
        solution = seed;
        const int max_iterations = 100;
        const double tolerance = 1e-3;

        for (int iter = 0; iter < max_iterations; ++iter) {
            // Compute current position
            Eigen::Vector3d current_pos = forward_kinematics(solution);

            // Check convergence
            double error = (target - current_pos).norm();
            if (error < tolerance) {
                return true;
            }

            // Compute Jacobian
            Eigen::MatrixXd J = compute_jacobian(solution);

            // Update: Δq = α * J^T * (target - current)
            Eigen::VectorXd delta_q = 0.1 * J.transpose() * (target - current_pos);
            solution += delta_q;
        }

        return false;  // Failed to converge
    }

    rclcpp::Service<robot_interfaces::srv::ComputeIK>::SharedPtr service_;
};
```

**Service design principles**:
- Timeout handling for non-convergent cases
- Seed state improves convergence speed
- Return success flag + diagnostic message
- Stateless operation (no side effects)

### 4.3 Controller Node Architecture

<!-- class="theory-concept" -->
**Integrated Control Node**

Controller orchestrates FK, IK, and hardware interface:

```python
class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Service client for IK
        self.ik_client = self.create_client(ComputeIK, 'compute_ik')

        # Subscriber to current joint states
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states',
            self.joint_callback, 10)

        # Publisher to joint commands
        self.cmd_pub = self.create_publisher(
            JointState, '/joint_commands', 10)

        # Action server for move_to_pose
        self._action_server = ActionServer(
            self, MoveToPose, 'move_to_pose',
            self.execute_move_callback)

        self.current_joints = None

    def execute_move_callback(self, goal_handle):
        target_pose = goal_handle.request.target_pose

        # Call IK service
        ik_request = ComputeIK.Request()
        ik_request.target_pose = target_pose
        ik_request.seed_state = self.current_joints

        future = self.ik_client.call_async(ik_request)
        rclpy.spin_until_future_complete(self, future)

        if not future.result().success:
            goal_handle.abort()
            return MoveToPose.Result(success=False)

        # Execute motion to target joints
        target_joints = future.result().joint_solution
        self.move_to_joints(goal_handle, target_joints)

        goal_handle.succeed()
        return MoveToPose.Result(success=True)

    def move_to_joints(self, goal_handle, target_joints):
        # Interpolate from current to target
        # Publish joint commands at control rate
        # Provide feedback on distance to goal
        pass
```

**System integration pattern**:
- Controller as central orchestrator
- FK/IK as specialized services
- Hardware interface abstraction
- Action interface for task-level commands

<!-- class="alternative-approach" -->
**MoveIt Integration**

For production systems, MoveIt2 provides:
- Multiple IK solvers (KDL, TRAC-IK, analytical)
- Planning scene with collision checking
- Trajectory optimization
- Standardized ROS2 action interfaces

Custom IK services useful for:
- Educational purposes
- Custom robot configurations
- Specialized optimization criteria
- Performance-critical applications

**Quiz: Distributed Kinematics**

Why implement inverse kinematics as a ROS2 service instead of a function call within the controller?

[(X)] To allow reuse by multiple nodes and decouple computation from control
[( )] Services are faster than function calls
[( )] It requires less code
[( )] Services can run on GPUs
***
<div>
Services enable modularity and reuse: multiple nodes can request IK solutions without duplicating code. The controller is decoupled from IK implementation details, allowing independent development and testing.
</div>
***

---

## Module 5: Launch Files and System Configuration

--{{0}}--
This module addresses multi-node system deployment and configuration management.

### 5.1 Launch File Architecture

<!-- class="theory-concept" -->
**Python Launch Files**

ROS2 launch files programmatically start nodes and configure systems:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_control',
            executable='forward_kinematics_node',
            name='fk_node',
            output='screen',
            parameters=[{'link_lengths': [0.3, 0.25, 0.2]}]
        ),
        Node(
            package='my_robot_control',
            executable='ik_service_node',
            name='ik_node',
            output='screen'
        ),
        Node(
            package='my_robot_control',
            executable='controller_node',
            name='controller',
            output='screen',
            remappings=[
                ('/joint_commands', '/robot/joint_commands')
            ]
        ),
    ])
```

**Launch actions**:
- **Node**: Start ROS2 node
- **IncludeLaunchDescription**: Compose launch files
- **ExecuteProcess**: Run arbitrary command
- **SetEnvironmentVariable**: Configure environment
- **DeclareLaunchArgument**: Parameterize launch file

**Advanced features**:
```python
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation instead of real hardware'
    )

    sim_nodes = GroupAction(
        actions=[
            Node(package='gazebo_ros', executable='gzserver'),
            Node(package='robot_sim', executable='simulator')
        ],
        condition=IfCondition(LaunchConfiguration('use_sim'))
    )

    return LaunchDescription([use_sim_arg, sim_nodes, ...])
```

### 5.2 Parameter Management

<!-- class="theory-concept" -->
**Parameter Declaration and Usage**

Nodes declare parameters with default values:

```cpp
class ConfigurableNode : public rclcpp::Node {
public:
    ConfigurableNode() : Node("configurable_node") {
        // Declare parameters with defaults
        this->declare_parameter("update_rate", 100.0);
        this->declare_parameter("joint_limits",
            std::vector<double>{-3.14, 3.14});
        this->declare_parameter("robot_name", "my_robot");

        // Get parameter values
        update_rate_ = this->get_parameter("update_rate").as_double();
        joint_limits_ = this->get_parameter("joint_limits")
            .as_double_array();
        robot_name_ = this->get_parameter("robot_name").as_string();

        // Parameter callback for runtime updates
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ConfigurableNode::parameters_callback, this,
                     std::placeholders::_1));
    }

private:
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;

        for (const auto &param : parameters) {
            if (param.get_name() == "update_rate") {
                if (param.as_double() > 0.0 && param.as_double() < 1000.0) {
                    update_rate_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(),
                               "Update rate changed to: %.2f", update_rate_);
                } else {
                    result.successful = false;
                    result.reason = "update_rate must be in (0, 1000]";
                }
            }
        }
        return result;
    }

    double update_rate_;
    std::vector<double> joint_limits_;
    std::string robot_name_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};
```

**Parameter files** (YAML):
```yaml
# config/robot_params.yaml
configurable_node:
  ros__parameters:
    update_rate: 50.0
    joint_limits: [-2.0, 2.0, -1.5, 1.5, -3.0, 3.0]
    robot_name: "test_robot"
```

**Loading parameters in launch file**:
```python
Node(
    package='my_package',
    executable='configurable_node',
    parameters=[os.path.join(pkg_share, 'config', 'robot_params.yaml')]
)
```

**Runtime parameter modification**:
```bash
ros2 param set /configurable_node update_rate 75.0
ros2 param get /configurable_node update_rate
ros2 param dump /configurable_node  # Save current values
```

### 5.3 Namespaces and Remapping

<!-- class="theory-concept" -->
**Topic Remapping**

Remapping connects nodes with different topic names:

```python
# Launch node with remapped topics
Node(
    package='robot_control',
    executable='controller',
    remappings=[
        ('/joint_states', '/robot_1/joint_states'),
        ('/joint_commands', '/robot_1/joint_commands')
    ]
)
```

**Namespaces**:
```python
# Push node into namespace
Node(
    package='robot_control',
    executable='controller',
    namespace='/robot_1'
)
# Topics become: /robot_1/joint_states, /robot_1/joint_commands
```

**Multi-robot systems**:
```python
def generate_multi_robot_launch():
    robots = []
    for i in range(3):
        robot_ns = f'/robot_{i}'
        robots.append(Node(
            package='robot_control',
            executable='controller',
            namespace=robot_ns,
            parameters=[{'robot_id': i}]
        ))
    return LaunchDescription(robots)
```

**Advantages**:
- Isolate multiple robot instances
- Reuse same node code for different robots
- Hierarchical organization of complex systems

<!-- class="historical-note" -->
**Launch System Evolution**

ROS launch systems evolved significantly:
- **ROS1**: XML launch files (roslaunch)
  - Declarative but limited flexibility
  - No programmatic logic
- **ROS2**: Python launch files
  - Full programming language power
  - Conditional logic, loops, functions
  - Type checking and IDE support
  - Can still include XML for compatibility

Modern launch files enable complex system composition and configuration management.

**Quiz: Launch and Configuration**

What is the primary advantage of Python launch files over XML launch files?

[( )] They run faster
[(X)] They allow programmatic logic like conditionals and loops
[( )] They are shorter
[( )] They work on more platforms
***
<div>
Python launch files provide full programming capabilities including conditional logic, loops, functions, and dynamic composition, enabling more flexible system configuration than declarative XML.
</div>
***

---

## Summary and Key Takeaways

This course established distributed systems foundations for robotics using ROS2:

1. **Distributed Systems**: Middleware architectures manage complexity through modular, communicating components

2. **Communication Patterns**: Topics (pub-sub), services (request-response), and actions (goal-oriented) address different system needs

3. **ROS2 Architecture**: Layered design separates DDS middleware from application code with quality-of-service policies

4. **Implementation**: Python and C++ nodes provide trade-offs between development speed and runtime performance

5. **Distributed Kinematics**: FK and IK as services enable modular, reusable computation accessible to multiple clients

6. **System Configuration**: Launch files and parameters enable flexible deployment and runtime reconfiguration

These principles scale from single manipulators to multi-robot fleets and complex autonomous systems.

---

## Optional Laboratory Exercises

The following exercises provide hands-on experience developing distributed robotic systems with ROS2.

---

### Laboratory 1: ROS2 Workspace Setup and Basic Nodes

**Duration**: 90 minutes

**Objective**: Create ROS2 workspace and implement basic publisher-subscriber nodes.

<!-- class="optional-exercise" -->
**Exercise 1.1: Workspace Creation**

Tasks:
1. Create ROS2 workspace directory structure
2. Create a new package `robot_basics`
3. Configure package.xml with dependencies
4. Build workspace with colcon
5. Source setup script

**Exercise 1.2: Publisher Node**

Implement a node that publishes simulated joint states:

Tasks:
1. Create Python node `joint_state_publisher`
2. Publish to `/joint_states` topic (sensor_msgs/JointState)
3. Generate sinusoidal joint angles
4. Publish at 10Hz
5. Add logging for published values

**Exercise 1.3: Subscriber Node**

Implement a node that subscribes to joint states:

Tasks:
1. Create Python node `joint_state_monitor`
2. Subscribe to `/joint_states`
3. Log received joint angles
4. Calculate and display joint velocities (numerical differentiation)
5. Verify communication using `ros2 topic echo`

Deliverable: Working pub-sub system with two nodes communicating via topics

<!-- class="exercise-tip" -->
**Development tip**: Use `ros2 node list`, `ros2 topic list`, and `ros2 topic info` to inspect your running system. The `rqt_graph` tool provides visualization of node-topic connections.

---

### Laboratory 2: Services and Kinematic Solvers

**Duration**: 150 minutes

**Objective**: Implement forward and inverse kinematics as ROS2 services.

<!-- class="optional-exercise" -->
**Exercise 2.1: Forward Kinematics Service**

Create custom service definition:

```
# ComputeFK.srv
float64[] joint_angles
---
geometry_msgs/Pose end_effector_pose
bool success
```

Tasks:
1. Define service in package
2. Implement C++ service server
3. Compute FK for 3-DOF planar arm (reuse RC-102 knowledge)
4. Return pose in Pose message format
5. Test using `ros2 service call`

**Exercise 2.2: Inverse Kinematics Service**

Implement IK service using Jacobian transpose:

Tasks:
1. Define IK service interface (target pose → joint angles)
2. Implement iterative Jacobian transpose solver
3. Add convergence checking and iteration limits
4. Handle failures gracefully (return success flag)
5. Test with various target poses

**Exercise 2.3: Service Client**

Create client node that uses both services:

Tasks:
1. Call FK service with known joints
2. Use result as target for IK service
3. Verify IK solution matches original joints
4. Measure round-trip computation time
5. Test error handling (unreachable targets)

Deliverable: FK and IK services with test client demonstrating bidirectional kinematic solving

<!-- class="exercise-advanced" -->
**Advanced challenge**: Implement analytical IK for 3-DOF planar arm. Compare convergence speed and accuracy to numerical Jacobian transpose method.

---

### Laboratory 3: Action Server for Motion Execution

**Duration**: 120 minutes

**Objective**: Implement action server for goal-oriented motion tasks.

<!-- class="optional-exercise" -->
**Exercise 3.1: Action Definition**

Define custom action:

```
# MoveToPose.action
geometry_msgs/Pose target_pose
float64 max_velocity
---
bool success
float64 final_error
---
float64 distance_to_goal
float64 percent_complete
```

**Exercise 3.2: Action Server Implementation**

Tasks:
1. Create action server node
2. Accept target pose goals
3. Call IK service to get target joints
4. Interpolate trajectory from current to target
5. Publish joint commands at control rate (10Hz)
6. Provide feedback (distance remaining, percent)
7. Return result upon completion

**Exercise 3.3: Action Client with Feedback**

Tasks:
1. Create action client node
2. Send goal pose
3. Display feedback during execution
4. Handle completion (success/failure)
5. Implement cancellation (stop mid-motion)

**Exercise 3.4: Multi-Goal Sequence**

Tasks:
1. Send sequence of poses as separate goals
2. Wait for each goal completion before sending next
3. Handle failures (skip to next goal vs. abort sequence)
4. Visualize robot path using RViz

Deliverable: Complete action-based motion system with feedback and cancellation

---

### Laboratory 4: Launch Files and System Integration

**Duration**: 90 minutes

**Objective**: Create launch files for full system deployment.

<!-- class="optional-exercise" -->
**Exercise 4.1: Basic Launch File**

Create launch file that starts all nodes:

Tasks:
1. Launch FK service node
2. Launch IK service node
3. Launch action server node
4. Launch joint state publisher (simulation)
5. Configure output to screen
6. Test single-command startup

**Exercise 4.2: Parameterized Launch**

Add configuration:

Tasks:
1. Create YAML parameter file with robot dimensions
2. Load parameters in launch file
3. Add launch arguments (use_sim, robot_name)
4. Conditional node launching based on arguments
5. Test different configurations

**Exercise 4.3: Multi-Robot Launch**

Extend to multiple robot instances:

Tasks:
1. Create namespace for each robot
2. Remap topics to avoid conflicts
3. Load robot-specific parameters
4. Launch 2-3 robot instances simultaneously
5. Verify isolation using `ros2 topic list`

**Exercise 4.4: System Monitoring**

Add monitoring and visualization:

Tasks:
1. Include RViz in launch file with config
2. Add rqt_graph for runtime visualization
3. Configure logging levels
4. Add robot_state_publisher for TF tree
5. Create complete system launch file

Deliverable: Complete launch system for multi-node, multi-robot deployment

---

## Further Study and Resources

### Recommended Reading

1. **ROS2 Documentation**: Official tutorials and design documents
2. **Morgan Quigley et al.**: *Programming Robots with ROS* (foundational concepts apply to ROS2)
3. **Arn Brunl**: *Embedded Robotics* (distributed embedded systems)

### Online Resources

- **ROS2 Official Documentation**: Comprehensive tutorials and API reference
- **ros2/demos repository**: Example implementations
- **The Construct**: Online ROS2 courses and simulations
- **ROS Discourse**: Community Q&A forum

### Next Steps

- **SOFTWARE-203**: Motion planning with MoveIt2
- **Advanced ROS2**: Lifecycle nodes, QoS tuning, real-time executors
- **Navigation Stack**: Nav2 for mobile robot navigation

---

**Course developed by Robot Campus Team**
Version 2.0.0 | Last updated: 2024
