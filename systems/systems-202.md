<!--
author:   Dr. Michael Chen
email:    michael.chen@academy-of-robotic-sciences.github.io
version:  2.0.0

language: en
narrator: US English Male

comment:  System Integration: Comprehensive treatment of distributed system architecture, ROS2 middleware, coordinate transformations, Quality of Service policies, containerization, and systematic integration methodologies for complex robotic systems.


mode:     Textbook

link:     https://raw.githubusercontent.com/Academy-of-Robotic-Sciences/courses/main/course-styles.css

import:   https://raw.githubusercontent.com/LiaTemplates/Pyodide/master/README.md
-->

# SYSTEMS-202: System Integration

> **Integration transforms isolated components into coordinated systems through rigorous interface specification and middleware architecture.**

## Course Overview

| | |
|---|---|
| **Course Code** | SYSTEMS-202 |
| **Duration** | 8 hours |
| **Level** | Advanced |
| **Prerequisites** | SYSTEMS-201, ROS fundamentals, Linux command line |
| **Theory-Practice** | 60% theory, 40% optional labs |

---

## Learning Objectives

Upon completion of this course, students will be able to:

### Theoretical Understanding

- Explain publish-subscribe middleware architecture and message-oriented communication
- Understand Data Distribution Service (DDS) protocol and Quality of Service (QoS) policies
- Analyze coordinate transformation theory and spatial reasoning in robotics
- Describe containerization principles and deployment architectures
- Understand real-time communication requirements and timing constraints
- Explain distributed system failure modes and mitigation strategies

### Practical Skills

- Debug ROS2 systems using introspection and diagnostic tools
- Configure QoS policies for reliable real-time communication
- Design and implement coordinate transformation trees (tf2)
- Create Docker containers for reproducible robotics applications
- Diagnose and resolve integration issues systematically
- Implement launch files for complex multi-node systems

---

## Module 1: Middleware and Communication Architectures

### 1.1 Message-Oriented Middleware

<!-- class="theory-concept" -->
**Publish-Subscribe Pattern**

In distributed robotic systems, components communicate asynchronously through message passing:

**Traditional Point-to-Point Communication**:
- Direct connections between components
- Tight coupling (sender knows receiver)
- Difficult to scale or modify

**Publish-Subscribe (Pub-Sub)**:
- Publishers send messages to topics (named channels)
- Subscribers receive messages from topics
- Loose coupling (publishers/subscribers independent)
- Many-to-many communication
- Dynamic discovery

**ROS2 Architecture**:
```
┌──────────┐         ┌─────────┐         ┌──────────┐
│Publisher │ ──msg──>│  Topic  │ ──msg──>│Subscriber│
└──────────┘         └─────────┘         └──────────┘
                          ↑
                          │ (multiple publishers/
                          │  subscribers allowed)
```

**Advantages**:
- Decoupling: Publishers/subscribers don't need to know about each other
- Scalability: Add/remove components without affecting others
- Flexibility: Runtime discovery and dynamic connections
- Reusability: Standard message interfaces

<!-- class="historical-note" -->
**Evolution of Robot Middleware**

**1990s**: Early robotic systems used custom communication protocols, leading to integration difficulties and limited code reuse.

**2000**: CORBA (Common Object Request Broker Architecture) was applied to robotics, but proved heavyweight and complex.

**2007**: **ROS (Robot Operating System)** introduced by Willow Garage, providing standardized pub-sub communication, tools, and libraries. Rapidly became the de facto standard for research robotics.

**2010s**: ROS limitations emerged for production systems:
- No real-time support
- Weak security
- Single-master architecture vulnerability
- Limited embedded system support

**2017**: **ROS2** development began, addressing production requirements:
- Based on DDS (Data Distribution Service) middleware
- Real-time capable
- Security features (encryption, authentication)
- Fully distributed (no master)
- Cross-platform (Linux, Windows, macOS, RTOS)

### 1.2 Data Distribution Service (DDS)

<!-- class="theory-concept" -->
**DDS Standard**

DDS is an OMG (Object Management Group) standard for data-centric publish-subscribe:

**Key Concepts**:

**Domain**: Logical network of communicating participants (isolates communication)

**Participant**: DDS entity representing a process in the domain

**Publisher/Subscriber**: Create and manage data writers/readers

**Topic**: Named data stream with specific type

**DataWriter**: Publishes data samples to topic

**DataReader**: Receives data samples from topic

**DDS Quality of Service (QoS)**: Configurable policies controlling data delivery

<!-- class="theory-concept" -->
**DDS vs. Traditional Messaging**

DDS provides:
- **Data-centric**: Focuses on data model, not endpoints
- **Automatic Discovery**: Participants find each other automatically
- **Fine-grained QoS**: Control reliability, latency, resource usage per topic
- **Content Filtering**: Subscribers can filter messages
- **Historical Data**: Late joiners can receive past data
- **Temporal Decoupling**: Publishers/subscribers need not exist simultaneously

**DDS Implementations** (multiple vendors):
- Fast DDS (eProsima) - ROS2 default
- CycloneDDS (Eclipse)
- RTI Connext DDS
- OpenDDS

### 1.3 Quality of Service (QoS) Policies

<!-- class="theory-concept" -->
**Configurable Communication Properties**

QoS policies allow tuning communication behavior per topic:

**Reliability**:
- **RELIABLE**: Guarantees delivery, retransmits lost messages
  - Use for: Commands, state information
  - Trade-off: Higher latency, more bandwidth
- **BEST_EFFORT**: No delivery guarantee, drops lost messages
  - Use for: High-frequency sensor data (newer data more valuable)
  - Trade-off: Lower latency, less bandwidth

**Durability**:
- **VOLATILE**: No historical data stored
- **TRANSIENT_LOCAL**: Keeps recent samples for late joiners
- **TRANSIENT**: Persistent across restarts
- **PERSISTENT**: Stored on disk

**History**:
- **KEEP_LAST(N)**: Keep only last N samples
- **KEEP_ALL**: Keep all samples (limited by resource limits)

**Deadline**: Expected publication period
- Subscriber notified if data not received within deadline
- Use for: Monitoring data freshness

**Lifespan**: Maximum data age
- Old data automatically discarded
- Prevents stale data processing

<!-- class="theory-concept" -->
**QoS Compatibility**

Publishers and subscribers must have compatible QoS:

**Requested vs. Offered**:
- Subscriber **requests** QoS properties
- Publisher **offers** QoS properties
- Connection only established if compatible

**Compatibility Rules**:
- RELIABLE publisher can match RELIABLE or BEST_EFFORT subscriber
- BEST_EFFORT publisher can only match BEST_EFFORT subscriber
- Durability: Offered ≥ Requested
- Deadline: Offered ≤ Requested

**Common Mismatch**: Publisher BEST_EFFORT, Subscriber RELIABLE → No connection!

**Quiz: Middleware Architecture**

In publish-subscribe systems, what is the primary benefit of loose coupling?

[( )] Faster communication
[(X)] Components can be added/removed without modifying others
[( )] Lower memory usage
[( )] Better security

A publisher with BEST_EFFORT reliability can communicate with which subscriber?

[(X)] BEST_EFFORT only
[( )] RELIABLE only
[( )] Both BEST_EFFORT and RELIABLE
[( )] Neither

---

## Module 2: ROS2 System Architecture

### 2.1 Nodes, Topics, Services, and Actions

<!-- class="theory-concept" -->
**ROS2 Communication Primitives**

**Nodes**: Independent processes performing computation
- Single responsibility principle
- Communicate via topics/services/actions
- Example: `camera_driver`, `object_detector`, `motion_planner`

**Topics** (Pub-Sub): Asynchronous, continuous data streams
- One-to-many, many-to-many
- Fire-and-forget semantics
- Example: `/camera/image`, `/odom`, `/cmd_vel`

**Services** (Request-Reply): Synchronous, transactional
- One-to-one communication
- Client blocks until server responds
- Example: `/get_map`, `/trigger_calibration`
- Use for: Infrequent, triggered actions requiring response

**Actions**: Long-running tasks with feedback
- Asynchronous goal submission
- Periodic feedback during execution
- Result upon completion
- Cancelable
- Example: `/navigate_to_pose`, `/grasp_object`

**Comparison**:

| Feature | Topics | Services | Actions |
|---------|--------|----------|---------|
| Pattern | Pub-Sub | Req-Reply | Goal-based |
| Blocking | No | Yes | No |
| Feedback | No | No | Yes |
| Cancel | N/A | No | Yes |
| Use Case | Continuous data | Quick transactions | Long tasks |

### 2.2 ROS2 Graph and Discovery

<!-- class="theory-concept" -->
**Computational Graph**

The ROS graph represents all nodes and their communication:

**Introspection Tools**:

**`ros2 node list`**: List all active nodes

**`ros2 topic list`**: List all topics

**`ros2 topic info /topic_name`**: Show publishers/subscribers and QoS

**`ros2 topic echo /topic_name`**: Display messages on topic

**`ros2 topic hz /topic_name`**: Measure publication rate

**`ros2 interface show msg_type`**: Display message definition

**`rqt_graph`**: Visualize node graph
- Nodes as ovals
- Topics as rectangles
- Edges show pub/sub relationships

<!-- class="theory-concept" -->
**Discovery Mechanism**

ROS2 uses DDS discovery to find participants:

**Simple Discovery Protocol (SDP)**:
1. Nodes multicast presence announcements
2. Other nodes respond with their topics/services
3. QoS compatibility checked
4. Matching endpoints connected

**No Centralized Master** (unlike ROS1):
- Fully distributed
- More robust (no single point of failure)
- Scales better for large systems

**Discovery Latency**: Can take 1-5 seconds for full graph convergence

### 2.3 Message and Interface Definitions

<!-- class="theory-concept" -->
**ROS2 Messages**

Messages define data structures exchanged between nodes:

**Message Definition Example** (`geometry_msgs/msg/Twist`):
```
Vector3 linear
Vector3 angular
```

Where `Vector3`:
```
float64 x
float64 y
float64 z
```

**Standard Message Packages**:
- `std_msgs`: Basic types (Int32, Float64, String, etc.)
- `geometry_msgs`: Poses, velocities, transforms
- `sensor_msgs`: Sensor data (Image, LaserScan, Imu, etc.)
- `nav_msgs`: Navigation (Odometry, Path)
- `tf2_msgs`: Transform trees

**Custom Messages**: Define in `msg/` directory
```
# my_package/msg/RobotStatus.msg
string robot_id
float32 battery_voltage
uint8 operational_mode
geometry_msgs/Pose current_pose
```

Build system generates language bindings (Python, C++, etc.)

---

## Module 3: Coordinate Transformations

### 3.1 Coordinate Frame Theory

<!-- class="theory-concept" -->
**Spatial Relationships in Robotics**

Robotic systems have multiple coordinate frames:
- **World/Map**: Fixed global reference
- **Odom**: Local odometry frame (drifts over time)
- **Base_link**: Robot's center
- **Sensor frames**: Camera, LiDAR, IMU positions
- **End-effector**: Gripper or tool

**Homogeneous Transformations**:

A rigid transformation (rotation + translation) from frame A to frame B:

$$^A T_B = \begin{bmatrix} ^A R_B & ^A P_B \\ 0 & 1 \end{bmatrix}$$

where:
- $^A R_B$ = 3×3 rotation matrix
- $^A P_B$ = 3×1 translation vector

**Transform Composition**: Chaining transforms

$$^A T_C = ^A T_B \cdot ^B T_C$$

Allows computing relative positions across multiple frames.

<!-- class="theory-concept" -->
**Rotation Representations**

**Rotation Matrices**: 3×3 orthogonal matrices
- Properties: $R^T R = I$, $\det(R) = 1$
- Composable, but 9 parameters (6 redundant)

**Euler Angles**: Roll, pitch, yaw (RPY)
- Intuitive for small rotations
- Gimbal lock problem
- Order-dependent (12 conventions!)

**Quaternions**: 4-parameter representation
- $q = [w, x, y, z]$ where $w^2 + x^2 + y^2 + z^2 = 1$
- No gimbal lock
- Efficient interpolation (SLERP)
- Standard in ROS: `geometry_msgs/Quaternion`

**Conversion**: Euler (roll, pitch, yaw) to Quaternion:
$$w = \cos(r/2)\cos(p/2)\cos(y/2) + \sin(r/2)\sin(p/2)\sin(y/2)$$
$$x = \sin(r/2)\cos(p/2)\cos(y/2) - \cos(r/2)\sin(p/2)\sin(y/2)$$
$$y = \cos(r/2)\sin(p/2)\cos(y/2) + \sin(r/2)\cos(p/2)\sin(y/2)$$
$$z = \cos(r/2)\cos(p/2)\sin(y/2) - \sin(r/2)\sin(p/2)\cos(y/2)$$

### 3.2 tf2 Transform Library

<!-- class="theory-concept" -->
**Transform Tree (tf2)**

ROS2 `tf2` library manages coordinate frame relationships:

**Tree Structure**:
- Each frame has exactly one parent
- Root frame (typically `world` or `map`)
- Leaves are sensors, end-effectors

**Transform Broadcasters**:
- Nodes publish transforms between frames
- Example: Odometry node publishes `odom` → `base_link`

**Transform Listeners**:
- Nodes query transforms
- tf2 automatically chains transforms through tree
- Time-synchronization: Get transform at specific timestamp

**Static vs. Dynamic Transforms**:

**Static**: Fixed relationships (sensor mounts)
```bash
ros2 run tf2_ros static_transform_publisher 0.1 0 0.2 0 0 0 base_link camera_link
```

**Dynamic**: Time-varying (robot motion, joint states)
- Published continuously (typically 10-100 Hz)

<!-- class="theory-concept" -->
**Transform Lookup**

Query transform from frame A to B at time t:

```python
from tf2_ros import Buffer, TransformListener

tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)

# Lookup transform from 'map' to 'camera_link'
try:
    transform = tf_buffer.lookup_transform(
        'map',              # target frame
        'camera_link',      # source frame
        rclpy.time.Time()   # latest available
    )
except TransformException as ex:
    node.get_logger().warn(f'Could not transform: {ex}')
```

**Common tf2 Errors**:
1. **Frame doesn't exist**: Not being published
2. **Extrapolation into past**: Transform too old (increase buffer size)
3. **Extrapolation into future**: Requested time not yet available
4. **Disconnected tree**: Missing intermediate transform

### 3.3 Time Synchronization

<!-- class="theory-concept" -->
**Temporal Consistency**

Sensor fusion requires temporally aligned data:

**Problem**: Sensors publish at different rates
- Camera: 30 Hz
- LiDAR: 10 Hz
- IMU: 100 Hz

For fusion at time t, need transforms valid at t.

**Message Timestamps**:
```python
from builtin_interfaces.msg import Time

msg.header.stamp = node.get_clock().now().to_msg()
msg.header.frame_id = 'camera_link'
```

**tf2 Buffer**: Stores transform history (default 10s)
- Enables lookup at past timestamps
- Interpolates between published transforms

**Message Filters**: Synchronize multiple topics
```python
from message_filters import ApproximateTimeSynchronizer, Subscriber

subs = [
    Subscriber(node, Image, '/camera/image'),
    Subscriber(node, LaserScan, '/scan')
]
sync = ApproximateTimeSynchronizer(subs, queue_size=10, slop=0.1)
sync.registerCallback(callback)
```

`slop=0.1`: Accept messages within 100ms of each other

**Quiz: Coordinate Transformations**

If frame A has position (1, 0, 0) in frame B, and frame C has position (2, 0, 0) in frame A, what is C's position in B?

[( )] (1, 0, 0)
[( )] (2, 0, 0)
[(X)] (3, 0, 0)
[( )] Cannot determine

What is the primary advantage of quaternions over Euler angles?

[( )] Easier to understand
[(X)] No gimbal lock
[( )] Require fewer parameters
[( )] Faster computation

---

## Module 4: Integration Debugging

### 4.1 Systematic Diagnostic Approach

<!-- class="theory-concept" -->
**Integration Failure Taxonomy**

Common integration problems and diagnostic approaches:

**1. No Communication (Nodes Don't Connect)**

**Symptoms**: `ros2 topic echo` shows no messages

**Diagnostics**:
```bash
# Check if topic exists
ros2 topic list

# Check publishers/subscribers
ros2 topic info /topic_name

# Verify QoS compatibility
ros2 topic info /topic_name --verbose
```

**Common Causes**:
- Topic name mismatch (typos, wrong namespace)
- QoS incompatibility (RELIABLE vs BEST_EFFORT)
- Node not running
- DDS discovery issue (firewall, network)

**2. Incorrect Data**

**Symptoms**: Messages received but contain wrong values

**Diagnostics**:
```bash
# View message content
ros2 topic echo /topic_name

# Check message type
ros2 topic info /topic_name

# Compare to interface definition
ros2 interface show geometry_msgs/msg/Pose
```

**Common Causes**:
- Unit mismatch (degrees vs radians, m vs mm)
- Coordinate frame mismatch
- Incorrect sensor calibration
- Logic error in publisher

**3. Transform Errors**

**Symptoms**: tf2 exceptions, incorrect spatial relationships

**Diagnostics**:
```bash
# View transform tree
ros2 run tf2_tools view_frames

# Check transform availability
ros2 run tf2_ros tf2_echo frame1 frame2

# Debug transform timing
ros2 topic echo /tf --no-arr
```

**Common Causes**:
- Missing transform broadcaster
- Incorrect parent frame
- Timestamp synchronization issues
- Publishing rate too low

**4. Timing Issues**

**Symptoms**: Intermittent failures, race conditions

**Diagnostics**:
```bash
# Measure publication rate
ros2 topic hz /topic_name

# Check message delays
ros2 topic echo /topic_name --field header.stamp
```

**Common Causes**:
- Insufficient publication rate
- Callback processing too slow
- Network latency
- CPU overload

### 4.2 ROS2 Introspection Tools

<!-- class="theory-concept" -->
**Command-Line Diagnostics**

**`ros2 doctor`**: System health check
- Checks network configuration
- Verifies DDS middleware
- Identifies common problems

**`ros2 bag`**: Record and replay data
```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /camera/image /scan

# Replay at half speed
ros2 bag play -r 0.5 my_bag
```

Use for:
- Reproducing bugs
- Testing without hardware
- Performance analysis

**`ros2 param`**: Parameter manipulation
```bash
# List node parameters
ros2 param list /node_name

# Get parameter value
ros2 param get /node_name param_name

# Set parameter value
ros2 param set /node_name param_name value
```

**`rqt`**: GUI tools
- `rqt_graph`: Visualize node graph
- `rqt_plot`: Plot numeric data
- `rqt_console`: View log messages
- `rqt_reconfigure`: Adjust parameters dynamically

### 4.3 Debugging Workflow

<!-- class="theory-concept" -->
**Systematic Integration Debugging**

**Step 1: Verify Individual Components**
- Test each node in isolation
- Confirm expected behavior
- Check inputs/outputs manually

**Step 2: Check Communication Infrastructure**
- Verify topics exist and have correct names
- Confirm publishers and subscribers present
- Validate QoS compatibility

**Step 3: Inspect Data Flow**
- Echo topics at each stage
- Verify message content and rate
- Check timestamps

**Step 4: Analyze Transform Tree**
- Generate tf tree visualization
- Verify all required frames present
- Check transform timestamps

**Step 5: Timing Analysis**
- Measure publication rates
- Check callback execution time
- Identify bottlenecks

**Step 6: Iterative Isolation**
- Remove components until system works
- Add back one at a time
- Identify problematic interaction

<!-- class="alternative-approach" -->
**Debugging Strategies**

**Binary Search**: Disable half the nodes, narrow down culprit

**Known Good State**: Compare to working configuration

**Incremental Integration**: Add components one at a time, test after each

**Simulation First**: Integrate in Gazebo before deploying to hardware

---

## Module 5: Containerization and Deployment

### 5.1 Docker Fundamentals

<!-- class="theory-concept" -->
**Containerization Concepts**

**Container vs. Virtual Machine**:

**Virtual Machine**:
- Full OS per VM
- Heavy (GBs)
- Slow startup (minutes)
- Hypervisor overhead

**Container**:
- Shares host kernel
- Lightweight (MBs)
- Fast startup (seconds)
- Minimal overhead

**Docker Architecture**:
- **Image**: Read-only template (filesystem + metadata)
- **Container**: Running instance of image
- **Dockerfile**: Script to build image
- **Registry**: Repository for images (Docker Hub, private registry)

<!-- class="theory-concept" -->
**Benefits for Robotics**

**Reproducibility**:
- Exact dependencies specified
- "Works on my machine" → "Works everywhere"
- Version control for entire environment

**Isolation**:
- Multiple projects on same machine
- No dependency conflicts
- Clean system

**Deployment**:
- Package once, deploy anywhere
- Consistent production environment
- Easy rollback to previous versions

**Testing**:
- CI/CD pipelines
- Automated testing in clean environment

### 5.2 Dockerfile for ROS2

<!-- class="theory-concept" -->
**Building ROS2 Images**

**Example Dockerfile**:

```dockerfile
# Base image with ROS2 Humble
FROM osrf/ros:humble-desktop

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws

# Copy source code
COPY src/ src/

# Build workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Source workspace in bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set entry point
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
```

**Multi-Stage Builds**: Reduce final image size
```dockerfile
# Build stage
FROM osrf/ros:humble-desktop AS builder
WORKDIR /ros2_ws
COPY src/ src/
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# Runtime stage
FROM osrf/ros:humble-ros-base
COPY --from=builder /ros2_ws/install /ros2_ws/install
```

### 5.3 Docker Compose for Multi-Container Systems

<!-- class="theory-concept" -->
**Orchestrating Multiple Containers**

**docker-compose.yml**:

```yaml
version: '3'

services:
  perception:
    build: ./perception
    environment:
      - ROS_DOMAIN_ID=42
    volumes:
      - ./config:/config
    network_mode: host

  navigation:
    build: ./navigation
    environment:
      - ROS_DOMAIN_ID=42
    depends_on:
      - perception
    network_mode: host

  visualization:
    build: ./visualization
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=42
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
    network_mode: host
```

**Key Concepts**:
- **Services**: Individual containers
- **network_mode: host**: Share host network (ROS2 multicast)
- **volumes**: Mount host directories
- **depends_on**: Start order
- **environment**: Environment variables

**Commands**:
```bash
# Build all services
docker-compose build

# Start all services
docker-compose up

# Start in background
docker-compose up -d

# View logs
docker-compose logs -f

# Stop all services
docker-compose down
```

**Quiz: Containerization**

What is the primary advantage of containers over virtual machines?

[( )] Better security
[(X)] Lower resource overhead
[( )] Easier to configure
[( )] More features

In Docker Compose, what does `network_mode: host` do?

[(X)] Container uses host network stack directly
[( )] Creates isolated network for containers
[( )] Disables networking
[( )] Enables internet access

---

## Module 6: Launch Systems

### 6.1 Launch File Architecture

<!-- class="theory-concept" -->
**ROS2 Launch System**

Launch files coordinate starting multiple nodes with parameters:

**Launch File Format**: Python (ROS2) vs XML (ROS1)

**Example Launch File**:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Node 1: Camera driver
    camera_node = Node(
        package='camera_driver',
        executable='camera_node',
        name='camera',
        parameters=[{
            'frame_id': 'camera_link',
            'fps': 30,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/image', '/camera/image_raw')
        ]
    )

    # Node 2: Object detector
    detector_node = Node(
        package='object_detection',
        executable='detector_node',
        name='detector',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('object_detection'),
                'config',
                'detector_params.yaml'
            ])
        ]
    )

    # Include another launch file
    nav_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),
            'launch',
            'navigation_launch.py'
        ])
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time'),
        camera_node,
        detector_node,
        nav_launch
    ])
```

**Launch Features**:
- Start multiple nodes
- Set parameters from YAML files
- Remap topics
- Include other launch files
- Conditional logic
- Event handlers

### 6.2 Parameter Configuration

<!-- class="theory-concept" -->
**YAML Parameter Files**

Externalize configuration for flexibility:

**params.yaml**:
```yaml
camera_node:
  ros__parameters:
    frame_id: "camera_link"
    fps: 30
    exposure: 10
    gain: 2.0
    image_width: 640
    image_height: 480

detector_node:
  ros__parameters:
    confidence_threshold: 0.7
    nms_threshold: 0.3
    classes: ["person", "robot", "obstacle"]
```

**Loading in Launch File**:
```python
Node(
    package='camera_driver',
    executable='camera_node',
    parameters=['/path/to/params.yaml']
)
```

**Dynamic Reconfiguration**:
```python
# In node callback
def parameter_callback(self, params):
    for param in params:
        if param.name == 'confidence_threshold':
            self.threshold = param.value
    return SetParametersResult(successful=True)

# Register callback
self.add_on_set_parameters_callback(self.parameter_callback)
```

---

## Summary

This course established integration methodologies for complex robotic systems:

1. **Middleware Architecture**: Analyzed pub-sub patterns and DDS communication
2. **QoS Policies**: Configured reliability, durability, and timing for real-time performance
3. **Coordinate Transformations**: Implemented tf2 trees for spatial reasoning
4. **Debugging**: Applied systematic diagnostic techniques to integration issues
5. **Containerization**: Created reproducible deployments using Docker
6. **Launch Systems**: Orchestrated multi-node systems with configuration management

**Key Takeaways**:
- Middleware enables loose coupling and scalability
- QoS policies must be compatible for communication
- Transform trees provide spatial context across sensors
- Systematic debugging isolates integration failures
- Containerization ensures reproducibility
- Launch files coordinate complex system startup

**Next Steps**: SYSTEMS-203 develops comprehensive testing and reliability engineering practices to ensure integrated systems remain functional.

---

## Optional Laboratory Exercises

### Lab 1: QoS Policy Configuration (2 hours)

<!-- class="optional-exercise" -->
**Objective**: Experiment with QoS policies to understand their effects on communication reliability and performance.

**Setup**: Two nodes: publisher and subscriber

**Tasks**:

1. **Baseline Communication** (20 min):
   - Create simple publisher (10 Hz, counter messages)
   - Create subscriber that logs received messages
   - Run both, verify all messages received

2. **Reliability Experiments** (30 min):
   - Publisher: BEST_EFFORT, Subscriber: BEST_EFFORT
   - Publisher: RELIABLE, Subscriber: RELIABLE
   - Publisher: BEST_EFFORT, Subscriber: RELIABLE (fails!)
   - Add artificial packet loss (network simulator or tc command)
   - Measure delivery rate for each configuration

3. **Durability Experiments** (30 min):
   - Publisher: TRANSIENT_LOCAL, History: KEEP_LAST(10)
   - Start publisher, send messages
   - Start subscriber after 5 seconds
   - Verify subscriber receives historical messages
   - Compare with VOLATILE durability

4. **Deadline Experiments** (30 min):
   - Set deadline to 200ms
   - Publisher rate: 10 Hz (meets deadline)
   - Reduce rate to 1 Hz (violates deadline)
   - Implement deadline callback in subscriber
   - Log missed deadlines

<!-- class="exercise-tip" -->
**Python QoS Configuration Example**:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

self.publisher = self.create_publisher(
    String,
    '/my_topic',
    qos_profile
)
```

**Analysis Questions**:
- When should you use BEST_EFFORT vs RELIABLE?
- What is the latency difference between them?
- How does TRANSIENT_LOCAL help late-joining subscribers?
- What happens when deadline is violated?

### Lab 2: Transform Tree Debugging (2 hours)

<!-- class="optional-exercise" -->
**Objective**: Build and debug a multi-frame robot system with sensor transformations.

**Scenario**: Mobile robot with LiDAR and camera

**Frame Structure**:
```
world → odom → base_link → laser_link
                         → camera_link
```

**Tasks**:

1. **Static Transform Publisher** (20 min):
   - Publish static transform: base_link → laser_link (0.1m forward, 0.2m up)
   - Publish static transform: base_link → camera_link (0.15m forward, 0.25m up, 10° pitch)
   - Verify with `ros2 run tf2_tools view_frames`

2. **Dynamic Transform** (30 min):
   - Create odometry node publishing odom → base_link
   - Simulate robot motion (circular trajectory)
   - Verify transform updates at 20 Hz
   - Check with `ros2 run tf2_ros tf2_echo odom base_link`

3. **Introduced Errors** (40 min):
   - **Error 1**: Remove laser_link broadcaster
     - Attempt transform lookup from odom to laser_link
     - Observe "frame does not exist" error
     - Fix by restoring broadcaster

   - **Error 2**: Publish base_link → odom (reversed!)
     - Observe disconnected tree
     - Fix direction

   - **Error 3**: Timestamp in future
     - Publish transform with time + 1 second
     - Observe extrapolation error
     - Fix timestamps

4. **Sensor Fusion Example** (30 min):
   - Transform laser scan from laser_link to odom frame
   - Transform camera point from camera_link to odom frame
   - Verify correct spatial relationships

<!-- class="exercise-advanced" -->
**Advanced Extensions**:
- Implement TF buffer with 30-second history
- Use message_filters to synchronize camera and LiDAR
- Add moving obstacle frame (second dynamic transform)
- Compute relative velocity between robot and obstacle

### Lab 3: Docker Integration Project (3 hours)

<!-- class="optional-exercise" -->
**Objective**: Containerize a multi-node ROS2 application and deploy with Docker Compose.

**System**: Simulated robot with perception and navigation

**Tasks**:

1. **Create Base Image** (30 min):
   - Write Dockerfile extending `osrf/ros:humble-desktop`
   - Install common dependencies
   - Set up workspace structure
   - Build and test

2. **Perception Container** (45 min):
   - Create package with camera driver and object detector
   - Write Dockerfile
   - Copy source code
   - Build ROS2 workspace in container
   - Test node launches correctly

3. **Navigation Container** (45 min):
   - Create package with motion planner
   - Write Dockerfile (can inherit from base image)
   - Build workspace
   - Verify communication with perception container

4. **Docker Compose Integration** (45 min):
   - Write docker-compose.yml for both containers
   - Configure shared ROS_DOMAIN_ID
   - Set network_mode to host
   - Mount configuration directory
   - Launch entire system with `docker-compose up`

5. **Verification** (15 min):
   - From host, run `ros2 topic list`
   - Verify topics from both containers visible
   - Echo messages to confirm communication
   - Test system with different configurations

<!-- class="exercise-tip" -->
**Dockerfile Best Practices**:
- Use specific image tags (not `latest`)
- Minimize layers (combine RUN commands)
- Clean apt cache to reduce size
- Use .dockerignore to exclude unnecessary files
- Multi-stage builds for smaller images

**Debugging Tips**:
```bash
# Enter running container
docker exec -it container_name bash

# View container logs
docker logs -f container_name

# Inspect container
docker inspect container_name

# Remove all stopped containers
docker container prune
```

### Lab 4: Launch File Design (1.5 hours)

<!-- class="optional-exercise" -->
**Objective**: Create comprehensive launch file for complex system with parameter management.

**System**: Autonomous navigation stack

**Components**:
- Sensor drivers (camera, LiDAR, IMU)
- Localization (AMCL)
- Mapping (SLAM Toolbox)
- Planning (Nav2)
- Visualization (RViz)

**Tasks**:

1. **Modular Launch Files** (30 min):
   - `sensors.launch.py`: All sensor drivers
   - `localization.launch.py`: Localization nodes
   - `navigation.launch.py`: Planning and control
   - `main.launch.py`: Include all above

2. **Parameter Management** (30 min):
   - Create YAML files for each component
   - Use LaunchConfiguration for runtime arguments
   - Implement conditional launching (sim vs. real)

3. **Namespace and Remapping** (20 min):
   - Launch same nodes twice with different namespaces (two robots)
   - Remap topics to avoid conflicts
   - Verify with rqt_graph

4. **Event Handlers** (10 min):
   - Launch visualization after navigation starts
   - Restart node if it crashes (respawn)

**Example Structure**:
```python
use_sim = LaunchConfiguration('sim', default='false')

return LaunchDescription([
    DeclareLaunchArgument('sim'),

    # Conditional: simulation or real sensors
    GroupAction(
        condition=IfCondition(use_sim),
        actions=[...simulation nodes...]
    ),
    GroupAction(
        condition=UnlessCondition(use_sim),
        actions=[...real sensor nodes...]
    ),

    # Always launch navigation
    IncludeLaunchDescription(navigation_launch),
])
```

---

*Version 2.0.0 - Academic curriculum emphasizing theoretical foundations with optional practical exercises*
