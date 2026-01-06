<!--
author:   Dr. Michael Chen
email:    michael.chen@academy-of-robotic-sciences.github.io
version:  2.0.0

language: en
narrator: US English Male

comment:  Reliability and Verification: Comprehensive treatment of software testing theory, simulation-based verification, continuous integration, fault tolerance, and formal methods for safety-critical robotic systems.


mode:     Textbook

link:     https://raw.githubusercontent.com/Academy-of-Robotic-Sciences/courses/main/course-styles.css

import:   https://raw.githubusercontent.com/LiaTemplates/Pyodide/master/README.md
-->

# SYSTEMS-203: Reliability and Verification

> **Reliable systems emerge from rigorous testing, formal verification, and systematic reliability engineering.**

## Course Overview

| | |
|---|---|
| **Course Code** | SYSTEMS-203 |
| **Duration** | 8 hours |
| **Level** | Advanced |
| **Prerequisites** | SYSTEMS-202, software testing fundamentals, statistics |
| **Theory-Practice** | 60% theory, 40% optional labs |

---

## Learning Objectives

Upon completion of this course, students will be able to:

### Theoretical Understanding

- Explain software testing theory including the testing pyramid and coverage metrics
- Understand reliability theory and failure rate analysis
- Apply fault tree analysis (FTA) and failure mode effects analysis (FMEA)
- Describe simulation-based verification methodologies
- Understand continuous integration and deployment (CI/CD) principles
- Explain formal verification methods for safety-critical systems

### Practical Skills

- Design comprehensive test suites for ROS2 nodes (unit, integration, system)
- Implement simulation-based testing using Gazebo
- Configure CI/CD pipelines with GitHub Actions
- Analyze test coverage and identify gaps
- Implement fault injection and recovery mechanisms
- Create automated regression test frameworks

---

## Module 1: Software Testing Theory

### 1.1 The Testing Pyramid

<!-- class="theory-concept" -->
**Levels of Testing**

The testing pyramid represents optimal distribution of tests:

```
           /\
          /  \  E2E (End-to-End)
         /----\
        /      \ Integration Tests
       /--------\
      /          \ Unit Tests
     /____________\
```

**Unit Tests** (Base, 70%):
- Test individual functions/classes in isolation
- Fast execution (<1ms per test)
- No external dependencies (mock I/O, databases, network)
- High quantity, fine-grained
- Example: Test PID controller calculation logic

**Integration Tests** (Middle, 20%):
- Test interactions between components
- Moderate speed (~100ms per test)
- Tests interfaces and data flow
- Example: Test communication between two ROS2 nodes

**End-to-End Tests** (Top, 10%):
- Test complete system behavior
- Slow execution (seconds to minutes)
- Tests in realistic environment (simulation or hardware)
- Example: Test robot navigating to goal in Gazebo

**Rationale**:
- Many fast unit tests catch bugs early
- Fewer integration tests verify component interactions
- Minimal E2E tests confirm system-level behavior
- Inverted pyramid (many E2E, few unit) is anti-pattern: slow, brittle, expensive

<!-- class="historical-note" -->
**Evolution of Software Testing**

**1950s**: **Ad-hoc Testing**. Manual testing by developers, no systematic approach.

**1960s**: **Debugging Phase**. Glenford Myers' "The Art of Software Testing" (1979) established testing as distinct from debugging.

**1970s**: **Structured Testing**. Introduction of code coverage metrics and systematic test case design.

**1980s**: **Test Automation**. Emergence of automated testing tools and frameworks.

**1990s**: **Unit Testing Frameworks**. Kent Beck's SUnit (Smalltalk) inspired JUnit (1997), spawning xUnit family.

**2000s**: **Test-Driven Development (TDD)**. Kent Beck popularized writing tests before code.

**2010s**: **Continuous Integration**. Automated testing integrated into build pipelines, DevOps culture.

**2020s**: **AI-Assisted Testing**. Machine learning generates test cases, analyzes coverage gaps.

### 1.2 Coverage Metrics

<!-- class="theory-concept" -->
**Measuring Test Completeness**

**Line Coverage**: Percentage of code lines executed by tests
$$\text{Line Coverage} = \frac{\text{Lines Executed}}{\text{Total Lines}} \times 100\%$$

**Branch Coverage**: Percentage of decision branches taken
- All if/else paths covered
- More stringent than line coverage

**Function Coverage**: Percentage of functions called

**Path Coverage**: All possible execution paths
- Combinatorial explosion for complex code
- Impractical for complete coverage

**Modified Condition/Decision Coverage (MC/DC)**:
- Every condition independently affects decision outcome
- Required for DO-178C (aviation software)
- Thorough but achievable

**Target Coverage**:
- Unit tests: 80-90% line coverage
- Critical systems: >95% with MC/DC
- 100% coverage ≠ bug-free (tests may not check correctness)

<!-- class="theory-concept" -->
**Mutation Testing**

Tests the tests by introducing bugs:

1. **Mutate** code (change operator, constant, condition)
2. **Run tests** on mutated code
3. **Check**: Did tests catch the bug?

**Mutation Score**:
$$\text{Mutation Score} = \frac{\text{Killed Mutants}}{\text{Total Mutants}} \times 100\%$$

High mutation score indicates quality tests, not just coverage.

**Quiz: Testing Theory**

In the testing pyramid, what percentage should be unit tests?

[( )] 10%
[( )] 30%
[( )] 50%
[(X)] 70%

What does 100% line coverage guarantee?

[( )] No bugs in code
[( )] All branches tested
[(X)] All lines executed at least once
[( )] Code is production-ready

---

## Module 2: Unit Testing for ROS2

### 2.1 Unit Testing Fundamentals

<!-- class="theory-concept" -->
**Principles of Good Unit Tests**

**FIRST Principles**:
- **Fast**: Execute quickly (<1ms ideal)
- **Independent**: No dependencies between tests
- **Repeatable**: Same result every run
- **Self-Validating**: Pass/fail, no manual interpretation
- **Timely**: Written alongside or before code (TDD)

**Arrange-Act-Assert (AAA) Pattern**:
```python
def test_pid_controller():
    # Arrange: Set up test conditions
    controller = PIDController(kp=1.0, ki=0.1, kd=0.01)

    # Act: Execute the code under test
    output = controller.compute(setpoint=10.0, measured=8.0)

    # Assert: Verify expected behavior
    assert output > 0  # Positive error should increase control
```

**Test Fixtures**: Setup/teardown for consistent state
- `setUp()`: Run before each test
- `tearDown()`: Clean up after each test

### 2.2 ROS2 Unit Testing with pytest

<!-- class="theory-concept" -->
**Testing ROS2 Nodes**

**Example: Testing a Publisher Node**

```python
import rclpy
from std_msgs.msg import String
import pytest

class PublisherNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'test_topic', 10)

    def publish_message(self, text):
        msg = String()
        msg.data = text
        self.publisher.publish(msg)

# Unit Test (no actual ROS2 execution)
def test_message_creation():
    """Test message creation without running ROS2"""
    node = PublisherNode()

    # We can test node initialization
    assert node.get_name() == 'publisher_node'

    # Test message creation
    msg = String()
    msg.data = "test"
    assert msg.data == "test"
```

**Mocking External Dependencies**:
```python
from unittest.mock import Mock, patch

def test_with_mock():
    """Test with mocked publisher"""
    node = PublisherNode()
    node.publisher = Mock()

    # Call method that publishes
    node.publish_message("hello")

    # Verify publish was called
    assert node.publisher.publish.called

    # Verify message content
    call_args = node.publisher.publish.call_args
    assert call_args[0][0].data == "hello"
```

### 2.3 Testing Pure Logic

<!-- class="theory-concept" -->
**Separating Logic from ROS2**

Best practice: Extract algorithms from ROS2 nodes for easier testing

**Before** (tightly coupled):
```python
class ControllerNode(Node):
    def callback(self, msg):
        # PID logic directly in callback
        error = self.setpoint - msg.data
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        # Publish output...
```

**After** (separated):
```python
# Pure Python class (easy to test)
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, measured, dt):
        error = setpoint - measured
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# ROS2 Node (thin wrapper)
class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller')
        self.controller = PIDController(1.0, 0.1, 0.01)

    def callback(self, msg):
        output = self.controller.compute(self.setpoint, msg.data, self.dt)
        # Publish output...

# Easy unit test (no ROS2 required)
def test_pid_proportional():
    pid = PIDController(kp=2.0, ki=0.0, kd=0.0)
    output = pid.compute(setpoint=10.0, measured=5.0, dt=0.1)
    assert output == 10.0  # P-only: output = kp * error = 2.0 * 5.0
```

---

## Module 3: Integration Testing

### 3.1 ROS2 Integration Testing

<!-- class="theory-concept" -->
**Testing Node Communication**

Integration tests verify nodes interact correctly:

**launch_testing Framework**:
```python
import unittest
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import rclpy
from std_msgs.msg import String

def generate_test_description():
    # Launch nodes for testing
    talker = Node(
        package='demo_nodes_cpp',
        executable='talker'
    )

    listener = Node(
        package='demo_nodes_py',
        executable='listener'
    )

    return (
        LaunchDescription([
            talker,
            listener,
            launch_testing.actions.ReadyToTest()
        ]),
        {'talker': talker, 'listener': listener}
    )

class TestTalkerListener(unittest.TestCase):
    def test_communication(self):
        """Verify talker and listener communicate"""
        rclpy.init()
        node = rclpy.create_node('test_node')

        received_msgs = []

        def callback(msg):
            received_msgs.append(msg.data)

        # Subscribe to topic
        subscription = node.create_subscription(
            String,
            'chatter',
            callback,
            10
        )

        # Wait for messages
        timeout = 10.0
        start = node.get_clock().now()
        while len(received_msgs) < 3:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (node.get_clock().now() - start).nanoseconds / 1e9 > timeout:
                break

        # Verify messages received
        self.assertGreater(len(received_msgs), 0, "No messages received")
        self.assertIn("Hello World", received_msgs[0])

        node.destroy_node()
        rclpy.shutdown()
```

### 3.2 Message Synchronization Testing

<!-- class="theory-concept" -->
**Testing Temporal Alignment**

For sensor fusion, verify timestamps align:

```python
def test_time_synchronization():
    """Test that camera and LiDAR messages are synchronized"""
    camera_times = []
    lidar_times = []

    def sync_callback(camera_msg, lidar_msg):
        camera_time = camera_msg.header.stamp
        lidar_time = lidar_msg.header.stamp

        # Compute time difference
        diff = abs((camera_time.sec + camera_time.nanosec / 1e9) -
                   (lidar_time.sec + lidar_time.nanosec / 1e9))

        # Assert synchronization within 100ms
        assert diff < 0.1, f"Messages not synchronized: {diff}s difference"
```

### 3.3 Transform Testing

<!-- class="theory-concept" -->
**Verifying tf2 Trees**

Test coordinate transformations:

```python
from tf2_ros import Buffer, TransformListener

def test_transform_tree():
    """Verify all required transforms exist and are valid"""
    rclpy.init()
    node = rclpy.create_node('test_transforms')

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    # Wait for transforms to be published
    time.sleep(2.0)

    # Test 1: Verify frames exist
    frames = ['map', 'odom', 'base_link', 'laser_link']
    for frame in frames:
        assert tf_buffer.can_transform('map', frame, rclpy.time.Time())

    # Test 2: Verify transform values
    transform = tf_buffer.lookup_transform('base_link', 'laser_link',
                                           rclpy.time.Time())

    # Laser should be 0.1m forward (x-axis)
    assert abs(transform.transform.translation.x - 0.1) < 0.01
    assert abs(transform.transform.translation.y) < 0.01

    node.destroy_node()
    rclpy.shutdown()
```

**Quiz: Integration Testing**

What distinguishes integration tests from unit tests?

[( )] Integration tests are faster
[(X)] Integration tests verify component interactions
[( )] Integration tests don't use assertions
[( )] Integration tests require hardware

---

## Module 4: Simulation-Based Testing

### 4.1 Gazebo Simulation Testing

<!-- class="theory-concept" -->
**Benefits of Simulation Testing**

**Advantages**:
- **Repeatability**: Deterministic environment
- **Speed**: Faster than real-time possible
- **Safety**: Test dangerous scenarios (collisions, failures)
- **Cost**: No hardware wear, automated testing
- **Coverage**: Test edge cases difficult in reality

**Limitations**:
- **Reality Gap**: Physics approximations
- **Sensor Modeling**: Simplified noise characteristics
- **Computational**: Resource-intensive for complex scenes
- **Validation**: Still need real-world testing

<!-- class="theory-concept" -->
**Simulation Test Structure**

**Example: Navigation Test**

```python
def test_navigation_to_goal():
    """Test robot navigates to goal without collisions"""

    # 1. Launch Gazebo with test world
    launch_gazebo(world='test_navigation.world')

    # 2. Spawn robot at starting position
    spawn_robot(x=0.0, y=0.0, theta=0.0)

    # 3. Send navigation goal
    send_goal(x=5.0, y=5.0)

    # 4. Monitor robot state
    timeout = 60.0  # seconds
    start_time = time.time()
    collision_detected = False
    goal_reached = False

    while time.time() - start_time < timeout:
        # Check for collisions
        if check_collision():
            collision_detected = True
            break

        # Check if goal reached
        robot_pose = get_robot_pose()
        distance = compute_distance(robot_pose, goal=(5.0, 5.0))
        if distance < 0.1:
            goal_reached = True
            break

        time.sleep(0.1)

    # 5. Assertions
    assert not collision_detected, "Robot collided with obstacle"
    assert goal_reached, "Robot did not reach goal within timeout"
```

### 4.2 Physics-Based Verification

<!-- class="theory-concept" -->
**Testing Physical Constraints**

Verify simulated behavior matches physics:

**Example: Testing Kinematics**

```python
def test_velocity_limits():
    """Verify robot respects velocity limits"""

    # Send high-speed command
    send_velocity_command(linear=10.0, angular=5.0)

    # Monitor actual velocities
    max_linear = 0.0
    max_angular = 0.0

    for _ in range(100):  # 10 seconds at 10Hz
        vel = get_robot_velocity()
        max_linear = max(max_linear, abs(vel.linear))
        max_angular = max(max_angular, abs(vel.angular))
        time.sleep(0.1)

    # Robot should not exceed physical limits
    assert max_linear <= 0.5, f"Linear velocity {max_linear} exceeds limit 0.5 m/s"
    assert max_angular <= 1.0, f"Angular velocity {max_angular} exceeds limit 1.0 rad/s"
```

**Example: Testing Dynamics**

```python
def test_acceleration_limits():
    """Verify realistic acceleration"""

    prev_velocity = 0.0
    max_acceleration = 0.0
    dt = 0.1

    # Command sudden velocity change
    send_velocity_command(linear=0.5)

    for _ in range(50):
        current_velocity = get_robot_velocity().linear
        acceleration = (current_velocity - prev_velocity) / dt
        max_acceleration = max(max_acceleration, abs(acceleration))
        prev_velocity = current_velocity
        time.sleep(dt)

    # Acceleration should be physically realistic
    assert max_acceleration < 2.0, "Unrealistic acceleration detected"
```

### 4.3 Fault Injection Testing

<!-- class="theory-concept" -->
**Testing Failure Modes**

Deliberately introduce faults to test robustness:

**Sensor Failure**:
```python
def test_lidar_failure_recovery():
    """Test system handles LiDAR failure gracefully"""

    # Normal operation
    send_goal(x=5.0, y=0.0)
    time.sleep(2.0)

    # Inject fault: Stop LiDAR publishing
    stop_sensor('/scan')

    # Monitor system behavior
    timeout = 5.0
    start = time.time()
    emergency_stop = False

    while time.time() - start < timeout:
        status = get_robot_status()
        if status == 'EMERGENCY_STOP':
            emergency_stop = True
            break
        time.sleep(0.1)

    # System should detect failure and stop safely
    assert emergency_stop, "System did not stop after sensor failure"

    # Verify robot velocity is zero
    vel = get_robot_velocity()
    assert abs(vel.linear) < 0.01, "Robot still moving after emergency stop"
```

**Network Latency**:
```python
def test_high_latency_tolerance():
    """Test system tolerates network delays"""

    # Inject network delay (simulate WiFi congestion)
    inject_latency('/cmd_vel', delay_ms=200)

    # Send navigation commands
    send_goal(x=3.0, y=3.0)

    # System should still reach goal despite delays
    goal_reached = wait_for_goal(timeout=120.0)
    assert goal_reached, "System failed under network latency"
```

---

## Module 5: Continuous Integration and Deployment

### 5.1 CI/CD Principles

<!-- class="theory-concept" -->
**Continuous Integration**

**Definition**: Automatically build, test, and integrate code changes frequently (multiple times per day).

**Goals**:
- Catch integration problems early
- Reduce integration effort
- Maintain deployable codebase
- Enable rapid iteration

**CI Workflow**:
1. Developer commits code to repository
2. CI server detects commit
3. Automatically builds code
4. Runs automated test suite
5. Reports results (pass/fail)
6. Optionally deploys if tests pass

**Benefits**:
- Bugs found within hours, not weeks
- Reduced integration risk
- Confidence in refactoring
- Automated quality checks

<!-- class="historical-note" -->
**Evolution of CI/CD**

**1990s**: **Daily Builds**. Microsoft's "daily build and smoke test" improved stability.

**2000**: **Continuous Integration**. Martin Fowler popularized CI practices with tools like CruiseControl.

**2005**: **Hudson/Jenkins**. First widely-adopted open-source CI server.

**2010s**: **Cloud CI**. Travis CI (2011), CircleCI (2011) offered hosted CI.

**2018**: **GitHub Actions**. Integrated CI/CD directly into GitHub, lowering barriers.

**2020s**: **GitOps and CD**. Continuous Deployment to production becomes standard for web services; robotics adoption growing.

### 5.2 GitHub Actions for ROS2

<!-- class="theory-concept" -->
**Configuring CI Pipeline**

**Example: `.github/workflows/ros2_ci.yml`**

```yaml
name: ROS2 CI

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]

jobs:
  test:
    runs-on: ubuntu-22.04

    steps:
    - name: Checkout repository
      uses: actions/checkout@v3

    - name: Setup ROS2
      uses: ros-tooling/setup-ros@v0.6
      with:
        required-ros-distributions: humble

    - name: Install dependencies
      run: |
        sudo apt-get update
        rosdep update
        rosdep install --from-paths src --ignore-src -r -y

    - name: Build workspace
      run: |
        source /opt/ros/humble/setup.bash
        colcon build --symlink-install

    - name: Run tests
      run: |
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        colcon test

    - name: Check test results
      run: |
        colcon test-result --verbose
```

**Advanced Features**:
- **Matrix Builds**: Test multiple ROS2 distributions
- **Caching**: Speed up builds by caching dependencies
- **Coverage**: Generate and upload coverage reports
- **Artifacts**: Store test results and build outputs

### 5.3 Test Automation Best Practices

<!-- class="theory-concept" -->
**Maintaining Reliable CI**

**Fast Feedback**:
- CI pipeline should complete in <10 minutes
- Use parallelization (run tests concurrently)
- Cache dependencies
- Run unit tests before integration tests (fail fast)

**Reliability**:
- Tests must be deterministic (no flaky tests)
- Avoid time-dependent tests (use simulation time)
- Proper cleanup between tests
- Retry only on known transient issues

**Scalability**:
- Separate fast and slow tests
- Run expensive tests nightly, not on every commit
- Use test tags to select subsets

**Notifications**:
- Alert team on failures
- Assign failures to committer
- Track test stability metrics

---

## Module 6: Reliability Engineering

### 6.1 Reliability Theory

<!-- class="theory-concept" -->
**Quantifying Reliability**

**Reliability** R(t): Probability system operates without failure for time t

For constant failure rate λ (exponential distribution):
$$R(t) = e^{-\lambda t}$$

**Failure Rate** λ: Failures per unit time

**Mean Time Between Failures (MTBF)**:
$$MTBF = \frac{1}{\lambda}$$

**Example**: If λ = 0.001 failures/hour, MTBF = 1000 hours

**Mean Time To Repair (MTTR)**: Average time to restore after failure

**Availability**: Fraction of time system is operational
$$A = \frac{MTBF}{MTBF + MTTR}$$

<!-- class="theory-concept" -->
**System Reliability**

**Series System** (all components must work):
$$R_{sys} = R_1 \times R_2 \times \cdots \times R_n$$

Example: If R₁ = R₂ = R₃ = 0.99, then R_sys = 0.99³ = 0.970

**Parallel System (Redundancy)** (at least one must work):
$$R_{sys} = 1 - (1-R_1)(1-R_2)\cdots(1-R_n)$$

Example: Two components R = 0.9 in parallel:
R_sys = 1 - (1-0.9)² = 1 - 0.01 = 0.99

**Design Implication**: Redundancy improves reliability but increases cost and complexity.

### 6.2 Fault Tree Analysis (FTA)

<!-- class="theory-concept" -->
**Top-Down Failure Analysis**

FTA identifies combinations of faults leading to system failure:

**Example: Robot Collision**

```
           Robot Collision (Top Event)
                    |
          +---------+---------+
          |                   |
    Sensor Failure      Control Failure
          |                   |
    +-----+-----+       +-----+-----+
    |           |       |           |
LiDAR Fail  Camera  Software  Actuator
              Fail    Bug       Jam
```

**Logic Gates**:
- **AND Gate**: All inputs must fail for output failure
- **OR Gate**: Any input failure causes output failure

**Quantitative Analysis**:

If independent events:
- OR gate: $P(A \cup B) = P(A) + P(B) - P(A)P(B)$
- AND gate: $P(A \cap B) = P(A) \times P(B)$

**Example Calculation**:

Assume:
- LiDAR failure: P = 0.01
- Camera failure: P = 0.02
- Software bug: P = 0.001
- Actuator jam: P = 0.005

Sensor Failure = LiDAR OR Camera:
$P = 0.01 + 0.02 - (0.01)(0.02) = 0.0298$

Control Failure = Software OR Actuator:
$P = 0.001 + 0.005 - (0.001)(0.005) = 0.006$

Collision = Sensor AND Control:
$P = 0.0298 \times 0.006 = 0.000179$ (1.79 in 10,000)

### 6.3 Failure Mode and Effects Analysis (FMEA)

<!-- class="theory-concept" -->
**Bottom-Up Failure Analysis**

FMEA systematically analyzes how each component can fail:

**FMEA Table Structure**:

| Component | Failure Mode | Effects | Severity | Likelihood | Detection | RPN |
|-----------|--------------|---------|----------|------------|-----------|-----|
| LiDAR | No data | Blind navigation | 9 | 2 | 8 | 144 |
| Battery | Low voltage | Unexpected shutdown | 8 | 5 | 2 | 80 |
| Motor | Stall | Immobility | 6 | 3 | 7 | 126 |

**Ratings (1-10)**:
- **Severity**: Impact of failure (1=minor, 10=catastrophic)
- **Likelihood**: Probability of occurrence (1=rare, 10=frequent)
- **Detection**: Ability to detect before harm (1=certain, 10=none)

**Risk Priority Number (RPN)**:
$$RPN = Severity \times Likelihood \times Detection$$

Higher RPN → higher priority for mitigation.

**Mitigation Strategies**:
- Reduce Severity: Add redundancy, graceful degradation
- Reduce Likelihood: Better components, preventive maintenance
- Improve Detection: Monitoring, diagnostics, self-tests

**Quiz: Reliability**

Two components with reliability 0.95 in series have combined reliability:

[( )] 0.95
[(X)] 0.9025
[( )] 0.9975
[( )] 1.90

What does MTBF stand for?

[( )] Mean Time Before Failure
[(X)] Mean Time Between Failures
[( )] Maximum Time Before Failure
[( )] Minimum Time Between Failures

---

## Summary

This course established comprehensive testing and reliability engineering practices:

1. **Testing Theory**: Applied testing pyramid and coverage metrics
2. **Unit Testing**: Tested isolated components with pytest
3. **Integration Testing**: Verified node communication and transforms
4. **Simulation Testing**: Used Gazebo for physics-based verification
5. **CI/CD**: Automated testing with GitHub Actions
6. **Reliability Engineering**: Applied FTA, FMEA, and reliability theory

**Key Takeaways**:
- Testing pyramid optimizes test distribution for speed and coverage
- Unit tests provide fast feedback, integration tests verify interactions
- Simulation enables comprehensive testing without hardware
- CI/CD ensures continuous code quality
- Reliability engineering quantifies and improves system dependability
- Formal methods complement testing for safety-critical systems

**Next Steps**: SYSTEMS-204 applies these quality assurance practices to production deployment with monitoring and maintenance.

---

## Optional Laboratory Exercises

### Lab 1: Unit Test Suite Development (2.5 hours)

<!-- class="optional-exercise" -->
**Objective**: Build comprehensive unit test suite with high coverage for a ROS2 package.

**Given**: Simple motion controller package with PID logic

**Tasks**:

1. **Setup Testing Infrastructure** (20 min):
   - Install pytest and coverage tools
   - Create test directory structure
   - Configure colcon for testing

2. **Write Unit Tests** (90 min):
   - Test PID computation (proportional, integral, derivative separately)
   - Test edge cases (zero error, saturation, negative values)
   - Test anti-windup logic
   - Test parameter validation
   - Achieve >90% line coverage

3. **Measure Coverage** (15 min):
   - Run pytest with coverage
   - Generate coverage report
   - Identify untested branches

4. **Improve Tests** (30 min):
   - Add tests for uncovered lines
   - Use parameterized tests for multiple inputs
   - Add negative tests (invalid inputs)

<!-- class="exercise-tip" -->
**Example Test Structure**:

```python
import pytest
from motion_controller.pid import PIDController

class TestPIDController:
    def test_proportional_only(self):
        """P-only controller output equals Kp * error"""
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0)
        output = pid.compute(setpoint=10.0, measured=5.0, dt=0.1)
        assert output == pytest.approx(10.0)  # 2.0 * 5.0

    @pytest.mark.parametrize("setpoint,measured,expected", [
        (10.0, 5.0, 10.0),
        (0.0, 0.0, 0.0),
        (5.0, 10.0, -10.0),
    ])
    def test_proportional_values(self, setpoint, measured, expected):
        """Test P controller with various inputs"""
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0)
        output = pid.compute(setpoint, measured, dt=0.1)
        assert output == pytest.approx(expected)
```

**Coverage Commands**:
```bash
# Run tests with coverage
pytest --cov=motion_controller --cov-report=html

# View coverage report
open htmlcov/index.html
```

### Lab 2: Integration Test with launch_testing (2 hours)

<!-- class="optional-exercise" -->
**Objective**: Write integration test verifying two nodes communicate correctly.

**System**: Publisher node and subscriber node with message transformation

**Tasks**:

1. **Create Test Launch Description** (30 min):
   - Launch both nodes in test
   - Configure QoS for reliable communication

2. **Write Communication Test** (45 min):
   - Subscribe to output topic
   - Verify messages received within timeout
   - Check message transformation correctness
   - Measure publication rate

3. **Test Error Conditions** (30 min):
   - Test with incompatible QoS (should fail)
   - Test with incorrect topic name (should timeout)
   - Verify error handling

4. **Performance Testing** (15 min):
   - Measure end-to-end latency
   - Verify meets real-time requirements

<!-- class="exercise-advanced" -->
**Advanced Extensions**:
- Test with multiple publishers (race conditions)
- Test message synchronization from different rates
- Benchmark throughput with large messages
- Test recovery from node restart

### Lab 3: Simulation Test Suite (2.5 hours)

<!-- class="optional-exercise" -->
**Objective**: Create automated test suite for robot navigation in Gazebo.

**Scenario**: Mobile robot must navigate obstacle course

**Tasks**:

1. **Setup Test Environment** (30 min):
   - Create Gazebo world with obstacles
   - Launch robot and navigation stack
   - Script waypoint goals

2. **Success Path Tests** (45 min):
   - Test 1: Navigate to goal in open space
   - Test 2: Navigate around single obstacle
   - Test 3: Navigate through narrow corridor
   - Verify: Goal reached, no collisions, reasonable time

3. **Failure Mode Tests** (60 min):
   - Test sensor failure (LiDAR stops publishing)
   - Test blocked path (moving obstacle)
   - Test impossible goal (inside obstacle)
   - Verify: System detects failure, stops safely

4. **Performance Tests** (15 min):
   - Measure path efficiency (actual vs. optimal distance)
   - Record computation time
   - Monitor CPU and memory usage

<!-- class="exercise-tip" -->
**Helper Functions**:

```python
def wait_for_goal(goal_pose, timeout=60.0):
    """Wait until robot reaches goal or timeout"""
    start = time.time()
    while time.time() - start < timeout:
        current_pose = get_robot_pose()
        distance = compute_distance(current_pose, goal_pose)
        if distance < 0.1:  # Within 10cm
            return True
        time.sleep(0.1)
    return False

def check_collision():
    """Check if robot collided with obstacle"""
    contacts = get_contacts()  # Gazebo contact sensor
    return len(contacts) > 0
```

### Lab 4: CI/CD Pipeline Setup (1.5 hours)

<!-- class="optional-exercise" -->
**Objective**: Configure GitHub Actions for automated testing of ROS2 package.

**Tasks**:

1. **Basic Workflow** (30 min):
   - Create `.github/workflows/ci.yml`
   - Setup ROS2 environment
   - Build workspace
   - Run unit tests

2. **Advanced Configuration** (30 min):
   - Add code coverage reporting
   - Upload test results as artifacts
   - Add status badge to README
   - Configure matrix builds (multiple ROS distributions)

3. **Integration Testing** (20 min):
   - Add integration test job
   - Launch nodes in CI environment
   - Run integration test suite

4. **Performance Monitoring** (10 min):
   - Measure build time
   - Track test execution time
   - Set performance budgets (fail if tests too slow)

**Example Matrix Build**:
```yaml
strategy:
  matrix:
    ros_distribution: [humble, iron]

steps:
  - uses: ros-tooling/setup-ros@v0.6
    with:
      required-ros-distributions: ${{ matrix.ros_distribution }}
```

---

*Version 2.0.0 - Academic curriculum emphasizing theoretical foundations with optional practical exercises*
