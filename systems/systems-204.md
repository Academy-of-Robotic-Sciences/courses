<!--
author:   Dr. Michael Chen
email:    michael.chen@robotcampus.dev
version:  2.0.0

language: en
narrator: US English Male

comment:  Production Deployment: Comprehensive treatment of configuration management, observability, logging architectures, health monitoring, documentation standards, and system lifecycle management for production robotic systems.

icon:     https://robotcampus.dev/logos/systems-204.png

mode:     Textbook

link:     https://robotcampus.dev/styles/course-styles.css

import:   https://raw.githubusercontent.com/LiaTemplates/Pyodide/master/README.md
-->

# SYSTEMS-204: Production Deployment

> **Production-ready systems require rigorous configuration management, comprehensive observability, and professional documentation.**

## Course Overview

| | |
|---|---|
| **Course Code** | SYSTEMS-204 |
| **Duration** | 4 hours |
| **Level** | Advanced |
| **Prerequisites** | SYSTEMS-203, DevOps fundamentals |
| **Theory-Practice** | 60% theory, 40% optional labs |

---

## Learning Objectives

Upon completion of this course, students will be able to:

### Theoretical Understanding

- Explain configuration management theory and externalization principles
- Understand observability triad: logging, metrics, and tracing
- Describe health monitoring architectures and heartbeat protocols
- Apply documentation standards for technical communication
- Understand system lifecycle management and maintenance strategies
- Explain deployment patterns and rollback procedures

### Practical Skills

- Design parameterized systems using YAML configuration
- Implement structured logging with appropriate severity levels
- Create health monitoring systems with diagnostics
- Write professional technical documentation (user, maintenance, developer)
- Configure deployment pipelines for robotic systems
- Implement version control and configuration baselines

---

## Module 1: Configuration Management

### 1.1 Configuration Theory

<!-- class="theory-concept" -->
**Externalization Principles**

**Hard-Coded Values** (anti-pattern):
```python
# BAD: Magic numbers embedded in code
MAX_VELOCITY = 0.5  # m/s
MIN_BATTERY = 11.1  # V
CAMERA_TOPIC = '/camera/image_raw'
```

**Externalized Configuration**:
```python
# GOOD: Load from configuration file
config = load_config('robot_params.yaml')
MAX_VELOCITY = config['motion']['max_velocity']
MIN_BATTERY = config['power']['min_battery_voltage']
CAMERA_TOPIC = config['topics']['camera']
```

**Benefits of Externalization**:
- **Flexibility**: Change behavior without recompiling
- **Environment-Specific**: Different configs for dev/test/production
- **Maintainability**: Configuration centralized and version-controlled
- **Auditability**: Track configuration changes over time

<!-- class="theory-concept" -->
**Configuration Hierarchy**

Configurations should be layered:

1. **Defaults**: Shipped with code, sensible for most use cases
2. **Environment**: Dev, staging, production overrides
3. **Instance**: Specific robot or deployment overrides
4. **Runtime**: Dynamic reconfiguration during operation

**Example**: ROS2 Parameter Override Chain
```
Default (in code) → YAML file → Launch file → Command line → Runtime reconfigure
```

Later values override earlier ones.

### 1.2 YAML Configuration Design

<!-- class="theory-concept" -->
**Structured Configuration Files**

**Example: robot_config.yaml**

```yaml
# Robot identity
robot:
  id: "robot_001"
  model: "turtlebot3_burger"
  location: "warehouse_a"

# Motion parameters
motion:
  max_linear_velocity: 0.22  # m/s
  max_angular_velocity: 2.84  # rad/s
  acceleration_limit: 1.0     # m/s^2
  control_frequency: 10       # Hz

# Sensor configuration
sensors:
  lidar:
    topic: "/scan"
    frame_id: "laser_link"
    qos: "RELIABLE"
  camera:
    topic: "/camera/image_raw"
    frame_id: "camera_link"
    fps: 30
    resolution: [640, 480]
  imu:
    topic: "/imu"
    frequency: 100

# Power management
power:
  battery_capacity: 2200    # mAh
  min_voltage: 11.1         # V
  critical_voltage: 10.8    # V
  shutdown_voltage: 10.5    # V

# Safety limits
safety:
  emergency_stop_distance: 0.3  # m
  watchdog_timeout: 1.0         # s
  max_tilt_angle: 30            # degrees
```

**Design Principles**:
- **Hierarchical**: Group related parameters
- **Units**: Always document units in comments
- **Validation**: Define acceptable ranges
- **Documentation**: Comment purpose of each parameter

### 1.3 Parameter Validation

<!-- class="theory-concept" -->
**Configuration Validation**

Always validate configuration at startup:

```python
class RobotConfig:
    def __init__(self, config_file):
        self.config = self.load_config(config_file)
        self.validate()

    def validate(self):
        """Validate all configuration parameters"""
        # Check required fields exist
        required = ['robot.id', 'motion.max_linear_velocity']
        for field in required:
            if not self.get_nested(field):
                raise ValueError(f"Required config field missing: {field}")

        # Check value ranges
        if self.config['motion']['max_linear_velocity'] <= 0:
            raise ValueError("max_linear_velocity must be positive")

        if not (10.0 <= self.config['power']['min_voltage'] <= 14.0):
            raise ValueError("min_voltage out of range [10.0, 14.0]")

        # Check QoS values
        valid_qos = ['RELIABLE', 'BEST_EFFORT']
        lidar_qos = self.config['sensors']['lidar']['qos']
        if lidar_qos not in valid_qos:
            raise ValueError(f"Invalid QoS: {lidar_qos}")
```

**Benefits**:
- Fail fast at startup, not during operation
- Clear error messages for configuration problems
- Prevents invalid states

**Quiz: Configuration Management**

What is the primary benefit of externalizing configuration?

[( )] Faster execution
[(X)] Change behavior without recompiling
[( )] Smaller file size
[( )] Better security

---

## Module 2: Observability and Logging

### 2.1 The Observability Triad

<!-- class="theory-concept" -->
**Three Pillars of Observability**

**Logs**: Discrete events with timestamps
- "Robot started navigation at 10:15:32"
- "Obstacle detected at distance 0.5m"
- Searchable, filterable records

**Metrics**: Time-series numerical data
- CPU usage over time
- Battery voltage trend
- Message publication rate
- Aggregatable, visualizable

**Traces**: Request flow through distributed system
- Track message from sensor through processing to actuator
- Identify bottlenecks
- Understand system-wide behavior

**Combined Use**: Logs answer "what happened", metrics answer "how much", traces answer "where is the problem".

<!-- class="historical-note" -->
**Evolution of Observability**

**1960s-1970s**: **Print Debugging**. Output to console, manual inspection.

**1980s**: **Log Files**. Persistent storage of events, basic log rotation.

**1990s**: **Centralized Logging**. Syslog protocol, aggregation of distributed logs.

**2000s**: **Monitoring Systems**. Nagios (1999), Ganglia (2000) introduced metrics collection and alerting.

**2010s**: **Observability Movement**. Coined by Twitter engineers, ELK stack (Elasticsearch, Logstash, Kibana) for log analysis, Prometheus (2012) for metrics.

**2020s**: **OpenTelemetry**. Unified standard for logs, metrics, and traces across distributed systems.

### 2.2 Structured Logging

<!-- class="theory-concept" -->
**Log Levels and Severity**

ROS2 logging severity levels (increasing):

**DEBUG**: Detailed diagnostic information
- Use for: Variable values, function entry/exit
- Example: "PID controller input: setpoint=1.5, measured=1.2"

**INFO**: Informational messages about normal operation
- Use for: State changes, milestones
- Example: "Navigation started to goal (5.0, 3.0)"

**WARN**: Potentially harmful situations that don't prevent operation
- Use for: Recoverable errors, degraded performance
- Example: "Battery voltage low (11.2V), returning to dock recommended"

**ERROR**: Error events that might still allow continued operation
- Use for: Failed operations, missing data
- Example: "Failed to read sensor data, using previous value"

**FATAL**: Severe errors causing shutdown
- Use for: Unrecoverable errors
- Example: "Emergency stop triggered, shutting down motors"

**Best Practices**:
- Production: INFO and above (suppress DEBUG)
- Development: DEBUG enabled for detailed diagnostics
- Never use logging in tight loops (performance impact)

<!-- class="theory-concept" -->
**Structured Logging Format**

**Unstructured** (hard to parse):
```
Robot moving to waypoint 5.0, 3.0 at speed 0.3
```

**Structured** (machine-readable):
```json
{
  "timestamp": "2025-01-15T10:15:32.123Z",
  "level": "INFO",
  "node": "navigation",
  "event": "waypoint_navigation",
  "waypoint": {"x": 5.0, "y": 3.0},
  "speed": 0.3,
  "units": "m/s"
}
```

**Benefits**:
- Searchable by field
- Aggregatable (e.g., count all "waypoint_navigation" events)
- Filterable (e.g., show only events from "navigation" node)

### 2.3 ROS2 Logging Best Practices

<!-- class="theory-concept" -->
**Effective Logging in ROS2**

```python
class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation')

        # Log startup with configuration
        self.get_logger().info(
            f"Navigation node initialized with "
            f"max_velocity={self.max_vel}, "
            f"goal_tolerance={self.goal_tol}"
        )

    def navigate_to_goal(self, x, y):
        # INFO: Normal operation milestone
        self.get_logger().info(
            f"Starting navigation to goal ({x:.2f}, {y:.2f})"
        )

        try:
            path = self.plan_path(x, y)

            # DEBUG: Detailed diagnostics
            self.get_logger().debug(
                f"Planned path with {len(path)} waypoints, "
                f"total distance {self.path_length(path):.2f}m"
            )

            self.execute_path(path)

        except PathPlanningError as e:
            # ERROR: Recoverable failure
            self.get_logger().error(
                f"Path planning failed: {str(e)}, retrying..."
            )
            return False

        except Exception as e:
            # FATAL: Unexpected error
            self.get_logger().fatal(
                f"Unexpected error in navigation: {str(e)}"
            )
            raise

        # INFO: Success
        self.get_logger().info(
            f"Reached goal ({x:.2f}, {y:.2f})"
        )
        return True

    def check_obstacle(self, distance):
        if distance < self.emergency_distance:
            # FATAL: Safety-critical
            self.get_logger().fatal(
                f"Obstacle too close ({distance:.2f}m < "
                f"{self.emergency_distance:.2f}m), emergency stop!"
            )
        elif distance < self.warning_distance:
            # WARN: Concerning but not critical
            self.get_logger().warn(
                f"Obstacle detected at {distance:.2f}m"
            )
```

**Guidelines**:
- Include context (node name, relevant parameters)
- Use appropriate severity level
- Format numeric values consistently (e.g., 2 decimal places)
- Include units where applicable
- Log state transitions and important events
- Avoid logging in high-frequency loops

---

## Module 3: Health Monitoring

### 3.1 Heartbeat Architecture

<!-- class="theory-concept" -->
**Watchdog and Heartbeat Patterns**

**Heartbeat**: Periodic "I'm alive" signals from components

**Watchdog**: Monitor that expects heartbeats, alerts if missing

**Architecture**:
```
┌─────────┐  heartbeat  ┌──────────┐
│Component├────────────>│ Watchdog │
└─────────┘    (1 Hz)   └────┬─────┘
                             │ timeout
                             ↓
                        ┌─────────┐
                        │  Alert  │
                        └─────────┘
```

**Implementation**:

```python
class HealthMonitor(Node):
    def __init__(self):
        super().__init__('health_monitor')

        # Track last heartbeat time for each component
        self.last_heartbeat = {}

        # Subscribe to heartbeat topics
        for component in ['navigation', 'perception', 'control']:
            self.create_subscription(
                Bool,
                f'/{component}/heartbeat',
                lambda msg, c=component: self.heartbeat_callback(c, msg),
                10
            )

        # Periodic check for timeouts
        self.create_timer(1.0, self.check_health)

    def heartbeat_callback(self, component, msg):
        """Record heartbeat timestamp"""
        self.last_heartbeat[component] = self.get_clock().now()

    def check_health(self):
        """Check for missing heartbeats"""
        now = self.get_clock().now()
        timeout = Duration(seconds=2.0)

        for component, last_time in self.last_heartbeat.items():
            if (now - last_time) > timeout:
                self.get_logger().error(
                    f"{component} heartbeat timeout! "
                    f"Last seen {(now - last_time).nanoseconds / 1e9:.1f}s ago"
                )
                self.trigger_safe_mode()
```

### 3.2 Diagnostic Framework

<!-- class="theory-concept" -->
**ROS2 Diagnostics**

ROS2 provides standardized diagnostic messages:

**Diagnostic Levels**:
- **OK**: Normal operation
- **WARN**: Minor issues, degraded performance
- **ERROR**: Serious problems, reduced functionality
- **STALE**: No recent updates (component not responding)

**Example: Battery Monitor**

```python
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )

        self.create_timer(1.0, self.publish_diagnostics)

    def publish_diagnostics(self):
        """Publish battery diagnostic status"""
        voltage = self.read_battery_voltage()

        # Determine status level
        if voltage > 12.0:
            level = DiagnosticStatus.OK
            message = "Battery healthy"
        elif voltage > 11.5:
            level = DiagnosticStatus.WARN
            message = "Battery low, consider recharging"
        elif voltage > 11.0:
            level = DiagnosticStatus.ERROR
            message = "Battery critical, return to dock"
        else:
            level = DiagnosticStatus.ERROR
            message = "Battery depleted, emergency shutdown imminent"

        # Create diagnostic message
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "battery"
        status.level = level
        status.message = message
        status.hardware_id = "battery_001"

        # Add key-value pairs
        status.values = [
            KeyValue(key="voltage", value=f"{voltage:.2f}V"),
            KeyValue(key="percentage", value=f"{self.voltage_to_percent(voltage):.0f}%"),
            KeyValue(key="time_remaining", value=f"{self.estimate_runtime(voltage):.0f} min")
        ]

        diag_msg.status.append(status)
        self.diagnostics_pub.publish(diag_msg)
```

**Benefits**:
- Standardized format across all components
- Aggregation by diagnostic aggregator node
- Visualization in rqt_robot_monitor
- Historical tracking and trending

### 3.3 Self-Test Procedures

<!-- class="theory-concept" -->
**Automated System Checks**

**Power-On Self-Test (POST)**:
```python
class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Run self-test before accepting commands
        if not self.run_self_test():
            self.get_logger().fatal("Self-test failed, not starting")
            raise RuntimeError("Self-test failed")

        self.get_logger().info("Self-test passed, ready for operation")

    def run_self_test(self):
        """Comprehensive system check"""
        tests = [
            ("Sensors", self.test_sensors),
            ("Motors", self.test_motors),
            ("Communication", self.test_communication),
            ("Transforms", self.test_transforms),
            ("Emergency Stop", self.test_estop),
        ]

        for name, test_fn in tests:
            self.get_logger().info(f"Testing {name}...")
            try:
                if not test_fn():
                    self.get_logger().error(f"{name} test FAILED")
                    return False
                self.get_logger().info(f"{name} test PASSED")
            except Exception as e:
                self.get_logger().error(f"{name} test EXCEPTION: {e}")
                return False

        return True

    def test_sensors(self):
        """Verify all sensors publishing"""
        required_topics = ['/scan', '/camera/image', '/imu']
        timeout = 5.0

        for topic in required_topics:
            if not self.wait_for_message(topic, timeout):
                self.get_logger().error(f"Sensor topic {topic} not publishing")
                return False
        return True

    def test_motors(self):
        """Verify motor controllers respond"""
        # Send small command, verify response
        self.send_velocity_command(0.01, 0.0)
        time.sleep(0.1)
        velocity = self.get_current_velocity()

        if abs(velocity.linear) < 0.005:
            self.get_logger().error("Motors not responding to commands")
            return False
        return True
```

**Quiz: Observability**

Which observability pillar answers "what happened at this timestamp"?

[(X)] Logs
[( )] Metrics
[( )] Traces
[( )] None

---

## Module 4: Documentation Standards

### 4.1 Documentation Hierarchy

<!-- class="theory-concept" -->
**Audience-Specific Documentation**

Different audiences require different documentation:

**User Documentation**:
- **Audience**: Operators, end-users
- **Goal**: Operate the system successfully
- **Content**: Setup instructions, operating procedures, troubleshooting
- **Format**: Step-by-step guides, screenshots, videos
- **Example**: "How to start autonomous navigation"

**Maintenance Documentation**:
- **Audience**: Technicians, field service
- **Goal**: Diagnose and repair problems
- **Content**: Diagnostic procedures, component replacement, calibration
- **Format**: Flowcharts, diagnostic trees, parts lists
- **Example**: "Replacing a faulty LiDAR sensor"

**Developer Documentation**:
- **Audience**: Software engineers, system integrators
- **Goal**: Understand, modify, and extend the system
- **Content**: Architecture diagrams, API reference, design decisions
- **Format**: Code comments, README files, architecture documents
- **Example**: "Navigation stack architecture and extension points"

### 4.2 User Manual Template

<!-- class="theory-concept" -->
**User Manual Structure**

**1. Introduction**
- System overview
- Intended use
- Safety warnings

**2. Getting Started**
- Unpacking and setup
- Initial configuration
- First run

**3. Operating Procedures**
- Normal operation
- Common tasks
- Safety procedures

**4. Troubleshooting**
- Common problems and solutions
- Error messages
- When to contact support

**5. Maintenance**
- Routine maintenance schedule
- Cleaning procedures
- Battery care

**Example: Troubleshooting Section**

```markdown
## Troubleshooting

### Robot Does Not Move

**Symptoms**: Robot is powered on but does not respond to navigation commands.

**Possible Causes**:

1. **Emergency stop engaged**
   - Check: Red button on robot is not pressed
   - Solution: Rotate red button clockwise to release

2. **Low battery**
   - Check: Battery indicator LED is not flashing red
   - Solution: Return robot to charging dock

3. **Software not started**
   - Check: Status display shows "Ready"
   - Solution: Press green "Start" button

**If problem persists**: Contact support at support@example.com
```

### 4.3 API Documentation

<!-- class="theory-concept" -->
**Code Documentation Standards**

**Docstring Format** (Python):

```python
def plan_path(self, start, goal, obstacles):
    """
    Plan collision-free path from start to goal.

    Uses A* algorithm with Euclidean heuristic. Accounts for
    robot footprint and safety margins around obstacles.

    Args:
        start (tuple): Starting position (x, y) in meters
        goal (tuple): Goal position (x, y) in meters
        obstacles (list): List of Obstacle objects

    Returns:
        list: Waypoints [(x1,y1), (x2,y2), ...] or None if no path

    Raises:
        ValueError: If start or goal outside map bounds
        PathPlanningError: If no collision-free path exists

    Example:
        >>> planner = PathPlanner()
        >>> path = planner.plan_path((0,0), (5,5), obstacles)
        >>> if path:
        >>>     print(f"Path with {len(path)} waypoints")
    """
```

**README.md Template**:

```markdown
# Robot Navigation Package

## Overview
Brief description of package purpose and functionality.

## Installation
```bash
git clone https://github.com/example/navigation.git
cd navigation
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Usage
```bash
ros2 launch navigation navigation_launch.py
```

## Configuration
See `config/params.yaml` for configurable parameters.

## Architecture
Description of major components and their interactions.

## API Reference
Link to auto-generated API docs or manual API description.

## Testing
```bash
colcon test
```

## Contributing
Guidelines for contributing code.

## License
License information.
```

---

## Module 5: Deployment and Lifecycle Management

### 5.1 Deployment Patterns

<!-- class="theory-concept" -->
**Blue-Green Deployment**

Run two identical production environments (Blue and Green):

```
┌──────┐     ┌──────────┐     ┌──────┐
│Users │────>│ Router   │────>│ Blue │ (Active)
└──────┘     └──────────┘     └──────┘
                   │
                   │          ┌──────┐
                   └─────────>│Green │ (Standby)
                              └──────┘
```

**Deployment Process**:
1. Deploy new version to Green (standby)
2. Test Green thoroughly
3. Switch router to Green
4. Blue becomes new standby

**Benefits**:
- Instant rollback (switch back to Blue)
- Zero-downtime deployment
- Full testing before cutover

<!-- class="alternative-approach" -->
**Canary Deployment**

Gradually roll out to subset of users:

1. **Deploy to 5%** of fleet
2. **Monitor** for errors or performance issues
3. **If good**: Increase to 25%, then 50%, then 100%
4. **If problems**: Rollback, fix, retry

**Benefits**:
- Early detection of issues
- Limited blast radius
- Real-world validation

### 5.2 Version Control and Baselines

<!-- class="theory-concept" -->
**Configuration Baselines**

**Baseline**: Known-good configuration snapshot

**Version Control All Configuration**:
```
robot-config/
├── v1.0.0/
│   ├── robot_params.yaml
│   ├── sensor_config.yaml
│   └── navigation_params.yaml
├── v1.1.0/
│   ├── robot_params.yaml
│   ├── sensor_config.yaml
│   └── navigation_params.yaml (modified)
└── current -> v1.1.0/
```

**Git Tags for Releases**:
```bash
git tag -a v1.0.0 -m "Initial production release"
git tag -a v1.1.0 -m "Improved path planning parameters"
```

**Benefits**:
- Reproducibility: Recreate exact configuration
- Traceability: Know what changed and when
- Rollback: Revert to known-good configuration

### 5.3 Maintenance Strategies

<!-- class="theory-concept" -->
**Preventive Maintenance**

**Time-Based**: Scheduled maintenance intervals
- Daily: Visual inspection, battery check
- Weekly: Sensor cleaning, loose fasteners
- Monthly: Full diagnostics, calibration check
- Quarterly: Deep inspection, wear parts replacement

**Condition-Based**: Maintenance triggered by metrics
- Battery cycle count reaches threshold
- Encoder drift exceeds limit
- Communication error rate increases
- Cumulative operating hours

**Predictive**: Use trends to predict failures
- Battery capacity degradation trend
- Motor current increase (bearing wear)
- Sensor noise level increase
- Log pattern analysis (anomaly detection)

**Maintenance Log**:
```yaml
- date: 2025-01-15
  robot_id: robot_001
  type: preventive
  procedure: quarterly_inspection
  findings:
    - Left motor bearing noise
    - Battery capacity 85% of original
  actions:
    - Scheduled motor replacement
    - Battery replacement recommended within 3 months
  next_service: 2025-04-15
```

---

## Summary

This course established production deployment practices for robotic systems:

1. **Configuration Management**: Externalized parameters with validation and version control
2. **Observability**: Implemented structured logging, metrics, and diagnostics
3. **Health Monitoring**: Created watchdog systems and self-test procedures
4. **Documentation**: Developed user, maintenance, and developer documentation
5. **Deployment**: Applied blue-green and canary deployment patterns
6. **Lifecycle Management**: Established maintenance strategies and baselines

**Key Takeaways**:
- Configuration externalization enables flexible deployment
- Observability is essential for diagnosing production issues
- Health monitoring enables proactive failure detection
- Documentation quality determines user and maintainer success
- Deployment strategies minimize risk and enable rollback
- Systematic maintenance extends system lifetime

**Graduation**: Students completing this course have mastered the complete systems engineering lifecycle from design through production deployment.

---

## Optional Laboratory Exercises

### Lab 1: Configuration System Design (1.5 hours)

<!-- class="optional-exercise" -->
**Objective**: Design and implement comprehensive configuration system with validation.

**Tasks**:

1. **Configuration Schema Design** (30 min):
   - Define YAML structure for robot configuration
   - Include motion, sensors, power, safety parameters
   - Document units and valid ranges

2. **Validation Implementation** (30 min):
   - Write Python class to load and validate configuration
   - Check required fields exist
   - Verify parameter ranges
   - Test with valid and invalid configurations

3. **Multi-Environment Support** (20 min):
   - Create dev, staging, production configurations
   - Implement override hierarchy
   - Test environment-specific behavior

4. **Runtime Reconfiguration** (10 min):
   - Implement ROS2 parameter callback
   - Validate new values before applying
   - Log configuration changes

<!-- class="exercise-tip" -->
**Validation Example**:

```python
class ConfigValidator:
    SCHEMA = {
        'motion.max_linear_velocity': (0.0, 2.0),
        'motion.max_angular_velocity': (0.0, 5.0),
        'power.min_voltage': (10.0, 14.0),
        'safety.emergency_stop_distance': (0.1, 1.0),
    }

    def validate(self, config):
        for path, (min_val, max_val) in self.SCHEMA.items():
            value = self.get_nested(config, path)
            if not (min_val <= value <= max_val):
                raise ValueError(
                    f"{path}={value} outside range [{min_val}, {max_val}]"
                )
```

### Lab 2: Health Monitoring System (2 hours)

<!-- class="optional-exercise" -->
**Objective**: Build comprehensive health monitoring system with diagnostics.

**Tasks**:

1. **Heartbeat Publishers** (30 min):
   - Modify 3 nodes to publish heartbeat messages
   - Include timestamp and status information

2. **Watchdog Monitor** (45 min):
   - Create health monitor node
   - Track heartbeats from all components
   - Detect timeouts and publish alerts
   - Trigger safe mode on critical failures

3. **Diagnostic Integration** (30 min):
   - Implement diagnostic publishers for each component
   - Monitor battery, sensors, communication
   - Publish to `/diagnostics` topic
   - Visualize in rqt_robot_monitor

4. **Self-Test Implementation** (15 min):
   - Create power-on self-test procedure
   - Verify all sensors publishing
   - Check transform tree complete
   - Test emergency stop functionality

<!-- class="exercise-advanced" -->
**Advanced Extensions**:
- Implement graceful degradation (reduced functionality)
- Add predictive failure detection using trends
- Create automated recovery procedures
- Log health metrics to time-series database

### Lab 3: Documentation Project (2.5 hours)

<!-- class="optional-exercise" -->
**Objective**: Create professional documentation suite for robotic system.

**Deliverables**:

1. **User Manual** (60 min):
   - Introduction and safety warnings
   - Quick start guide with screenshots
   - Operating procedures for common tasks
   - Troubleshooting guide (5+ scenarios)
   - Maintenance schedule

2. **Maintenance Guide** (45 min):
   - Component location diagram
   - Diagnostic flowcharts
   - Calibration procedures
   - Parts list with suppliers
   - Service intervals

3. **Developer Documentation** (45 min):
   - System architecture diagram
   - ROS2 node graph
   - API documentation (docstrings)
   - README with setup instructions
   - Configuration reference

<!-- class="exercise-tip" -->
**Documentation Tools**:
- **Diagrams**: draw.io, PlantUML, Mermaid
- **API Docs**: Sphinx (Python), Doxygen (C++)
- **User Guides**: Markdown, MkDocs, GitBook
- **Screenshots**: Use annotation tools (arrows, callouts)
- **Video**: Record common procedures (< 5 min each)

**Evaluation Criteria**:
- Clarity: Can target audience understand?
- Completeness: All necessary information included?
- Accuracy: Technical content correct?
- Professionalism: Formatting, grammar, organization

### Lab 4: Deployment Pipeline (2 hours)

<!-- class="optional-exercise" -->
**Objective**: Implement deployment pipeline with versioning and rollback capability.

**Tasks**:

1. **Configuration Versioning** (30 min):
   - Create Git repository for configurations
   - Tag versions (v1.0.0, v1.1.0, etc.)
   - Document changes in CHANGELOG.md
   - Test checkout of specific version

2. **Deployment Script** (45 min):
   - Write script to deploy configuration to robot
   - Validate configuration before deployment
   - Backup current configuration
   - Apply new configuration
   - Restart services

3. **Rollback Procedure** (30 min):
   - Implement automatic rollback on failure
   - Test rollback to previous version
   - Verify system returns to functional state

4. **Deployment Verification** (15 min):
   - Run automated tests after deployment
   - Check all services started
   - Verify configuration applied correctly
   - Monitor for errors in first 5 minutes

**Example Deploy Script**:
```bash
#!/bin/bash
# deploy_config.sh

VERSION=$1
ROBOT_ID=$2

echo "Deploying configuration $VERSION to robot $ROBOT_ID"

# Backup current config
ssh robot@$ROBOT_ID "tar -czf /backup/config_$(date +%Y%m%d_%H%M%S).tar.gz /opt/robot/config"

# Copy new configuration
scp -r config/$VERSION/* robot@$ROBOT_ID:/opt/robot/config/

# Validate configuration
ssh robot@$ROBOT_ID "/opt/robot/bin/validate_config.py /opt/robot/config/robot_params.yaml"

if [ $? -ne 0 ]; then
    echo "Configuration validation failed, rolling back"
    # Restore backup...
    exit 1
fi

# Restart services
ssh robot@$ROBOT_ID "systemctl restart robot.service"

echo "Deployment complete"
```

---

*Version 2.0.0 - Academic curriculum emphasizing theoretical foundations with optional practical exercises*
