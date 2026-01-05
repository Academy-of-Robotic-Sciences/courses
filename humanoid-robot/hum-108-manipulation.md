---
id: hum-108-manipulation
title: "HUM-108: Object Manipulation & Coordination"
sidebar_position: 9
version: 2.0.0
---
<link rel="stylesheet" href="/styles/textbook.css" />

# Chapter 8: Object Manipulation & Whole-Body Coordination

<div class="chapter-header">
<div class="course-info">

| **Course Code** | HUM-108 |
|---|---|
| **Duration** | 8 hours (full day) |
| **Level** | Advanced Integration |
| **Prerequisites** | HUM-107 (Dynamic Walking) |
| **Primary Role** | Software Engineer, AI Engineer |
| **Secondary Role** | Systems Engineer |

</div>

<div class="abstract">
**Abstract**: This chapter addresses the fundamental challenge of coordinated whole-body control in humanoid robotics: enabling simultaneous locomotion and manipulation while maintaining dynamic balance. Through task-space optimization, hierarchical control architectures, and multi-objective constraint satisfaction, we develop methods for integrated mobile manipulation. Students implement coordinated reaching behaviors, vision-guided grasping, and human-robot object transfer protocols, culminating in autonomous bi-manual manipulation during dynamic walking.
</div>
</div>

## 8.1 Introduction to Whole-Body Coordination

The transition from isolated subsystem control to integrated whole-body coordination represents one of the most significant challenges in humanoid robotics. While previous chapters addressed balance, locomotion, and manipulation independently, real-world tasks demand simultaneous coordination of all degrees of freedom while satisfying multiple competing objectives.

### 8.1.1 Historical Development

The concept of whole-body control emerged from early work in redundant manipulator control in the 1980s. Khatib's operational space formulation (1987) provided the mathematical foundation for task-space control, enabling specification of desired end-effector behavior while resolving internal redundancy. The extension to humanoid robots, pioneered by researchers at the University of Tokyo and Stanford in the early 2000s, introduced balance constraints as the highest-priority task in hierarchical control frameworks.

Sentis and Khatib (2005) formalized whole-body operational space control for humanoids, demonstrating manipulation tasks while maintaining balance. Subsequent work by Mansard and Chaumette (2007) on task function approach and by Escande et al. (2014) on hierarchical quadratic programming advanced the mathematical rigor and computational efficiency of these methods.

### 8.1.2 The Multi-Objective Control Problem

Whole-body coordination involves solving a constrained optimization problem with competing objectives:

**Problem Statement**: Given a humanoid robot with state **q** ∈ ℝⁿ and desired task specifications **x**ᵈ, find joint velocities **q̇** that satisfy:

<div class="equation-block">

$$
\begin{aligned}
\text{minimize} \quad & \sum_{i=1}^{m} w_i \|\mathbf{e}_i(\mathbf{q})\|^2 \\
\text{subject to} \quad & \mathbf{J}_{\text{balance}}(\mathbf{q})\dot{\mathbf{q}} = \dot{\mathbf{x}}_{\text{balance}} & \text{(priority 1)} \\
& \mathbf{J}_{\text{collision}}(\mathbf{q})\dot{\mathbf{q}} \geq \dot{\mathbf{x}}_{\text{safe}} & \text{(priority 2)} \\
& \mathbf{J}_{\text{task}}(\mathbf{q})\dot{\mathbf{q}} = \dot{\mathbf{x}}_{\text{task}} & \text{(priority 3)} \\
& \dot{\mathbf{q}}_{\min} \leq \dot{\mathbf{q}} \leq \dot{\mathbf{q}}_{\max} & \text{(bounds)}
\end{aligned}
$$

</div>

where **J**ᵢ represents the Jacobian mapping joint velocities to task-space velocities for objective i, **e**ᵢ are task errors, and wᵢ are weighting factors.

### 8.1.3 The Center of Mass Displacement Problem

One of the primary challenges in manipulation during standing is the coupling between arm motion and balance. When the robot reaches forward, the center of mass shifts, requiring compensatory motion in the lower body.

**Theoretical Framework**: Consider a humanoid robot modeled as a system of rigid bodies. The total center of mass is:

<div class="equation-block">

$$
\mathbf{p}_{\text{CoM}} = \frac{1}{M} \sum_{i=1}^{N} m_i \mathbf{p}_i
$$

</div>

where M is total mass, mᵢ is the mass of link i, and **p**ᵢ is the position of link i's center of mass.

When the arm moves from configuration **q**₁ to **q**₂, the CoM displacement is:

<div class="equation-block">

$$
\Delta \mathbf{p}_{\text{CoM}} = \frac{1}{M} \sum_{i \in \text{arm}} m_i (\mathbf{p}_i(\mathbf{q}_2) - \mathbf{p}_i(\mathbf{q}_1))
$$

</div>

For static stability, this displacement must be compensated such that the vertical projection of the CoM remains within the support polygon:

<div class="equation-block">

$$
\mathbf{p}_{\text{CoM}}^{\text{xy}} + \Delta \mathbf{p}_{\text{CoM}}^{\text{xy}} + \mathbf{p}_{\text{comp}}^{\text{xy}} \in \mathcal{S}
$$

</div>

where 𝓢 is the support polygon and **p**ᶜᵒᵐᵖ is the compensatory motion.

<div class="quiz">

**Quiz 8.1: Whole-Body Control Fundamentals**

1. In hierarchical task control, why must balance constraints have higher priority than manipulation objectives?
   - a) Balance requires faster computation
   - b) Manipulation tasks are less important
   - c) Loss of balance causes system failure; manipulation can be temporarily suspended
   - d) Balance uses fewer degrees of freedom

2. A humanoid's arm (mass 0.6 kg) extends 20 cm forward from its initial position. If the total robot mass is 4 kg, what is the approximate forward CoM shift?
   - a) 3 mm
   - b) 30 mm
   - c) 300 mm
   - d) 50 mm

3. Which mathematical tool maps joint velocities to task-space velocities in operational space control?
   - a) Hessian matrix
   - b) Jacobian matrix
   - c) Inertia matrix
   - d) Coriolis matrix

**Answers**: 1-c, 2-b (Δp = 0.6 kg × 20 cm / 4 kg = 3 cm = 30 mm), 3-b

</div>

---

## 8.2 Task-Space Control Theory

Task-space control provides a framework for specifying robot behavior in operational coordinates (end-effector position, orientation) rather than joint coordinates. This formulation is particularly powerful for manipulation tasks where the desired outcome is naturally expressed in Cartesian space.

### 8.2.1 Operational Space Formulation

The relationship between joint velocities **q̇** and task-space velocities **ẋ** is given by the Jacobian:

<div class="equation-block">

$$
\dot{\mathbf{x}} = \mathbf{J}(\mathbf{q})\dot{\mathbf{q}}
$$

</div>

For a redundant manipulator (n > m, where n is the number of joints and m is the task dimension), infinitely many joint configurations can achieve the same task-space position. The general solution is:

<div class="equation-block">

$$
\dot{\mathbf{q}} = \mathbf{J}^{\dagger}\dot{\mathbf{x}} + (\mathbf{I} - \mathbf{J}^{\dagger}\mathbf{J})\dot{\mathbf{q}}_0
$$

</div>

where **J**† is the Moore-Penrose pseudoinverse, and **q̇**₀ represents motion in the null space that does not affect the primary task.

### 8.2.2 Weighted Pseudoinverse for Multi-Objective Control

When multiple tasks must be satisfied simultaneously, we use a weighted pseudoinverse approach. Define a positive-definite weighting matrix **W**, typically chosen as the joint inertia matrix **M**(**q**):

<div class="equation-block">

$$
\mathbf{J}^{\dagger}_W = \mathbf{W}^{-1}\mathbf{J}^T(\mathbf{J}\mathbf{W}^{-1}\mathbf{J}^T)^{-1}
$$

</div>

This weighted pseudoinverse minimizes the kinetic energy of motion while achieving the task objective.

### 8.2.3 Hierarchical Task Prioritization

For strict task prioritization, we use recursive null-space projection. Given two tasks with Jacobians **J**₁ (high priority) and **J**₂ (low priority):

<div class="equation-block">

$$
\begin{aligned}
\dot{\mathbf{q}}_1 &= \mathbf{J}_1^{\dagger}\dot{\mathbf{x}}_1 \\
\mathbf{N}_1 &= \mathbf{I} - \mathbf{J}_1^{\dagger}\mathbf{J}_1 \\
\dot{\mathbf{q}}_2 &= \mathbf{J}_1^{\dagger}\dot{\mathbf{x}}_1 + (\mathbf{J}_2\mathbf{N}_1)^{\dagger}(\dot{\mathbf{x}}_2 - \mathbf{J}_2\dot{\mathbf{q}}_1)
\end{aligned}
$$

</div>

where **N**₁ is the null-space projector that ensures secondary task motion does not interfere with the primary task.

### 8.2.4 Task Error Dynamics

To achieve smooth convergence, we typically specify task-space velocities using proportional-derivative control:

<div class="equation-block">

$$
\dot{\mathbf{x}}_{\text{des}} = \dot{\mathbf{x}}_{\text{ref}} + K_p(\mathbf{x}_{\text{ref}} - \mathbf{x}) + K_d(\dot{\mathbf{x}}_{\text{ref}} - \dot{\mathbf{x}})
$$

</div>

where **x**ᵣₑ𝒻 is the reference trajectory, and K_p, K_d are gain matrices chosen to achieve desired convergence characteristics.

---

## 8.3 Guided Project: Coordinated Reaching and Balance

### Objective
Implement a whole-body controller that enables the humanoid to reach for objects while automatically compensating for balance disturbances. This project integrates task-space control, CoM regulation, and real-time optimization.

### 8.3.1 Implementation: Whole-Body Controller

**Step 1: Task Space Controller Class**

```python
import numpy as np
from scipy.linalg import pinv

class WholeBodyController:
    """
    Hierarchical whole-body controller for humanoid manipulation.

    Implements three-level task hierarchy:
    1. Balance maintenance (CoM within support polygon)
    2. Self-collision avoidance
    3. End-effector task achievement
    """

    def __init__(self, robot_model):
        self.robot = robot_model
        self.n_joints = robot_model.n_joints

        # Task definitions
        self.tasks = {
            'balance': {'priority': 1, 'weight': 10.0},
            'collision': {'priority': 2, 'weight': 5.0},
            'manipulation': {'priority': 3, 'weight': 1.0},
            'posture': {'priority': 4, 'weight': 0.1}
        }

        # Control gains
        self.K_p = np.diag([50.0, 50.0, 50.0])  # Position gains
        self.K_d = np.diag([10.0, 10.0, 10.0])  # Velocity gains

    def compute_jacobian(self, task_name, q):
        """
        Compute task Jacobian for given configuration.

        Args:
            task_name: Identifier for task type
            q: Joint configuration [n x 1]

        Returns:
            J: Task Jacobian [m x n]
        """
        if task_name == 'balance':
            # Jacobian mapping joint velocities to CoM velocity
            return self.robot.compute_com_jacobian(q)
        elif task_name == 'manipulation':
            # Jacobian for end-effector
            return self.robot.compute_end_effector_jacobian(q)
        elif task_name == 'posture':
            # Identity for joint-space posture task
            return np.eye(self.n_joints)
        else:
            raise ValueError(f"Unknown task: {task_name}")

    def compute_task_error(self, task_name, x_desired, x_current,
                          v_desired, v_current):
        """
        Compute task-space error with PD control law.

        Returns:
            e: Desired task-space velocity [m x 1]
        """
        position_error = x_desired - x_current
        velocity_error = v_desired - v_current

        # PD control law
        e = v_desired + self.K_p @ position_error + self.K_d @ velocity_error

        return e

    def solve_hierarchical(self, q, q_dot, tasks_desired):
        """
        Solve hierarchical task control using null-space projection.

        Args:
            q: Current joint positions [n x 1]
            q_dot: Current joint velocities [n x 1]
            tasks_desired: Dict of desired task states

        Returns:
            q_dot_cmd: Commanded joint velocities [n x 1]
        """
        # Sort tasks by priority
        sorted_tasks = sorted(self.tasks.items(),
                            key=lambda x: x[1]['priority'])

        q_dot_cmd = np.zeros(self.n_joints)
        N = np.eye(self.n_joints)  # Null-space projector

        for task_name, task_props in sorted_tasks:
            if task_name not in tasks_desired:
                continue

            # Compute task Jacobian
            J = self.compute_jacobian(task_name, q)

            # Get desired task state
            task_des = tasks_desired[task_name]
            x_des = task_des['position']
            v_des = task_des.get('velocity', np.zeros_like(x_des))

            # Current task state
            x_curr = self.robot.forward_kinematics(task_name, q)
            v_curr = J @ q_dot

            # Compute task error
            e = self.compute_task_error(task_name, x_des, x_curr,
                                       v_des, v_curr)

            # Project Jacobian into current null space
            J_proj = J @ N

            # Weighted pseudoinverse
            W = self.robot.get_inertia_matrix(q)
            J_proj_pinv = self.weighted_pseudoinverse(J_proj, W)

            # Add task contribution
            q_dot_task = J_proj_pinv @ (e - J @ q_dot_cmd)
            q_dot_cmd += q_dot_task

            # Update null-space projector
            N = N @ (np.eye(self.n_joints) - J_proj_pinv @ J_proj)

        return q_dot_cmd

    def weighted_pseudoinverse(self, J, W):
        """
        Compute weighted pseudoinverse: W^-1 J^T (J W^-1 J^T)^-1
        """
        W_inv = np.linalg.inv(W)
        J_W_inv = J @ W_inv
        return W_inv @ J.T @ pinv(J_W_inv @ J.T)
```

**Mathematical Analysis**: This implementation uses recursive null-space projection to ensure strict task hierarchy. The weighted pseudoinverse with inertia matrix **W** = **M**(**q**) minimizes instantaneous kinetic energy:

<div class="equation-block">

$$
\frac{1}{2}\dot{\mathbf{q}}^T \mathbf{M}(\mathbf{q}) \dot{\mathbf{q}}
$$

</div>

This choice is optimal for dynamic systems as it accounts for the configuration-dependent inertial properties.

### 8.3.2 Implementation: Balance-Aware Reaching

**Step 2: Coordinated Reaching with CoM Compensation**

```python
class BalanceAwareReaching:
    """
    Implements reaching behaviors with automatic balance compensation.
    """

    def __init__(self, controller, robot):
        self.controller = controller
        self.robot = robot
        self.support_polygon = None

    def update_support_polygon(self):
        """
        Compute current support polygon from foot contact points.
        """
        # Get foot contact positions
        left_foot_contacts = self.robot.get_foot_contacts('left')
        right_foot_contacts = self.robot.get_foot_contacts('right')

        # Convex hull of all contact points
        all_contacts = np.vstack([left_foot_contacts, right_foot_contacts])
        self.support_polygon = self.compute_convex_hull(all_contacts)

    def predict_com_shift(self, q_current, q_target):
        """
        Predict CoM displacement from arm motion.

        Args:
            q_current: Current joint configuration
            q_target: Target joint configuration

        Returns:
            delta_com: Predicted CoM displacement [3 x 1]
        """
        # Current CoM
        com_current = self.robot.compute_com(q_current)

        # Predicted CoM at target
        com_target = self.robot.compute_com(q_target)

        return com_target - com_current

    def compute_compensation_motion(self, delta_com):
        """
        Compute hip and ankle adjustments to compensate CoM shift.

        Uses simplified model: CoM compensation through lower-body joints.

        Args:
            delta_com: Predicted CoM shift [3 x 1]

        Returns:
            q_comp: Compensatory joint angles dict
        """
        q_comp = {}

        # Horizontal compensation
        if abs(delta_com[0]) > 0.02:  # 2 cm threshold
            # Forward/backward shift - use hip and ankle pitch
            q_comp['hip_pitch'] = -delta_com[0] * 2.0  # deg per cm
            q_comp['ankle_pitch'] = -delta_com[0] * 1.0
        else:
            # Within support polygon - minimal ankle adjustment
            q_comp['hip_pitch'] = 0.0
            q_comp['ankle_pitch'] = -delta_com[0] * 0.5

        # Lateral compensation
        if abs(delta_com[1]) > 0.02:
            q_comp['hip_roll'] = -delta_com[1] * 2.0
            q_comp['ankle_roll'] = -delta_com[1] * 1.0
        else:
            q_comp['hip_roll'] = 0.0
            q_comp['ankle_roll'] = 0.0

        return q_comp

    def reach_with_balance(self, target_position, duration=2.0):
        """
        Execute reaching motion with balance compensation.

        Args:
            target_position: Desired end-effector position [x, y, z]
            duration: Motion duration in seconds
        """
        # Compute target arm configuration
        q_arm_target = self.robot.inverse_kinematics_arm(target_position)

        # Get current full-body configuration
        q_current = self.robot.get_joint_positions()

        # Predict CoM shift
        delta_com = self.predict_com_shift(q_current, q_arm_target)

        # Compute compensation
        q_compensation = self.compute_compensation_motion(delta_com)

        # Generate smooth trajectory
        trajectory = self.generate_coordinated_trajectory(
            q_current, q_arm_target, q_compensation, duration
        )

        # Execute coordinated motion
        for q_des, q_dot_des, t in trajectory:
            # Set up task hierarchy
            tasks = {
                'balance': {
                    'position': self.compute_desired_com(q_current,
                                                        q_compensation),
                    'velocity': np.zeros(3)
                },
                'manipulation': {
                    'position': self.robot.forward_kinematics('hand', q_des),
                    'velocity': q_dot_des
                },
                'posture': {
                    'position': self.robot.get_nominal_posture(),
                    'velocity': np.zeros(self.robot.n_joints)
                }
            }

            # Solve whole-body control
            q_dot_cmd = self.controller.solve_hierarchical(
                q_des, q_dot_des, tasks
            )

            # Send commands to robot
            self.robot.set_joint_velocities(q_dot_cmd)

            # Wait for next control cycle
            time.sleep(0.01)  # 100 Hz control

    def generate_coordinated_trajectory(self, q_start, q_arm_target,
                                       q_comp, duration):
        """
        Generate smooth trajectory using minimum-jerk interpolation.
        """
        n_steps = int(duration * 100)  # 100 Hz
        trajectory = []

        for i in range(n_steps):
            # Normalized time [0, 1]
            s = i / n_steps

            # Minimum-jerk trajectory scaling
            tau = 10*s**3 - 15*s**4 + 6*s**5
            tau_dot = (30*s**2 - 60*s**3 + 30*s**4) / duration

            # Interpolate configuration
            q = q_start + tau * (q_arm_target - q_start)

            # Add compensation
            for joint, angle in q_comp.items():
                q[joint] += tau * angle

            # Velocity
            q_dot = tau_dot * (q_arm_target - q_start)
            for joint, angle in q_comp.items():
                q_dot[joint] += tau_dot * angle

            trajectory.append((q, q_dot, i * 0.01))

        return trajectory
```

**Theoretical Foundation**: The minimum-jerk trajectory τ(s) = 10s³ - 15s⁴ + 6s⁵ minimizes the time integral of jerk (rate of change of acceleration):

<div class="equation-block">

$$
\min \int_0^T \|\dddot{\mathbf{x}}(t)\|^2 dt
$$

</div>

This produces smooth, natural-looking motions that are optimal for minimizing actuator wear and energy consumption.

### 8.3.3 Testing Protocol

**Experimental Validation**:

1. **Static Reaching Test**:
   - Place target at (30, 0, 20) cm relative to robot base
   - Execute reaching motion
   - Measure: (a) CoM displacement, (b) stability margin, (c) task error

2. **Multi-Direction Reaching**:
   - Targets: forward, lateral, diagonal, high, low
   - Record success rate and stability metrics

3. **Dynamic Reaching**:
   - Reach while transitioning from single to double support
   - Measure balance recovery time

<div class="quiz">

**Quiz 8.2: Task-Space Control**

1. Why is the weighted pseudoinverse with inertia matrix preferred over the standard pseudoinverse?
   - a) It's computationally faster
   - b) It minimizes kinetic energy, accounting for mass distribution
   - c) It always produces a unique solution
   - d) It works for non-redundant systems

2. In null-space projection, what does the projector **N** = **I** - **J**†**J** ensure?
   - a) Minimum energy motion
   - b) Secondary task motion doesn't affect primary task
   - c) Faster computation
   - d) Singularity avoidance

3. The minimum-jerk trajectory is optimal for:
   - a) Minimizing execution time
   - b) Minimizing energy consumption
   - c) Producing smooth, natural motion
   - d) Avoiding obstacles

**Answers**: 1-b, 2-b, 3-c

</div>

---

## 8.4 Vision-Guided Grasping

Vision-guided manipulation integrates computer vision with motion control to achieve autonomous object interaction. This section develops the theoretical framework and practical implementation for closed-loop visual servoing.

### 8.4.1 Visual Servoing Theory

Visual servoing uses visual feedback to control robot motion. Two primary approaches exist:

**Position-Based Visual Servoing (PBVS)**: Reconstruct 3D object pose, then control in Cartesian space.

**Image-Based Visual Servoing (IBVS)**: Control directly in image space without explicit 3D reconstruction.

For IBVS, the relationship between image feature velocities **ṡ** and camera velocity **v**_c is:

<div class="equation-block">

$$
\dot{\mathbf{s}} = \mathbf{L}_s \mathbf{v}_c
$$

</div>

where **L**_s is the interaction matrix (image Jacobian). For a point feature with image coordinates (x, y) and depth Z:

<div class="equation-block">

$$
\mathbf{L}_s = \begin{bmatrix}
-\frac{1}{Z} & 0 & \frac{x}{Z} & xy & -(1+x^2) & y \\
0 & -\frac{1}{Z} & \frac{y}{Z} & 1+y^2 & -xy & -x
\end{bmatrix}
$$

</div>

### 8.4.2 3D Position Estimation from Vision

Using a calibrated pinhole camera model, we can estimate object position from image observations.

**Camera Model**: The relationship between 3D point **P** = (X, Y, Z) and image point **p** = (u, v) is:

<div class="equation-block">

$$
\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \frac{1}{Z} \mathbf{K} \begin{bmatrix} X \\ Y \\ Z \end{bmatrix}
$$

where $\mathbf{K} = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}$

</div>

**Depth from Known Object Size**: If we know the real-world height h of an object, and observe its image height h_img:

<div class="equation-block">

$$
Z = \frac{f_y \cdot h}{h_{\text{img}}}
$$

</div>

---

## 8.5 Guided Project: Visual Grasping Pipeline

### Objective
Implement a complete vision-guided grasping system that detects objects, estimates their 3D position, and executes coordinated whole-body motion to grasp them.

### 8.5.1 Implementation: Vision System

```python
import cv2
import numpy as np
from ultralytics import YOLO

class VisionGraspingSystem:
    """
    Computer vision system for object detection and 3D localization.
    """

    def __init__(self, camera_id=0, camera_matrix=None):
        self.camera = cv2.VideoCapture(camera_id)
        self.detector = YOLO('yolov8n.pt')  # Lightweight YOLO

        # Camera calibration parameters
        if camera_matrix is None:
            # Default calibration for typical webcam
            self.K = np.array([
                [600.0, 0, 320.0],
                [0, 600.0, 240.0],
                [0, 0, 1.0]
            ])
        else:
            self.K = camera_matrix

        # Object database (real-world dimensions in cm)
        self.object_database = {
            'cup': {'height': 10.0, 'width': 8.0},
            'bottle': {'height': 20.0, 'width': 6.0},
            'book': {'height': 20.0, 'width': 15.0},
            'cell phone': {'height': 14.0, 'width': 7.0}
        }

    def detect_objects(self, confidence_threshold=0.5):
        """
        Detect objects in current camera frame.

        Returns:
            objects: List of detected objects with bounding boxes
        """
        ret, frame = self.camera.read()
        if not ret:
            return []

        # Run YOLO detection
        results = self.detector(frame, verbose=False)

        objects = []
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Extract detection info
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                confidence = float(box.conf[0])
                class_id = int(box.cls[0])
                class_name = self.detector.names[class_id]

                if confidence > confidence_threshold:
                    objects.append({
                        'class': class_name,
                        'bbox': np.array([x1, y1, x2, y2]),
                        'confidence': confidence,
                        'center': np.array([(x1+x2)/2, (y1+y2)/2])
                    })

        return objects

    def estimate_3d_position(self, bbox, object_class):
        """
        Estimate 3D position from bounding box and known object size.

        Args:
            bbox: [x1, y1, x2, y2] bounding box
            object_class: Object type from database

        Returns:
            position_3d: [X, Y, Z] in camera frame (cm)
        """
        if object_class not in self.object_database:
            # Unknown object - use default size
            real_height = 10.0  # cm
        else:
            real_height = self.object_database[object_class]['height']

        # Image measurements
        x1, y1, x2, y2 = bbox
        h_img = y2 - y1  # Height in pixels

        # Depth estimation using pinhole model
        f_y = self.K[1, 1]
        Z = (real_height * f_y) / h_img

        # Convert pixel coordinates to camera coordinates
        u_center = (x1 + x2) / 2
        v_center = (y1 + y2) / 2

        c_x = self.K[0, 2]
        c_y = self.K[1, 2]
        f_x = self.K[0, 0]

        X = (u_center - c_x) * Z / f_x
        Y = (v_center - c_y) * Z / f_y

        return np.array([X, Y, Z])

    def transform_to_robot_frame(self, position_camera):
        """
        Transform position from camera frame to robot base frame.

        Assumes camera is mounted on robot head with known transform.
        """
        # Camera-to-robot transformation (example values)
        # In practice, obtain from robot kinematics
        R_cam_to_robot = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0]
        ])
        t_cam_to_robot = np.array([0, 0, 50])  # Camera 50cm above base

        position_robot = R_cam_to_robot @ position_camera + t_cam_to_robot

        return position_robot
```

### 8.5.2 Implementation: Grasp Execution

```python
class GraspController:
    """
    Coordinates vision, motion planning, and gripper control for grasping.
    """

    def __init__(self, vision_system, arm_controller, gripper):
        self.vision = vision_system
        self.arm = arm_controller
        self.gripper = gripper

    def grasp_object(self, target_class='cup', max_attempts=3):
        """
        Complete grasp pipeline: detect, approach, grasp, lift.

        Args:
            target_class: Type of object to grasp
            max_attempts: Number of detection attempts

        Returns:
            success: Boolean indicating grasp success
        """
        # Step 1: Object detection
        object_detected = False
        for attempt in range(max_attempts):
            objects = self.vision.detect_objects()

            # Find target object
            targets = [obj for obj in objects
                      if obj['class'] == target_class]

            if targets:
                # Select closest/most confident
                target = max(targets, key=lambda x: x['confidence'])
                object_detected = True
                break

            time.sleep(0.5)

        if not object_detected:
            print(f"Could not detect {target_class}")
            return False

        # Step 2: 3D position estimation
        pos_camera = self.vision.estimate_3d_position(
            target['bbox'], target['class']
        )
        pos_robot = self.vision.transform_to_robot_frame(pos_camera)

        print(f"Object located at: {pos_robot} cm")

        # Step 3: Plan approach trajectory
        # Pre-grasp position (5 cm above object)
        pos_pregrasp = pos_robot.copy()
        pos_pregrasp[2] += 5.0

        # Grasp position
        pos_grasp = pos_robot.copy()

        # Post-grasp position (lift 10 cm)
        pos_postgrasp = pos_robot.copy()
        pos_postgrasp[2] += 10.0

        # Step 4: Execute grasp sequence
        try:
            # Move to pre-grasp
            self.arm.reach_with_balance(pos_pregrasp, duration=2.0)
            time.sleep(0.5)

            # Open gripper
            self.gripper.open()
            time.sleep(0.5)

            # Move to grasp position
            self.arm.reach_with_balance(pos_grasp, duration=1.0)
            time.sleep(0.5)

            # Close gripper
            grasp_force = self.gripper.close()
            time.sleep(0.5)

            # Check grasp success
            if grasp_force < self.gripper.min_grasp_force:
                print("Grasp failed - insufficient force")
                return False

            # Lift object
            self.arm.reach_with_balance(pos_postgrasp, duration=1.5)
            time.sleep(0.5)

            # Verify object is held
            if not self.gripper.has_object():
                print("Object dropped during lift")
                return False

            print(f"Successfully grasped {target_class}")
            return True

        except Exception as e:
            print(f"Grasp execution error: {e}")
            self.arm.return_to_neutral()
            return False
```

**Performance Analysis**: The grasp success rate depends on several factors:

1. **Detection accuracy**: YOLO achieves ~90% accuracy on common objects
2. **Depth estimation error**: Typically ±2-3 cm for objects 30-50 cm away
3. **Execution precision**: Joint control accuracy ±1-2 degrees

Expected overall grasp success rate: 70-85% for known objects under good lighting conditions.

---

## 8.6 Mobile Manipulation

Mobile manipulation combines locomotion and manipulation, enabling the robot to transport objects while walking. This requires careful coordination to maintain stability under varying loads.

### 8.6.1 Load-Adapted Gait

When carrying an object, the robot's mass distribution changes, affecting stability. The adapted gait must account for:

1. **Shifted center of mass**: Additional mass in hands moves CoM forward/upward
2. **Reduced stability margin**: Less tolerance for balance errors
3. **Modified dynamics**: Changed inertia affects required control torques

**Mathematical Model**: Consider a robot with mass M carrying an object with mass m_obj at position **p**_obj relative to the base. The new CoM is:

<div class="equation-block">

$$
\mathbf{p}_{\text{CoM}}^{\text{new}} = \frac{M \mathbf{p}_{\text{CoM}}^{\text{robot}} + m_{\text{obj}} \mathbf{p}_{\text{obj}}}{M + m_{\text{obj}}}
$$

</div>

The gait parameters must be adjusted to maintain the ZMP within the support polygon despite this shift.

### 8.6.2 Implementation: Walking with Object

```python
class MobileManipulator:
    """
    Integrated locomotion and manipulation for object transport.
    """

    def __init__(self, walker, arm_controller, gripper):
        self.walker = walker
        self.arm = arm_controller
        self.gripper = gripper

        self.carrying_object = False
        self.object_mass = 0.0

    def walk_with_object(self, distance, object_mass=0.2):
        """
        Walk while carrying an object.

        Args:
            distance: Distance to walk (cm)
            object_mass: Mass of carried object (kg)
        """
        self.object_mass = object_mass
        self.carrying_object = True

        # Adapt gait parameters for carrying
        original_params = self.walker.get_gait_params()
        adapted_params = self.adapt_gait_for_load(original_params,
                                                   object_mass)
        self.walker.set_gait_params(adapted_params)

        # Lock arms in carrying position
        carrying_pose = {
            'shoulder_pitch': 45,  # Arms forward
            'shoulder_roll': 0,
            'elbow': 90,  # Bent at 90 degrees
            'wrist_pitch': 0,
            'wrist_roll': 0
        }
        self.arm.hold_position(carrying_pose)

        # Execute walking
        steps_needed = int(distance / adapted_params['step_length'])

        for step in range(steps_needed):
            # Compensate for object weight
            self.compensate_for_load(object_mass)

            # Take step with stability check
            success = self.walker.step()

            if not success:
                print(f"Stability lost at step {step}")
                self.emergency_stop()
                break

            # Monitor gripper force
            if self.gripper.get_force() < self.gripper.min_grasp_force:
                print("Object slipping - stopping")
                self.walker.stop()
                break

        # Restore normal gait
        self.walker.set_gait_params(original_params)
        self.carrying_object = False

    def adapt_gait_for_load(self, params, object_mass):
        """
        Modify gait parameters based on carried load.

        Adaptations:
        - Reduce step length for stability
        - Increase step time for smoother motion
        - Reduce lateral sway
        - Eliminate arm swing
        """
        adapted = params.copy()

        # Load factor (0 to 1 representing fraction of robot mass)
        load_factor = object_mass / 4.0  # Assuming 4kg robot

        # Reduce step length: 20% reduction per 1kg load
        adapted['step_length'] *= (1.0 - 0.2 * load_factor)

        # Increase step time: 30% increase per 1kg load
        adapted['step_time'] *= (1.0 + 0.3 * load_factor)

        # Reduce lateral sway
        adapted['lateral_sway'] *= (1.0 - 0.5 * load_factor)

        # Disable arm swing
        adapted['arm_swing_amplitude'] = 0.0

        return adapted

    def compensate_for_load(self, object_mass):
        """
        Real-time compensation for object weight during walking.

        Adjusts hip and ankle torques to maintain balance under load.
        """
        # Estimate load-induced CoM shift
        # Object held at ~30cm forward, 40cm up from base
        object_position = np.array([30, 0, 40])

        # CoM displacement
        robot_mass = 4.0  # kg
        delta_com = (object_mass * object_position) / robot_mass

        # Compensate with hip pitch
        hip_compensation = -delta_com[0] * 0.5  # degrees per cm

        # Apply compensation
        self.walker.add_hip_pitch_offset(hip_compensation)
```

---

## 8.7 Human-Robot Object Handoff

Safe and natural object transfer between robot and human requires compliance, force sensing, and appropriate timing.

### 8.7.1 Handoff Protocol

**Safety Requirements**:
1. Approach speed < 10 cm/s when human is near
2. Grasp force sufficient to hold but not damage object
3. Release only when human has secure grasp
4. Retract slowly to avoid collisions

### 8.7.2 Implementation: Safe Handoff

```python
class HumanHandoffController:
    """
    Manages safe object transfer to humans.
    """

    def __init__(self, vision, arm, gripper):
        self.vision = vision
        self.arm = arm
        self.gripper = gripper

    def handoff_to_human(self, speak_func=None):
        """
        Execute safe handoff sequence.

        Returns:
            success: Boolean indicating successful handoff
        """
        # Step 1: Detect human hand
        hand_detected = False
        for _ in range(10):  # 5 seconds at 2 Hz
            hand_pos = self.detect_human_hand()
            if hand_pos is not None:
                hand_detected = True
                break
            time.sleep(0.5)

        if not hand_detected:
            if speak_func:
                speak_func("Please extend your hand")
            time.sleep(2)
            hand_pos = self.detect_human_hand()
            if hand_pos is None:
                return False

        # Step 2: Move object to handoff position
        # Position slightly below detected hand
        handoff_pos = hand_pos.copy()
        handoff_pos[2] -= 3  # 3 cm below hand

        # Slow approach for safety
        self.arm.reach_with_balance(handoff_pos, duration=3.0)
        time.sleep(0.5)

        # Step 3: Signal readiness
        if speak_func:
            speak_func("Please take the object")

        # Step 4: Wait for human to grasp
        handoff_complete = False
        timeout = 10.0  # seconds
        start_time = time.time()

        while time.time() - start_time < timeout:
            # Monitor gripper force
            current_force = self.gripper.get_force()

            # Detect pull force (human grasping)
            if self.detect_pull_force(threshold=0.5):  # 0.5 N threshold
                # Human has grasped - release gripper
                self.gripper.open(speed=0.3)  # Slow opening
                time.sleep(0.5)

                if speak_func:
                    speak_func("Here you go")

                handoff_complete = True
                break

            time.sleep(0.1)  # 10 Hz monitoring

        if not handoff_complete:
            if speak_func:
                speak_func("Handoff timeout")
            return False

        # Step 5: Retract arm safely
        neutral_pos = self.arm.get_neutral_position()
        self.arm.reach_with_balance(neutral_pos, duration=2.0)

        return True

    def detect_human_hand(self):
        """
        Detect human hand position using computer vision.

        Returns:
            hand_pos: 3D position or None if not detected
        """
        # Use MediaPipe or similar for hand detection
        # Simplified implementation
        objects = self.vision.detect_objects()

        hands = [obj for obj in objects if obj['class'] == 'hand']
        if not hands:
            return None

        # Return closest hand
        closest_hand = min(hands, key=lambda x: x['depth'])
        return self.vision.estimate_3d_position(
            closest_hand['bbox'], 'hand'
        )

    def detect_pull_force(self, threshold):
        """
        Detect if external force is applied to gripper.

        Uses force sensor or motor current sensing.
        """
        force = self.gripper.get_external_force()
        return force > threshold
```

---

## 8.8 Bi-Manual Manipulation

Coordinating both arms enables manipulation of larger objects and more complex tasks.

### 8.8.1 Theoretical Framework

For bi-manual manipulation, we must coordinate two kinematic chains subject to a mutual constraint (holding opposite ends of an object).

**Constraint Equation**: If both hands grasp a rigid object, the relative pose between end-effectors is constrained:

<div class="equation-block">

$$
\mathbf{T}_{\text{left}}^{-1} \mathbf{T}_{\text{right}} = \mathbf{T}_{\text{grasp}}
$$

</div>

where **T**_grasp is the constant transformation between grasp points on the object.

This constraint can be enforced using closed-loop inverse kinematics or through coordinated task-space control.

### 8.8.2 Implementation: Bi-Manual Coordination

```python
class BimanualController:
    """
    Coordinates two arms for bi-manual manipulation tasks.
    """

    def __init__(self, left_arm, right_arm):
        self.left_arm = left_arm
        self.right_arm = right_arm

    def hold_box(self, box_width=20, box_center=[30, 0, 20]):
        """
        Grasp a box with both hands.

        Args:
            box_width: Width of box in cm
            box_center: Center position [x, y, z] in cm
        """
        # Compute target positions for each hand
        left_target = np.array(box_center)
        left_target[1] -= box_width / 2  # Left side

        right_target = np.array(box_center)
        right_target[1] += box_width / 2  # Right side

        # Synchronize motion using threads
        import threading

        left_done = threading.Event()
        right_done = threading.Event()

        def move_left():
            self.left_arm.reach_with_balance(left_target, duration=2.0)
            left_done.set()

        def move_right():
            self.right_arm.reach_with_balance(right_target, duration=2.0)
            right_done.set()

        # Start simultaneous motion
        thread_left = threading.Thread(target=move_left)
        thread_right = threading.Thread(target=move_right)

        thread_left.start()
        thread_right.start()

        # Wait for both to complete
        left_done.wait()
        right_done.wait()

        print("Both hands positioned")

        # Close grippers simultaneously
        self.left_arm.gripper.close()
        self.right_arm.gripper.close()
        time.sleep(0.5)

        # Verify grasp
        left_grasped = self.left_arm.gripper.has_object()
        right_grasped = self.right_arm.gripper.has_object()

        return left_grasped and right_grasped

    def coordinated_lift(self, lift_height=10):
        """
        Lift held object with coordinated motion.
        """
        # Get current positions
        left_current = self.left_arm.get_end_effector_position()
        right_current = self.right_arm.get_end_effector_position()

        # Compute lift targets
        left_target = left_current.copy()
        left_target[2] += lift_height

        right_target = right_current.copy()
        right_target[2] += lift_height

        # Execute synchronized lift
        # Use position control to maintain relative position
        n_steps = 100
        for i in range(n_steps):
            alpha = (i + 1) / n_steps

            left_inter = left_current + alpha * (left_target - left_current)
            right_inter = right_current + alpha * (right_target - right_current)

            self.left_arm.move_to(left_inter)
            self.right_arm.move_to(right_inter)

            time.sleep(0.02)  # 50 Hz
```

---

## 8.9 Chapter Summary and Integration Demo

### 8.9.1 Key Concepts Mastered

1. **Whole-Body Coordination**: Hierarchical task control with null-space projection enables simultaneous achievement of multiple objectives while respecting task priorities

2. **Task-Space Control**: Operating in task coordinates with weighted pseudoinverse provides intuitive control and optimizes performance metrics (e.g., kinetic energy)

3. **Vision-Guided Manipulation**: Integration of computer vision, 3D estimation, and motion control creates autonomous grasping capabilities

4. **Mobile Manipulation**: Load-adapted gait enables stable walking while carrying objects

5. **Human-Robot Interaction**: Safe handoff protocols with force sensing enable natural object transfer

6. **Bi-Manual Coordination**: Synchronized multi-arm control allows manipulation of larger objects

### 8.9.2 Integration Demonstration Sequence

**Final Demo: Table Clearing Task** (Complete autonomous sequence)

```python
def table_clearing_demo():
    """
    Autonomous demo: Detect objects on table, pick them up,
    carry to bin, and return.
    """
    # Initialize systems
    vision = VisionGraspingSystem()
    arm = BalanceAwareReaching(WholeBodyController(robot), robot)
    gripper = GripperController()
    walker = DynamicWalker()
    mobile_manip = MobileManipulator(walker, arm, gripper)

    # Task parameters
    table_position = np.array([100, 0, 0])  # 1 meter ahead
    bin_position = np.array([100, 50, 0])    # 1 meter ahead, 50cm right

    # Detect all objects on table
    robot.walk_to(table_position)
    objects = vision.detect_objects()

    print(f"Detected {len(objects)} objects")

    for i, obj in enumerate(objects):
        print(f"Processing object {i+1}: {obj['class']}")

        # Grasp object
        success = grasp_controller.grasp_object(obj['class'])
        if not success:
            print(f"Failed to grasp {obj['class']}, skipping")
            continue

        # Walk to bin while carrying
        mobile_manip.walk_with_object(
            distance=np.linalg.norm(bin_position - table_position),
            object_mass=0.2
        )

        # Release object over bin
        arm.reach_with_balance(bin_position, duration=1.5)
        gripper.open()
        time.sleep(0.5)

        # Return to table
        mobile_manip.walk_with_object(
            distance=np.linalg.norm(table_position - bin_position),
            object_mass=0.0
        )

    print("Table clearing complete!")
```

<div class="quiz">

**Quiz 8.3: Integration Concepts**

1. When walking while carrying a 0.5 kg object held 30 cm forward, what is the primary adaptation needed in the gait?
   - a) Increase step frequency
   - b) Reduce step length and increase stability margin
   - c) Increase lateral sway
   - d) Remove all joint constraints

2. In the handoff protocol, why do we wait for pull force detection rather than timing-based release?
   - a) It's computationally simpler
   - b) It ensures human has secure grasp before robot releases
   - c) Timing is unreliable
   - d) Force sensors are more accurate than timers

3. What is the primary advantage of bi-manual manipulation for a humanoid robot?
   - a) Faster execution
   - b) Ability to manipulate larger/heavier objects
   - c) Simpler control algorithms
   - d) Lower power consumption

**Answers**: 1-b, 2-b, 3-b

</div>

---

## 8.10 Advanced Topics and Future Directions

### 8.10.1 Compliance and Impedance Control

While this chapter focused on position control, many manipulation tasks benefit from compliance - the ability to yield to external forces. Impedance control allows specifying the dynamic relationship between force and motion:

<div class="equation-block">

$$
\mathbf{F} = \mathbf{M}_d(\ddot{\mathbf{x}} - \ddot{\mathbf{x}}_d) + \mathbf{D}_d(\dot{\mathbf{x}} - \dot{\mathbf{x}}_d) + \mathbf{K}_d(\mathbf{x} - \mathbf{x}_d)
$$

</div>

where **M**_d, **D**_d, **K**_d are desired inertia, damping, and stiffness matrices.

### 8.10.2 Learning-Based Manipulation

Recent advances in imitation learning and reinforcement learning offer alternatives to analytical control design. Systems like LeRobot (covered in HUM-109) can learn manipulation behaviors from demonstration, potentially discovering strategies that exceed hand-designed controllers.

### 8.10.3 Tactile Sensing

While vision provides rich information, tactile feedback is crucial for fine manipulation. Integration of force/torque sensors, tactile arrays, or even slip detection would significantly improve grasp robustness and enable force-controlled assembly tasks.

---

## References and Further Reading

**Foundational Papers**:
1. Khatib, O. (1987). "A unified approach for motion and force control of robot manipulators: The operational space formulation." *IEEE Journal on Robotics and Automation*, 3(1), 43-53.

2. Sentis, L., & Khatib, O. (2005). "Synthesis of whole-body behaviors through hierarchical control of behavioral primitives." *International Journal of Humanoid Robotics*, 2(04), 505-518.

3. Mansard, N., & Chaumette, F. (2007). "Task sequencing for high-level sensor-based control." *IEEE Transactions on Robotics*, 23(1), 60-72.

**Visual Servoing**:
4. Chaumette, F., & Hutchinson, S. (2006). "Visual servo control, Part I: Basic approaches." *IEEE Robotics & Automation Magazine*, 13(4), 82-90.

**Mobile Manipulation**:
5. Stilman, M. (2010). "Global manipulation planning in robot joint space with task constraints." *IEEE Transactions on Robotics*, 26(3), 576-584.

**Recommended Textbooks**:
- Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer.
- Lynch, K. M., & Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press.

---

## Exercises

### Conceptual Questions

1. **Explain** why the weighted pseudoinverse using the inertia matrix **M**(**q**) is advantageous compared to the unweighted pseudoinverse for whole-body control. What physical quantity does this minimize?

2. **Derive** the image Jacobian **L**_s for a point feature at image coordinates (x, y) with depth Z. Show how small camera motions affect image feature velocities.

3. **Analyze** the stability implications of carrying a 1 kg object at different positions: (a) close to the body at 10 cm forward, (b) arms extended at 30 cm forward. Which configuration is more stable and why?

### Computational Exercises

4. **Implement** a null-space projector and verify that secondary task motion does not affect primary task achievement. Use a simple 2D example with joint limits.

5. **Simulate** the minimum-jerk trajectory τ(s) = 10s³ - 15s⁴ + 6s⁵ and compare position, velocity, and acceleration profiles to a linear interpolation. Comment on smoothness.

6. **Calculate** the depth estimation error for an object 50 cm away using the pinhole model, given: (a) 10% error in object size knowledge, (b) ±5 pixel error in bounding box measurement, (c) 10% error in focal length calibration.

### Implementation Projects

7. **Extend** the `WholeBodyController` to include joint limit avoidance as a secondary task. The joint limit task should activate only when joints approach limits (within 10% of range).

8. **Implement** visual servoing using the image-based approach (IBVS). Control the robot to center a detected object in the image using only image-space errors, without 3D reconstruction.

9. **Design** an adaptive gait controller that automatically adjusts parameters based on measured stability margins. If stability drops below threshold, reduce step length and increase step time.

10. **Create** a complete pick-and-place demo that integrates all components: vision-based object detection, coordinated reaching with balance compensation, grasping, walking while carrying, and placing at a specified location.

---

<div class="chapter-footer">

**Next Chapter**: HUM-109 - AI-Powered Behaviors
*Building on manipulation capabilities, we integrate machine learning for adaptive behavior, imitation learning, and natural human-robot interaction.*

**Previous Chapter**: HUM-107 - Dynamic Walking
*Dynamic bipedal locomotion with gait generation and environmental adaptation.*

</div>
