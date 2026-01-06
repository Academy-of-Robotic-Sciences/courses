<!--
author:   Dr. Maria Torres
email:    maria.torres@academy-of-robotic-sciences.github.io
version:  2.0.0

language: en
narrator: US English Female

comment:  Kinematics and Parametric Assemblies: Comprehensive treatment of robot kinematics, Denavit-Hartenberg notation, mechanism analysis, and advanced parametric assembly modeling.


mode:     Textbook

link:     https://raw.githubusercontent.com/Academy-of-Robotic-Sciences/courses/main/course-styles.css

-->

# DESIGN-204: Kinematics and Parametric Assemblies

> **Kinematic analysis transforms static geometry into dynamic mechanisms through mathematical representation of motion and constraints.**

## Course Overview

| | |
|---|---|
| **Course Code** | DESIGN-204 |
| **Duration** | 4 hours |
| **Level** | Advanced |
| **Prerequisites** | DESIGN-203, linear algebra, calculus |
| **Theory-Practice** | 60% theory, 40% optional labs |

---

## Learning Objectives

Upon completion of this course, students will be able to:

### Theoretical Understanding

- Explain robot kinematic fundamentals: forward and inverse kinematics
- Derive transformation matrices using Denavit-Hartenberg (DH) convention
- Analyze mechanisms using Grübler's equation for degrees of freedom
- Understand joint types and their mathematical representations
- Compare serial, parallel, and hybrid kinematic architectures
- Evaluate workspace analysis and singularity conditions

### Practical Skills

- Apply DH parameters to robot manipulators
- Create parametric assemblies with motion constraints in Fusion 360
- Simulate mechanism motion and validate kinematic performance
- Design linkages for specified motion requirements
- Implement motion studies for gripper mechanisms
- Optimize design parameters for workspace and dexterity

---

## Module 1: Foundations of Robot Kinematics

### 1.1 Coordinate Frames and Transformations

<!-- class="theory-concept" -->
**Spatial Description of Robot Configuration**

**Homogeneous Coordinates** (4×1):
Represent 3D point as:
$$\mathbf{p} = \begin{bmatrix} x \\ y \\ z \\ 1 \end{bmatrix}$$

The "1" enables translation through matrix multiplication.

**Homogeneous Transformation Matrix** (4×4):
$$T = \begin{bmatrix}
R_{3\times 3} & \mathbf{d}_{3\times 1} \\
\mathbf{0}_{1\times 3} & 1
\end{bmatrix}$$

where:
- **R**: 3×3 rotation matrix (orientation)
- **d**: 3×1 translation vector (position)

**Elementary Rotations**:

Rotation about Z-axis by θ:
$$R_z(\theta) = \begin{bmatrix}
\cos\theta & -\sin\theta & 0 \\
\sin\theta & \cos\theta & 0 \\
0 & 0 & 1
\end{bmatrix}$$

Similarly for R_x(α) and R_y(β).

**Composition**: Transform from frame {0} to {2} through {1}:
$$T_0^2 = T_0^1 \cdot T_1^2$$

Matrix multiplication order matters (non-commutative).

<!-- class="historical-note" -->
**Development of Robot Kinematics**

- **1955**: Denavit and Hartenberg published systematic method for kinematic analysis
- **1960s**: Industrial robots emerge (Unimate, 1961)
- **1969**: Victor Scheinman developed Stanford Arm (first electrically-powered robot)
- **1970s**: Mathematical formalism established (Paul, Craig, Whitney)
- **1980s**: Computational algorithms for real-time control
- **1990s**: Parallel manipulators (Stewart platform, Delta robot)
- **2000s**: Collaborative robots with advanced kinematics
- **2010s**: Soft robotics challenges classical kinematic models

### 1.2 Denavit-Hartenberg Convention

<!-- class="theory-concept" -->
**Standard Notation for Serial Kinematics**

The DH convention systematically assigns frames to each joint using four parameters.

**DH Parameters** (for joint i):

| Parameter | Symbol | Description |
|-----------|--------|-------------|
| Link length | $a_i$ or $a_{i-1}$ | Distance along $x_i$ from $z_{i-1}$ to $z_i$ |
| Link twist | $\alpha_i$ or $\alpha_{i-1}$ | Angle about $x_i$ from $z_{i-1}$ to $z_i$ |
| Link offset | $d_i$ | Distance along $z_{i-1}$ from $x_{i-1}$ to $x_i$ |
| Joint angle | $\theta_i$ | Angle about $z_{i-1}$ from $x_{i-1}$ to $x_i$ |

**Variable Convention**:
- **Revolute joint**: θ_i is variable, d_i is constant
- **Prismatic joint**: d_i is variable, θ_i is constant

**Transformation from frame i-1 to frame i**:
$$T_{i-1}^{i} = R_z(\theta_i) \cdot T_z(d_i) \cdot T_x(a_i) \cdot R_x(\alpha_i)$$

Expanded form:
$$T_i = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

**Forward Kinematics**:
$$T_0^n = T_1 \cdot T_2 \cdot \ldots \cdot T_n$$

Gives end-effector pose (position + orientation) from joint angles.

<!-- class="alternative-approach" -->
**Modified DH Convention**

Alternative formulation (used by some textbooks):
- Different frame assignment rules
- Parameters have slightly different meanings
- Results are equivalent, just different notation
- Important to be consistent within a project

### 1.3 Example: 2-DOF Planar Arm

<!-- class="theory-concept" -->
**Illustrative Case Study**

Two revolute joints in a plane (RR manipulator):

**DH Parameters**:
| Joint | θ_i | d_i | a_i | α_i |
|-------|-----|-----|-----|-----|
| 1 | θ₁ (var) | 0 | L₁ | 0 |
| 2 | θ₂ (var) | 0 | L₂ | 0 |

**Forward Kinematics**:
End-effector position:
$$x = L_1 \cos\theta_1 + L_2 \cos(\theta_1 + \theta_2)$$
$$y = L_1 \sin\theta_1 + L_2 \sin(\theta_1 + \theta_2)$$

**Inverse Kinematics** (given x, y, find θ₁, θ₂):

Using law of cosines:
$$\cos\theta_2 = \frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}$$
$$\theta_2 = \pm \arccos\left(\frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}\right)$$

Two solutions: "elbow up" and "elbow down".

**Workspace**: Annulus with inner radius |L₁ - L₂| and outer radius L₁ + L₂.

**Quiz: Kinematics Fundamentals**

In the DH convention for a revolute joint, which parameter is the variable?

[( )] a_i (link length)
[(X)] θ_i (joint angle)
[( )] α_i (link twist)
[( )] d_i (link offset)

For a 2-DOF planar arm, how many inverse kinematic solutions exist (generally)?

[( )] 1
[(X)] 2
[( )] 4
[( )] Infinite

---

## Module 2: Mechanism Analysis

### 2.1 Degrees of Freedom

<!-- class="theory-concept" -->
**Mobility of Mechanisms**

**Grübler's Equation** (planar mechanisms):
$$DOF = 3(n - 1) - 2j_1 - j_2$$

where:
- **n**: Number of links (including ground/base)
- **j₁**: Number of 1-DOF joints (revolute, prismatic)
- **j₂**: Number of 2-DOF joints (rarely used)

**Spatial Mechanisms** (3D):
$$DOF = 6(n - 1) - 5j_1 - 4j_2 - 3j_3 - 2j_4 - j_5$$

**Example**: 4-bar linkage
- n = 4 links (including ground)
- j₁ = 4 revolute joints
- DOF = 3(4-1) - 2(4) = 9 - 8 = 1

Single input drives entire mechanism (1-DOF).

### 2.2 Joint Types and Constraints

<!-- class="theory-concept" -->
**Kinematic Pairs**

**Lower Pairs** (surface contact):
1. **Revolute (R)**: 1-DOF, rotation about axis
2. **Prismatic (P)**: 1-DOF, translation along axis
3. **Helical (H)**: 1-DOF, coupled rotation and translation (screw)
4. **Cylindrical (C)**: 2-DOF, rotation + independent translation
5. **Planar**: 3-DOF, motion in plane
6. **Spherical (S)**: 3-DOF, rotation about point (ball joint)

**Constraint Count**:
- Revolute: 5 constraints (removes 5 DOF)
- Prismatic: 5 constraints
- Spherical: 3 constraints

**Robot Architectures** (by joint types):
- **RRR**: 3 revolute joints (e.g., SCARA arm)
- **RPR**: Revolute-Prismatic-Revolute
- **Spherical wrist**: 3 intersecting revolute joints (roll-pitch-yaw)

### 2.3 Serial vs. Parallel Mechanisms

<!-- class="theory-concept" -->
**Architectural Comparison**

**Serial Manipulators**:
- **Structure**: Open kinematic chain
- **Workspace**: Large, complex shape
- **Stiffness**: Low (cantilevered)
- **Accuracy**: Errors accumulate
- **Examples**: Industrial arms, SO-101

**Parallel Manipulators**:
- **Structure**: Closed kinematic loops
- **Workspace**: Smaller, simpler shape
- **Stiffness**: High (triangulated)
- **Accuracy**: High (errors average out)
- **Examples**: Stewart platform, Delta robot, surgical robots

**Delta Robot**:
- 3 parallel chains with spherical joints
- 3-DOF (XYZ translation)
- Extremely fast (food packaging)
- Compact workspace

**Hybrid**:
Combine serial and parallel (e.g., parallel base + serial wrist).

---

## Module 3: Parametric Assembly Design

### 3.1 Component-Based Modeling

<!-- class="theory-concept" -->
**Multi-Part CAD Systems**

**Hierarchy**:
- **Part**: Single continuous geometry (one STL)
- **Component**: Instance of a part (can be reused)
- **Assembly**: Collection of components with relationships

**Design Intent in Assemblies**:
- Mate/constraint relationships maintain design logic
- Parameters can drive multiple components
- Top-down vs. bottom-up design approaches

**Top-Down Design**:
1. Define assembly skeleton (layout)
2. Create component envelopes in context
3. Detail individual parts
4. Maintains inter-part relationships

**Bottom-Up Design**:
1. Design parts independently
2. Assemble afterward with mates
3. More modular, easier to reuse parts

### 3.2 Joint and Constraint Types

<!-- class="theory-concept" -->
**Fusion 360 Assembly Joints**

**As-Built Joint**: Position components without motion

**Joint Types** (for motion):

1. **Rigid**: 0-DOF, fixes components together
2. **Revolute**: 1-DOF, rotation about axis
3. **Slider**: 1-DOF, translation along axis
4. **Cylindrical**: 2-DOF, rotation + translation
5. **Pin-Slot**: 2-DOF, rotation + constrained translation
6. **Planar**: 3-DOF, motion in plane
7. **Ball**: 3-DOF, rotation about point

**Joint Limits**:
- Minimum/maximum angle for revolute
- Minimum/maximum distance for slider
- Prevents unrealistic configurations

**Motion Links**:
- Couple multiple joints (gears, rack-pinion)
- Define gear ratios, linear relationships
- Example: Symmetric gripper (two fingers mirror each other)

### 3.3 Motion Studies

<!-- class="theory-concept" -->
**Kinematic Simulation**

**Animation Types**:
1. **Manual**: Drag components by joint
2. **Prescribed Motion**: Define joint trajectory over time
   - Linear: constant velocity
   - Harmonic: sinusoidal motion
   - Spline: arbitrary path
3. **Contact-Based**: Simulate collisions (requires physics engine)

**Analysis Outputs**:
- Joint angles vs. time
- Component trajectories
- Velocity and acceleration
- Interference detection (collisions)

**Validation**:
- Verify full range of motion
- Check for part interference
- Measure workspace envelope
- Optimize linkage proportions

---

## Module 4: Gripper Mechanism Design

### 4.1 Gripper Kinematics

<!-- class="theory-concept" -->
**Parallel Jaw Grippers**

**Two-Finger Gripper**:
- 2 DOF (2 joint angles)
- 1 actuator with linkage coupling
- Effective 1-DOF (synchronized motion)

**Grasp Force Analysis**:
For applied actuator force F_a, gripper force F_g:
$$F_g = \frac{F_a}{MA}$$

where **MA** is mechanical advantage from linkage geometry.

**Four-Bar Linkage Gripper**:
- Input link (actuator)
- Coupler link
- Output link (finger)
- Ground link (frame)

Provides:
- Parallel jaw motion
- Force multiplication
- Adjustable grasp range

### 4.2 Linkage Synthesis

<!-- class="theory-concept" -->
**Designing for Specified Motion**

**Position Synthesis**: Design linkage to pass through specified positions.

**Example: Two-Position Synthesis**
Given two desired finger positions, solve for link lengths.

**Graphical Method**:
1. Plot desired positions
2. Construct geometric constraints
3. Determine link lengths from geometry

**Analytical Method**:
Set up equations:
$$\mathbf{p}_2 = T(\theta_1, \theta_2, L_1, L_2)$$

Solve for link lengths and joint angles.

**Motion Generation**:
More complex: specify entire trajectory
- Requires optimization
- Multiple solutions exist
- Trade-offs between metrics (workspace, force, compactness)

### 4.3 Workspace Analysis

<!-- class="theory-concept" -->
**Reachable Space**

**Workspace** = Set of all end-effector positions robot can reach.

**Types**:
- **Reachable workspace**: All positions reachable in some orientation
- **Dexterous workspace**: Positions reachable in all orientations

**Analysis Methods**:
1. **Analytical**: Solve kinematic equations for boundary
2. **Numerical**: Sample joint space, map to Cartesian space
3. **Monte Carlo**: Random joint configurations, plot results

**Gripper Workspace**:
- Grasp range: minimum to maximum opening
- Approach angles: accessible orientations
- Clearance: avoid collisions with object/environment

---

## Module 5: Advanced Topics

### 5.1 Singularities

<!-- class="theory-concept" -->
**Kinematic Degeneracies**

**Singularity**: Configuration where DOF is lost.

**Mathematical Definition**:
Jacobian matrix J maps joint velocities to end-effector velocities:
$$\dot{\mathbf{x}} = J \dot{\mathbf{q}}$$

**Singular** when det(J) = 0.

**Consequences**:
- Cannot move end-effector in certain directions
- Infinite joint velocities required for finite end-effector velocity
- Loss of control

**Example**: Arm fully extended (elbow singularity)
- Cannot move end-effector radially outward
- Must "fold" arm to escape singularity

**Design Implication**: Avoid working near singularities.

### 5.2 Redundancy

<!-- class="alternative-approach" -->
**Extra Degrees of Freedom**

**Kinematically Redundant**: More DOF than required for task.

**Example**: 7-DOF arm for 6-DOF end-effector pose
- Extra DOF allows: obstacle avoidance, singularity avoidance, optimized posture
- Inverse kinematics has infinite solutions

**Optimization**:
Minimize:
$$\|\dot{\mathbf{q}}\|^2 \quad \text{(joint velocity)}$$
$$\sum w_i |q_i - q_{i,mid}|^2 \quad \text{(stay near mid-range)}$$

Subject to: achieving desired end-effector velocity.

**Applications**:
- Collaborative robots (UR5, Sawyer)
- Humanoid arms
- Surgical robots

---

## Summary

This course established comprehensive understanding of kinematics and assembly modeling:

1. **Kinematic Foundations**: Homogeneous transformations and DH parameters
2. **Mechanism Analysis**: DOF calculation, joint types, serial vs. parallel
3. **Parametric Assemblies**: Component-based design with motion constraints
4. **Gripper Design**: Linkage synthesis and workspace analysis
5. **Advanced Topics**: Singularities and redundancy

**Key Takeaways**:
- DH convention provides systematic method for robot kinematic analysis
- Forward kinematics determines end-effector pose from joint configuration
- Mechanism DOF follows from topology (Grübler's equation)
- Parametric assemblies enable design validation through motion simulation

**Next Steps**: DESIGN-205 integrates all skills in a comprehensive capstone design project.

---

## Optional Laboratory Exercises

### Lab 1: DH Parameter Derivation (90 minutes)

<!-- class="optional-exercise" -->
**Objective**: Apply Denavit-Hartenberg convention to analyze robot kinematics.

**Robot**: 3-DOF planar arm (RRR configuration)
- Link lengths: L₁ = 100mm, L₂ = 80mm, L₃ = 60mm

**Tasks**:
1. **Frame Assignment**:
   - Sketch robot in zero configuration
   - Assign coordinate frames following DH convention
   - Identify joint axes

2. **DH Table**:
   - Fill in DH parameters (θ, d, a, α) for each joint
   - Identify which parameters are variable

3. **Forward Kinematics**:
   - Write transformation matrices T₁, T₂, T₃
   - Compute T₀³ = T₁·T₂·T₃
   - Extract end-effector position (x, y) equations

4. **Numerical Verification**:
   - Choose joint angles: θ₁=30°, θ₂=45°, θ₃=-20°
   - Calculate end-effector position
   - Verify using geometric construction

<!-- class="exercise-tip" -->
**Tip**: Draw diagrams for each step. Visual representation helps avoid sign errors.

### Lab 2: Parametric Gripper Assembly (120 minutes)

<!-- class="optional-exercise" -->
**Objective**: Create fully-parametric two-finger gripper with motion simulation.

**Components**:
- Base plate
- Two gripper fingers (mirror symmetric)
- Mounting hardware

**Requirements**:
- Grasp range: 0-50mm adjustable
- Synchronized finger motion
- Parametric: finger_length, finger_width, jaw_opening

**Tasks**:
1. **Component Design**:
   - Model base plate with mounting features
   - Design finger with grip surface
   - Use parameters for all key dimensions

2. **Assembly**:
   - Create new assembly
   - Insert base (fixed)
   - Insert two fingers
   - Define revolute joints for each finger

3. **Motion Coupling**:
   - Add motion link between finger joints
   - Set symmetric relationship (θ₂ = -θ₁)
   - Define joint limits (0° to 45°)

4. **Animation**:
   - Create motion study showing gripper opening/closing
   - Verify no interference
   - Measure maximum grasp width

<!-- class="exercise-advanced" -->
**Advanced Extension**:
- Add four-bar linkage actuator
- Analyze mechanical advantage
- Design for specific grasp force
- Create configuration table for different object sizes

### Lab 3: Workspace Visualization (90 minutes)

<!-- class="optional-exercise" -->
**Objective**: Numerically compute and visualize manipulator workspace.

**Method**: Monte Carlo sampling

**Tasks**:
1. **Python Script**:
```python
import numpy as np
import matplotlib.pyplot as plt

# Robot parameters
L1 = 100  # mm
L2 = 80   # mm

# Sample joint space
n_samples = 10000
theta1 = np.random.uniform(0, 2*np.pi, n_samples)
theta2 = np.random.uniform(-np.pi, np.pi, n_samples)

# Forward kinematics
x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)

# Plot workspace
plt.figure(figsize=(8, 8))
plt.scatter(x, y, s=1, alpha=0.1)
plt.xlabel('X (mm)')
plt.ylabel('Y (mm)')
plt.title('2-DOF Arm Workspace')
plt.axis('equal')
plt.grid(True)
plt.show()
```

2. **Analysis**:
   - Identify workspace boundaries
   - Measure workspace area
   - Add joint limits: θ₁ ∈ [0°, 180°], θ₂ ∈ [-90°, 90°]
   - Compare restricted vs. full workspace

3. **3D Extension**:
   - Add third joint (out of plane)
   - Visualize 3D workspace
   - Use voxel grid for volume calculation

<!-- class="exercise-tip" -->
**Deliverable**: Report with workspace plots, area calculations, and discussion of how joint limits affect reachability.

---

*Version 2.0.0 - Academic curriculum emphasizing theoretical foundations with optional practical exercises*
