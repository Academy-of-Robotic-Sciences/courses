---
id: hum-104-arm-integration
title: "HUM-104: Arm Integration and Manipulation Kinematics"
sidebar_position: 5
version: 2.0.0
link: https://robotcampus.dev/styles/course-styles.css
---

# HUM-104: Arm Integration and Manipulation Kinematics

> Design and construction of five degree-of-freedom anthropomorphic arms with analysis of forward/inverse kinematics, workspace optimization, and whole-body dynamics.

## Course Overview

| Parameter | Details |
|---|---|
| **Course Code** | HUM-104 |
| **Duration** | 8 contact hours |
| **Format** | Theory (40%) + Guided Project (60%) |
| **Prerequisites** | HUM-103, robotics kinematics |
| **Deliverables** | Two complete 5-DOF arms, kinematic solver, gripper design |

---

## Learning Objectives

### Theoretical Knowledge
- Derive forward kinematics for serial manipulators using DH parameters
- Implement analytical and numerical inverse kinematics solutions
- Analyze manipulability and dexterity metrics
- Understand dynamic coupling between arm motion and balance

### Practical Skills
- Design and fabricate multi-DOF arm mechanisms
- Implement end-effector control (grippers or hands)
- Develop trajectory planning for smooth motion
- Integrate arm control with existing bipedal platform

---

## Part 1: Theoretical Foundations (3 hours)

### Anthropomorphic Arm Architecture

<div class="theory-concept">

**Human Arm as Design Template**

The human arm provides approximately 7 degrees of freedom (shoulder: 3, elbow: 1, wrist: 3), enabling remarkable dexterity. For humanoid robots, a 5-DOF design balances capability with complexity:

**Shoulder (2 DOF):** Pitch and roll (simplified from 3-DOF ball joint)
**Elbow (1 DOF):** Flexion/extension
**Wrist (2 DOF):** Pitch and roll
**Gripper (1 DOF):** Open/close (not counted in arm DOF)

This configuration provides sufficient workspace for most manipulation tasks while remaining kinematically tractable.

</div>

#### Denavit-Hartenberg Parameterization

**Standard DH parameters** for 5-DOF arm (from shoulder to wrist):

| Joint | $\theta_i$ | $d_i$ | $a_i$ | $\alpha_i$ |
|-------|-----------|-------|-------|-----------|
| Shoulder Pitch | $\theta_1$ | 0 | 0 | 90° |
| Shoulder Roll | $\theta_2$ | 0 | $L_{upper}$ | 0° |
| Elbow | $\theta_3$ | 0 | $L_{forearm}$ | 0° |
| Wrist Pitch | $\theta_4$ | 0 | 0 | 90° |
| Wrist Roll | $\theta_5$ | $L_{hand}$ | 0 | 0° |

Where:
- $L_{upper} = 150mm$ (upper arm length)
- $L_{forearm} = 150mm$ (forearm length)
- $L_{hand} = 80mm$ (hand/gripper length)

**Forward Kinematics:** Position and orientation of end-effector in shoulder frame:

$${}^{shoulder}\mathbf{T}_{ee} = \prod_{i=1}^5 {}^{i-1}\mathbf{T}_i(\theta_i, d_i, a_i, \alpha_i)$$

Each transformation:

$${}^{i-1}\mathbf{T}_i = \begin{bmatrix}
c\theta_i & -s\theta_i c\alpha_i & s\theta_i s\alpha_i & a_i c\theta_i \\
s\theta_i & c\theta_i c\alpha_i & -c\theta_i s\alpha_i & a_i s\theta_i \\
0 & s\alpha_i & c\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

Where $c\theta_i = \cos\theta_i$, $s\theta_i = \sin\theta_i$.

#### Inverse Kinematics

**Problem statement:** Given desired end-effector pose ${}^{shoulder}\mathbf{T}_{desired}$, find joint angles $\boldsymbol{\theta} = [\theta_1, \theta_2, \theta_3, \theta_4, \theta_5]^T$.

**Analytical solution** (for 5-DOF arm with spherical wrist):

1. **Position IK for first 3 joints** (shoulder-elbow chain):

Wrist center position: $\mathbf{p}_{wrist} = \mathbf{p}_{ee} - L_{hand} \cdot \mathbf{z}_{ee}$

Distance from shoulder to wrist:
$$r = \sqrt{p_x^2 + p_y^2 + p_z^2}$$

**Elbow angle** (law of cosines):
$$\theta_3 = \cos^{-1}\left(\frac{r^2 - L_{upper}^2 - L_{forearm}^2}{2 L_{upper} L_{forearm}}\right)$$

**Shoulder angles:**
$$\theta_1 = \text{atan2}(p_y, \sqrt{p_x^2 + p_z^2})$$
$$\theta_2 = \text{atan2}(p_x, p_z) - \text{atan2}(L_{forearm}\sin\theta_3, L_{upper} + L_{forearm}\cos\theta_3)$$

2. **Orientation IK for wrist joints** (decouple from position):

Given desired rotation matrix $\mathbf{R}_{desired}$ and known $\mathbf{R}_{0:3}$ from first 3 joints:

$$\mathbf{R}_{wrist} = \mathbf{R}_{0:3}^{-1} \mathbf{R}_{desired}$$

Extract Euler angles for wrist pitch and roll.

<div class="theory-concept">

**Multiple Solutions and Singularities**

A 5-DOF arm typically has **multiple IK solutions** (elbow-up vs elbow-down configurations). Selection criteria:
1. Minimize joint motion from current configuration
2. Avoid joint limits
3. Maximize manipulability
4. Maintain balance for humanoid platform

**Singularities** occur when:
- Fully extended arm ($\theta_3 = 0$)
- Wrist aligned with shoulder (lose rotational DOF)

</div>

### Workspace Analysis

**Reachable workspace:** Set of all points end-effector can reach (ignoring orientation).

For planar arm in x-z plane:
$$W_{reachable} = \{(x,z) : |L_{upper} - L_{forearm}| \leq \sqrt{x^2 + z^2} \leq L_{upper} + L_{forearm}\}$$

This creates an annular region with:
- Inner radius: $|L_{upper} - L_{forearm}| = 0$ (if equal length)
- Outer radius: $L_{upper} + L_{forearm} = 300mm$

**Dexterous workspace:** Set of points reachable with arbitrary orientation (much smaller subset).

#### Manipulability Analysis

**Manipulability ellipsoid** characterizes ease of motion in different directions:

$$m(\boldsymbol{\theta}) = \sqrt{\det(\mathbf{J}(\boldsymbol{\theta}) \mathbf{J}^T(\boldsymbol{\theta}))}$$

Where $\mathbf{J}$ is the Jacobian matrix relating joint velocities to end-effector velocities:

$$\dot{\mathbf{x}} = \mathbf{J}(\boldsymbol{\theta}) \dot{\boldsymbol{\theta}}$$

**High manipulability** (m ≈ 1): Robot can move easily in all directions
**Low manipulability** (m ≈ 0): Near singularity, restricted motion

<div class="historical-note">

**Evolution of Humanoid Arm Design**

Early humanoid arms (WABOT-1, 1973) used pneumatic cylinders with limited DOF. Key developments:

- **1986:** Waseda's WABOT-2 achieves dexterous manipulation with 6-DOF arms
- **1996:** Honda P2 demonstrates coordinated arm-leg motion during walking
- **2000:** ASIMO integrates force sensing for compliant manipulation
- **2013:** DARPA Robotics Challenge drives development of robust manipulation
- **2020:** Modern humanoids (Optimus, Atlas) use compact harmonic drives for high torque density

Current trend: Modular designs with integrated sensors and distributed control.

</div>

### Whole-Body Dynamics and Balance

<div class="theory-concept">

**Dynamic Coupling Between Arms and Balance**

Arm motion creates reaction forces that affect the robot's center of mass. For a 400g arm extending horizontally at 0.3m from torso:

**Moment about CoM:**
$$M = m_{arm} \cdot r \cdot \ddot{\theta} = 0.4kg \times 0.3m \times 5 rad/s^2 = 0.6 N \cdot m$$

This moment must be compensated by:
1. **Opposite arm motion** (most efficient)
2. **Hip torque** (affects balance)
3. **Ankle torque** (last resort, limited authority)

</div>

**Design principle:** Symmetric bilateral arm motion minimizes disturbance to balance.

### Knowledge Check Quiz

1. **Question:** For arms with upper arm = forearm = 150mm, what is the reachable workspace radius?
   - (a) 150 mm
   - (b) 212 mm
   - (c) 300 mm
   - (d) 450 mm

   **Answer:** (c) - Maximum reach = $L_{upper} + L_{forearm} = 300mm$

2. **Question:** Why does extending one arm affect robot balance?
   - (a) Increases total mass
   - (b) Shifts center of mass laterally
   - (c) Changes ground contact
   - (d) Reduces power available

   **Answer:** (b) - CoM shifts toward extended arm, requiring compensation

3. **Question:** At a singularity, the manipulability measure equals:
   - (a) 1.0
   - (b) Maximum value
   - (c) 0
   - (d) Undefined

   **Answer:** (c) - Jacobian loses rank, determinant becomes zero

---

## Part 2: Guided Project - Arm Assembly (4.5 hours)

### Phase 1: Arm Design and Preparation (45 min)

#### Component Selection

**Per arm (5 servos + gripper):**

**Servo Requirements:**
- 2× Medium-torque servos (12-15 kg·cm) for shoulder joints
- 1× Medium-torque servo (12 kg·cm) for elbow
- 2× Standard servos (9-12 kg·cm) for wrist joints
- 1× Small servo (5-9 kg·cm) for gripper

**Mechanical Components:**
- Upper arm segment (150mm length)
- Forearm segment (150mm length)
- Wrist assembly bracket
- Gripper mechanism (parallel jaw or 3-finger)
- Servo brackets and horns
- M3 hardware kit

#### Arm Mass Budget Analysis

**Target:** Each arm < 400g to limit impact on balance

```python
def calculate_arm_mass():
    """Estimate total arm mass and CoM"""
    components = {
        'shoulder_servos': {'mass': 55*2, 'position': 0.02},   # 2 servos
        'upper_arm': {'mass': 40, 'position': 0.075},
        'elbow_servo': {'mass': 55, 'position': 0.15},
        'forearm': {'mass': 35, 'position': 0.225},
        'wrist_servos': {'mass': 25*2, 'position': 0.30},      # 2 servos
        'gripper': {'mass': 30, 'position': 0.35}
    }

    total_mass = sum(c['mass'] for c in components.values())
    com_position = sum(c['mass'] * c['position'] for c in components.values()) / total_mass

    print(f"Total arm mass: {total_mass}g")
    print(f"Arm CoM position: {com_position*1000:.1f}mm from shoulder")

    # Check against budget
    if total_mass > 400:
        print(f"⚠️ Exceeds budget by {total_mass - 400}g. Consider lighter components.")
    else:
        print(f"✓ Within budget ({400 - total_mass}g margin)")

    return total_mass, com_position

calculate_arm_mass()
```

### Phase 2: Shoulder and Upper Arm (60 min)

#### 2-DOF Shoulder Joint

**Assembly sequence:**

1. **Shoulder Pitch Servo** (30 min)
   - Mounts to torso side panel
   - Provides forward/backward arm swing
   - **Critical alignment:** Axis horizontal, perpendicular to torso

   ```
   [Torso Side Mount]
          |
   [Pitch Servo] ← Forward/back motion
          |
   [Pitch Output Bracket]
   ```

2. **Shoulder Roll Servo** (30 min)
   - Mounts perpendicular to pitch servo output
   - Provides arm raise/lower motion
   - Creates 2-DOF compound shoulder

   ```
   [Pitch Output]
          |
   [Roll Servo] ← Arm raise/lower
          |
   [Upper Arm Attachment]
   ```

**Verification:**
- Both axes intersect at ideal shoulder pivot point
- Full range of motion without collision with torso
- Symmetric installation for left/right arms

### Phase 3: Elbow and Forearm (60 min)

#### 1-DOF Elbow Joint

1. **Upper Arm Installation** (20 min)
   - Attach to shoulder roll servo horn
   - Verify arm straight down when servo at 90°
   - Secure with minimum 4× M3 bolts

2. **Elbow Servo Mounting** (25 min)
   - Mount servo at end of upper arm
   - **Alignment:** Elbow axis perpendicular to upper arm
   - Ensure robust connection (elbow carries forearm + wrist + gripper)

3. **Forearm Integration** (15 min)
   - Attach forearm to elbow servo horn
   - Route servo cables through forearm interior
   - Verify straight arm at elbow angle 0°

**Testing reach:**

```cpp
void test_arm_reach() {
  // Test maximum vertical reach
  set_shoulder_pitch(0);   // Neutral
  set_shoulder_roll(0);    // Arm raised vertical
  set_elbow(0);            // Straight
  delay(2000);

  // Test maximum forward reach
  set_shoulder_pitch(90);  // Forward
  set_shoulder_roll(90);   // Horizontal
  set_elbow(0);            // Straight
  delay(2000);

  // Test bent elbow configuration
  set_shoulder_pitch(45);
  set_shoulder_roll(60);
  set_elbow(90);           // 90° elbow bend
  delay(2000);
}
```

### Phase 4: Wrist and Gripper (90 min)

#### 2-DOF Wrist Mechanism

**Wrist design:**

```
[Forearm End]
       |
[Wrist Pitch Servo] ← Hand up/down
       |
[Wrist Roll Servo]  ← Hand rotation
       |
[Gripper Attachment]
```

1. **Wrist Pitch** (25 min)
   - Allows hand to tilt up/down
   - Critical for grasping objects at different heights
   - Mount to forearm end

2. **Wrist Roll** (25 min)
   - Provides hand rotation
   - Enables tool-like manipulation
   - Mount perpendicular to pitch axis

#### Gripper Design Options

**Option A: Parallel Jaw Gripper**

Simple, robust design for cylindrical objects:

```python
class ParallelGripper:
    """Control parallel jaw gripper"""
    def __init__(self, servo_pin, open_angle=90, close_angle=45):
        self.servo_pin = servo_pin
        self.open_angle = open_angle
        self.close_angle = close_angle

    def open(self):
        """Open gripper to maximum"""
        set_servo(self.servo_pin, self.open_angle)

    def close(self):
        """Close gripper fully"""
        set_servo(self.servo_pin, self.close_angle)

    def set_width(self, width_mm):
        """Set gripper to specific width (0-60mm)"""
        angle = self.open_angle - (width_mm / 60.0) * (self.open_angle - self.close_angle)
        set_servo(self.servo_pin, angle)

    def grasp_object(self, estimated_width):
        """Grasp object with adaptive closure"""
        self.open()
        delay(500)
        # Gradually close until current spike (object contact)
        for width in range(estimated_width, 0, -2):
            self.set_width(width)
            if self.detect_contact():  # Monitor servo current
                return True
            delay(100)
        return False  # Failed to grasp
```

**Option B: 3-Finger Adaptive Gripper**

More complex, better conformability to irregular shapes:

- Three fingers arranged 120° apart
- Single servo drives all fingers via linkage
- Passive compliance in finger joints
- Better for delicate objects

**Implementation** (40 min):
- 3D print gripper mechanism
- Mount servo to wrist roll output
- Install mechanical linkages
- Test grasp of various objects (cubes, cylinders, spheres)

<div class="exercise-tip">

**Gripper Force Considerations**

Maximum safe grip force from servo torque:

$$F_{grip} = \frac{\tau_{servo}}{r_{finger}}$$

For 5 kg·cm servo and 20mm finger length:
$$F = \frac{0.05 N \cdot m}{0.02 m} = 2.5 N$$

This is sufficient for 200-300g objects but inadequate for heavy items.

**Force limiting:** Use current sensing to prevent crushing delicate objects.

</div>

### Phase 5: Integration and Testing (45 min)

#### Complete Bilateral Assembly

1. **Wire Both Arms** (20 min)
   - Left arm servos: Arduino pins 14-19
   - Right arm servos: Arduino pins 20-25
   - Common 12V power bus
   - Organized cable routing through torso

2. **Calibration Procedure** (15 min)

```cpp
// Arm servo enumeration
enum ArmServo {
  L_SHOULDER_PITCH = 14, L_SHOULDER_ROLL, L_ELBOW, L_WRIST_PITCH, L_WRIST_ROLL, L_GRIPPER,
  R_SHOULDER_PITCH = 20, R_SHOULDER_ROLL, R_ELBOW, R_WRIST_PITCH, R_WRIST_ROLL, R_GRIPPER
};

// Neutral positions (arms at sides)
int arm_neutral[12] = {
  90, 90, 90, 90, 90, 90,  // Left arm
  90, 90, 90, 90, 90, 90   // Right arm
};

void calibrate_arms() {
  Serial.println("Arm calibration sequence:");

  // Move to neutral
  Serial.println("Moving to neutral position...");
  for(int i = 0; i < 12; i++) {
    arm_servos[i].write(arm_neutral[i]);
  }
  delay(2000);

  // Test each joint individually
  for(int i = 0; i < 12; i++) {
    Serial.print("Testing servo "); Serial.println(i);

    // Sweep through range
    for(int angle = 45; angle <= 135; angle += 5) {
      arm_servos[i].write(angle);
      delay(50);
    }

    // Return to neutral
    arm_servos[i].write(arm_neutral[i]);
    delay(500);
  }

  Serial.println("Calibration complete. Document any offset adjustments needed.");
}
```

3. **Symmetric Motion Test** (10 min)

```cpp
void test_symmetric_motion() {
  // Raise both arms simultaneously
  Serial.println("Raising arms...");
  for(int angle = 90; angle >= 0; angle -= 2) {
    arm_servos[L_SHOULDER_ROLL].write(angle);
    arm_servos[R_SHOULDER_ROLL].write(angle);
    delay(30);
  }

  // Forward reach
  Serial.println("Reaching forward...");
  for(int angle = 90; angle <= 135; angle += 2) {
    arm_servos[L_SHOULDER_PITCH].write(angle);
    arm_servos[R_SHOULDER_PITCH].write(angle);
    delay(30);
  }

  // Return to neutral
  Serial.println("Returning to neutral...");
  for(int i = 0; i < 12; i++) {
    arm_servos[i].write(arm_neutral[i]);
  }
}
```

---

## Part 3: Kinematic Implementation and Validation (30 min)

### Forward Kinematics Implementation

```python
import numpy as np

class ArmKinematics:
    """5-DOF arm kinematics solver"""

    def __init__(self, L_upper=150, L_forearm=150, L_hand=80):
        self.L_upper = L_upper
        self.L_forearm = L_forearm
        self.L_hand = L_hand

    def dh_transform(self, theta, d, a, alpha):
        """Compute DH transformation matrix"""
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)

        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d   ],
            [0,   0,      0,     1   ]
        ])

    def forward_kinematics(self, joint_angles):
        """
        Compute end-effector pose from joint angles
        joint_angles: [shoulder_pitch, shoulder_roll, elbow, wrist_pitch, wrist_roll]
        Returns: 4×4 transformation matrix
        """
        theta = np.radians(joint_angles)

        # DH parameters
        T1 = self.dh_transform(theta[0], 0, 0, np.pi/2)
        T2 = self.dh_transform(theta[1], 0, self.L_upper, 0)
        T3 = self.dh_transform(theta[2], 0, self.L_forearm, 0)
        T4 = self.dh_transform(theta[3], 0, 0, np.pi/2)
        T5 = self.dh_transform(theta[4], self.L_hand, 0, 0)

        # Chain transformations
        T = T1 @ T2 @ T3 @ T4 @ T5

        return T

    def get_position(self, joint_angles):
        """Extract position from FK"""
        T = self.forward_kinematics(joint_angles)
        return T[:3, 3]

    def get_orientation(self, joint_angles):
        """Extract rotation matrix from FK"""
        T = self.forward_kinematics(joint_angles)
        return T[:3, :3]

# Test forward kinematics
arm = ArmKinematics()
test_angles = [0, 0, 0, 0, 0]  # All joints at neutral
position = arm.get_position(test_angles)
print(f"End-effector position: {position}")
# Expected: [0, 0, L_upper + L_forearm + L_hand] = [0, 0, 380]mm
```

### Inverse Kinematics Implementation

```python
def inverse_kinematics_position(self, target_position):
    """
    Solve IK for position (first 3 joints)
    Returns: [shoulder_pitch, shoulder_roll, elbow] or None if unreachable
    """
    px, py, pz = target_position

    # Account for wrist/hand length
    wrist_target = target_position - np.array([0, 0, self.L_hand])
    wx, wy, wz = wrist_target

    # Distance from shoulder to wrist
    r = np.sqrt(wx**2 + wy**2 + wz**2)

    # Check reachability
    L_max = self.L_upper + self.L_forearm
    L_min = abs(self.L_upper - self.L_forearm)

    if r > L_max or r < L_min:
        return None  # Target unreachable

    # Elbow angle (law of cosines)
    cos_elbow = (r**2 - self.L_upper**2 - self.L_forearm**2) / (2 * self.L_upper * self.L_forearm)
    cos_elbow = np.clip(cos_elbow, -1, 1)
    elbow = np.arccos(cos_elbow)

    # Shoulder angles
    shoulder_pitch = np.arctan2(wy, np.sqrt(wx**2 + wz**2))

    alpha = np.arctan2(wx, wz)
    beta = np.arctan2(self.L_forearm * np.sin(elbow),
                     self.L_upper + self.L_forearm * np.cos(elbow))
    shoulder_roll = alpha - beta

    return np.degrees([shoulder_pitch, shoulder_roll, elbow])

# Test IK
target = np.array([200, 0, 200])  # Point in front and to the right
solution = arm.inverse_kinematics_position(target)
if solution:
    print(f"IK solution: {solution}")
    # Verify by FK
    computed_pos = arm.get_position(list(solution) + [0, 0])
    error = np.linalg.norm(computed_pos - target)
    print(f"Position error: {error:.2f}mm")
```

### Workspace Visualization

```python
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def generate_workspace(resolution=20):
    """Generate reachable workspace points"""
    workspace_points = []

    # Sample joint space
    for sp in np.linspace(-90, 90, resolution):
        for sr in np.linspace(-90, 90, resolution):
            for elbow in np.linspace(0, 150, resolution):
                angles = [sp, sr, elbow, 0, 0]
                pos = arm.get_position(angles)
                workspace_points.append(pos)

    return np.array(workspace_points)

# Generate and plot
workspace = generate_workspace(resolution=15)

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(workspace[:,0], workspace[:,1], workspace[:,2], s=1, alpha=0.3)
ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_zlabel('Z (mm)')
ax.set_title('5-DOF Arm Reachable Workspace')
plt.show()
```

### Knowledge Check Quiz

4. **Question:** If IK returns two solutions (elbow-up and elbow-down), which should be selected?
   - (a) Always elbow-up
   - (b) Random choice
   - (c) Closest to current configuration
   - (d) Higher manipulability

   **Answer:** (c) or (d) - Minimize motion or maximize dexterity

5. **Question:** Why is the wrist excluded from position IK calculation?
   - (a) Wrist doesn't affect position
   - (b) Position IK uses only first 3 joints
   - (c) Wrist is decoupled due to spherical geometry
   - (d) Insufficient DOF

   **Answer:** (c) - Spherical wrist decouples position from orientation

---

## Deliverables and Assessment

### Required Submissions

1. **Functional Hardware**
   - Two complete 5-DOF arms with grippers
   - Symmetric bilateral installation
   - All joints demonstrate full range of motion
   - Successful grasp of test objects (cube, cylinder, sphere)

2. **Kinematic Analysis**
   - Forward kinematics implementation with validation
   - Inverse kinematics solver (position and orientation)
   - Workspace visualization (3D plot)
   - Manipulability analysis at key configurations

3. **Code Repository**
   - Complete arm control library
   - IK solver with multiple solution handling
   - Gripper control with force limiting
   - Example trajectories

4. **Integration Report**
   - Mass budget analysis
   - Balance impact assessment
   - Calibration procedure documentation
   - Troubleshooting log

### Assessment Rubric

**Mechanical Construction (30%)**
- Robust joint assembly
- Symmetric bilateral design
- Functional gripper mechanism
- Professional cable management

**Kinematic Implementation (35%)**
- Correct FK derivation and code
- Working IK solver
- Workspace analysis
- Singularity handling

**System Integration (20%)**
- Successful whole-body assembly
- Coordinated bilateral motion
- Balance preservation during arm motion
- Grasp demonstrations

**Documentation (15%)**
- Clear technical explanations
- Reproducible calibration procedure
- Performance metrics
- Design justifications

---

## Extensions and Further Study

<div class="optional-exercise">

**Advanced Challenges**

1. **7-DOF Arms:** Add redundant joint for obstacle avoidance
2. **Compliant Control:** Implement force sensing and impedance control
3. **Visual Servoing:** Use camera feedback for closed-loop grasping
4. **Dual-Arm Coordination:** Bimanual manipulation tasks
5. **Trajectory Optimization:** Minimum-time or energy-optimal paths

</div>

### Recommended Reading

- **Textbook:** "Robot Modeling and Control" by Spong et al., Chapters 3-4
- **Paper:** Siciliano & Khatib (2008), "Springer Handbook of Robotics" - Chapter on Kinematics
- **Tutorial:** "A Mathematical Introduction to Robotic Manipulation" by Murray et al.

---

## Connection to HUM-105

With arms integrated, the humanoid now requires a comprehensive sensor suite to perceive its environment and internal state. HUM-105 will add IMUs, force sensors, cameras, and sensor fusion algorithms to create a complete sensory nervous system.

---

*Manipulation capability transforms humanoid from mobile platform to versatile robot. Proceed to HUM-105 to add sensory perception.*
