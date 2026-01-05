---
id: hum-103-leg-mechanics
title: "HUM-103: Bipedal Leg Mechanics and Assembly"
sidebar_position: 4
version: 2.0.0
link: https://robotcampus.dev/styles/course-styles.css
---

# HUM-103: Bipedal Leg Mechanics and Assembly

> Design and construction of six degree-of-freedom bipedal legs with analysis of inverse kinematics, torque requirements, and structural loading in humanoid locomotion.

## Course Overview

| Parameter | Details |
|---|---|
| **Course Code** | HUM-103 |
| **Duration** | 8 contact hours |
| **Format** | Theory (40%) + Guided Project (60%) |
| **Prerequisites** | HUM-102, kinematics fundamentals |
| **Deliverables** | Two complete legs, kinematic analysis, standing robot |

---

## Learning Objectives

### Theoretical Knowledge
- Derive inverse kinematics for 6-DOF leg chains
- Calculate joint torques from ground reaction forces
- Analyze stability conditions for bipedal stance
- Understand Zero Moment Point (ZMP) criterion

### Practical Skills
- Assemble complex multi-DOF mechanical linkages
- Configure servo motors for high-torque applications
- Implement forward and inverse kinematics in code
- Calibrate symmetric bilateral systems

---

## Part 1: Theoretical Foundations (3 hours)

### Bipedal Leg Architecture

<div class="theory-concept">

**Human-Inspired Leg Design**

The human leg provides the template for bipedal robot design:

**Hip (3 DOF):** Flexion/extension (sagittal), abduction/adduction (frontal), internal/external rotation (transverse)
**Knee (1 DOF):** Flexion/extension only (simplified from human 2 DOF)
**Ankle (2 DOF):** Dorsiflexion/plantarflexion, inversion/eversion

This 6-DOF configuration provides sufficient workspace for walking while remaining computationally tractable.

</div>

#### Kinematic Chain Analysis

**Denavit-Hartenberg Parameters:**

For the leg modeled as a serial kinematic chain from hip to foot:

| Joint | $\theta_i$ | $d_i$ | $a_i$ | $\alpha_i$ |
|-------|-----------|-------|-------|-----------|
| Hip Yaw | $\theta_1$ | 0 | 0 | 90° |
| Hip Roll | $\theta_2$ | 0 | 0 | 90° |
| Hip Pitch | $\theta_3$ | 0 | $L_{thigh}$ | 0° |
| Knee | $\theta_4$ | 0 | $L_{shin}$ | 0° |
| Ankle Pitch | $\theta_5$ | 0 | 0 | 90° |
| Ankle Roll | $\theta_6$ | 0 | $L_{foot}$ | 0° |

**Forward Kinematics:** Position of foot in hip frame:

$${}^{hip}\mathbf{T}_{foot} = {}^{hip}\mathbf{T}_1 \cdot {}^1\mathbf{T}_2 \cdot {}^2\mathbf{T}_3 \cdot {}^3\mathbf{T}_4 \cdot {}^4\mathbf{T}_5 \cdot {}^5\mathbf{T}_6$$

Where each transformation matrix:

$${}^{i-1}\mathbf{T}_i = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

#### Inverse Kinematics

For a desired foot position $\mathbf{p} = [x, y, z]^T$ in hip frame, solve for joint angles:

**Simplified 2D planar solution** (sagittal plane, hip-knee-ankle):

Given target position $(x, z)$ from hip:

$$L = \sqrt{x^2 + z^2}$$

**Knee angle** (law of cosines):
$$\theta_{knee} = \cos^{-1}\left(\frac{L^2 - L_{thigh}^2 - L_{shin}^2}{2 L_{thigh} L_{shin}}\right)$$

**Hip angle:**
$$\theta_{hip} = \tan^{-1}\left(\frac{x}{z}\right) - \tan^{-1}\left(\frac{L_{shin}\sin\theta_{knee}}{L_{thigh} + L_{shin}\cos\theta_{knee}}\right)$$

**Ankle angle** (to maintain foot parallel to ground):
$$\theta_{ankle} = -(\theta_{hip} + \theta_{knee})$$

<div class="theory-concept">

**Singularities and Workspace Limitations**

The leg becomes singular when:
1. **Fully extended:** $\theta_{knee} = 0$ (lose ability to move vertically)
2. **Fully collapsed:** $\theta_{knee} = 180°$ (physically impossible)

Practical workspace: Keep knee in range 30° ≤ $\theta_{knee}$ ≤ 150°

</div>

### Structural Loading Analysis

#### Ground Reaction Forces

During single-leg stance, entire robot weight concentrates on one foot:

$$F_{ground} = m_{robot} \cdot g \approx 4kg \times 9.81 m/s^2 = 39.2 N$$

This force propagates up the kinematic chain, creating joint torques.

#### Torque Calculation

**Static case** (standing still):

For knee joint supporting mass $m_{upper}$ (torso + head + arms) at distance $r$ from knee pivot:

$$\tau_{knee} = m_{upper} \cdot g \cdot r$$

With $m_{upper} = 2.5kg$, $r = L_{shin}\sin\theta_{knee} = 0.15m \times \sin(10°) = 0.026m$:

$$\tau_{knee} = 2.5 \times 9.81 \times 0.026 = 0.64 N \cdot m = 6.4 kg \cdot cm$$

**Dynamic case** (walking):

During push-off, leg must accelerate body upward. With vertical acceleration $a = 2 m/s^2$:

$$\tau_{knee,dynamic} = m_{upper}(g + a) \cdot r = 2.5 \times (9.81 + 2) \times 0.026 = 0.77 N \cdot m = 7.7 kg \cdot cm$$

Add safety factor 2×: **Required knee torque ≥ 15 kg·cm**

<div class="historical-note">

**Evolution of Bipedal Leg Design**

Early walking robots (WABOT-1, 1973) used large pneumatic actuators, limiting portability. Key advances:

- **1986:** Honda E0 uses custom DC motors, achieves first dynamic walking
- **1996:** Honda P2 introduces compact harmonic drive actuators
- **2000:** ASIMO demonstrates 1.6 km/h walking speed
- **2013:** Boston Dynamics Atlas uses hydraulic actuation (90 kg, 1.5m tall)
- **2021:** Tesla Optimus design targets low-cost electric actuation

Modern trend: High-torque, low-cost electric servo motors enable accessible bipedal platforms.

</div>

### Zero Moment Point Criterion

<div class="theory-concept">

**Dynamic Stability Condition**

The Zero Moment Point (ZMP) is the point on the ground where the sum of moments from gravity and inertial forces equals zero. For stable walking:

**ZMP must remain inside the support polygon** (convex hull of ground contact points)

</div>

**ZMP Calculation:**

For robot with CoM at $[x_{CoM}, y_{CoM}, z_{CoM}]$, acceleration $[\ddot{x}, \ddot{y}, \ddot{z}]$:

$$x_{ZMP} = x_{CoM} - z_{CoM}\frac{\ddot{x}}{\ddot{z} + g}$$

$$y_{ZMP} = y_{CoM} - z_{CoM}\frac{\ddot{y}}{\ddot{z} + g}$$

**Static case** ($\ddot{x} = \ddot{y} = 0$): ZMP = projection of CoM onto ground

**Dynamic case:** ZMP shifts based on acceleration direction

### Knowledge Check Quiz

1. **Question:** In a 6-DOF leg, which joints contribute most to forward/backward motion during walking?
   - (a) Hip yaw and ankle roll
   - (b) Hip pitch and knee
   - (c) Hip roll and ankle pitch
   - (d) All equally

   **Answer:** (b) - Sagittal plane motion (hip pitch, knee) drives forward walking

2. **Question:** If thigh and shin are both 15cm, what is maximum horizontal reach of the foot?
   - (a) 15 cm
   - (b) 21 cm
   - (c) 30 cm
   - (d) 45 cm

   **Answer:** (c) - Fully extended: $L_{max} = L_{thigh} + L_{shin} = 30cm$

3. **Question:** Why does knee torque requirement increase during dynamic walking compared to standing?
   - (a) Knee angle changes
   - (b) Additional inertial forces from acceleration
   - (c) Friction increases
   - (d) Servo efficiency decreases

   **Answer:** (b) - Must counteract both gravity and acceleration forces

---

## Part 2: Guided Project - Leg Assembly (4.5 hours)

### Phase 1: Component Preparation (45 min)

#### Parts Inventory (per leg)

**Servo Motors:**
- 1× High-torque servo (20 kg·cm) for knee
- 2× Medium-torque servos (15 kg·cm) for hip pitch, ankle pitch
- 3× Standard servos (12 kg·cm) for hip roll/yaw, ankle roll

**Mechanical Components:**
- 2× Thigh segments (150mm length, 3D printed or aluminum)
- 2× Shin segments (150mm length)
- 2× Foot plates (100×80mm)
- 12× Servo brackets (various angles)
- 6× Servo horns (matched to servo splines)
- Hardware kit (M3 bolts, nuts, washers)

**Wiring:**
- 6× Servo extension cables (300mm)
- Cable management (spiral wrap, zip ties)

#### Servo Centering Procedure

Before assembly, center all servos to 90° neutral position:

```cpp
#include <Servo.h>

Servo servos[12];  // 6 servos per leg × 2 legs
int servo_pins[] = {2,3,4,5,6,7,8,9,10,11,12,13};  // Arduino Mega pins

void setup() {
  Serial.begin(115200);

  for(int i = 0; i < 12; i++) {
    servos[i].attach(servo_pins[i]);
    servos[i].write(90);  // Center position
    delay(500);
  }

  Serial.println("All servos centered. Remove power and install horns.");
}

void loop() {
  // Keep servos at center
}
```

**Critical:** Attach servo horns only after centering to ensure symmetric range of motion.

### Phase 2: Hip Assembly (60 min)

#### 3-DOF Hip Joint Construction

**Assembly sequence** (creates compound joint with 3 orthogonal axes):

1. **Hip Yaw (Rotation):** Mounts to torso bottom plate
   ```
   [Torso Mount Point]
          |
   [Yaw Servo] ← Allows leg to swing laterally
          |
   [Transition Bracket]
   ```

2. **Hip Roll (Abduction/Adduction):** Perpendicular to yaw
   ```
   [Yaw Output Horn]
          |
   [Roll Servo] ← Side-to-side leg spreading
          |
   [Roll Bracket]
   ```

3. **Hip Pitch (Flexion/Extension):** Main walking motion
   ```
   [Roll Output Horn]
          |
   [Pitch Servo] ← Forward/backward swing
          |
   [Thigh Attachment Point]
   ```

**Alignment verification:**
- All three axes intersect at single point (ideal hip pivot)
- Axes mutually perpendicular (verify with square)
- Full range of motion without bracket collision

#### Dual Hip Installation

Mount both hip assemblies to torso with symmetric spacing:

```python
# Verify symmetric installation
hip_spacing = 100  # mm, center-to-center distance
tolerance = 2      # mm, acceptable error

def verify_symmetry():
    """Measure and verify left/right hip alignment"""
    left_position = measure_hip_center('left')
    right_position = measure_hip_center('right')

    measured_spacing = abs(left_position[1] - right_position[1])

    if abs(measured_spacing - hip_spacing) < tolerance:
        print("✓ Hip spacing within tolerance")
    else:
        print(f"✗ Spacing error: {measured_spacing - hip_spacing}mm")

    # Check height alignment
    height_diff = abs(left_position[2] - right_position[2])
    if height_diff < tolerance:
        print("✓ Hip heights aligned")
    else:
        print(f"✗ Height misalignment: {height_diff}mm")
```

### Phase 3: Leg Linkages (90 min)

#### Thigh-Knee-Shin Assembly

1. **Thigh Attachment** (20 min)
   - Connect thigh segment to hip pitch servo horn
   - Ensure thigh vertical when servo at 90°
   - Secure with M3 bolts (minimum 4 per connection)

2. **Knee Joint Installation** (30 min)
   - Mount high-torque knee servo to thigh end
   - **Critical alignment:** Knee axis perpendicular to thigh
   - Reinforce knee mounting (highest load joint)
   - Add mechanical stops to prevent hyperextension

3. **Shin Integration** (20 min)
   - Attach shin to knee servo horn
   - Route servo cables through hollow shin structure
   - Verify straight leg at knee angle 0°

4. **Testing Planar Kinematics** (20 min)
   ```python
   import numpy as np

   def test_2d_leg_reach():
       """Verify leg can reach expected workspace"""
       L_thigh = 150  # mm
       L_shin = 150   # mm

       # Test points in workspace
       test_points = [
           (0, 300),    # Straight down (fully extended)
           (100, 250),  # Forward and down
           (150, 200),  # Maximum forward
           (0, 200)     # Bent knee, straight down
       ]

       for x, z in test_points:
           angles = inverse_kinematics_2d(x, z, L_thigh, L_shin)
           if angles:
               print(f"Point ({x},{z}): Hip={angles[0]:.1f}°, Knee={angles[1]:.1f}°")
               set_leg_angles(angles[0], angles[1], -(angles[0]+angles[1]))
               input("Press Enter after verifying position...")
           else:
               print(f"Point ({x},{z}): UNREACHABLE")

   def inverse_kinematics_2d(x, z, L1, L2):
       """Solve IK for planar 2-link leg"""
       L = np.sqrt(x**2 + z**2)

       # Check if point is reachable
       if L > L1 + L2 or L < abs(L1 - L2):
           return None

       # Knee angle (law of cosines)
       cos_knee = (L**2 - L1**2 - L2**2) / (2 * L1 * L2)
       knee = np.arccos(np.clip(cos_knee, -1, 1))

       # Hip angle
       alpha = np.arctan2(x, z)
       beta = np.arctan2(L2 * np.sin(knee), L1 + L2 * np.cos(knee))
       hip = alpha - beta

       # Convert to degrees
       return (np.degrees(hip), np.degrees(knee))
   ```

### Phase 4: Ankle and Foot (60 min)

#### 2-DOF Ankle Joint

**Ankle mechanism:**

```
[Shin End]
     |
[Ankle Pitch Servo] ← Dorsiflexion/plantarflexion
     |
[Ankle Roll Servo]  ← Inversion/eversion
     |
[Foot Plate]
```

**Foot design considerations:**

- **Size:** 100×80mm provides adequate support polygon
- **Material:** Rigid base (PLA/aluminum) with rubber contact pads
- **Sensor mounts:** Integrate 4× force-sensing resistors (FSRs) at corners
- **Compliance:** Optional foam layer for impact absorption

#### Foot Plate Assembly

```cpp
// Foot force sensor integration (preparation for HUM-105)
const int FSR_PINS[] = {A0, A1, A2, A3};  // Four corner sensors
const int NUM_FSR = 4;

void read_foot_forces() {
  int forces[NUM_FSR];
  int total_force = 0;

  for(int i = 0; i < NUM_FSR; i++) {
    forces[i] = analogRead(FSR_PINS[i]);
    total_force += forces[i];
  }

  // Calculate center of pressure (simplified)
  float cop_x = (forces[1] + forces[2] - forces[0] - forces[3]) / (float)total_force;
  float cop_y = (forces[2] + forces[3] - forces[0] - forces[1]) / (float)total_force;

  Serial.print("Total Force: "); Serial.print(total_force);
  Serial.print(" | CoP: ("); Serial.print(cop_x);
  Serial.print(", "); Serial.print(cop_y); Serial.println(")");
}
```

### Phase 5: Integration and Standing (45 min)

#### Complete System Wiring

**Power distribution:**
- All 12 leg servos connect to 12V servo bus
- Signal wires to Arduino Mega (pins 2-13)
- Common ground between Arduino and servo power

**Cable management strategy:**
- Group wires by leg segment
- Use spiral wrap for protection
- Leave slack at each joint for full range of motion
- Secure to structure with zip ties every 50mm

#### First Stand Sequence

```cpp
// Standing position controller
#include <Servo.h>

// Servo indices
enum LegServo {
  L_HIP_YAW = 0, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL,
  R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL
};

Servo servos[12];
int servo_pins[] = {2,3,4,5,6,7,8,9,10,11,12,13};

// Standing pose (all angles in degrees)
int standing_pose[12] = {
  90, 90, 90, 90, 90, 90,  // Left leg: neutral position
  90, 90, 90, 90, 90, 90   // Right leg: neutral position
};

void setup() {
  Serial.begin(115200);

  // Attach all servos
  for(int i = 0; i < 12; i++) {
    servos[i].attach(servo_pins[i]);
  }

  Serial.println("Robot ready. Type 'stand' to execute standing sequence.");
}

void loop() {
  if(Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if(cmd == "stand") {
      execute_stand_sequence();
    } else if(cmd == "sit") {
      execute_sit_sequence();
    }
  }
}

void execute_stand_sequence() {
  Serial.println("Standing up...");

  // Gradual transition to standing pose
  int current_angles[12];
  for(int i = 0; i < 12; i++) {
    current_angles[i] = servos[i].read();
  }

  // Interpolate over 2 seconds
  for(int step = 0; step <= 100; step++) {
    for(int i = 0; i < 12; i++) {
      int target = standing_pose[i];
      int current = current_angles[i];
      int interpolated = current + (target - current) * step / 100;
      servos[i].write(interpolated);
    }
    delay(20);  // 20ms × 100 steps = 2 seconds
  }

  Serial.println("Standing complete!");
}

void execute_sit_sequence() {
  // Bend knees to lower robot to ground
  Serial.println("Sitting down...");

  for(int angle = 90; angle <= 140; angle++) {
    servos[L_KNEE].write(angle);
    servos[R_KNEE].write(angle);
    delay(20);
  }

  Serial.println("Sitting complete.");
}
```

**Testing procedure:**

1. **Visual inspection:** Verify all connections before power
2. **Power-up:** Turn on servo power with robot supported
3. **Gradual stand:** Execute stand sequence while supporting robot
4. **Release support:** Slowly remove hands, monitor stability
5. **Balance test:** Robot should stand unsupported for 10+ seconds
6. **Gentle push test:** Apply small lateral force, verify recovery

<div class="exercise-tip">

**Troubleshooting Common Issues**

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Robot leans forward/back | CoM not centered | Adjust hip pitch offsets ±5° |
| Robot leans left/right | Hip heights misaligned | Verify symmetric assembly |
| Servos jittering | Insufficient power | Check voltage under load, upgrade power supply |
| One leg collapsed | Weak knee servo | Verify high-torque servo at knee, check torque spec |
| Foot not flat on ground | Ankle angle error | Calibrate ankle to maintain foot parallel |

</div>

---

## Part 3: Validation and Analysis (30 min)

### Kinematic Validation

**Workspace verification:**

```python
import numpy as np
import matplotlib.pyplot as plt

def generate_workspace(L1, L2, resolution=100):
    """Generate reachable workspace for planar leg"""
    workspace_points = []

    theta1_range = np.linspace(-60, 60, resolution)
    theta2_range = np.linspace(0, 150, resolution)

    for theta1 in theta1_range:
        for theta2 in theta2_range:
            # Forward kinematics
            x = L1 * np.sin(np.radians(theta1)) + \
                L2 * np.sin(np.radians(theta1 + theta2))
            z = L1 * np.cos(np.radians(theta1)) + \
                L2 * np.cos(np.radians(theta1 + theta2))
            workspace_points.append((x, z))

    return np.array(workspace_points)

# Generate and plot
workspace = generate_workspace(150, 150)
plt.figure(figsize=(8, 8))
plt.scatter(workspace[:,0], workspace[:,1], s=1, alpha=0.5)
plt.xlabel('X (mm)')
plt.ylabel('Z (mm)')
plt.title('Leg Reachable Workspace')
plt.axis('equal')
plt.grid(True)
plt.show()
```

### Standing Stability Analysis

**Center of Mass calculation:**

```python
def calculate_system_com(torso_com, leg_angles):
    """Calculate total robot CoM from component positions"""

    # Component masses and positions
    components = {
        'torso': {'mass': 0.5, 'pos': torso_com},
        'head': {'mass': 0.3, 'pos': torso_com + np.array([0, 0, 0.15])},
        'leg_left': {'mass': 0.6, 'pos': compute_leg_com('left', leg_angles)},
        'leg_right': {'mass': 0.6, 'pos': compute_leg_com('right', leg_angles)}
    }

    total_mass = sum(c['mass'] for c in components.values())
    com = sum(c['mass'] * c['pos'] for c in components.values()) / total_mass

    return com

def verify_stability(com, foot_polygon):
    """Check if CoM projects inside support polygon"""
    from shapely.geometry import Point, Polygon

    com_projection = Point(com[0], com[1])  # Project to ground (x,y)
    support = Polygon(foot_polygon)

    stable = support.contains(com_projection)
    margin = support.boundary.distance(com_projection)

    return stable, margin

# Test standing stability
com = calculate_system_com(np.array([0, 0, 0.3]), standing_angles)
foot_polygon = [(0.05, 0.04), (-0.05, 0.04), (-0.05, -0.04), (0.05, -0.04)]

stable, margin = verify_stability(com, foot_polygon)
print(f"Stability: {stable}, Margin: {margin*1000:.1f}mm")
```

### Knowledge Check Quiz

4. **Question:** During standing, if CoM is shifted 2cm forward from center of foot, what happens to ZMP?
   - (a) ZMP remains at center of foot
   - (b) ZMP shifts forward 2cm
   - (c) ZMP shifts backward 2cm
   - (d) Cannot determine without acceleration data

   **Answer:** (b) - In static case, ZMP = projection of CoM

5. **Question:** Why is the knee servo specified at higher torque than other joints?
   - (a) Knee moves faster
   - (b) Knee supports largest moment arm from CoM
   - (c) Knee has larger range of motion
   - (d) Knee requires more precision

   **Answer:** (b) - Entire upper body mass creates moment about knee

---

## Deliverables and Assessment

### Required Submissions

1. **Functional Hardware**
   - Two complete 6-DOF legs mechanically integrated with torso
   - Robot stands unsupported for minimum 30 seconds
   - All joints demonstrate full range of motion

2. **Kinematic Analysis**
   - Derivation of inverse kinematics for planar leg
   - Workspace plot showing reachable positions
   - Validation of 5 test points with physical measurement

3. **Code Repository**
   - Servo control with named joint definitions
   - IK solver implementation
   - Standing sequence with smooth interpolation

4. **Technical Report**
   - Torque calculation for each joint with safety factors
   - CoM analysis showing stability margin
   - Documentation of any design deviations from plan

### Assessment Rubric

**Mechanical Construction (35%)**
- Symmetric bilateral assembly
- Robust joint connections
- Full range of motion without binding
- Professional cable management

**Theoretical Analysis (25%)**
- Correct IK derivation
- Accurate torque calculations
- Workspace analysis
- Stability verification

**Software Implementation (25%)**
- Working IK solver
- Smooth motion control
- Named constants for maintainability
- Comprehensive testing

**System Integration (15%)**
- Successful standing demonstration
- Balanced weight distribution
- Documented calibration values
- Troubleshooting log

---

## Extensions and Further Study

<div class="optional-exercise">

**Advanced Challenges**

1. **Optimization:** Minimize leg mass while maintaining torque requirements
2. **Dynamics:** Implement forward dynamics simulation (predict motion from torques)
3. **Compliance:** Add series elastic actuators for impact absorption
4. **Alternative IK:** Implement Jacobian-based numerical IK for full 6-DOF
5. **Gait Preview:** Generate simple walking trajectory (preparation for HUM-107)

</div>

### Recommended Reading

- **Textbook:** "Introduction to Humanoid Robotics" by Kajita et al., Chapters 2-3
- **Paper:** Vukobratović & Borovac (2004), "Zero-Moment Point - Thirty Five Years of its Life"
- **Reference:** "Robot Modeling and Control" by Spong et al., Chapter 3 (Kinematics)

---

## Connection to HUM-104

With bipedal legs complete, the next course adds upper-body manipulation capability through arm integration. You will design 5-DOF arms and develop coordinated whole-body control where arm motion compensates for dynamic effects on balance.

---

*Bipedal legs transform static structure into dynamic platform. Proceed to HUM-104 to add manipulation capability.*
