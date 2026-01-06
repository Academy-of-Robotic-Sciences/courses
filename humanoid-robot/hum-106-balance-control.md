---
id: hum-106-balance-control
title: "HUM-106: Balance Control and Stability"
sidebar_position: 7
version: 2.0.0
link: https://raw.githubusercontent.com/Academy-of-Robotic-Sciences/courses/main/course-styles.css
---

# HUM-106: Balance Control and Stability

> Implementing closed-loop balance controllers that enable the humanoid to stand, resist disturbances, and maintain stability using ZMP theory and hierarchical control strategies.

## Course Overview

| Parameter | Details |
|---|---|
| **Course Code** | HUM-106 |
| **Duration** | 8 contact hours |
| **Format** | Theory (40%) + Guided Project (60%) |
| **Prerequisites** | HUM-105 completed, control theory fundamentals |
| **Deliverables** | Self-balancing humanoid, disturbance rejection demonstration |

---

## Learning Objectives

### Theoretical Knowledge
- Derive the Zero Moment Point (ZMP) stability criterion
- Analyze ankle and hip balance strategies mathematically
- Design PID and LQR controllers for bipedal stability
- Understand the Linear Inverted Pendulum Model (LIPM)

### Practical Skills
- Implement real-time balance controllers at 100 Hz
- Tune multi-loop PID controllers systematically
- Deploy hierarchical control strategies (ankle → hip → stepping)
- Characterize stability regions experimentally

### Project Integration
- Transform the humanoid from statically stable to dynamically balanced
- Create the control foundation for walking (HUM-107)
- Develop emergency stop and fall detection systems

---

## Part 1: Theoretical Foundations (190 minutes)

### The Zero Moment Point (ZMP) Criterion

<div class="theory-concept">

**Zero Moment Point Definition**

The **Zero Moment Point** is the point on the ground where the net moment of inertial and gravitational forces acting on the robot equals zero.

For a robot to remain balanced:
$$\text{ZMP must lie within the support polygon}$$

The **support polygon** is the convex hull of all ground contact points.

</div>

#### Mathematical Derivation of ZMP

Consider a humanoid with center of mass (CoM) at position $\mathbf{p}_{CoM} = [x_{CoM}, y_{CoM}, z_{CoM}]^T$ and acceleration $\ddot{\mathbf{p}}_{CoM}$.

The ground reaction force at the ZMP creates moments that balance inertial effects:

$$\boldsymbol{\tau}_{ground} + \boldsymbol{\tau}_{inertia} + \boldsymbol{\tau}_{gravity} = 0$$

In the sagittal plane (forward-backward):

$$x_{ZMP} = x_{CoM} - \frac{z_{CoM} \ddot{x}_{CoM}}{g + \ddot{z}_{CoM}}$$

For a humanoid of height $h$ with constant CoM height ($\ddot{z}_{CoM} \approx 0$):

$$x_{ZMP} = x_{CoM} - \frac{h \ddot{x}_{CoM}}{g}$$

**Physical interpretation:** Forward acceleration ($\ddot{x}_{CoM} > 0$) shifts ZMP **backward** (toward the heel). The robot must ensure ZMP remains within the foot.

<div class="historical-note">

**Historical Development of Bipedal Balance Theory**

- **1969:** Miomir Vukobratović introduces the ZMP concept at the Mihailo Pupin Institute
- **1972:** ZMP criterion published, becomes foundation of humanoid robot control
- **1973:** WABOT-1 at Waseda University demonstrates quasi-static walking using ZMP
- **1996:** Honda P2 achieves dynamic walking with real-time ZMP control
- **2000:** ASIMO refines ZMP walking to near-human smoothness
- **2010s:** ZMP extended with angular momentum for more dynamic behaviors

ZMP remains the dominant stability criterion for humanoid robots due to its computational simplicity and real-time applicability.

</div>

### Linear Inverted Pendulum Model (LIPM)

Approximate the humanoid as an inverted pendulum with point mass $m$ at height $h$:

**Equation of motion** in the sagittal plane:

$$\ddot{x}_{CoM} = \frac{g}{h}(x_{CoM} - x_{ZMP})$$

This is a **second-order unstable system** with characteristic equation:

$$s^2 - \frac{g}{h} = 0 \implies s = \pm \sqrt{g/h}$$

The positive real root indicates exponential divergence without control.

**Control objective:** Manipulate $x_{ZMP}$ (via ankle/hip torques) to stabilize $x_{CoM}$.

#### State-Space Representation

Define state vector: $\mathbf{x} = [x_{CoM}, \dot{x}_{CoM}]^T$

Input: $u = x_{ZMP}$

$$\dot{\mathbf{x}} = \begin{bmatrix} 0 & 1 \\ g/h & 0 \end{bmatrix}\mathbf{x} + \begin{bmatrix} 0 \\ -g/h \end{bmatrix}u$$

For a humanoid with $h = 0.5$ m:

$$\mathbf{A} = \begin{bmatrix} 0 & 1 \\ 19.62 & 0 \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} 0 \\ -19.62 \end{bmatrix}$$

This system is **controllable** (can verify via controllability matrix), so stabilizing feedback exists.

### Hierarchical Balance Strategies

<div class="theory-concept">

**Three-Level Balance Hierarchy**

1. **Ankle Strategy:** Rotate ankles to shift ZMP (small disturbances, <5°)
2. **Hip Strategy:** Bend hips/knees to shift CoM (medium disturbances, 5-15°)
3. **Stepping Strategy:** Take a step to reposition support polygon (large disturbances, >15°)

</div>

#### Ankle Strategy

**Principle:** Keep body rigid, apply torque at ankles to rotate entire body.

Ankle torque: $\tau_{ankle} = I \ddot{\theta}$

Where $I$ is moment of inertia about ankle, $\theta$ is body tilt angle.

For small angles: $\theta \approx x_{CoM}/h$

$$\tau_{ankle} \approx \frac{I}{h}\ddot{x}_{CoM}$$

**Advantages:**
- Minimal motion, energy efficient
- Fast response
- Intuitive implementation

**Limitations:**
- Limited torque authority (~20 N·m for hobby servos)
- Ineffective for large disturbances
- Requires stiff ankle joints

#### Hip Strategy

**Principle:** Counter body tilt by moving hips opposite to tilt direction, shifting CoM.

Hip motion creates counteracting moment:

$$\Delta x_{CoM} = \frac{m_{upper} \Delta x_{hip}}{m_{total}}$$

Where $m_{upper}$ is upper body mass, $\Delta x_{hip}$ is hip displacement.

**Advantages:**
- Can handle larger disturbances
- Does not require large ankle torques
- More human-like response

**Limitations:**
- Slower than ankle strategy (larger inertias)
- Requires coordinated hip-knee motion
- Can destabilize if not coordinated properly

### PID Control for Balance

For angle regulation, the PID control law:

$$\tau(t) = K_p e(t) + K_i \int_0^t e(\tau)d\tau + K_d \frac{de(t)}{dt}$$

Where $e(t) = \theta_{desired} - \theta_{measured}$.

**Component roles:**
- **$K_p$:** Proportional to error; provides restoring force
- **$K_i$:** Eliminates steady-state error from gravity/bias
- **$K_d$:** Derivative (angular velocity); provides damping

**Discrete-time implementation:**

$$u[k] = K_p e[k] + K_i \sum_{j=0}^k e[j]\Delta t + K_d \frac{e[k] - e[k-1]}{\Delta t}$$

**Anti-windup:** Limit integrator to prevent saturation during large errors:

```cpp
integral += error * dt;
if (integral > integral_max) integral = integral_max;
if (integral < -integral_max) integral = -integral_max;
```

### LQR Control for Optimal Balance

For advanced students, **Linear Quadratic Regulator (LQR)** provides optimal gains.

**Cost function:**

$$J = \int_0^\infty (\mathbf{x}^T \mathbf{Q} \mathbf{x} + u^T \mathbf{R} u) dt$$

Where:
- $\mathbf{Q}$: State cost matrix (penalizes position and velocity errors)
- $\mathbf{R}$: Control cost matrix (penalizes control effort)

Solve the **Algebraic Riccati Equation (ARE)**:

$$\mathbf{A}^T \mathbf{P} + \mathbf{P}\mathbf{A} - \mathbf{P}\mathbf{B}\mathbf{R}^{-1}\mathbf{B}^T\mathbf{P} + \mathbf{Q} = 0$$

Optimal feedback gain:

$$\mathbf{K} = \mathbf{R}^{-1}\mathbf{B}^T\mathbf{P}$$

Control law: $u = -\mathbf{K}\mathbf{x}$

**Example for LIPM:**

```python
import numpy as np
from scipy.linalg import solve_continuous_are

# System matrices
A = np.array([[0, 1], [19.62, 0]])
B = np.array([[0], [-19.62]])

# Cost matrices
Q = np.diag([10, 1])   # Penalize position more than velocity
R = np.array([[0.1]])  # Moderate control cost

# Solve Riccati equation
P = solve_continuous_are(A, B, Q, R)

# Compute optimal gain
K = np.linalg.inv(R) @ B.T @ P
print(f"Optimal gains: K = {K}")
# K ≈ [31.6, 7.1] for typical values
```

### Knowledge Check Quiz

1. **Question:** Why does the ZMP shift backward during forward acceleration?
   - (a) Ground friction opposes motion
   - (b) Inertial force creates a backward moment
   - (c) Center of mass moves forward
   - (d) Ankle torque is insufficient

   **Answer:** (b) - Forward acceleration creates inertia $ma$ acting backward at CoM, creating moment about ZMP

2. **Question:** What is the natural frequency of an inverted pendulum with $h = 0.5$ m?
   - (a) 2.23 rad/s
   - (b) 4.43 rad/s
   - (c) 19.62 rad/s
   - (d) The system is unstable, no natural frequency

   **Answer:** (d) - Positive eigenvalue $\sqrt{g/h} = 4.43$ indicates instability, not oscillation

3. **Question:** In PID control, increasing $K_d$ primarily:
   - (a) Reduces steady-state error
   - (b) Increases response speed
   - (c) Adds damping to reduce overshoot
   - (d) Eliminates disturbances

   **Answer:** (c) - Derivative term acts as virtual damping, opposing velocity

---

## Part 2: Guided Project - Balance Controller Implementation (290 minutes)

### Phase 1: Ankle Strategy Controller (70 minutes)

#### PID Controller Class

```cpp
class PIDController {
private:
  float kp, ki, kd;
  float integral;
  float previous_error;
  float dt;
  float integral_limit;

public:
  PIDController(float p, float i, float d, float timestep, float int_limit = 100.0)
    : kp(p), ki(i), kd(d), dt(timestep), integral(0), previous_error(0), integral_limit(int_limit) {}

  float compute(float setpoint, float measured_value) {
    float error = setpoint - measured_value;

    // Proportional term
    float P = kp * error;

    // Integral term with anti-windup
    integral += error * dt;
    if (integral > integral_limit) integral = integral_limit;
    if (integral < -integral_limit) integral = -integral_limit;
    float I = ki * integral;

    // Derivative term (with filtering to reduce noise)
    float derivative = (error - previous_error) / dt;
    float D = kd * derivative;

    previous_error = error;

    return P + I + D;
  }

  void reset() {
    integral = 0;
    previous_error = 0;
  }

  void setGains(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
  }

  void getGains(float &p, float &i, float &d) {
    p = kp;
    i = ki;
    d = kd;
  }
};
```

#### Ankle Balance Implementation

```cpp
class AnkleBalanceController {
private:
  PIDController pitch_pid;
  PIDController roll_pid;

  // Servo limits (degrees)
  const float ANKLE_PITCH_MIN = 60;
  const float ANKLE_PITCH_MAX = 120;
  const float ANKLE_ROLL_MIN = 60;
  const float ANKLE_ROLL_MAX = 120;

  // Balance thresholds
  const float BALANCE_THRESHOLD = 5.0;  // degrees

public:
  AnkleBalanceController()
    : pitch_pid(2.5, 0.1, 0.8, 0.01),  // Initial PID gains
      roll_pid(2.0, 0.1, 0.6, 0.01) {}

  void update(float pitch_angle, float roll_angle) {
    // Target is upright (0 degrees)
    float pitch_correction = pitch_pid.compute(0.0, pitch_angle);
    float roll_correction = roll_pid.compute(0.0, roll_angle);

    // Convert to servo angles (90 = neutral)
    float left_ankle_pitch = 90 + pitch_correction;
    float right_ankle_pitch = 90 + pitch_correction;
    float left_ankle_roll = 90 - roll_correction;
    float right_ankle_roll = 90 + roll_correction;

    // Apply limits
    left_ankle_pitch = constrain(left_ankle_pitch, ANKLE_PITCH_MIN, ANKLE_PITCH_MAX);
    right_ankle_pitch = constrain(right_ankle_pitch, ANKLE_PITCH_MIN, ANKLE_PITCH_MAX);
    left_ankle_roll = constrain(left_ankle_roll, ANKLE_ROLL_MIN, ANKLE_ROLL_MAX);
    right_ankle_roll = constrain(right_ankle_roll, ANKLE_ROLL_MIN, ANKLE_ROLL_MAX);

    // Send to servos
    setServo(LEFT_ANKLE_PITCH, left_ankle_pitch);
    setServo(RIGHT_ANKLE_PITCH, right_ankle_pitch);
    setServo(LEFT_ANKLE_ROLL, left_ankle_roll);
    setServo(RIGHT_ANKLE_ROLL, right_ankle_roll);
  }

  bool isBalanced(float pitch, float roll) {
    return (abs(pitch) < BALANCE_THRESHOLD && abs(roll) < BALANCE_THRESHOLD);
  }

  void tunePID(float kp_pitch, float ki_pitch, float kd_pitch,
               float kp_roll, float ki_roll, float kd_roll) {
    pitch_pid.setGains(kp_pitch, ki_pitch, kd_pitch);
    roll_pid.setGains(kp_roll, ki_roll, kd_roll);
  }
};
```

#### Ziegler-Nichols Tuning Implementation

```cpp
class PIDTuner {
private:
  PIDController *controller;
  float test_duration;  // seconds

public:
  PIDTuner(PIDController *ctrl) : controller(ctrl), test_duration(10.0) {}

  void autoTune() {
    Serial.println("Starting Ziegler-Nichols auto-tuning...");

    // Step 1: Set I and D to zero
    controller->setGains(0.1, 0, 0);

    float kp_critical = 0;
    float period_critical = 0;
    bool oscillating = false;

    // Step 2: Increase Kp until sustained oscillation
    for (float kp = 0.1; kp < 20.0; kp += 0.1) {
      controller->setGains(kp, 0, 0);

      // Test stability
      if (testForOscillation(period_critical)) {
        kp_critical = kp;
        oscillating = true;
        Serial.print("Critical gain found: ");
        Serial.println(kp_critical);
        Serial.print("Critical period: ");
        Serial.println(period_critical);
        break;
      }
    }

    if (!oscillating) {
      Serial.println("Failed to find critical gain!");
      return;
    }

    // Step 3: Apply Ziegler-Nichols formulas
    float kp = 0.6 * kp_critical;
    float ki = 2.0 * kp / period_critical;
    float kd = kp * period_critical / 8.0;

    controller->setGains(kp, ki, kd);

    Serial.println("Ziegler-Nichols tuning complete:");
    Serial.print("Kp = "); Serial.println(kp);
    Serial.print("Ki = "); Serial.println(ki);
    Serial.print("Kd = "); Serial.println(kd);
  }

private:
  bool testForOscillation(float &period) {
    // Collect data for analysis
    const int samples = 500;
    float angles[samples];
    unsigned long start_time = millis();

    for (int i = 0; i < samples; i++) {
      angles[i] = readOrientation();
      delay(20);  // 50 Hz sampling
    }

    // Detect oscillation by counting zero crossings
    int zero_crossings = 0;
    unsigned long first_crossing = 0;
    unsigned long last_crossing = 0;

    for (int i = 1; i < samples; i++) {
      if ((angles[i-1] < 0 && angles[i] >= 0) ||
          (angles[i-1] > 0 && angles[i] <= 0)) {
        zero_crossings++;
        if (zero_crossings == 1) first_crossing = i * 20;
        last_crossing = i * 20;
      }
    }

    // Need at least 4 zero crossings (2 complete cycles)
    if (zero_crossings >= 4) {
      period = (last_crossing - first_crossing) / ((zero_crossings - 1) / 2.0) / 1000.0;
      return true;
    }

    return false;
  }
};
```

<div class="exercise-tip">

**Manual PID Tuning Guidelines**

If automatic tuning fails, use this systematic manual approach:

1. **Start with all gains at zero**
2. **Increase $K_p$** until the robot oscillates steadily (critical gain)
3. **Reduce $K_p$** to 60% of critical value
4. **Add $K_d$** to dampen oscillations (start with $K_p/3$)
5. **Fine-tune $K_d$** to minimize overshoot
6. **Add $K_i$** to eliminate steady-state error (start small, $K_p/10$)
7. **Adjust $K_i$** to balance error elimination vs. stability

| Symptom | Action |
|---------|--------|
| Continuous oscillation | Reduce $K_p$, increase $K_d$ |
| Slow response | Increase $K_p$ |
| Overshoots setpoint | Increase $K_d$ |
| Steady-state error | Increase $K_i$ |
| Unstable with $K_i$ | Reduce $K_i$, add integral limit |

</div>

### Phase 2: Hip Strategy Controller (60 minutes)

```cpp
class HipBalanceController {
private:
  PIDController hip_pitch_pid;
  PIDController hip_roll_pid;
  PIDController knee_pid;

  const float HIP_ACTIVATION_THRESHOLD = 5.0;  // degrees
  float hip_engagement;  // 0.0 to 1.0

public:
  HipBalanceController()
    : hip_pitch_pid(1.5, 0.05, 0.4, 0.01),
      hip_roll_pid(1.2, 0.05, 0.3, 0.01),
      knee_pid(1.0, 0.02, 0.2, 0.01),
      hip_engagement(0.0) {}

  void update(float pitch_angle, float roll_angle) {
    // Calculate disturbance magnitude
    float disturbance = sqrt(pitch_angle*pitch_angle + roll_angle*roll_angle);

    // Smooth transition from ankle to hip strategy
    if (disturbance > HIP_ACTIVATION_THRESHOLD) {
      hip_engagement += 0.05;  // Ramp up
    } else {
      hip_engagement -= 0.02;  // Ramp down
    }
    hip_engagement = constrain(hip_engagement, 0.0, 1.0);

    if (hip_engagement > 0.01) {
      // Hip strategy: move hips opposite to tilt
      float hip_pitch_correction = hip_pitch_pid.compute(0.0, pitch_angle);
      float hip_roll_correction = hip_roll_pid.compute(0.0, roll_angle);

      // Knee compensation (bend knees when hips move)
      float knee_compensation = knee_pid.compute(0.0, abs(hip_pitch_correction));

      // Apply with engagement factor
      float left_hip_pitch = 90 - hip_pitch_correction * hip_engagement;
      float right_hip_pitch = 90 - hip_pitch_correction * hip_engagement;
      float left_hip_roll = 90 + hip_roll_correction * hip_engagement;
      float right_hip_roll = 90 - hip_roll_correction * hip_engagement;
      float left_knee = 90 + knee_compensation * hip_engagement;
      float right_knee = 90 + knee_compensation * hip_engagement;

      // Send to servos
      setServo(LEFT_HIP_PITCH, left_hip_pitch);
      setServo(RIGHT_HIP_PITCH, right_hip_pitch);
      setServo(LEFT_HIP_ROLL, left_hip_roll);
      setServo(RIGHT_HIP_ROLL, right_hip_roll);
      setServo(LEFT_KNEE, left_knee);
      setServo(RIGHT_KNEE, right_knee);
    }
  }

  float getEngagement() { return hip_engagement; }
};
```

### Phase 3: Hierarchical Controller Integration (50 minutes)

```cpp
class HierarchicalBalanceController {
private:
  AnkleBalanceController ankle_controller;
  HipBalanceController hip_controller;

  enum BalanceStrategy {
    ANKLE_ONLY,
    ANKLE_HIP_BLEND,
    HIP_DOMINANT,
    STEPPING_REQUIRED
  };

  BalanceStrategy current_strategy;

public:
  HierarchicalBalanceController() : current_strategy(ANKLE_ONLY) {}

  void update(float pitch, float roll, FootForces left_foot, FootForces right_foot) {
    // Determine appropriate strategy
    float disturbance = sqrt(pitch*pitch + roll*roll);

    if (disturbance < 5.0) {
      current_strategy = ANKLE_ONLY;
    } else if (disturbance < 15.0) {
      current_strategy = ANKLE_HIP_BLEND;
    } else if (disturbance < 25.0) {
      current_strategy = HIP_DOMINANT;
    } else {
      current_strategy = STEPPING_REQUIRED;
    }

    // Execute strategy
    switch (current_strategy) {
      case ANKLE_ONLY:
        ankle_controller.update(pitch, roll);
        break;

      case ANKLE_HIP_BLEND:
        ankle_controller.update(pitch * 0.7, roll * 0.7);
        hip_controller.update(pitch, roll);
        break;

      case HIP_DOMINANT:
        ankle_controller.update(pitch * 0.3, roll * 0.3);
        hip_controller.update(pitch, roll);
        break;

      case STEPPING_REQUIRED:
        Serial.println("WARNING: Stepping required!");
        // In HUM-107, this will trigger a step
        emergencyStabilize();
        break;
    }

    // Log telemetry
    logBalanceData(pitch, roll, current_strategy);
  }

private:
  void emergencyStabilize() {
    // Crouch to lower CoM
    for (int angle = 90; angle <= 120; angle += 2) {
      setServo(LEFT_KNEE, angle);
      setServo(RIGHT_KNEE, angle);
      delay(20);
    }
  }
};
```

### Phase 4: ZMP Monitoring and Visualization (40 minutes)

```cpp
struct ZMPState {
  float x_zmp, y_zmp;           // ZMP position (mm)
  float x_com, y_com, z_com;    // CoM position (mm)
  bool stable;
  float stability_margin;       // Distance to polygon edge (mm)
};

ZMPState computeZMP(float pitch, float roll, FootForces left_foot, FootForces right_foot) {
  ZMPState state;

  // Estimate CoM position (simplified model)
  state.x_com = 0;  // Assume centered initially
  state.y_com = 0;
  state.z_com = 500;  // 50cm height

  // Estimate CoM acceleration from IMU
  // pitch angle ≈ x_com / z_com for small angles
  float pitch_rad = pitch * PI / 180.0;
  float roll_rad = roll * PI / 180.0;

  float ddx_com = (g / state.z_com) * state.x_com;  // From LIPM
  float ddy_com = (g / state.z_com) * state.y_com;

  // Calculate ZMP
  state.x_zmp = state.x_com - (state.z_com * ddx_com) / g;
  state.y_zmp = state.y_com - (state.z_com * ddy_com) / g;

  // Determine support polygon (simplified: single foot)
  float polygon_x_min = -30, polygon_x_max = 30;  // mm
  float polygon_y_min = -40, polygon_y_max = 40;  // mm

  if (left_foot.total > 1.0 && right_foot.total > 1.0) {
    // Double support: expand polygon
    polygon_y_min = -100;  // Wider lateral stability
    polygon_y_max = 100;
  }

  // Check stability
  state.stable = (state.x_zmp >= polygon_x_min && state.x_zmp <= polygon_x_max &&
                  state.y_zmp >= polygon_y_min && state.y_zmp <= polygon_y_max);

  // Calculate margin
  float margin_x = min(state.x_zmp - polygon_x_min, polygon_x_max - state.x_zmp);
  float margin_y = min(state.y_zmp - polygon_y_min, polygon_y_max - state.y_zmp);
  state.stability_margin = min(margin_x, margin_y);

  return state;
}
```

**Python Visualization:**

```python
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

def update(frame):
    # Read data from serial
    data = read_serial_data()  # pitch, roll, zmp_x, zmp_y, cop_x, cop_y

    # Left plot: Orientation
    ax1.clear()
    ax1.set_xlim([-30, 30])
    ax1.set_ylim([-30, 30])
    ax1.set_xlabel('Roll (degrees)')
    ax1.set_ylabel('Pitch (degrees)')
    ax1.set_title('Robot Orientation')
    ax1.grid(True)

    # Draw stable region
    stable_region = patches.Circle((0, 0), 5, fill=False, edgecolor='green', linestyle='--')
    ax1.add_patch(stable_region)

    # Plot current orientation
    ax1.scatter([data['roll']], [data['pitch']], s=200, c='blue', marker='o')

    # Right plot: ZMP and CoP
    ax2.clear()
    ax2.set_xlim([-50, 50])
    ax2.set_ylim([-60, 60])
    ax2.set_xlabel('X (mm)')
    ax2.set_ylabel('Y (mm)')
    ax2.set_title('ZMP and Center of Pressure')
    ax2.grid(True)

    # Draw support polygon (foot outline)
    foot = patches.Rectangle((-30, -40), 60, 80, fill=False, edgecolor='black', linewidth=2)
    ax2.add_patch(foot)

    # Plot ZMP
    ax2.scatter([data['zmp_x']], [data['zmp_y']], s=200, c='red', marker='x', label='ZMP')

    # Plot CoP
    ax2.scatter([data['cop_x']], [data['cop_y']], s=200, c='green', marker='+', label='CoP')

    ax2.legend()

ani = FuncAnimation(fig, update, interval=50)
plt.tight_layout()
plt.show()
```

### Phase 5: Disturbance Rejection Testing (70 minutes)

**Systematic Testing Protocol:**

```cpp
class DisturbanceTest {
private:
  struct TestResult {
    float disturbance_magnitude;
    float max_angle;
    float recovery_time;
    bool recovered;
  };

  std::vector<TestResult> results;

public:
  void runPushTest(float push_force, String direction) {
    Serial.print("Push test: ");
    Serial.print(push_force);
    Serial.print(" N ");
    Serial.println(direction);

    TestResult result;
    result.disturbance_magnitude = push_force;

    // Wait for user to apply push
    Serial.println("Apply push in 3 seconds...");
    delay(3000);

    // Monitor recovery
    unsigned long start_time = millis();
    float max_deviation = 0;

    while (millis() - start_time < 10000) {  // 10 second window
      float pitch = readPitch();
      float roll = readRoll();
      float deviation = sqrt(pitch*pitch + roll*roll);

      if (deviation > max_deviation) {
        max_deviation = deviation;
      }

      // Check if recovered
      if (millis() - start_time > 2000 && deviation < 2.0) {
        result.recovered = true;
        result.recovery_time = (millis() - start_time) / 1000.0;
        result.max_angle = max_deviation;
        break;
      }

      delay(10);
    }

    if (!result.recovered) {
      result.recovered = false;
      result.recovery_time = -1;
      result.max_angle = max_deviation;
      Serial.println("FAILED to recover!");
    } else {
      Serial.print("Recovered in ");
      Serial.print(result.recovery_time);
      Serial.println(" seconds");
    }

    results.push_back(result);
  }

  void printSummary() {
    Serial.println("\n=== DISTURBANCE TEST SUMMARY ===");
    Serial.println("Force\tMax Angle\tRecovery Time\tStatus");
    for (auto &r : results) {
      Serial.print(r.disturbance_magnitude);
      Serial.print("\t");
      Serial.print(r.max_angle);
      Serial.print("\t\t");
      if (r.recovered) {
        Serial.print(r.recovery_time);
        Serial.println("\t\tPASS");
      } else {
        Serial.println("N/A\t\tFAIL");
      }
    }
  }
};
```

**Performance Metrics:**

Target specifications:
- **Static balance:** Stand for 5+ minutes without falling
- **Disturbance rejection:** Recover from 3 N push within 2 seconds
- **Orientation accuracy:** Maintain ±2° of vertical
- **Response time:** Begin corrective action within 100 ms
- **Stability margin:** Keep ZMP >10 mm from polygon edge

---

## Part 3: Analysis and Optimization (30 minutes)

### LQR Implementation (Optional Advanced)

```python
import numpy as np
from scipy.linalg import solve_continuous_are

class LQRBalanceController:
    def __init__(self, height=0.5):
        # LIPM system matrices
        g = 9.81
        self.A = np.array([[0, 1], [g/height, 0]])
        self.B = np.array([[0], [-g/height]])

        # Cost matrices (tune these)
        Q = np.diag([100, 1])    # Penalize position heavily
        R = np.array([[1]])      # Control cost

        # Solve ARE
        P = solve_continuous_are(self.A, self.B, Q, R)

        # Optimal gain
        self.K = np.linalg.inv(R) @ self.B.T @ P

        print(f"LQR gains: K = {self.K[0]}")

    def compute_control(self, x_com, dx_com):
        # State vector
        x = np.array([[x_com], [dx_com]])

        # Optimal control
        u = -self.K @ x

        return u[0, 0]  # Desired ZMP position
```

### Knowledge Check Quiz

4. **Question:** In hierarchical balance control, why use ankle strategy before hip strategy?
   - (a) Ankle servos are stronger
   - (b) Hip strategy is more complex to implement
   - (c) Ankle strategy is faster and more energy-efficient
   - (d) Hip strategy can cause falls

   **Answer:** (c) - Ankle strategy has lower inertia, faster response, and minimal energy cost

5. **Question:** What does a negative stability margin indicate?
   - (a) The robot is very stable
   - (b) ZMP is outside the support polygon (unstable)
   - (c) Sensors are miscalibrated
   - (d) Control gains are too high

   **Answer:** (b) - Negative margin means ZMP has exited the stable region

---

## Deliverables and Assessment

### Required Submissions

1. **Functional Demonstration**
   - Video: Robot standing for 2+ minutes
   - Video: Recovery from manual pushes (4 directions)
   - Video: Smooth transition between ankle and hip strategies

2. **Technical Analysis**
   - Documented PID gains with tuning methodology
   - ZMP trajectory plots during disturbance tests
   - Comparison of ankle vs. hip strategy performance

3. **Code Repository**
   - Complete balance controller implementation
   - Calibration and testing scripts
   - Real-time visualization tools

4. **Performance Report**
   - Quantitative metrics (recovery time, max stable angle, etc.)
   - Discussion of controller limitations
   - Proposed improvements for walking

### Assessment Rubric

**Functional Performance (40%)**
- Stands unsupported for 5 minutes: Excellent
- Recovers from 3 N push within 3 seconds: Required
- Smooth ankle-hip strategy transition: Excellent
- No falls during testing: Required

**Theoretical Understanding (30%)**
- Correct ZMP derivation: Full marks
- Clear explanation of LIPM dynamics: Full marks
- Justification of PID tuning approach: Full marks

**Implementation Quality (20%)**
- Clean hierarchical controller architecture: Professional
- Real-time performance (100 Hz): Required
- Robust error handling: Required

**Documentation (10%)**
- Comprehensive testing data: Required
- Clear performance analysis: Full marks

---

## Extensions and Further Study

<div class="optional-exercise">

**Advanced Topics**

1. **Adaptive Control:** Dynamically adjust PID gains based on disturbance level
2. **Whole-Body ZMP Control:** Incorporate arm motion into ZMP calculation
3. **Model Predictive Control (MPC):** Preview-based balance for walking
4. **Terrain Estimation:** Adapt balance strategy to slopes and uneven ground
5. **Learning-Based Tuning:** Use reinforcement learning to optimize controller parameters

</div>

### Recommended Reading

- **Textbook:** "Introduction to Humanoid Robotics" by Kajita et al., Chapter 4
- **Paper:** Vukobratović and Borovac (2004), "Zero-Moment Point—Thirty Five Years of its Life"
- **Tutorial:** "Bipedal Walking Control" by Pratt and Tedrake (MIT)
- **Video:** Boston Dynamics Atlas balance demonstrations

---

## Connection to HUM-107

Your balance controller is the **foundation** for walking. In HUM-107:

- **Static walking:** Use ZMP control to shift weight safely
- **Dynamic walking:** Maintain ZMP within moving support polygon
- **Gait generation:** Coordinate balance with leg swing trajectories

The hierarchical structure extends naturally:
- **Ankle strategy** → Continuous balance during stance phase
- **Hip strategy** → Weight shifting between steps
- **Stepping strategy** → Foot placement for next step

Your robot is now stable enough to attempt locomotion.

---

*From unstable oscillations to robust balance—your humanoid stands strong. Proceed to HUM-107 to make it walk.*
