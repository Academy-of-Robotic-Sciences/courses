---
id: hum-101-foundations
title: "HUM-101: Foundations of Humanoid Balance and Control"
sidebar_position: 2
version: 2.0.0
link: https://raw.githubusercontent.com/Academy-of-Robotic-Sciences/courses/main/course-styles.css
---

# HUM-101: Foundations of Humanoid Balance and Control

> An introduction to the fundamental challenges of bipedal robotics through theoretical analysis and practical construction of an inverted pendulum balancing system.

## Course Overview

| Parameter | Details |
|---|---|
| **Course Code** | HUM-101 |
| **Duration** | 4 contact hours |
| **Format** | Theory (40%) + Guided Project (60%) |
| **Prerequisites** | RC-101 to RC-106, linear algebra, basic dynamics |
| **Deliverables** | Working inverted pendulum balancer, theoretical analysis |

---

## Learning Objectives

### Theoretical Knowledge
- Understand the mathematical model of the inverted pendulum
- Analyze stability conditions for bipedal systems
- Derive PID control equations for balance
- Explain the relationship between simplified models and humanoid robots

### Practical Skills
- Construct a single degree-of-freedom balancing mechanism
- Implement digital control loops with real-time constraints
- Tune PID controllers using systematic methodologies
- Analyze system performance through experimental data

### Project Integration
- Build the foundational subsystem for later integration
- Establish control patterns applicable to full humanoid balance
- Develop intuition for dynamic stability

---

## Part 1: Theoretical Foundations (90 minutes)

### The Inverted Pendulum Problem

<div class="theory-concept">

**Why Study the Inverted Pendulum?**

The inverted pendulum represents the simplest nontrivial model of bipedal balance. A humanoid robot in stance phase behaves as a multi-link inverted pendulum—inherently unstable without active control. Mastering single-link dynamics provides the foundation for more complex humanoid systems.

</div>

#### Mathematical Model

Consider a pendulum of length $L$ and mass $m$ pivoting about a fixed point. The equation of motion is:

$$\ddot{\theta} = \frac{g}{L}\sin\theta - \frac{b}{mL^2}\dot{\theta} + \frac{\tau}{mL^2}$$

Where:
- $\theta$: Angular displacement from vertical
- $g$: Gravitational acceleration (9.81 m/s²)
- $b$: Damping coefficient
- $\tau$: Applied control torque

<div class="theory-concept">

**Linearization for Small Angles**

For $|\theta| < 15°$, we can use the small-angle approximation $\sin\theta \approx \theta$:

$$\ddot{\theta} \approx \frac{g}{L}\theta + \frac{\tau}{mL^2}$$

This linear model simplifies analysis while remaining valid for practical balancing scenarios.

</div>

#### Stability Analysis

**Open-Loop Behavior:** Without control ($\tau = 0$), the characteristic equation is:

$$s^2 - \frac{g}{L} = 0$$

Roots: $s = \pm\sqrt{g/L}$

The positive real root indicates exponential divergence—the system is **unstable**.

**Closed-Loop Requirements:** To achieve stability, we must design $\tau$ to move all eigenvalues to the left half-plane.

### Historical Context

<div class="historical-note">

**Evolution of Bipedal Balance Control**

The inverted pendulum was first studied mathematically by Lagrange (1788) and later became a classic control problem. Key milestones in bipedal robotics:

- **1969:** Vukobratović introduces the Zero Moment Point (ZMP) criterion
- **1973:** WABOT-1 achieves quasi-static walking at Waseda University
- **1996:** Honda P2 demonstrates dynamic walking with active balance
- **2005:** Boston Dynamics' BigDog shows robust balance on rough terrain
- **2013:** Atlas robot performs running and jumping maneuvers

These advances built on increasingly sophisticated applications of inverted pendulum control theory.

</div>

### Control Strategies

#### Proportional-Integral-Derivative (PID) Control

The general PID control law:

$$\tau(t) = K_p e(t) + K_i \int_0^t e(\tau)d\tau + K_d \frac{de(t)}{dt}$$

Where $e(t) = \theta_{desired} - \theta(t)$ is the tracking error.

**Component Analysis:**
- **Proportional ($K_p$):** Reacts to current error; higher gain increases responsiveness but may cause overshoot
- **Integral ($K_i$):** Eliminates steady-state error by accumulating past errors
- **Derivative ($K_d$):** Provides damping by predicting future error trends

<div class="theory-concept">

**Discrete-Time Implementation**

Digital controllers operate at discrete time steps $\Delta t$:

$$u[k] = K_p e[k] + K_i \sum_{j=0}^k e[j]\Delta t + K_d \frac{e[k] - e[k-1]}{\Delta t}$$

For real-time systems, we must ensure:
1. Loop frequency ≥ 100 Hz for balance control
2. Computational latency < 10 ms
3. Sensor noise filtering without excessive phase lag

</div>

### Knowledge Check Quiz

1. **Question:** Why is the inverted pendulum unstable without control?
   - (a) Damping is too high
   - (b) It has a positive real eigenvalue
   - (c) Gravity acts downward
   - (d) The mass is too small

   **Answer:** (b) - The positive eigenvalue causes exponential divergence

2. **Question:** In PID control, which term eliminates steady-state error?
   - (a) Proportional
   - (b) Integral
   - (c) Derivative
   - (d) None—PID cannot eliminate steady-state error

   **Answer:** (b) - The integral term accumulates error until it drives steady-state to zero

3. **Question:** For a 30 cm pendulum, what is the approximate natural frequency?
   - (a) 1.8 rad/s
   - (b) 5.7 rad/s
   - (c) 32.7 rad/s
   - (d) 0.56 rad/s

   **Answer:** (b) - $\omega = \sqrt{g/L} = \sqrt{9.81/0.3} \approx 5.7$ rad/s

---

## Part 2: Guided Project - Inverted Pendulum Balancer (150 minutes)

### Design Phase (30 minutes)

#### System Architecture

You will construct one of two configurations:

**Option A: Reaction Wheel Pendulum**

Principle: Angular momentum exchange between pendulum and flywheel

```
Components:
- Pendulum arm (length L = 20-30 cm)
- DC motor with flywheel mounted at top
- IMU sensor (MPU6050) at center of mass
- Microcontroller (Arduino/Pico)
- Power supply (5V logic, 12V motor)
```

Dynamics:
$$J_{pendulum}\ddot{\theta} + mgL\sin\theta = -J_{wheel}\ddot{\phi}$$

Where $\phi$ is wheel angular velocity.

**Option B: Cart-Pole System**

Principle: Horizontal base acceleration to counteract pendulum tilt

```
Components:
- Vertical pendulum (length L = 30-40 cm)
- Linear rail (60 cm minimum)
- Motor-driven cart
- IMU at pendulum tip
- Position encoder for cart
```

<div class="alternative-approach">

**Alternative Sensors**

While an IMU provides both angle and angular velocity, alternatives include:
- **Rotary encoder:** Direct angle measurement, compute velocity via differentiation
- **Accelerometer-only:** Measure tilt via gravity vector projection
- **Gyroscope-only:** Integrate to obtain angle (subject to drift)

The MPU6050 combines accelerometer and gyroscope with hardware sensor fusion, providing optimal performance.

</div>

#### Mechanical Considerations

**Moment of Inertia:** For a uniform rod of length $L$ and mass $m$:

$$J = \frac{1}{3}mL^2$$

**Torque Requirements:** To accelerate the pendulum at $\alpha = 10$ rad/s²:

$$\tau_{required} = J\alpha = \frac{1}{3}mL^2\alpha$$

For $m = 0.1$ kg, $L = 0.25$ m: $\tau \approx 0.021$ N·m

Select motor and wheel inertia accordingly.

### Assembly Phase (45 minutes)

#### Hardware Assembly Procedure

1. **Mechanical Structure** (20 min)
   - Mount pivot bearing to base plate
   - Attach pendulum arm ensuring free rotation
   - Position motor/cart mechanism
   - Verify mechanical balance

2. **Sensor Integration** (15 min)
   - Mount IMU at center of mass
   - Ensure sensor axes align with rotation plane
   - Use vibration-damping material if necessary
   - Route sensor cable to avoid entanglement

3. **Electrical Connections** (10 min)
   - **IMU:** VCC → 3.3V, GND → GND, SDA → Pin A4, SCL → Pin A5
   - **Motor:** Via motor driver, connect to PWM output and 12V supply
   - **Power:** Separate 5V for logic, 12V for motor
   - **Common ground** for all components

<div class="exercise-tip">

**Preventing Noise Issues**

Motors generate electrical noise that can corrupt IMU readings. Best practices:
- Use separate power rails (5V logic, 12V motor)
- Add 100 µF capacitor across motor terminals
- Twist motor power wires together
- Keep IMU wires short and away from motor

</div>

### Software Implementation Phase (45 minutes)

#### Sensor Calibration

Before implementing control, calibrate the IMU:

```cpp
// IMU Calibration Routine
#include <Wire.h>
#include <MPU6050.h>

MPU6050 imu;

void calibrateIMU() {
  const int samples = 1000;
  float gyro_offset[3] = {0, 0, 0};

  Serial.println("Hold pendulum vertical and still...");
  delay(2000);

  for(int i = 0; i < samples; i++) {
    int16_t gx, gy, gz;
    imu.getRotation(&gx, &gy, &gz);

    gyro_offset[0] += gx;
    gyro_offset[1] += gy;
    gyro_offset[2] += gz;

    delay(3);
  }

  // Average offsets
  gyro_offset[0] /= samples;
  gyro_offset[1] /= samples;
  gyro_offset[2] /= samples;

  // Store offsets for use in main loop
  imu.setXGyroOffset(gyro_offset[0]);
  imu.setYGyroOffset(gyro_offset[1]);
  imu.setZGyroOffset(gyro_offset[2]);

  Serial.println("Calibration complete");
}
```

#### PID Controller Implementation

```cpp
class PIDController {
  private:
    float kp, ki, kd;
    float integral;
    float previous_error;
    float dt;

  public:
    PIDController(float p, float i, float d, float timestep)
      : kp(p), ki(i), kd(d), dt(timestep), integral(0), previous_error(0) {}

    float compute(float setpoint, float measured_value) {
      // Calculate error
      float error = setpoint - measured_value;

      // Proportional term
      float P = kp * error;

      // Integral term (with anti-windup)
      integral += error * dt;
      if(integral > 100) integral = 100;  // Limit integral
      if(integral < -100) integral = -100;
      float I = ki * integral;

      // Derivative term
      float derivative = (error - previous_error) / dt;
      float D = kd * derivative;

      // Update for next iteration
      previous_error = error;

      // Return control signal
      return P + I + D;
    }

    void reset() {
      integral = 0;
      previous_error = 0;
    }

    void setGains(float p, float i, float d) {
      kp = p; ki = i; kd = d;
    }
};
```

#### Main Control Loop

```cpp
#include <Wire.h>
#include <MPU6050.h>

MPU6050 imu;
PIDController pid(50.0, 0.5, 10.0, 0.01);  // Initial gains

const int MOTOR_PIN = 9;  // PWM pin for motor
const float DT = 0.01;    // 10ms loop time (100 Hz)

void setup() {
  Serial.begin(115200);
  Wire.begin();

  imu.initialize();
  if(!imu.testConnection()) {
    Serial.println("IMU connection failed!");
    while(1);
  }

  calibrateIMU();
  pinMode(MOTOR_PIN, OUTPUT);
}

void loop() {
  unsigned long start_time = millis();

  // Read IMU
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Complementary filter for angle estimation
  float acc_angle = atan2(ax, az) * 180 / PI;
  float gyro_rate = gy / 131.0;  // Convert to deg/s

  static float angle = 0;
  angle = 0.98 * (angle + gyro_rate * DT) + 0.02 * acc_angle;

  // PID control
  float target_angle = 0.0;  // Upright position
  float motor_command = pid.compute(target_angle, angle);

  // Limit motor command
  motor_command = constrain(motor_command, -255, 255);

  // Apply to motor
  if(motor_command > 0) {
    analogWrite(MOTOR_PIN, motor_command);
  } else {
    analogWrite(MOTOR_PIN, 0);
    // Direction control if using H-bridge
  }

  // Data logging
  Serial.print(angle);
  Serial.print(",");
  Serial.println(motor_command);

  // Maintain loop timing
  while(millis() - start_time < DT * 1000);
}
```

<div class="exercise-advanced">

**Advanced Implementation: State-Space Control**

For students familiar with linear systems, implement an LQR (Linear Quadratic Regulator) controller:

State vector: $\mathbf{x} = [\theta, \dot{\theta}]^T$

System matrices:
$$\mathbf{A} = \begin{bmatrix} 0 & 1 \\ g/L & 0 \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} 0 \\ 1/(mL^2) \end{bmatrix}$$

Solve the Riccati equation for optimal gains $\mathbf{K}$, then apply:
$$u = -\mathbf{K}\mathbf{x}$$

</div>

### Tuning Phase (30 minutes)

#### Systematic PID Tuning Methodology

**Ziegler-Nichols Method:**

1. Set $K_i = 0$ and $K_d = 0$
2. Increase $K_p$ until sustained oscillation occurs
3. Record critical gain $K_{p,crit}$ and oscillation period $T_{crit}$
4. Calculate PID gains:
   - $K_p = 0.6 K_{p,crit}$
   - $K_i = 2K_p / T_{crit}$
   - $K_d = K_p T_{crit} / 8$

**Manual Tuning Guidelines:**

| Symptom | Adjustment |
|---------|-----------|
| Oscillates continuously | Reduce $K_p$, increase $K_d$ |
| Slow response | Increase $K_p$ |
| Steady-state error | Increase $K_i$ |
| Overshoot | Increase $K_d$, reduce $K_p$ |

**Performance Metrics:**
- **Rise time:** Time to reach 90% of setpoint
- **Settling time:** Time to stay within 5% of setpoint
- **Overshoot:** Maximum deviation beyond setpoint
- **Steady-state error:** Final error after transient

Target: Rise time < 0.5s, settling time < 2s, overshoot < 10%, steady-state error < 2°

---

## Part 3: Analysis and Reflection (30 minutes)

### Experimental Data Collection

Record system performance:
1. **Step response:** Apply sudden disturbance, measure recovery
2. **Continuous operation:** Balance duration before failure
3. **Disturbance rejection:** Apply varying force levels, measure max recoverable disturbance

### From Pendulum to Humanoid

<div class="theory-concept">

**Scaling Complexity**

The humanoid robot represents a multi-link inverted pendulum:

- **Single-link (today):** 1 DOF, 2 state variables, single PID loop
- **Double-link (ankle + hip):** 2 DOF, 4 state variables, coupled control
- **Full humanoid:** 20+ DOF, 40+ state variables, hierarchical control with task prioritization

Key principles remain constant:
- Real-time sensing of deviation from equilibrium
- Proportional-derivative feedback for stabilization
- Predictive control to anticipate disturbances

</div>

### Knowledge Check Quiz

4. **Question:** In your pendulum, increasing $K_d$ primarily affects:
   - (a) Steady-state accuracy
   - (b) Response speed
   - (c) Oscillation damping
   - (d) Motor torque

   **Answer:** (c) - Derivative gain provides damping to reduce oscillations

5. **Question:** What is the primary limitation of the linearized pendulum model?
   - (a) Computational complexity
   - (b) Invalid for large angles (>15°)
   - (c) Requires expensive sensors
   - (d) Cannot be implemented digitally

   **Answer:** (b) - The small-angle approximation breaks down for large deviations

---

## Deliverables and Assessment

### Required Submissions

1. **Working Demonstration**
   - Video showing pendulum balancing for minimum 30 seconds
   - Recovery from manual disturbance
   - Return to equilibrium within 2 seconds

2. **Technical Analysis**
   - Derivation of linearized equation of motion
   - Documentation of final PID gains with justification
   - Performance metrics (rise time, settling time, overshoot)

3. **Code Repository**
   - Commented source code
   - Calibration procedure
   - Serial output data logged during testing

4. **Reflection Document**
   - Discussion of challenges encountered
   - Explanation of how pendulum dynamics relate to humanoid balance
   - Hypotheses for how multi-link systems will differ

### Assessment Rubric

**Functional Performance (40%)**
- Balances for 30 seconds: Minimum requirement
- Balances for 5 minutes: Excellent performance
- Recovers from disturbances: Required
- No oscillations in steady-state: Optimal tuning

**Theoretical Understanding (30%)**
- Correct derivation of equations: Full marks
- Accurate PID tuning justification: Full marks
- Clear explanation of control principles: Full marks

**Documentation Quality (20%)**
- Well-commented code: Professional standard
- Clear experimental data presentation: Required
- Thoughtful reflection: Demonstrated learning

**Innovation (10%)**
- Implementation of advanced features (LQR, adaptive tuning, etc.)
- Novel mechanical designs
- Exceptional performance

---

## Extensions and Further Study

<div class="optional-exercise">

**Challenge Extensions**

For students seeking additional depth:

1. **Double Inverted Pendulum:** Add a second link to create a cart-double-pendulum system
2. **Adaptive Control:** Implement gain scheduling based on angle magnitude
3. **Optimal Control:** Design LQR or MPC controller and compare to PID
4. **Disturbance Observer:** Estimate external torques and compensate proactively
5. **Hardware Optimization:** Design custom PCB for integrated sensor and control

</div>

### Recommended Reading

- **Textbook:** "Feedback Systems" by Åström and Murray, Chapter 10
- **Paper:** Yamakita et al. (1991), "Robust Swing-Up Control of Double Pendulum"
- **Tutorial:** "PID Control in the Third Millennium" by Åström and Hägglund
- **Video:** Steve Brunton's Control Bootcamp (YouTube series)

---

## Connection to HUM-102

In the next course, you will design and construct the humanoid torso—the structural foundation onto which legs and arms will attach. The power systems and controller architecture established here will scale to manage 20+ servo motors.

Key concepts to carry forward:
- **Real-time loop timing:** 100 Hz control frequency
- **Sensor fusion:** Combining multiple data sources
- **Modular design:** Independent subsystems with clean interfaces

Your inverted pendulum serves as the conceptual and software foundation for the full humanoid balance controller implemented in HUM-106.

---

*Mastery of the inverted pendulum provides the essential foundation for all bipedal robotics. Proceed to HUM-102 to build the physical platform for humanoid locomotion.*
