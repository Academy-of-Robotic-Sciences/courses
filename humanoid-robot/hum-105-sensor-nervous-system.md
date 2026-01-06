---
id: hum-105-sensor-nervous-system
title: "HUM-105: Sensor Integration and State Estimation"
sidebar_position: 6
version: 2.0.0
link: https://raw.githubusercontent.com/Academy-of-Robotic-Sciences/courses/main/course-styles.css
---

# HUM-105: Sensor Integration and State Estimation

> Implementing the sensory nervous system that enables proprioception, exteroception, and real-time state estimation for humanoid balance and control.

## Course Overview

| Parameter | Details |
|---|---|
| **Course Code** | HUM-105 |
| **Duration** | 6 contact hours |
| **Format** | Theory (40%) + Guided Project (60%) |
| **Prerequisites** | HUM-104 completed, signal processing fundamentals |
| **Deliverables** | Integrated sensor suite, calibrated state estimator |

---

## Learning Objectives

### Theoretical Knowledge
- Understand sensor types and their operational principles
- Analyze sensor fusion algorithms including complementary and Kalman filtering
- Derive state estimation equations for pose and velocity
- Explain the role of proprioceptive and exteroceptive sensing in humanoid control

### Practical Skills
- Integrate IMU sensors with sensor fusion algorithms
- Deploy force-sensitive resistors for ground contact detection
- Implement real-time state estimation at 100+ Hz
- Calibrate multi-modal sensor arrays

### Project Integration
- Establish the sensory foundation for balance control (HUM-106)
- Create data logging infrastructure for system analysis
- Develop safety monitoring through current sensing

---

## Part 1: Theoretical Foundations (140 minutes)

### Sensor Taxonomy for Humanoid Robots

<div class="theory-concept">

**Proprioception vs. Exteroception**

**Proprioceptive sensors** measure the robot's internal state:
- Joint encoders: Angular position and velocity
- Inertial Measurement Units (IMUs): Angular velocity and linear acceleration
- Force/torque sensors: Ground reaction forces and contact detection

**Exteroceptive sensors** measure the environment:
- Cameras: Visual information for object detection and localization
- LiDAR/ultrasonic: Distance to obstacles
- Tactile sensors: Contact forces and object properties

For bipedal balance, proprioception dominates—the robot must know its own state before reacting to the world.

</div>

#### Inertial Measurement Units

An IMU typically combines:
1. **3-axis accelerometer:** Measures linear acceleration $\mathbf{a} = [a_x, a_y, a_z]^T$ (m/s²)
2. **3-axis gyroscope:** Measures angular velocity $\boldsymbol{\omega} = [\omega_x, \omega_y, \omega_z]^T$ (rad/s)
3. **3-axis magnetometer (optional):** Measures magnetic field for absolute heading

**Accelerometer Model:**

The accelerometer measures the sum of linear acceleration and gravity:

$$\mathbf{a}_{measured} = \mathbf{a}_{true} - \mathbf{g} + \boldsymbol{\eta}_a$$

Where $\mathbf{g} = [0, 0, -9.81]^T$ m/s² in the world frame, and $\boldsymbol{\eta}_a$ is sensor noise.

When stationary, $\mathbf{a}_{true} = 0$, so the accelerometer measures only gravity:

$$\theta_{pitch} = \arctan\left(\frac{a_x}{\sqrt{a_y^2 + a_z^2}}\right)$$

$$\theta_{roll} = \arctan\left(\frac{a_y}{\sqrt{a_x^2 + a_z^2}}\right)$$

**Gyroscope Model:**

The gyroscope provides angular velocity in the body frame:

$$\boldsymbol{\omega}_{measured} = \boldsymbol{\omega}_{true} + \mathbf{b}_g + \boldsymbol{\eta}_g$$

Where $\mathbf{b}_g$ is the gyroscope bias (drift) and $\boldsymbol{\eta}_g$ is white noise.

Integrating angular velocity yields orientation:

$$\boldsymbol{\theta}(t) = \boldsymbol{\theta}(t_0) + \int_{t_0}^t \boldsymbol{\omega}(\tau) d\tau$$

However, integration amplifies bias, causing **drift** over time.

### Sensor Fusion Theory

<div class="theory-concept">

**The Complementary Nature of Accelerometer and Gyroscope**

- **Accelerometer:** Accurate at low frequencies (long-term), noisy at high frequencies (vibrations)
- **Gyroscope:** Accurate at high frequencies (short-term), drifts at low frequencies

Solution: **Complementary Filter** combines their strengths.

</div>

#### Complementary Filter

The discrete-time complementary filter:

$$\theta[k] = \alpha \left(\theta[k-1] + \omega[k-1] \Delta t\right) + (1-\alpha) \theta_{acc}[k]$$

Where:
- $\alpha \in [0.95, 0.99]$: Trust factor for gyroscope
- $\theta[k-1] + \omega[k-1]\Delta t$: Prediction from gyroscope integration
- $\theta_{acc}[k]$: Measurement from accelerometer
- $\Delta t$: Sample period

**Interpretation:** Trust the gyroscope for fast changes (high $\alpha$), but slowly correct drift using accelerometer.

#### Kalman Filter for State Estimation

For optimal fusion, the **Kalman filter** provides minimum-variance state estimates.

**State vector:** $\mathbf{x} = [\theta, \dot{\theta}]^T$

**System model:**
$$\mathbf{x}[k] = \mathbf{A}\mathbf{x}[k-1] + \mathbf{B}\mathbf{u}[k] + \mathbf{w}[k]$$

$$\mathbf{z}[k] = \mathbf{H}\mathbf{x}[k] + \mathbf{v}[k]$$

For angle estimation:
$$\mathbf{A} = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} 0 \\ 0 \end{bmatrix}$$

$$\mathbf{H} = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}$$

Measurements: $\mathbf{z} = [\theta_{acc}, \omega_{gyro}]^T$

**Kalman filter algorithm:**

**Prediction:**
$$\hat{\mathbf{x}}^-[k] = \mathbf{A}\hat{\mathbf{x}}[k-1]$$
$$\mathbf{P}^-[k] = \mathbf{A}\mathbf{P}[k-1]\mathbf{A}^T + \mathbf{Q}$$

**Update:**
$$\mathbf{K}[k] = \mathbf{P}^-[k]\mathbf{H}^T(\mathbf{H}\mathbf{P}^-[k]\mathbf{H}^T + \mathbf{R})^{-1}$$
$$\hat{\mathbf{x}}[k] = \hat{\mathbf{x}}^-[k] + \mathbf{K}[k](\mathbf{z}[k] - \mathbf{H}\hat{\mathbf{x}}^-[k])$$
$$\mathbf{P}[k] = (\mathbf{I} - \mathbf{K}[k]\mathbf{H})\mathbf{P}^-[k]$$

Where:
- $\mathbf{Q}$: Process noise covariance
- $\mathbf{R}$: Measurement noise covariance
- $\mathbf{K}$: Kalman gain
- $\mathbf{P}$: Estimate error covariance

### Force Sensing and Ground Contact Detection

**Force-Sensitive Resistors (FSRs)** change resistance under applied force:

$$R_{FSR} = \frac{R_0}{1 + (F/F_0)^{\gamma}}$$

Where $\gamma \approx 0.5$ to $0.8$ for typical FSRs.

**Center of Pressure (CoP):**

With four force sensors per foot at positions $(x_i, y_i)$ measuring forces $F_i$:

$$x_{CoP} = \frac{\sum_{i=1}^4 F_i x_i}{\sum_{i=1}^4 F_i}, \quad y_{CoP} = \frac{\sum_{i=1}^4 F_i y_i}{\sum_{i=1}^4 F_i}$$

The CoP must remain within the **support polygon** (convex hull of ground contacts) for static stability.

<div class="historical-note">

**Historical Development of Sensor Fusion**

- **1960:** Kalman publishes the Kalman filter, revolutionizing state estimation
- **1980s:** MEMS accelerometers and gyroscopes emerge from semiconductor technology
- **1990s:** First low-cost IMUs appear in consumer electronics
- **2000s:** MPU-6050 and similar integrated IMUs democratize robotics sensing
- **2010s:** Sensor fusion algorithms become standard in smartphones and drones
- **Present:** Advanced IMUs with integrated sensor fusion processors

This progression enabled affordable, reliable orientation sensing crucial for humanoid robotics.

</div>

### Knowledge Check Quiz

1. **Question:** Why does gyroscope integration cause drift over time?
   - (a) Gyroscopes are fundamentally inaccurate
   - (b) Integration accumulates bias errors
   - (c) Angular velocity cannot be measured precisely
   - (d) Digital sampling introduces errors

   **Answer:** (b) - Even small constant bias terms grow linearly when integrated

2. **Question:** In a complementary filter with $\alpha = 0.98$, which sensor dominates short-term estimates?
   - (a) Accelerometer
   - (b) Gyroscope
   - (c) Both equally
   - (d) Neither

   **Answer:** (b) - High $\alpha$ weights gyroscope integration heavily

3. **Question:** What indicates static stability in terms of center of pressure?
   - (a) CoP equals zero
   - (b) CoP is constant over time
   - (c) CoP lies within the support polygon
   - (d) CoP matches the center of mass

   **Answer:** (c) - CoP must be inside the convex hull of ground contacts

---

## Part 2: Guided Project - Sensor Integration (220 minutes)

### Phase 1: IMU Integration and Calibration (60 minutes)

#### Hardware Setup

**MPU-6050 Specifications:**
- Accelerometer range: ±2g, ±4g, ±8g, ±16g
- Gyroscope range: ±250°/s, ±500°/s, ±1000°/s, ±2000°/s
- I²C communication at 400 kHz
- Integrated Digital Motion Processor (DMP) for sensor fusion

**Mounting Considerations:**
1. **Location:** Mount IMU at robot's center of mass (typically torso, near hip level)
2. **Orientation:** Align IMU axes with robot body frame (X forward, Y left, Z up)
3. **Rigidity:** Hard-mount to avoid vibration-induced noise
4. **Isolation:** Keep away from motors (magnetic interference) and high-current wires

**I²C Wiring:**
```
MPU-6050        Arduino Mega
VCC      →      3.3V (NOT 5V—will damage sensor)
GND      →      GND
SDA      →      SDA (Pin 20)
SCL      →      SCL (Pin 21)
```

<div class="exercise-tip">

**Preventing I²C Communication Issues**

Common problems and solutions:
- **No device detected:** Check power voltage (must be 3.3V), verify I²C address (0x68 or 0x69)
- **Intermittent data:** Add 4.7kΩ pull-up resistors on SDA and SCL lines
- **Corrupted readings:** Reduce I²C clock speed or shorten wire length
- **Multiple devices:** Use I²C multiplexer or change AD0 pin to select alternate address

</div>

#### IMU Calibration Procedure

```cpp
#include <Wire.h>
#include <MPU6050.h>

MPU6050 imu;

struct IMUCalibration {
  float accel_offset[3];  // Accelerometer bias
  float gyro_offset[3];   // Gyroscope bias
  float accel_scale[3];   // Accelerometer scale factors
};

IMUCalibration calibration;

void calibrateIMU() {
  const int samples = 2000;
  float accel_sum[3] = {0, 0, 0};
  float gyro_sum[3] = {0, 0, 0};

  Serial.println("IMU Calibration: Keep robot completely still");
  delay(3000);

  // Collect samples
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    accel_sum[0] += ax;
    accel_sum[1] += ay;
    accel_sum[2] += az;
    gyro_sum[0] += gx;
    gyro_sum[1] += gy;
    gyro_sum[2] += gz;

    delay(5);  // 200 Hz sampling
  }

  // Calculate gyroscope bias (should be zero at rest)
  for (int i = 0; i < 3; i++) {
    calibration.gyro_offset[i] = gyro_sum[i] / samples;
  }

  // Calculate accelerometer bias
  // Z-axis should measure -1g when upright
  calibration.accel_offset[0] = accel_sum[0] / samples;
  calibration.accel_offset[1] = accel_sum[1] / samples;
  calibration.accel_offset[2] = accel_sum[2] / samples - 16384;  // 1g at ±2g scale

  Serial.println("Calibration complete:");
  Serial.print("Gyro offsets: ");
  Serial.print(calibration.gyro_offset[0]); Serial.print(", ");
  Serial.print(calibration.gyro_offset[1]); Serial.print(", ");
  Serial.println(calibration.gyro_offset[2]);

  // Store in EEPROM for persistence
  saveCalibration(calibration);
}

void getCompensatedIMU(float accel[3], float gyro[3]) {
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to physical units and apply calibration
  // Accelerometer: LSB = 16384 for ±2g range
  accel[0] = (ax - calibration.accel_offset[0]) / 16384.0 * 9.81;
  accel[1] = (ay - calibration.accel_offset[1]) / 16384.0 * 9.81;
  accel[2] = (az - calibration.accel_offset[2]) / 16384.0 * 9.81;

  // Gyroscope: LSB = 131 for ±250°/s range
  gyro[0] = (gx - calibration.gyro_offset[0]) / 131.0 * PI / 180.0;  // rad/s
  gyro[1] = (gy - calibration.gyro_offset[1]) / 131.0 * PI / 180.0;
  gyro[2] = (gz - calibration.gyro_offset[2]) / 131.0 * PI / 180.0;
}
```

### Phase 2: Complementary Filter Implementation (50 minutes)

```cpp
class ComplementaryFilter {
private:
  float alpha;          // Trust factor for gyroscope (0.95-0.99)
  float pitch, roll;    // Estimated angles (radians)
  float dt;             // Sample period (seconds)

public:
  ComplementaryFilter(float trust_gyro = 0.98, float sample_time = 0.01)
    : alpha(trust_gyro), pitch(0), roll(0), dt(sample_time) {}

  void update(float accel[3], float gyro[3]) {
    // Accelerometer-based angle (noisy but no drift)
    float accel_pitch = atan2(accel[0], sqrt(accel[1]*accel[1] + accel[2]*accel[2]));
    float accel_roll = atan2(accel[1], sqrt(accel[0]*accel[0] + accel[2]*accel[2]));

    // Gyroscope integration (smooth but drifts)
    float gyro_pitch = pitch + gyro[1] * dt;  // Pitch from Y-axis rotation
    float gyro_roll = roll + gyro[0] * dt;    // Roll from X-axis rotation

    // Complementary fusion
    pitch = alpha * gyro_pitch + (1 - alpha) * accel_pitch;
    roll = alpha * gyro_roll + (1 - alpha) * accel_roll;
  }

  float getPitch() { return pitch * 180.0 / PI; }  // Return in degrees
  float getRoll() { return roll * 180.0 / PI; }

  void reset() {
    pitch = 0;
    roll = 0;
  }
};
```

**Main Loop Integration:**

```cpp
ComplementaryFilter orientation(0.98, 0.01);  // 98% gyro trust, 10ms loop

void loop() {
  unsigned long start_time = micros();

  // Read calibrated sensor data
  float accel[3], gyro[3];
  getCompensatedIMU(accel, gyro);

  // Update orientation estimate
  orientation.update(accel, gyro);

  // Get current orientation
  float pitch = orientation.getPitch();
  float roll = orientation.getRoll();

  // Data logging for analysis
  Serial.print(pitch); Serial.print(",");
  Serial.print(roll); Serial.print(",");
  Serial.println(millis());

  // Maintain 100 Hz loop rate
  while (micros() - start_time < 10000);  // 10ms = 100 Hz
}
```

<div class="exercise-advanced">

**Advanced Implementation: Kalman Filter**

For students comfortable with linear algebra, implement a Kalman filter:

```cpp
class KalmanFilter {
private:
  float x[2];      // State: [angle, angular_velocity]
  float P[2][2];   // Error covariance
  float Q[2][2];   // Process noise covariance
  float R[2][2];   // Measurement noise covariance
  float dt;

public:
  void predict() {
    // State prediction: x_k = A * x_{k-1}
    x[0] = x[0] + x[1] * dt;
    // x[1] remains constant (random walk model)

    // Covariance prediction: P_k = A * P_{k-1} * A^T + Q
    P[0][0] += 2*dt*P[0][1] + dt*dt*P[1][1] + Q[0][0];
    P[0][1] += dt*P[1][1] + Q[0][1];
    P[1][0] += dt*P[1][1] + Q[1][0];
    P[1][1] += Q[1][1];
  }

  void update(float angle_acc, float gyro) {
    float z[2] = {angle_acc, gyro};  // Measurements

    // Innovation: y = z - H*x
    float y[2] = {z[0] - x[0], z[1] - x[1]};

    // Innovation covariance: S = H*P*H^T + R
    float S[2][2];
    S[0][0] = P[0][0] + R[0][0];
    S[0][1] = P[0][1] + R[0][1];
    S[1][0] = P[1][0] + R[1][0];
    S[1][1] = P[1][1] + R[1][1];

    // Kalman gain: K = P*H^T * inv(S)
    float det = S[0][0]*S[1][1] - S[0][1]*S[1][0];
    float K[2][2];
    K[0][0] = (P[0][0]*S[1][1] - P[0][1]*S[1][0]) / det;
    K[0][1] = (P[0][1]*S[0][0] - P[0][0]*S[0][1]) / det;
    K[1][0] = (P[1][0]*S[1][1] - P[1][1]*S[1][0]) / det;
    K[1][1] = (P[1][1]*S[0][0] - P[1][0]*S[0][1]) / det;

    // State update: x = x + K*y
    x[0] += K[0][0]*y[0] + K[0][1]*y[1];
    x[1] += K[1][0]*y[0] + K[1][1]*y[1];

    // Covariance update: P = (I - K*H)*P
    float I_KH[2][2] = {{1-K[0][0], -K[0][1]}, {-K[1][0], 1-K[1][1]}};
    float P_new[2][2];
    P_new[0][0] = I_KH[0][0]*P[0][0] + I_KH[0][1]*P[1][0];
    P_new[0][1] = I_KH[0][0]*P[0][1] + I_KH[0][1]*P[1][1];
    P_new[1][0] = I_KH[1][0]*P[0][0] + I_KH[1][1]*P[1][0];
    P_new[1][1] = I_KH[1][0]*P[0][1] + I_KH[1][1]*P[1][1];

    memcpy(P, P_new, sizeof(P));
  }
};
```

</div>

### Phase 3: Force Sensor Integration (60 minutes)

#### FSR Circuit Design

For each FSR, implement a voltage divider:

```
VCC (5V)
   |
   FSR (R_fsr varies with force)
   |
   +---- Analog Input (A0-A7)
   |
   10kΩ resistor
   |
  GND
```

Output voltage: $V_{out} = \frac{V_{CC} \cdot R_{fixed}}{R_{FSR} + R_{fixed}}$

**Calibration Procedure:**

```cpp
struct FSRCalibration {
  float zero_voltage;     // Voltage at zero force
  float full_voltage;     // Voltage at maximum force
  float force_scale;      // Newtons per volt
};

FSRCalibration fsr_cal[8];  // 4 sensors per foot

void calibrateFSR(int sensor_id) {
  Serial.println("FSR Calibration - Remove all weight");
  delay(3000);

  // Zero measurement
  int zero_samples = 100;
  float zero_sum = 0;
  for (int i = 0; i < zero_samples; i++) {
    zero_sum += analogRead(sensor_id);
    delay(10);
  }
  fsr_cal[sensor_id].zero_voltage = (zero_sum / zero_samples) * 5.0 / 1023.0;

  Serial.println("Place known weight (500g) on sensor");
  delay(5000);

  // Full force measurement
  int full_samples = 100;
  float full_sum = 0;
  for (int i = 0; i < full_samples; i++) {
    full_sum += analogRead(sensor_id);
    delay(10);
  }
  fsr_cal[sensor_id].full_voltage = (full_sum / full_samples) * 5.0 / 1023.0;

  // Calculate scale factor (500g = 4.905 N)
  float voltage_range = fsr_cal[sensor_id].full_voltage - fsr_cal[sensor_id].zero_voltage;
  fsr_cal[sensor_id].force_scale = 4.905 / voltage_range;
}

float readForce(int sensor_id) {
  float voltage = analogRead(sensor_id) * 5.0 / 1023.0;
  float force = (voltage - fsr_cal[sensor_id].zero_voltage) * fsr_cal[sensor_id].force_scale;
  return max(force, 0.0);  // Clamp to non-negative
}
```

#### Center of Pressure Calculation

```cpp
struct FootForces {
  float heel_left, heel_right;
  float toe_left, toe_right;
  float total;
  float cop_x, cop_y;  // Center of pressure in foot frame
};

// Sensor positions in foot frame (mm)
const float SENSOR_POS[4][2] = {
  {-30, -20},  // Heel left
  {-30,  20},  // Heel right
  { 30, -20},  // Toe left
  { 30,  20}   // Toe right
};

FootForces computeFootForces(int foot_sensor_base) {
  FootForces forces;

  // Read all four sensors
  float F[4];
  for (int i = 0; i < 4; i++) {
    F[i] = readForce(foot_sensor_base + i);
  }

  forces.heel_left = F[0];
  forces.heel_right = F[1];
  forces.toe_left = F[2];
  forces.toe_right = F[3];
  forces.total = F[0] + F[1] + F[2] + F[3];

  // Calculate CoP
  if (forces.total > 0.5) {  // Threshold to avoid division by zero
    forces.cop_x = 0;
    forces.cop_y = 0;
    for (int i = 0; i < 4; i++) {
      forces.cop_x += F[i] * SENSOR_POS[i][0];
      forces.cop_y += F[i] * SENSOR_POS[i][1];
    }
    forces.cop_x /= forces.total;
    forces.cop_y /= forces.total;
  } else {
    forces.cop_x = 0;
    forces.cop_y = 0;
  }

  return forces;
}

bool isFootInContact(FootForces forces, float threshold = 1.0) {
  return forces.total > threshold;  // Newtons
}
```

### Phase 4: Current Sensing for Motor Protection (30 minutes)

```cpp
// ACS712 current sensor (5A version)
// Sensitivity: 185 mV/A
// Zero current output: 2.5V

const float ACS712_SENSITIVITY = 0.185;  // V/A
const float ACS712_ZERO_CURRENT = 2.5;   // V
const float CURRENT_LIMIT = 3.0;         // Amps
const int OVERCURRENT_TIME = 500;        // ms

struct CurrentMonitor {
  float current;
  unsigned long overcurrent_start;
  bool alarm_active;
};

CurrentMonitor motor_current[4];  // Left leg, right leg, left arm, right arm

float readCurrent(int sensor_pin) {
  // Average multiple samples to reduce noise
  float voltage_sum = 0;
  for (int i = 0; i < 10; i++) {
    voltage_sum += analogRead(sensor_pin) * 5.0 / 1023.0;
  }
  float voltage = voltage_sum / 10.0;

  float current = (voltage - ACS712_ZERO_CURRENT) / ACS712_SENSITIVITY;
  return abs(current);  // Magnitude only
}

void monitorMotorCurrent(int motor_group, int sensor_pin) {
  motor_current[motor_group].current = readCurrent(sensor_pin);

  if (motor_current[motor_group].current > CURRENT_LIMIT) {
    if (!motor_current[motor_group].alarm_active) {
      motor_current[motor_group].overcurrent_start = millis();
      motor_current[motor_group].alarm_active = true;
    }

    if (millis() - motor_current[motor_group].overcurrent_start > OVERCURRENT_TIME) {
      // Persistent overcurrent - shut down motor group
      Serial.print("OVERCURRENT ALARM: Motor group ");
      Serial.println(motor_group);
      disableMotorGroup(motor_group);
    }
  } else {
    motor_current[motor_group].alarm_active = false;
  }
}
```

### Phase 5: Sensor Fusion Dashboard (20 minutes)

Create a Python script for real-time visualization:

```python
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

# Connect to Arduino
ser = serial.Serial('/dev/ttyUSB0', 115200)

# Data buffers
time_data = []
pitch_data = []
roll_data = []
cop_x_data = []
cop_y_data = []

fig, axes = plt.subplots(2, 2, figsize=(12, 8))

def update(frame):
    if ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        data = line.split(',')

        if len(data) == 5:
            t, pitch, roll, cop_x, cop_y = map(float, data)

            time_data.append(t / 1000.0)  # Convert ms to seconds
            pitch_data.append(pitch)
            roll_data.append(roll)
            cop_x_data.append(cop_x)
            cop_y_data.append(cop_y)

            # Keep only last 200 samples
            if len(time_data) > 200:
                time_data.pop(0)
                pitch_data.pop(0)
                roll_data.pop(0)
                cop_x_data.pop(0)
                cop_y_data.pop(0)

    # Clear and redraw
    axes[0, 0].clear()
    axes[0, 0].plot(time_data, pitch_data, 'b-', label='Pitch')
    axes[0, 0].set_ylabel('Pitch (degrees)')
    axes[0, 0].set_ylim([-45, 45])
    axes[0, 0].axhline(0, color='r', linestyle='--', alpha=0.3)
    axes[0, 0].legend()
    axes[0, 0].grid(True)

    axes[0, 1].clear()
    axes[0, 1].plot(time_data, roll_data, 'g-', label='Roll')
    axes[0, 1].set_ylabel('Roll (degrees)')
    axes[0, 1].set_ylim([-45, 45])
    axes[0, 1].axhline(0, color='r', linestyle='--', alpha=0.3)
    axes[0, 1].legend()
    axes[0, 1].grid(True)

    axes[1, 0].clear()
    if cop_x_data and cop_y_data:
        axes[1, 0].scatter([cop_x_data[-1]], [cop_y_data[-1]], s=200, c='red')
    axes[1, 0].set_xlim([-40, 40])
    axes[1, 0].set_ylim([-30, 30])
    axes[1, 0].set_xlabel('CoP X (mm)')
    axes[1, 0].set_ylabel('CoP Y (mm)')
    axes[1, 0].set_title('Center of Pressure')
    axes[1, 0].grid(True)

    # Draw foot outline
    foot_outline = plt.Rectangle((-30, -20), 60, 40, fill=False, edgecolor='black')
    axes[1, 0].add_patch(foot_outline)

    axes[1, 1].clear()
    axes[1, 1].plot(time_data, cop_x_data, 'm-', label='CoP X')
    axes[1, 1].set_ylabel('CoP X (mm)')
    axes[1, 1].set_ylim([-40, 40])
    axes[1, 1].legend()
    axes[1, 1].grid(True)

ani = FuncAnimation(fig, update, interval=50)
plt.tight_layout()
plt.show()
```

---

## Part 3: Analysis and System Validation (20 minutes)

### Performance Metrics

**IMU Performance:**
- Update rate: 100 Hz minimum
- Angle accuracy: ±2° RMS
- Drift rate: <1°/minute after calibration

**Force Sensor Performance:**
- Response time: <10 ms
- Force resolution: 0.1 N
- Repeatability: ±5%

**State Estimation Quality:**
```python
def analyze_sensor_performance(data_log):
    """Analyze logged sensor data for quality metrics"""
    import numpy as np

    # Calculate RMS noise on accelerometer (when stationary)
    stationary_periods = identify_stationary_periods(data_log)
    accel_noise = []
    for period in stationary_periods:
        accel_noise.append(np.std(period['accel']))

    print(f"Accelerometer noise: {np.mean(accel_noise):.4f} m/s² RMS")

    # Measure gyroscope drift
    drift_rate = (data_log['angle'][-1] - data_log['angle'][0]) / data_log['duration']
    print(f"Gyroscope drift: {drift_rate:.3f} degrees/minute")

    # Complementary filter performance
    # Compare with ground truth (if available)
    if 'ground_truth' in data_log:
        angle_error = data_log['estimated_angle'] - data_log['ground_truth']
        print(f"Angle estimation error: {np.std(angle_error):.2f}° RMS")
```

### Knowledge Check Quiz

4. **Question:** In the complementary filter, what happens if $\alpha$ is too low (e.g., 0.5)?
   - (a) Orientation estimate becomes very noisy
   - (b) Drift increases dramatically
   - (c) Sensor fusion fails
   - (d) No significant effect

   **Answer:** (a) - Low $\alpha$ trusts the noisy accelerometer too much, reducing smoothness

5. **Question:** Why are four force sensors per foot better than one?
   - (a) Higher total force capacity
   - (b) Redundancy for sensor failure
   - (c) Enables center of pressure calculation
   - (d) Reduces noise through averaging

   **Answer:** (c) - Multiple sensors allow computation of CoP location, critical for balance

---

## Deliverables and Assessment

### Required Submissions

1. **Functional Sensor System**
   - All sensors calibrated and streaming data at specified rates
   - Complementary filter providing stable orientation estimates
   - Center of pressure calculation verified

2. **Technical Documentation**
   - Sensor calibration coefficients with methodology
   - Filter tuning parameters and justification
   - Data sheets showing sensor noise characteristics

3. **Data Logs**
   - 1-minute recording of stationary operation
   - Recording of manual tilt test in all axes
   - Force sensor data during weight shifting

4. **Analysis Report**
   - Quantitative assessment of sensor performance
   - Comparison of complementary vs. Kalman filter (if implemented)
   - Discussion of sensor fusion trade-offs

### Assessment Rubric

**Functional Performance (40%)**
- IMU orientation accurate to ±3°: Required
- 100 Hz update rate maintained: Required
- Center of pressure tracking weight shifts: Excellent
- Current monitoring detecting stall conditions: Required

**Theoretical Understanding (30%)**
- Correct derivation of filter equations: Full marks
- Clear explanation of sensor fusion rationale: Full marks
- Understanding of noise vs. drift trade-offs: Full marks

**Code Quality (20%)**
- Well-structured sensor abstraction layers: Professional
- Proper calibration persistence (EEPROM): Required
- Real-time constraints met: Critical

**Documentation (10%)**
- Clear plots of sensor data: Required
- Quantitative performance analysis: Full marks

---

## Extensions and Further Study

<div class="optional-exercise">

**Advanced Topics**

1. **Extended Kalman Filter (EKF):** Implement nonlinear state estimation for full 3D orientation (quaternions)
2. **Sensor Diagnostics:** Automated detection of sensor failures and degradation
3. **Adaptive Filtering:** Dynamically adjust filter parameters based on motion state
4. **Multi-Rate Fusion:** Integrate sensors operating at different frequencies
5. **Outlier Rejection:** Implement RANSAC or similar algorithms for robust estimation

</div>

### Recommended Reading

- **Textbook:** "Probabilistic Robotics" by Thrun et al., Chapters 2-3
- **Paper:** Madgwick (2011), "An Efficient Orientation Filter for IMUs"
- **Tutorial:** "Kalman Filter for Dummies" by Bilgin
- **Application Note:** MPU-6050 Register Map and Descriptions (InvenSense)

---

## Connection to HUM-106

The sensor infrastructure established here provides the **feedback** for balance control. In HUM-106, you will close the control loop:

$$\text{IMU} \rightarrow \text{State Estimator} \rightarrow \text{Balance Controller} \rightarrow \text{Actuators} \rightarrow \text{Robot Motion} \rightarrow \text{IMU}$$

Key data flows:
- **Pitch/Roll angles** → Ankle and hip balance strategies
- **Center of Pressure** → ZMP stability monitoring
- **Angular velocities** → Derivative control for damping

Your sensor system is the robot's perception of itself—the foundation for all subsequent control.

---

*From raw sensor noise to reliable state estimates—you've built the humanoid nervous system. Proceed to HUM-106 to make your robot balance.*
