<!--
author:   Dr. Elena Martinez
email:    elena.martinez@robotcampus.dev
version:  2.0.0

language: en
narrator: US English Female

comment:  Sensor Integration and Signal Processing: Comprehensive treatment of sensor physics, communication protocols (I2C, SPI, UART), inertial measurement units, encoders, and sensor fusion algorithms for robotics.

icon:     https://robotcampus.dev/logos/hardware-203.png

mode:     Textbook

link:     https://robotcampus.dev/styles/course-styles.css

import:   https://raw.githubusercontent.com/LiaTemplates/Pyodide/master/README.md
-->

# HARDWARE-203: Sensor Integration and Signal Processing

> **Sensor systems provide the perceptual foundation for autonomous robotic behavior.**

## Course Overview

| | |
|---|---|
| **Course Code** | HARDWARE-203 |
| **Duration** | 4 hours |
| **Level** | Intermediate |
| **Prerequisites** | HARDWARE-202, linear algebra fundamentals |
| **Theory-Practice** | 60% theory, 40% optional labs |

---

## Learning Objectives

Upon completion of this course, students will be able to:

### Theoretical Understanding

- Explain sensor physics and transduction mechanisms for common robotic sensors
- Understand serial communication protocols (I2C, SPI, UART) and their tradeoffs
- Analyze inertial measurement unit (IMU) operation and orientation mathematics
- Describe quadrature encoding and motor feedback systems
- Apply sensor fusion algorithms including complementary and Kalman filters

### Practical Skills

- Interface multiple I2C devices on a shared bus
- Read high-frequency encoder signals using interrupts
- Extract orientation data from IMUs using sensor fusion libraries
- Implement camera-based object detection for robot perception
- Build multi-sensor data acquisition systems with real-time constraints

---

## Module 1: Communication Protocols

### 1.1 Serial Communication Fundamentals

<!-- class="theory-concept" -->
**Parallel vs. Serial Communication**

**Parallel Communication**:
- Multiple data lines (8, 16, 32 bits)
- Faster data transfer
- More complex wiring, prone to skew
- Example: Old printer ports, memory buses

**Serial Communication**:
- One or few data lines
- Sequential bit transmission
- Simpler wiring, longer distances
- Example: USB, Ethernet, I2C, SPI

**Bit Rate vs. Baud Rate**:
- **Bit rate**: Bits per second (bps)
- **Baud rate**: Symbol changes per second
- For binary signaling: bit rate = baud rate

<!-- class="historical-note" -->
**Evolution of Serial Protocols**

**1960s**: RS-232 standardized for teletypewriters and modems
**1970s**: SPI developed at Motorola for short-distance chip communication
**1980s**: I2C created by Philips (now NXP) for TV internal components
**1990s**: USB revolutionized PC peripherals
**2000s**: CAN bus dominated automotive, I2C/SPI became standard in embedded systems
**2010s**: High-speed serial (PCIe, SATA, DisplayPort) replaced parallel buses

### 1.2 I2C (Inter-Integrated Circuit)

<!-- class="theory-concept" -->
**I2C Protocol Architecture**

I2C is a two-wire, multi-master, multi-slave serial protocol:

**Signal Lines**:
- **SDA (Serial Data)**: Bidirectional data line
- **SCL (Serial Clock)**: Clock signal from master
- Both require pull-up resistors (typically 4.7kΩ)

**Addressing**:
- 7-bit addresses (128 devices max)
- Extended 10-bit addressing available
- Each device has unique address
- Address conflicts prevented by device design

**Communication Sequence**:
```
1. Master generates START condition (SDA falls while SCL high)
2. Master sends 7-bit address + R/W bit
3. Addressed slave sends ACK (pulls SDA low)
4. Data bytes transferred (8 bits + ACK each)
5. Master generates STOP condition (SDA rises while SCL high)
```

<!-- class="theory-concept" -->
**I2C Clock Speed**

**Standard Mode**: 100 kHz
**Fast Mode**: 400 kHz
**Fast Mode Plus**: 1 MHz
**High Speed Mode**: 3.4 MHz

**Pull-Up Resistor Calculation**:

Bus capacitance and resistor value determine rise time:

$$t_r = 0.8473 \cdot R_{pull} \cdot C_{bus}$$

For 400 kHz (Fast Mode): $t_r < 300$ ns

**Advantages**:
- Only 2 wires for multiple devices
- Built-in addressing
- Multi-master capable
- ACK/NACK error detection

**Disadvantages**:
- Slower than SPI
- Pull-up resistors limit speed
- Limited distance (~1m typical)

### 1.3 SPI (Serial Peripheral Interface)

<!-- class="theory-concept" -->
**SPI Protocol Architecture**

SPI is a four-wire, full-duplex, master-slave protocol:

**Signal Lines**:
- **MOSI** (Master Out Slave In): Data from master to slave
- **MISO** (Master In Slave Out): Data from slave to master
- **SCK** (Serial Clock): Clock from master
- **SS/CS** (Slave Select/Chip Select): Select active slave

**Communication Characteristics**:
- Full duplex: Simultaneous transmit and receive
- No addressing: SS/CS line per slave
- No acknowledgment mechanism
- Clock polarity (CPOL) and phase (CPHA) configurable

**SPI Modes** (CPOL, CPHA combinations):

| Mode | CPOL | CPHA | Clock Idle | Sample Edge |
|------|------|------|------------|-------------|
| 0    | 0    | 0    | Low        | Rising      |
| 1    | 0    | 1    | Low        | Falling     |
| 2    | 1    | 0    | High       | Falling     |
| 3    | 1    | 1    | High       | Rising      |

**Clock Speed**:
- Typical: 1-10 MHz
- Maximum: 50+ MHz for some devices

**Advantages**:
- Fast (10-100× faster than I2C)
- Full duplex
- Simple hardware
- No addressing overhead

**Disadvantages**:
- More wires (4 + additional SS for each slave)
- No error checking
- No multi-master capability
- No flow control

### 1.4 UART (Universal Asynchronous Receiver-Transmitter)

<!-- class="theory-concept" -->
**UART Protocol**

UART provides asynchronous serial communication:

**Signal Lines**:
- **TX** (Transmit): Data output
- **RX** (Receive): Data input
- **Ground**: Common reference

**Frame Structure**:
```
[START bit] [7-8 DATA bits] [Optional PARITY bit] [1-2 STOP bits]
```

**Configuration Parameters**:
- **Baud Rate**: 9600, 115200 bps common
- **Data Bits**: 7 or 8
- **Parity**: None, Even, Odd
- **Stop Bits**: 1 or 2

**Timing**:
No shared clock—both devices must use same baud rate (within ~2% tolerance).

**Sample Period**: $T = 1 / \text{baud rate}$

**Advantages**:
- Simple, two-wire connection
- Asynchronous (no clock)
- Universal support
- Good for point-to-point

**Disadvantages**:
- Only two devices (without additional hardware)
- No addressing
- Baud rate mismatch causes errors
- Moderate speed

**Quiz: Communication Protocols**

Which protocol requires pull-up resistors?

[( )] SPI
[(X)] I2C
[( )] UART
[( )] All of the above

What is the maximum number of devices on a single SPI bus with one master?

[( )] 128 (7-bit addressing)
[( )] 2 (one master, one slave)
[(X)] Limited only by available chip select lines
[( )] 256 (8-bit addressing)

---

## Module 2: Inertial Measurement Units

### 2.1 Accelerometer Physics

<!-- class="theory-concept" -->
**Acceleration Measurement**

Accelerometers measure proper acceleration (acceleration relative to freefall):

**MEMS Accelerometer Operation**:
- Proof mass suspended by springs
- Displacement measured capacitively or piezoresistively
- Acceleration: $a = k \cdot \Delta x / m$ (spring-mass system)

**Three-Axis Measurement**:
Accelerometers provide acceleration in body frame coordinates:

$$\vec{a} = [a_x, a_y, a_z]^T$$

**Gravity Sensing**:
At rest, accelerometer measures gravitational acceleration:

$$\vec{a}_{\text{rest}} = -\vec{g}_{\text{body}}$$

This enables tilt sensing by computing angle to gravity vector.

**Sensitivity and Range**:
- **Range**: ±2g to ±16g typical (g = 9.81 m/s²)
- **Resolution**: 12-16 bits typical
- Higher range → lower resolution

**Noise and Drift**:
- High-frequency noise (vibration)
- Bias drift with temperature
- Requires filtering for orientation estimation

### 2.2 Gyroscope Physics

<!-- class="theory-concept" -->
**Angular Velocity Measurement**

Gyroscopes measure rotational rate around three axes:

$$\vec{\omega} = [\omega_x, \omega_y, \omega_z]^T \quad \text{(rad/s or deg/s)}$$

**MEMS Gyroscope Operation**:
- Vibrating proof mass
- Coriolis force proportional to angular velocity: $F_c = 2m(\vec{v} \times \vec{\omega})$
- Force measured capacitively

**Integration for Orientation**:
Integrate angular velocity to estimate orientation:

$$\theta(t) = \theta(t_0) + \int_{t_0}^{t} \omega(\tau) \, d\tau$$

**Drift Problem**:
- Gyroscope bias causes unbounded error growth
- 1°/s bias → 60°/min error
- Must fuse with absolute reference (accelerometer, magnetometer)

### 2.3 Magnetometer and Sensor Fusion

<!-- class="theory-concept" -->
**Magnetometer Operation**

Magnetometers measure Earth's magnetic field vector:

$$\vec{B} = [B_x, B_y, B_z]^T$$

**Applications**:
- Absolute heading (yaw angle) reference
- Compass functionality
- Complements accelerometer (pitch/roll) and gyroscope

**Magnetic Interference**:
- **Hard iron**: Permanent magnets on robot (fixed offset)
- **Soft iron**: Ferromagnetic materials causing field distortion
- Calibration required for accurate heading

<!-- class="theory-concept" -->
**9-DOF IMU**

Modern IMUs combine:
- **3-axis accelerometer** (3 DOF)
- **3-axis gyroscope** (3 DOF)
- **3-axis magnetometer** (3 DOF)

Total: 9 degrees of freedom

**Example**: MPU-6050 (6-DOF: accel + gyro), MPU-9250 (9-DOF: accel + gyro + mag)

### 2.4 Orientation Representation and Fusion

<!-- class="theory-concept" -->
**Euler Angles**

Orientation described by three rotations:
- **Roll** (φ): Rotation about X-axis
- **Pitch** (θ): Rotation about Y-axis
- **Yaw** (ψ): Rotation about Z-axis

**Extracting from Accelerometer** (static case):

$$\text{Roll} = \phi = \text{atan2}(a_y, a_z)$$
$$\text{Pitch} = \theta = \text{atan2}(-a_x, \sqrt{a_y^2 + a_z^2})$$

**Gimbal Lock Problem**: Singularity when pitch = ±90°

<!-- class="theory-concept" -->
**Complementary Filter**

Fuses accelerometer (low-pass) and gyroscope (high-pass) for tilt:

$$\theta_{\text{fused}} = \alpha(\theta_{\text{gyro}} + \omega \Delta t) + (1-\alpha)\theta_{\text{accel}}$$

where α ≈ 0.95-0.98

**Rationale**:
- Accelerometer: Accurate long-term, noisy short-term
- Gyroscope: Accurate short-term, drifts long-term
- Complementary filter: Trust gyro for fast changes, accelerometer for slow corrections

<!-- class="alternative-approach" -->
**Kalman Filter**

Optimal state estimator fusing sensor measurements:

**State**: $\vec{x} = [\theta, \omega]^T$ (angle, angular velocity)

**Prediction** (from gyroscope):
$$\hat{x}_{k|k-1} = F \hat{x}_{k-1|k-1}$$

**Update** (from accelerometer):
$$\hat{x}_{k|k} = \hat{x}_{k|k-1} + K(z_k - H\hat{x}_{k|k-1})$$

**Advantages**:
- Optimal for Gaussian noise
- Handles sensor noise statistics
- Widely used in robotics

**Complexity**: More computationally intensive than complementary filter

**Quiz: IMU Fundamentals**

Why does a stationary accelerometer read approximately 9.8 m/s² (1g) on one axis?

[(X)] It measures the normal force counteracting gravity
[( )] It's measuring gravitational acceleration
[( )] The sensor is miscalibrated
[( )] Air pressure causes the reading

Gyroscope drift is:

[( )] High-frequency noise
[(X)] Slow bias that accumulates over time
[( )] Caused by magnetic interference
[( )] Only a problem in low-cost sensors

---

## Module 3: Encoder Systems

### 3.1 Rotary Encoder Principles

<!-- class="theory-concept" -->
**Incremental Encoders**

Incremental encoders measure relative position change:

**Types**:
1. **Optical**: LED + photodetector, slotted disk
2. **Magnetic**: Hall effect sensor, magnetized disk
3. **Capacitive**: Capacitance changes with rotation

**Resolution**:
- **Counts Per Revolution (CPR)**: Number of state changes per rotation
- **Pulses Per Revolution (PPR)**: Number of pulses (1/4 of CPR for quadrature)

Example:
- 360 CPR encoder
- Resolution: 1° per count
- With 4× decoding (quadrature): 0.25° per count

### 3.2 Quadrature Encoding

<!-- class="theory-concept" -->
**Two-Channel Encoding**

Quadrature encoders use two output channels (A and B) phase-shifted 90°:

**Phase Relationship**:
```
Channel A:  ▁▁▁▁▔▔▔▔▁▁▁▁▔▔▔▔
Channel B:  ▁▁▔▔▔▔▁▁▁▁▔▔▔▔▁▁
            ←  CW rotation →
```

**Direction Detection**:
- **Clockwise**: A leads B (A rises before B)
- **Counter-clockwise**: B leads A (B rises before A)

**4× Decoding** (Quadrature):
Count on all edges of both channels:
- A rising + B low → +1
- A rising + B high → -1
- A falling + B high → +1
- A falling + B low → -1
- (B rising/falling with similar logic)

**Position Calculation**:
$$\theta = \frac{\text{count}}{4 \times \text{PPR}} \times 360°$$

### 3.3 Velocity Estimation from Encoders

<!-- class="theory-concept" -->
**Differentiation Methods**

**Method 1: Fixed Time Interval**
Measure position change over fixed Δt:

$$\omega = \frac{\Delta\theta}{\Delta t}$$

**Advantages**: Simple, good for high speeds
**Disadvantages**: Quantization noise at low speeds

**Method 2: Fixed Count Interval**
Measure time for fixed position change:

$$\omega = \frac{\Delta\theta}{\Delta t_{\text{measured}}}$$

**Advantages**: Better resolution at low speeds
**Disadvantages**: Variable update rate

**Method 3: Hybrid Approach**
Use both methods and switch based on speed

<!-- class="theory-concept" -->
**Filtering and Noise Rejection**

**Low-Pass Filter**: Smooth noisy velocity estimates

$$\omega_{\text{filtered}}[n] = \alpha \cdot \omega[n] + (1-\alpha) \cdot \omega_{\text{filtered}}[n-1]$$

where α ≈ 0.1-0.3

**Moving Average**:
$$\omega_{\text{avg}} = \frac{1}{N}\sum_{i=0}^{N-1}\omega[n-i]$$

### 3.4 Absolute vs. Incremental Encoders

<!-- class="theory-concept" -->
**Absolute Encoders**

Provide unique position code for each shaft angle:

**Encoding**:
- Gray code (prevents ambiguous transitions)
- Binary output (parallel or serial)
- No homing required on power-up

**Example**: 12-bit absolute encoder
- 4096 unique positions
- Resolution: 360° / 4096 = 0.088°

**Advantages**:
- Known position on startup
- No accumulated error

**Disadvantages**:
- More expensive
- More complex wiring (multi-bit parallel) or protocol (serial)

**Incremental Encoders**:
- Relative position only
- Require homing procedure
- Less expensive
- Standard in servo motors

---

## Module 4: Vision Sensors

### 4.1 Camera Fundamentals

<!-- class="theory-concept" -->
**Digital Image Formation**

Camera converts light into digital image:

**Image Sensor Types**:
- **CCD** (Charge-Coupled Device): High quality, expensive
- **CMOS** (Complementary Metal-Oxide-Semiconductor): Lower cost, power; modern standard

**Resolution**: Width × Height pixels (e.g., 640×480 VGA, 1920×1080 Full HD)

**Frame Rate**: Images per second (e.g., 30 fps, 60 fps)

**Color Encoding**:
- **Bayer Pattern**: RGGB filter array
- **Debayering**: Interpolation to full RGB
- **Grayscale**: Single intensity channel

### 4.2 Simple Vision Sensors for Robotics

<!-- class="theory-concept" -->
**Pixy2 Smart Camera**

Pixy2 is a dedicated vision sensor for robotics:

**Features**:
- On-board image processing (NXP LPC4330 processor)
- Color-based object tracking
- Line following algorithms
- I2C, SPI, UART interfaces
- Returns object coordinates, not full images

**Object Detection**:
1. Train on specific colors using teach button
2. Camera identifies blobs of trained colors
3. Returns: (x, y, width, height, signature) for each detected object

**Interface Example (I2C)**:
```cpp
Pixy2 pixy;
pixy.init();

pixy.ccc.getBlocks();
if (pixy.ccc.numBlocks) {
    Block largest = pixy.ccc.blocks[0];
    Serial.print("Object at (");
    Serial.print(largest.m_x);
    Serial.print(", ");
    Serial.print(largest.m_y);
    Serial.println(")");
}
```

**Advantages for Robotics**:
- Low latency (<50ms)
- Low CPU overhead (processing on camera)
- Easy interfacing
- Real-time tracking

<!-- class="alternative-approach" -->
**OpenMV Camera**

MicroPython-programmable camera module:

**Features**:
- ARM Cortex-M7 processor
- MicroPython scripting
- Image processing libraries (blob detection, QR codes, AprilTags)
- UART output or SD card logging

**Applications**:
- Vision-based navigation
- Object recognition
- Barcode/QR code reading
- Custom image processing algorithms

---

## Module 5: Multi-Sensor Integration

### 5.1 I2C Bus Management

<!-- class="theory-concept" -->
**Multi-Device I2C Systems**

**Address Conflicts**:
Some sensors have fixed addresses (e.g., MPU-6050 at 0x68 or 0x69)

**Solutions**:
1. **AD0 pin**: Some devices have address select pin
2. **I2C Multiplexer** (e.g., TCA9548A): Switch between 8 I2C channels
3. **Different sensor models**: Use sensors with different addresses

**I2C Multiplexer Example**:
```cpp
TCA9548A mux(0x70);

// Select channel 0
mux.selectChannel(0);
imu1.read();

// Select channel 1
mux.selectChannel(1);
imu2.read();
```

### 5.2 Sensor Dashboard Design

<!-- class="theory-concept" -->
**Real-Time Data Acquisition Architecture**

**Requirements**:
- Read multiple sensors at different rates
- Non-blocking execution
- Formatted output

**Design Pattern**:
```cpp
class SensorDashboard {
private:
    IMU imu;
    Encoder encoder;
    Pixy2 camera;

    unsigned long lastIMU, lastEncoder, lastCamera;
    const int IMU_RATE = 100;      // 100 Hz
    const int ENCODER_RATE = 50;   // 50 Hz
    const int CAMERA_RATE = 30;    // 30 Hz

public:
    void update() {
        unsigned long now = millis();

        if (now - lastIMU >= 1000/IMU_RATE) {
            imu.read();
            lastIMU = now;
        }

        if (now - lastEncoder >= 1000/ENCODER_RATE) {
            encoder.update();
            lastEncoder = now;
        }

        if (now - lastCamera >= 1000/CAMERA_RATE) {
            camera.getBlocks();
            lastCamera = now;
        }
    }

    void display() {
        Serial.println("=== Sensor Dashboard ===");
        Serial.print("IMU (Y,P,R): ");
        Serial.print(imu.yaw); Serial.print(", ");
        Serial.print(imu.pitch); Serial.print(", ");
        Serial.println(imu.roll);

        Serial.print("Encoder: ");
        Serial.println(encoder.position);

        Serial.print("Objects detected: ");
        Serial.println(camera.numBlocks);
        Serial.println("========================");
    }
};
```

### 5.3 Synchronization and Timestamping

<!-- class="theory-concept" -->
**Temporal Consistency**

Different sensors provide data at different times:

**Timestamping Strategy**:
```cpp
struct SensorReading {
    unsigned long timestamp;
    float value;
};

SensorReading imuReading;
imuReading.timestamp = micros();
imuReading.value = imu.read();
```

**Interpolation** for sensor fusion:
If sensor A updates at 100 Hz and sensor B at 50 Hz, interpolate B to match A's timestamps.

**Quiz: Sensor Integration**

An I2C bus with two devices using the same address can be resolved by:

[( )] Changing the baud rate
[(X)] Using an I2C multiplexer
[( )] Switching to SPI
[( )] Increasing pull-up resistors

For real-time sensor data acquisition, the preferred approach is:

[( )] Block on each sensor read sequentially
[(X)] Non-blocking reads with individual timing for each sensor
[( )] Read all sensors simultaneously
[( )] Use delays between each sensor

---

## Summary

This course established comprehensive sensor integration capabilities:

1. **Communication Protocols**: Examined I2C, SPI, and UART with tradeoffs and applications
2. **Inertial Sensors**: Understood IMU physics, sensor fusion, and orientation estimation
3. **Encoders**: Analyzed quadrature encoding for precision position feedback
4. **Vision**: Explored smart cameras for robot perception
5. **Integration**: Designed multi-sensor systems with real-time constraints

**Key Takeaways**:
- Protocol selection depends on speed, complexity, and number of devices
- IMUs require sensor fusion to overcome individual sensor limitations
- Encoders provide high-resolution position feedback essential for control
- Smart vision sensors offload processing from main microcontroller
- Multi-sensor systems demand non-blocking, rate-based architectures

**Next Steps**: HARDWARE-204 examines power systems, battery management, and safety engineering.

---

## Optional Laboratory Exercises

### Lab 1: Multi-Sensor I2C System (2 hours)

<!-- class="optional-exercise" -->
**Objective**: Build a system integrating IMU, time-of-flight sensor, and OLED display on shared I2C bus.

**Components**:
- MPU-6050 IMU (address 0x68)
- VL53L0X time-of-flight sensor (address 0x29)
- SSD1306 OLED display (address 0x3C)
- STM32 or Arduino

**Tasks**:
1. Wire all three devices to I2C bus (SDA, SCL, power, ground)
2. Verify each device responds (I2C scanner sketch)
3. Read orientation from IMU (use library with sensor fusion)
4. Read distance from ToF sensor
5. Display both readings on OLED
6. Update at 10 Hz

<!-- class="exercise-tip" -->
**Tips**:
- Use 4.7kΩ pull-up resistors on SDA and SCL
- Initialize devices in setup()
- Use non-blocking reads in loop()
- Format OLED output for readability:
  ```
  Pitch: 12.3°
  Roll:  -5.2°
  Dist:  47 cm
  ```

**Target Performance**:
- All sensors communicate without conflicts
- Stable orientation readings (filter noise)
- Distance updates smoothly
- Display refresh at ≥10 Hz

### Lab 2: Quadrature Encoder Velocity Control (2.5 hours)

<!-- class="optional-exercise" -->
**Objective**: Implement closed-loop velocity control using encoder feedback.

**Components**:
- DC motor with quadrature encoder
- H-bridge motor driver
- Arduino or STM32
- Potentiometer (setpoint input)

**Tasks**:
1. Connect encoder A/B channels to interrupt pins
2. Implement quadrature decoding ISRs
3. Calculate velocity from position derivative
4. Implement low-pass filter on velocity
5. Create PI controller:
   $$u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau$$
   where e(t) = desired velocity - actual velocity
6. Output PWM to motor driver
7. Plot actual vs. desired velocity

<!-- class="exercise-advanced" -->
**Advanced Extensions**:
- Implement feed-forward term for faster response
- Add anti-windup for integral term
- Tune PID gains using Ziegler-Nichols method
- Create acceleration ramp for setpoint changes

**Analysis Questions**:
- How does filter time constant affect velocity noise vs. response speed?
- What encoder resolution is needed for smooth low-speed control?
- How do P and I gains affect stability and steady-state error?

### Lab 3: Sensor Fusion Dashboard (2 hours)

<!-- class="optional-exercise" -->
**Objective**: Build a complete multi-sensor acquisition and display system.

**Components**:
- MPU-6050 or MPU-9250 IMU
- Quadrature encoder (motor or standalone)
- Pixy2 or similar vision sensor
- STM32 (for performance)

**Tasks**:
1. Interface all sensors (I2C for IMU, interrupts for encoder, I2C/SPI for camera)
2. Read each sensor at appropriate rate:
   - IMU: 100 Hz
   - Encoder: 50 Hz
   - Camera: 20 Hz
3. Implement sensor fusion for IMU orientation (complementary or Kalman filter)
4. Create formatted serial output updating at 10 Hz
5. Calculate and display CPU utilization

**Output Format**:
```
=== Robot Sensor Dashboard ===
Time: 12.345 s
IMU Orientation (Y,P,R): [45.2°, 12.3°, -5.8°]
Encoder Position: 1234 counts (68.3°)
Encoder Velocity: 125.4 RPM
Objects Detected: 2
  Object 0: (150, 120), size 45×30
  Object 1: (210, 145), size 32×28
CPU Utilization: 42%
==============================
```

<!-- class="exercise-tip" -->
**CPU Utilization Calculation**:
```cpp
unsigned long activeTime = 0;
unsigned long periodStart = millis();

void loop() {
    unsigned long taskStart = micros();

    // Do work
    updateSensors();

    activeTime += micros() - taskStart;

    if (millis() - periodStart >= 1000) {
        float cpuUtil = 100.0 * activeTime / (1000.0 * 1000);
        Serial.print("CPU: ");
        Serial.print(cpuUtil);
        Serial.println("%");

        activeTime = 0;
        periodStart = millis();
    }
}
```

---

*Version 2.0.0 - Academic curriculum emphasizing theoretical foundations with optional practical exercises*
