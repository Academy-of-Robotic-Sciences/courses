<!--
author:   Dr. Elena Martinez
email:    elena.martinez@robotcampus.dev
version:  2.0.0

language: en
narrator: US English Female

comment:  System Integration and Assembly: Capstone project synthesizing mechanical assembly, electrical integration, calibration, and system-level testing for complete robot construction.

icon:     https://robotcampus.dev/logos/hardware-205.png

mode:     Textbook

link:     https://robotcampus.dev/styles/course-styles.css

import:   https://raw.githubusercontent.com/LiaTemplates/Pyodide/master/README.md
-->

# HARDWARE-205: System Integration and Assembly

> **System integration transforms individual components into a cohesive robotic platform.**

## Course Overview

| | |
|---|---|
| **Course Code** | HARDWARE-205 |
| **Duration** | 8 hours |
| **Level** | Capstone Project |
| **Prerequisites** | HARDWARE-201, 202, 203, 204 |
| **Theory-Practice** | 40% theory, 60% guided assembly |

---

## Learning Objectives

Upon completion of this course, students will be able to:

### Theoretical Understanding

- Explain systems engineering principles for mechatronic integration
- Understand tolerance analysis and mechanical assembly sequencing
- Describe calibration theory and sensor-actuator correspondence
- Analyze system-level failure modes and diagnostic strategies
- Apply verification and validation methodologies

### Practical Skills

- Assemble complex mechanical structures following technical documentation
- Integrate electronic subsystems with proper cable management
- Calibrate multi-degree-of-freedom robotic systems
- Diagnose and troubleshoot hardware faults systematically
- Conduct acceptance testing and performance verification

---

## Module 1: Systems Engineering Fundamentals

### 1.1 The V-Model of Development

<!-- class="theory-concept" -->
**System Development Lifecycle**

The V-Model represents the relationship between development and testing:

```
Requirements ──────────────────→ Acceptance Testing
     ↓                                   ↑
System Design ──────────────→ System Testing
     ↓                                   ↑
Subsystem Design ────────→ Integration Testing
     ↓                                   ↑
Component Design ──────→ Unit Testing
     ↓                           ↑
Implementation ─────────────────┘
```

**Left Side (Development)**:
- Requirements: What the system must do
- Design: How the system will be structured
- Implementation: Building the components

**Right Side (Verification)**:
- Unit Testing: Verify individual components
- Integration: Verify interfaces between subsystems
- System Testing: Verify complete system
- Acceptance: Verify customer requirements

<!-- class="historical-note" -->
**Evolution of Systems Engineering**

Systems engineering emerged from complex projects:

**1940s-1950s**: Bell Labs telephone systems, early missile programs
**1960s**: NASA Apollo program formalized systems engineering
**1970s**: Commercial aviation (Boeing 747) applied rigorous SE methods
**1980s**: ISO 9000 standards codified quality systems
**1990s**: Embedded systems in automotive (ABS, airbags)
**2000s-Present**: Agile methods adapted for hardware (rapid prototyping, iterative integration)

### 1.2 Interface Specifications

<!-- class="theory-concept" -->
**Defining System Boundaries**

Interfaces define how subsystems interact:

**Mechanical Interfaces**:
- Mounting points and hole patterns
- Dimensional tolerances
- Load-bearing capacity
- Material compatibility

**Electrical Interfaces**:
- Voltage levels and current capacity
- Connector types and pinouts
- Communication protocols (I2C, SPI, UART)
- Grounding and shielding

**Software Interfaces**:
- Function signatures and data types
- Communication formats (serial, binary, JSON)
- Timing constraints and update rates
- Error handling conventions

**Example: Motor-Controller Interface Specification**

```
Mechanical:
- Mounting: NEMA 17 bolt pattern (31mm × 31mm, M3 screws)
- Shaft: 5mm diameter, 20mm length
- Torque: 0.4 Nm holding, 0.3 Nm running

Electrical:
- Power: 12V ±10%, 1.5A peak per phase
- Control: Step/Direction signals (3.3V or 5V logic)
- Feedback: Quadrature encoder, 400 CPR, 5V open-collector

Software:
- Step pulse: Minimum 1μs width, maximum 200 kHz
- Direction: Must be stable 5μs before step edge
- Encoder: Phase offset 90° ±5°
```

### 1.3 Tolerance Analysis

<!-- class="theory-concept" -->
**Cumulative Tolerances in Assembly**

Manufacturing tolerances accumulate in assemblies:

**Worst-Case Tolerance Stacking**:

For linear dimension chain:
$$L_{total} = \sum_{i=1}^{n} L_i \pm \sum_{i=1}^{n} \delta_i$$

where:
- $L_i$ = nominal dimension of component i
- $\delta_i$ = tolerance of component i

**Example**: 5-link robot arm
- Each link: 200mm ± 0.5mm
- Worst case length: $5 \times 200 \pm 5 \times 0.5 = 1000 \pm 2.5$ mm

**Statistical Tolerance (RSS - Root Sum Square)**:

Assumes independent, normally-distributed errors:
$$\delta_{total} = \sqrt{\sum_{i=1}^{n} \delta_i^2}$$

For the same robot arm:
$$\delta_{total} = \sqrt{5 \times 0.5^2} = 1.12 \text{ mm}$$

Much tighter than worst-case, more realistic for manufactured parts.

<!-- class="theory-concept" -->
**Tolerance Allocation Strategy**

**Critical Dimensions**: Tight tolerances (expensive)
- Motor shaft alignment: ±0.05mm
- Bearing fits: ±0.01mm
- Gear mesh clearance: ±0.02mm

**Non-Critical Dimensions**: Loose tolerances (economical)
- Cosmetic panels: ±0.5mm
- Cable routing clearance: ±2mm
- Access hole locations: ±1mm

**Design for Manufacturing**:
- Use standard sizes (metric fasteners: M3, M4, M5)
- Avoid tight tolerances across joints (use adjustment)
- Specify tolerances only where functionally necessary

---

## Module 2: Mechanical Assembly

### 2.1 Assembly Sequencing

<!-- class="theory-concept" -->
**Determining Assembly Order**

**Constraints**:
1. **Accessibility**: Must be able to reach fasteners/connectors
2. **Precedence**: Some parts must be installed before others
3. **Stability**: Partially assembled structure must be stable
4. **Tooling**: Special tools or fixtures may be needed

**Assembly Sequence Example (Robot Arm)**:

```
1. Base mounting plate
   ├─ 2. Base servo motor
   │    └─ 3. First joint bracket
   │         ├─ 4. Shoulder servo motor
   │         │    └─ 5. Upper arm link
   │         │         ├─ 6. Elbow servo motor
   │         │         │    └─ 7. Forearm link
   │         │         │         ├─ 8. Wrist servo motor
   │         │         │         │    └─ 9. Gripper assembly
```

**Assembly Process Flow**:
- Identify subassemblies that can be built separately
- Assemble subunits off the main structure
- Test subassemblies before integration
- Final assembly brings subassemblies together

### 2.2 Fastener Selection and Torque

<!-- class="theory-concept" -->
**Threaded Fasteners**

**Fastener Specifications**:
- **Thread**: Metric (M3, M4, M5) or Imperial (#4-40, 1/4-20)
- **Material**: Steel (strong), stainless (corrosion-resistant), nylon (electrical insulation)
- **Head**: Socket cap, button head, flat head, pan head
- **Drive**: Hex socket (Allen), Phillips, Torx

**Torque Specifications**:

Prevents under-tightening (loosening) and over-tightening (stripping):

| Fastener | Torque (Nm) | Torque (in-lb) |
|----------|-------------|----------------|
| M2       | 0.08-0.10   | 0.7-0.9        |
| M2.5     | 0.15-0.18   | 1.3-1.6        |
| M3       | 0.35-0.40   | 3.1-3.5        |
| M4       | 0.80-0.90   | 7.1-8.0        |
| M5       | 1.5-1.8     | 13-16          |

**Use Threadlocker**:
- **Blue (Medium)**: Removable, most common
- **Red (High)**: Permanent, high vibration
- **Green (Wicking)**: Thin, applied after assembly

### 2.3 Bearing Installation and Alignment

<!-- class="theory-concept" -->
**Bearing Types and Fits**

**Ball Bearings**:
- Support radial and limited axial loads
- Low friction, high speed
- Require precise alignment

**Press Fit vs. Slip Fit**:
- **Inner race to shaft**: Press fit (interference)
- **Outer race to housing**: Slip fit (clearance)
- Allows thermal expansion without binding

**Installation**:
- Apply even pressure to fitted race only
- Use arbor press or bearing installation tool
- Never hammer directly on bearing

**Alignment**:
- Shaft runout: <0.05mm typical
- Parallel alignment of coupled shafts: <0.1mm
- Angular alignment: <0.5° typical
- Use shims or adjustable mounts for fine tuning

---

## Module 3: Electrical Integration

### 3.1 Cable Routing and Management

<!-- class="theory-concept" -->
**Cable Management Principles**

**Separation**:
- **Power cables** (12V motors): Keep separate from signal cables
- **High-frequency signals** (PWM, encoders): Twisted-pair or shielded
- **Analog sensors**: Away from switching power supplies

**Routing**:
- Follow structure (avoid free spans)
- Avoid sharp bends (minimum radius: 10× cable diameter)
- Keep away from moving parts
- Allow service loops at connections

**Strain Relief**:
- At every connector
- At panel pass-throughs
- Use cable glands or grommets
- Avoid tension on solder joints

**Securing Methods**:
- Cable ties (not overtightened)
- Adhesive cable mounts
- Braided cable sleeve
- Cable chains for moving joints

### 3.2 Grounding and Noise Reduction

<!-- class="theory-concept" -->
**Ground Loops and Star Grounding**

**Ground Loop Problem**:
Multiple ground paths create current loops that induce noise.

**Star Grounding**:
All subsystem grounds connect to single point:

```
Power Supply Ground (Star Point)
    ├─── Motor Controller Ground
    ├─── Microcontroller Ground
    ├─── Sensor Board Ground
    └─── Safety Circuit Ground
```

**Single-Point Grounding**:
- Analog circuits: Separate analog and digital grounds, connect at one point only
- Power grounds: Heavy gauge wire to minimize resistance
- Signal grounds: Follow signal routing

**Shielding**:
- Connect shield at one end only (typically source)
- Shields carry displacement current, not signal current
- Use shielded twisted pair for analog sensors

### 3.3 Connector Assembly and Testing

<!-- class="theory-concept" -->
**Crimping vs. Soldering**

**Crimping** (Preferred for field servicing):
- Gas-tight connection
- Vibration-resistant
- Requires correct crimp tool and technique
- Inspect: Wire strands fully inserted, insulation not crimped

**Soldering** (Permanent connections):
- Strong electrical and mechanical connection
- Not easily field-serviceable
- Risk of cold joints
- Inspect: Shiny, concave fillet, complete wetting

**Continuity Testing**:
After assembly, test each connection:

```
1. Verify power rails (12V, 5V, 3.3V) at each connector
2. Check ground continuity (<1Ω)
3. Verify signal connectivity with multimeter
4. Test under light mechanical stress (wiggle test)
5. Document any intermittent connections
```

---

## Module 4: Calibration and Testing

### 4.1 Sensor Calibration

<!-- class="theory-concept" -->
**Zero-Offset Calibration**

Many sensors require offset correction:

**Accelerometer**:
At rest on level surface, should read (0, 0, -1g)

$$a_{calibrated} = a_{raw} - a_{offset}$$

**Gyroscope**:
At rest, should read (0, 0, 0)

$$\omega_{calibrated} = \omega_{raw} - \omega_{bias}$$

Measure bias over 10 seconds, compute average.

**Magnetometer (Hard-Iron Calibration)**:
Rotate sensor through 360° in all axes, record min/max:

$$B_{calibrated} = B_{raw} - \frac{B_{max} + B_{min}}{2}$$

<!-- class="theory-concept" -->
**Actuator Calibration**

**Servo Position Calibration**:

Map pulse width to actual angle:

```
Procedure:
1. Send 1000μs pulse → Measure actual angle → θ_min
2. Send 2000μs pulse → Measure actual angle → θ_max
3. Compute scaling: θ = θ_min + (pulse - 1000) × (θ_max - θ_min) / 1000
```

**Encoder Index Pulse Homing**:

Find absolute zero position:

```
Procedure:
1. Slowly rotate motor in one direction
2. Detect index pulse (once per revolution)
3. Stop motor
4. Reset encoder count to 0
5. Record this as home position
```

### 4.2 End-to-End System Testing

<!-- class="theory-concept" -->
**Functional Testing Sequence**

**Power-On Sequence**:
```
1. Visual inspection (no loose wires, correct connections)
2. Measure battery voltage (within spec)
3. Connect power, observe LED indicators
4. Measure voltage rails (12V, 5V, 3.3V)
5. Verify no abnormal heating or smells
6. Test E-Stop function
```

**Communication Testing**:
```
1. Serial monitor shows boot messages
2. I2C scan detects all sensors
3. Query each sensor, verify reasonable values
4. Test encoder counts (rotate by hand)
5. Camera detects test object
```

**Actuator Testing**:
```
1. Command each motor to small angle (10°)
2. Verify motion in correct direction
3. Check for unusual noise or vibration
4. Test full range of motion
5. Verify encoder feedback matches command
```

### 4.3 Performance Characterization

<!-- class="theory-concept" -->
**Quantitative Performance Metrics**

**Repeatability**:
Command same position multiple times, measure variance

$$\sigma_{position} = \sqrt{\frac{1}{N-1}\sum_{i=1}^{N}(\theta_i - \bar{\theta})^2}$$

Target: σ < 1° for educational robot

**Accuracy**:
Difference between commanded and actual position

$$\epsilon_{accuracy} = |\theta_{commanded} - \theta_{actual}|$$

Measured using external instrumentation (protractor, calipers)

**Speed**:
Time to complete standard motion

$$v = \frac{\Delta\theta}{\Delta t}$$

**Load Capacity**:
Maximum payload before performance degradation
- Measure position error vs. payload mass
- Determine safe operating limit

---

## Module 5: Diagnostic Methods

### 5.1 Systematic Troubleshooting

<!-- class="theory-concept" -->
**Fault Isolation Strategy**

**Divide and Conquer**:

```
System not working
├─ Power subsystem?
│  ├─ Battery voltage OK? → Test with multimeter
│  ├─ Regulators outputting correct voltages? → Test rails
│  └─ Fuses intact? → Visual + continuity test
│
├─ Communication subsystem?
│  ├─ Microcontroller booting? → Check serial messages
│  ├─ Sensors responding? → I2C scan
│  └─ Wiring correct? → Check pinouts
│
└─ Actuator subsystem?
   ├─ Motors receiving power? → Measure voltage at terminals
   ├─ Control signals present? → Oscilloscope on PWM
   └─ Mechanical binding? → Manual movement test
```

**Half-Splitting Method**:
Test midpoint of system, determine which half has fault, repeat.

### 5.2 Common Failure Modes

<!-- class="theory-concept" -->
**Electrical Failures**

**Short Circuit**:
- Symptoms: Blown fuse, regulator hot, battery drains quickly
- Diagnosis: Disconnect loads, measure current draw in sections
- Common causes: Stray solder, wire insulation damage, reversed polarity

**Open Circuit**:
- Symptoms: No response, intermittent operation
- Diagnosis: Continuity test, wiggle test
- Common causes: Cold solder joint, broken wire, loose connector

**Voltage Drop**:
- Symptoms: Motors run slow, brownouts under load
- Diagnosis: Measure voltage at load during operation
- Common causes: Undersized wire, high resistance connection, weak battery

<!-- class="theory-concept" -->
**Mechanical Failures**

**Binding**:
- Symptoms: Motor stalls, high current, jerky motion
- Diagnosis: Manual movement test, check alignment
- Common causes: Misalignment, interference, over-constrained linkage

**Backlash**:
- Symptoms: Position inaccuracy, oscillation
- Diagnosis: Move actuator, observe delay in output
- Common causes: Worn gears, loose fasteners, flexible coupling

**Vibration**:
- Symptoms: Noise, reduced lifespan, position error
- Diagnosis: Observation, measure with accelerometer
- Common causes: Imbalance, resonance, loose parts

### 5.3 Documentation and Iteration

<!-- class="theory-concept" -->
**Build Documentation**

Essential records for debugging and future builds:

**As-Built Documentation**:
- Schematic with actual component values
- Photos of assembly at key stages
- Wiring diagram with color codes
- Calibration constants and procedures
- Bill of materials (BOM) with suppliers

**Test Results**:
- Checklist of functional tests passed/failed
- Performance metrics measured
- Issues encountered and resolutions
- Modifications made during assembly

**Maintenance Log**:
- Operating hours
- Failures and repairs
- Component replacements
- Performance degradation over time

---

## Module 6: The SO-101 Assembly Project

### 6.1 Project Overview

<!-- class="theory-concept" -->
**SO-101 Educational Robot Arm**

**Specifications**:
- **Degrees of Freedom**: 5 (base rotation, shoulder, elbow, wrist pitch, gripper)
- **Reach**: 400mm horizontal
- **Payload**: 100g at full extension
- **Repeatability**: ±2°
- **Controller**: STM32F103 (Blue Pill)
- **Power**: 3S LiPo (11.1V nominal)

**Subsystems**:
1. **Mechanical**: 3D-printed links, aluminum brackets, ball bearings
2. **Actuation**: 5× MG996R servos
3. **Sensing**: MPU-6050 IMU (base orientation), 5× encoder feedback
4. **Power**: Multi-voltage distribution (12V, 5V), E-Stop circuit
5. **Control**: STM32 running servo control firmware

### 6.2 Assembly Procedure

<!-- class="optional-exercise" -->
**Phase 1: Mechanical Assembly (3 hours)**

**Base Assembly**:
1. Install base servo into mounting plate
2. Attach base rotation bearing
3. Mount first joint bracket to servo horn
4. Verify smooth 360° rotation

**Arm Linkage**:
5. Install shoulder servo in joint bracket
6. Attach upper arm link with through-shaft bearing
7. Install elbow servo at upper arm end
8. Attach forearm link
9. Verify no binding through full range of motion

**End Effector**:
10. Install wrist pitch servo
11. Mount gripper base plate
12. Install gripper servo
13. Attach gripper jaws with linkage

**Quality Checks**:
- All fasteners torqued to specification
- Bearings rotate freely without play
- No interference between links
- Center of mass within base footprint

<!-- class="optional-exercise" -->
**Phase 2: Electrical Integration (3 hours)**

**Power System**:
1. Mount power distribution board to base
2. Connect battery via XT60 connector
3. Verify 12V and 5V outputs
4. Install E-Stop button and relay
5. Test E-Stop function

**Microcontroller**:
6. Mount STM32 Blue Pill on mounting plate
7. Connect 5V power from distribution board
8. Connect programmer (ST-Link)
9. Upload and test blink program

**Sensors**:
10. Mount IMU on base (aligned with axes)
11. Connect I2C (SDA, SCL) to STM32
12. Run I2C scan, verify address 0x68
13. Test IMU orientation readings

**Actuators**:
14. Connect each servo to power (12V, GND)
15. Connect servo signal wires to STM32 PWM pins
16. Connect encoder signals to interrupt-capable pins
17. Secure all wiring with cable ties

**Wiring Harness**:
18. Bundle power wires separately from signals
19. Add strain relief at all connectors
20. Label all wires at both ends
21. Conduct continuity tests

<!-- class="optional-exercise" -->
**Phase 3: Calibration and Testing (2 hours)**

**Software Setup**:
1. Clone firmware repository
2. Configure pin assignments in code
3. Compile and upload firmware
4. Open serial monitor at 115200 baud

**Sensor Calibration**:
5. Run IMU calibration routine (10 sec stationary)
6. Record offset values
7. Manually rotate each joint, verify encoder counts
8. Run encoder homing routine

**Servo Calibration**:
9. Command each servo to 90° (mid-range)
10. Measure actual angle with protractor
11. Adjust software scaling factors
12. Verify end stops (0° and 180°)
13. Test full range, check for binding

**System Tests**:
14. Run diagnostic program (lights each servo in sequence)
15. Command predefined poses
16. Test gripper open/close
17. Verify E-Stop during motion
18. Measure position repeatability (±2°)

**Final Inspection**:
19. Check all fasteners are secure
20. Verify no loose wires
21. Test full range of motion without load
22. Test with 50g payload at full extension
23. Document any issues or adjustments

### 6.3 Acceptance Criteria

<!-- class="theory-concept" -->
**Pass/Fail Checklist**

**Safety** (Must Pass All):
- [ ] E-Stop immediately cuts motor power
- [ ] No exposed high voltage (>30V)
- [ ] No sharp edges or pinch points
- [ ] Stable (does not tip under operation)

**Functionality** (Must Pass ≥80%):
- [ ] All joints move through full range
- [ ] Position repeatability <3°
- [ ] Gripper holds 50g object
- [ ] IMU reports orientation within ±5°
- [ ] All encoders report position
- [ ] Servo response time <500ms

**Quality** (Must Pass ≥80%):
- [ ] All fasteners torqued correctly
- [ ] Wiring is neat and labeled
- [ ] No mechanical binding
- [ ] No unusual noise or vibration
- [ ] Power consumption <3A at 12V (idle)

**Documentation** (Must Pass All):
- [ ] Assembly photos at key stages
- [ ] Wiring diagram with color codes
- [ ] Calibration constants recorded
- [ ] Test results documented

---

## Summary

This capstone course synthesized all hardware engineering concepts:

1. **Systems Engineering**: Applied V-model, interface specs, and tolerance analysis
2. **Mechanical Assembly**: Sequenced assembly, selected fasteners, aligned bearings
3. **Electrical Integration**: Routed cables, managed grounding, assembled connectors
4. **Calibration**: Calibrated sensors and actuators for accurate control
5. **Testing**: Conducted functional tests and performance characterization
6. **Troubleshooting**: Applied systematic diagnostic methods
7. **Integration**: Assembled complete SO-101 robot from components

**Key Takeaways**:
- System integration requires careful planning and documentation
- Interface specifications prevent integration failures
- Calibration transforms raw hardware into precision instruments
- Systematic testing catches problems before deployment
- Good mechanical assembly and wiring are foundations of reliability

**Certification**: Students who successfully complete the SO-101 assembly and pass the acceptance criteria receive the **Hardware Engineering Specialization Certificate**.

**Next Steps**:
- **Software Engineering Track**: Learn to program the robot you built
- **Advanced Projects**: Design custom robots for specific applications
- **Mentorship**: Guide the next cohort through hardware assembly

---

## Optional Advanced Extensions

### Extension 1: Force Feedback Integration (3 hours)

<!-- class="exercise-advanced" -->
**Objective**: Add force sensing to gripper for controlled grasping.

**Components**:
- FSR (Force-Sensitive Resistor)
- Analog input channel on STM32
- Modified gripper jaw

**Tasks**:
1. Design mounting for FSR in gripper jaw
2. Interface FSR to ADC
3. Calibrate force vs. voltage curve
4. Implement PID control for target grip force
5. Test with objects of varying fragility (foam, plastic, metal)

**Theory**:
Force control prevents crushing delicate objects while ensuring secure grip.

### Extension 2: Wireless Control Interface (2 hours)

<!-- class="exercise-advanced" -->
**Objective**: Add Bluetooth or WiFi control for untethered operation.

**Components**:
- ESP32 or Bluetooth module (HC-05)
- Mobile app or web interface

**Tasks**:
1. Interface wireless module to STM32 (UART)
2. Define serial command protocol
3. Implement command parser on STM32
4. Create control interface (app or web page)
5. Test wireless latency and range

### Extension 3: Vision-Guided Pick and Place (4 hours)

<!-- class="exercise-advanced" -->
**Objective**: Integrate vision sensor for autonomous object manipulation.

**Components**:
- Pixy2 camera
- Mounting bracket
- Colored objects

**Tasks**:
1. Mount camera with field-of-view covering workspace
2. Train camera on object signatures
3. Implement coordinate transformation (camera frame → robot base frame)
4. Calculate inverse kinematics for reach point
5. Implement pick-and-place routine:
   - Detect object location
   - Plan trajectory
   - Move to object
   - Grasp
   - Move to target
   - Release

**Theory**:
Coordinate transformations:

$$\begin{bmatrix} x_{robot} \\ y_{robot} \end{bmatrix} = R(\theta) \begin{bmatrix} x_{camera} \\ y_{camera} \end{bmatrix} + \begin{bmatrix} t_x \\ t_y \end{bmatrix}$$

where R(θ) is rotation matrix, (t_x, t_y) is camera offset.

---

*Version 2.0.0 - Academic curriculum emphasizing theoretical foundations with optional practical exercises*
