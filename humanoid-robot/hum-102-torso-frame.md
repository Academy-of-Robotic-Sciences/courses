---
id: hum-102-torso-frame
title: "HUM-102: Structural Design and Torso Assembly"
sidebar_position: 3
version: 2.0.0
link: https://raw.githubusercontent.com/Academy-of-Robotic-Sciences/courses/main/course-styles.css
---

# HUM-102: Structural Design and Torso Assembly

> Application of mechanical design principles to construct a rigid, lightweight humanoid torso with integrated power distribution and computational subsystems.

## Course Overview

| Parameter | Details |
|---|---|
| **Course Code** | HUM-102 |
| **Duration** | 8 contact hours |
| **Format** | Theory (40%) + Guided Project (60%) |
| **Prerequisites** | HUM-101, basic CAD knowledge |
| **Deliverables** | Assembled torso, CAD models, power system documentation |

---

## Learning Objectives

### Theoretical Knowledge
- Apply structural analysis to humanoid frame design
- Calculate center of mass and moment of inertia distributions
- Understand power system architecture for mobile robots
- Analyze thermal management requirements

### Practical Skills
- Create parametric CAD models for robotic structures
- Fabricate components using 3D printing and machining
- Design and wire multi-voltage power distribution systems
- Integrate microcontrollers for hierarchical control

---

## Part 1: Theoretical Foundations (3 hours)

### Structural Design Principles

<div class="theory-concept">

**Design Requirements for Humanoid Torsos**

The torso serves as the structural foundation, housing:
1. **Computational systems** (Raspberry Pi, Arduino)
2. **Power distribution** (batteries, voltage regulators)
3. **Mounting points** for 20+ servo motors
4. **Cable management** infrastructure

Key design constraints:
- **Mass budget:** < 500g to maintain favorable mass distribution
- **Rigidity:** Minimal flexure under 4 kg total robot mass
- **Accessibility:** Serviceable components without complete disassembly
- **Thermal:** Passive cooling for continuous operation

</div>

#### Structural Analysis

**Beam Bending:** For a cantilever beam (arm attachment point) of length $L$ with point load $F$:

$$\delta = \frac{FL^3}{3EI}$$

Where:
- $\delta$: Deflection at free end
- $E$: Young's modulus (PLA: ~3.5 GPa, Aluminum: ~69 GPa)
- $I$: Second moment of area

For rectangular cross-section: $I = \frac{bh^3}{12}$

**Example:** PLA beam, 100mm × 10mm × 3mm, 200g load:
$$\delta = \frac{(0.2 \times 9.81) \times 0.1^3}{3 \times 3.5 \times 10^9 \times 2.25 \times 10^{-11}} \approx 0.8 \text{ mm}$$

<div class="exercise-tip">

**Material Selection Trade-offs**

| Material | Density | Strength | Stiffness | Fabrication |
|----------|---------|----------|-----------|-------------|
| PLA (3D printed) | 1.25 g/cm³ | Moderate | Low | Excellent |
| Aluminum 6061 | 2.70 g/cm³ | High | High | Machining required |
| Carbon Fiber | 1.60 g/cm³ | Very High | Very High | Specialized |

For student projects, hybrid approach optimal: PLA for custom joints, aluminum extrusions for primary structure.

</div>

#### Center of Mass Calculation

For system with $n$ components:

$$\mathbf{r}_{CoM} = \frac{\sum_{i=1}^n m_i \mathbf{r}_i}{\sum_{i=1}^n m_i}$$

**Critical for bipedal stability:** CoM must project within support polygon during stance.

### Power System Architecture

<div class="theory-concept">

**Multi-Voltage Power Distribution**

Humanoid robots require multiple voltage rails:

1. **12V Rail:** High-current for servo motors (up to 10A peak)
2. **5V Rail:** Logic power for microcontrollers (2-3A)
3. **Battery:** 3S LiPo (11.1V nominal, 12.6V fully charged)

Power budget analysis:
- 20 servos × 1A average = 20A @ 12V = 240W
- Controllers: 2A @ 5V = 10W
- **Total:** ~250W peak, ~100W continuous

</div>

#### Battery Selection

Energy capacity required for 45-minute operation:

$$E = P \times t = 100W \times 0.75h = 75 Wh$$

At 11.1V: $C = \frac{75Wh}{11.1V} = 6.75 Ah$

**Practical choice:** 3S 2200mAh LiPo provides ~25Wh (20 minutes active use)

#### Voltage Regulation

Buck converter efficiency:

$$\eta = \frac{V_{out} \times I_{out}}{V_{in} \times I_{in}}$$

For 5V @ 2A output from 12V input at 85% efficiency:
$$I_{in} = \frac{5V \times 2A}{12V \times 0.85} = 0.98A$$

### Knowledge Check Quiz

1. **Question:** What is the primary advantage of aluminum over PLA for structural members?
   - (a) Lower mass
   - (b) Higher stiffness
   - (c) Easier fabrication
   - (d) Lower cost

   **Answer:** (b) - Aluminum has ~20× higher elastic modulus

2. **Question:** For a 4kg humanoid with torso at 30cm height, hip at 0cm, what is the contribution of torso to CoM height if torso mass is 0.5kg?
   - (a) 15 cm
   - (b) 3.75 cm
   - (c) 30 cm
   - (d) 7.5 cm

   **Answer:** (b) - $(0.5kg \times 30cm) / 4kg = 3.75cm$ contribution

---

## Part 2: CAD Design and Fabrication (2 hours)

### Parametric CAD Modeling

#### Design Parameters

```python
# Parametric torso design specification
torso_height = 300      # mm
torso_width = 180       # mm (shoulder spacing)
torso_depth = 100       # mm (front-to-back)
wall_thickness = 3      # mm (3D printed walls)
hip_spacing = 100       # mm (between leg attachment points)

# Component integration
battery_volume = [120, 60, 40]  # mm (length, width, height)
pi_footprint = [85, 56, 17]     # mm (Raspberry Pi 4 dimensions)
servo_mount_spacing = 30        # mm (mounting hole centers)
```

<div class="alternative-approach">

**CAD Software Options**

- **FreeCAD:** Open-source, parametric, Python scriptable (recommended for students)
- **Fusion 360:** Professional features, free for education, cloud-based
- **SolidWorks:** Industry standard, available in university labs
- **OnShape:** Browser-based, collaborative features

All can export STL files for 3D printing.

</div>

#### Design Workflow

1. **Create base structure** with parametric dimensions
2. **Add mounting features:** Servo bracket holes, standoffs for electronics
3. **Design cable routing:** Channels for wires, zip-tie anchor points
4. **Integrate access panels:** Removable sections for battery/electronics
5. **Validate clearances:** Ensure no servo collision during full range of motion

### Guided Project: Torso Assembly (3 hours)

#### Phase 1: Component Fabrication (90 min)

**3D Printing Guidelines:**

Critical print parameters:
- **Layer height:** 0.2mm (balance of speed and strength)
- **Wall thickness:** 3-4 perimeters for structural parts
- **Infill:** 30% gyroid pattern (optimal strength-to-weight)
- **Supports:** Necessary for overhangs > 45°

**Aluminum cutting** (if using extrusions):
- 2× 300mm vertical spines
- 2× 180mm shoulder cross-braces
- 4× 100mm servo mounting rails

#### Phase 2: Power System Integration (60 min)

**Battery Management System (BMS):**

```
[3S LiPo Battery] → [BMS Protection Board] → [Main Power Switch]
         ↓                                            ↓
    [Balance Plug]                              [Power Distribution]
                                                      ↓
                                           ┌──────────┴──────────┐
                                           ↓                     ↓
                                      [12V Bus]            [Buck Converter]
                                           ↓                     ↓
                                     [Servo Power]          [5V Logic]
```

**Wiring specifications:**
- **12V bus:** 18 AWG wire (minimum), rated for 10A continuous
- **5V distribution:** 22 AWG acceptable for logic
- **Fusing:** 10A fuse on main 12V line, 3A on 5V rail

<div class="exercise-tip">

**Safety-Critical Wiring Practices**

1. **Polarity protection:** Diodes or keyed connectors
2. **Strain relief:** Secure wires before solder joints
3. **Color coding:** Red (+), Black (GND), White (signal)
4. **Testing sequence:**
   - Verify voltages with no load
   - Test with single servo before full connection
   - Monitor current during initial power-up

</div>

#### Phase 3: Controller Integration (30 min)

**Computational Architecture:**

```python
# Controller mounting and communication

Raspberry Pi 4 (High-level control):
- ROS2 nodes for planning and vision
- Python for AI/ML algorithms
- USB/Ethernet communication

Arduino Mega 2560 (Low-level real-time):
- Servo PWM generation (54 digital pins)
- Sensor reading (100 Hz update rate)
- Serial communication with Pi (115200 baud)

Communication protocol:
Pi → Arduino: Joint angle commands
Arduino → Pi: Sensor data, status
```

**Test procedure:**

```cpp
// Arduino communication test
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if(Serial.available()) {
    char cmd = Serial.read();
    if(cmd == 'H') {
      Serial.println("Arduino Ready");
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }
}
```

```python
# Raspberry Pi test script
import serial
import time

arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Allow Arduino to reset

arduino.write(b'H')
response = arduino.readline().decode('utf-8').strip()

if response == "Arduino Ready":
    print("Communication established!")
else:
    print(f"Unexpected response: {response}")
```

---

## Part 3: Validation and Documentation (1 hour)

### Structural Testing

**Load testing procedure:**

1. **Static load:** Apply 4kg weight, measure deflection (target: < 3mm)
2. **Vibration:** Shake test for loose components
3. **Thermal:** Monitor temperatures during 10-minute powered operation

### Power System Validation

**Electrical tests:**

```python
# Test checklist
power_tests = {
    'battery_voltage': (11.1, 12.6),    # V, acceptable range
    'servo_rail_12v': (11.5, 12.0),     # V under no load
    'logic_rail_5v': (4.9, 5.1),        # V under load
    'idle_current': (0, 500),           # mA, no servos active
    'ripple_voltage': (0, 100),         # mV p-p, acceptable noise
}

def validate_power_system():
    results = {}
    for test, (min_val, max_val) in power_tests.items():
        measured = measure(test)  # Function to read multimeter
        passed = min_val <= measured <= max_val
        results[test] = {'value': measured, 'pass': passed}
    return results
```

### Knowledge Check Quiz

3. **Question:** Why use separate 12V and 5V rails rather than powering everything from 12V?
   - (a) Lower cost
   - (b) Reduce noise coupling; protect sensitive logic
   - (c) Batteries naturally output both voltages
   - (d) Arduino cannot handle 12V

   **Answer:** (b) - Isolation prevents motor noise from affecting microcontrollers

4. **Question:** If servos draw 8A peak, what minimum wire gauge is required for safety with a 10A rating?
   - (a) 24 AWG
   - (b) 22 AWG
   - (c) 18 AWG
   - (d) 14 AWG

   **Answer:** (c) - 18 AWG rated for 10A, provides safety margin over 8A peak

---

## Deliverables and Assessment

### Required Submissions

1. **CAD Model Package**
   - Complete torso assembly in STEP or F3D format
   - STL files for all 3D printed components
   - Bill of materials with specifications

2. **Assembled Hardware**
   - Functional torso with rigid structure
   - Power system operational (verified voltages)
   - Controllers mounted and communicating

3. **Documentation**
   - Wiring diagram with component labels
   - Assembly instructions with photographs
   - Test results (structural and electrical)

### Assessment Rubric

**Design Quality (30%)**
- Parametric CAD model with appropriate constraints
- Lightweight design (< 500g target)
- Accessibility and serviceability considerations

**Fabrication (30%)**
- Precise component fits
- Clean assembly without structural defects
- Professional cable management

**Electrical Integration (25%)**
- Correct power distribution architecture
- Safe wiring practices (fusing, polarity protection)
- Verified voltage rails within specification

**Documentation (15%)**
- Clear, reproducible assembly instructions
- Complete electrical schematics
- Test data presented professionally

---

## Extensions and Further Study

<div class="optional-exercise">

**Advanced Challenges**

1. **Topology Optimization:** Use FEA to create minimal-mass structure
2. **Thermal Management:** Add active cooling for extended operation
3. **Modular Design:** Quick-release connectors for rapid assembly/disassembly
4. **Custom PCB:** Design power distribution board replacing breadboard
5. **BLDC Motors:** Upgrade to brushless motors for higher efficiency

</div>

<div class="historical-note">

**Evolution of Humanoid Torso Design**

Early humanoids (WABOT-1, 1970s) used heavy aluminum frames with external electronics. Modern designs (Atlas, Optimus) employ:
- Carbon fiber monocoque structures
- Integrated electronics within sealed housings
- Custom motor controllers with embedded microprocessors
- Active thermal management systems

These advances reduced mass by 60% while increasing computational power 1000×.

</div>

---

## Connection to HUM-103

The torso now provides mounting points for the bipedal leg assemblies. In HUM-103, you will:
- Design and assemble 6-DOF legs
- Calculate required joint torques based on total system mass
- Integrate legs mechanically and electrically with the torso

Ensure your torso design accommodates hip servo mounting with adequate clearance.

---

*A well-designed torso is the foundation of a capable humanoid. Proceed to HUM-103 to add bipedal mobility.*
