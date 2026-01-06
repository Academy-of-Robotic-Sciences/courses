<!--
author:   Dr. Elena Martinez
email:    elena.martinez@academy-of-robotic-sciences.github.io
version:  2.0.0

language: en
narrator: US English Female

comment:  Electronics Fundamentals: Comprehensive treatment of circuit theory, soldering techniques, diagnostic instrumentation, and power supply design for robotic systems.


mode:     Textbook

link:     https://raw.githubusercontent.com/Academy-of-Robotic-Sciences/courses/main/course-styles.css

import:   https://raw.githubusercontent.com/LiaTemplates/Pyodide/master/README.md
-->

# HARDWARE-201: Electronics Fundamentals

> **Robust electronic systems form the foundation of reliable robotic platforms.**

## Course Overview

| | |
|---|---|
| **Course Code** | HARDWARE-201 |
| **Duration** | 4 hours |
| **Level** | Intermediate |
| **Prerequisites** | Basic physics, algebra, RC-105 or equivalent electronics experience |
| **Theory-Practice** | 60% theory, 40% optional labs |

---

## Learning Objectives

Upon completion of this course, students will be able to:

### Theoretical Understanding

- Apply Ohm's law, Kirchhoff's laws, and circuit analysis techniques to robot electronics
- Explain voltage regulation theory and power supply design principles
- Understand metallurgical bonding in soldering and thermal management
- Analyze RC and RL transient response in signal conditioning circuits
- Describe oscilloscope operation and signal measurement theory

### Practical Skills

- Design and fabricate permanent circuits using soldering techniques
- Use diagnostic instruments (multimeter, oscilloscope, power supply) for circuit analysis
- Build regulated power supplies for multi-voltage robot systems
- Diagnose and repair electronic faults using systematic troubleshooting methods
- Transfer prototypes from breadboard to production-grade perfboard assemblies

---

## Module 1: Circuit Theory Foundations

### 1.1 Fundamental Laws and Analysis

<!-- class="theory-concept" -->
**Ohm's Law and Basic Circuit Analysis**

The relationship between voltage (V), current (I), and resistance (R) is fundamental to all electronics:

$$V = IR$$

**Power Dissipation** in resistive elements:

$$P = VI = I^2R = \frac{V^2}{R}$$

where power P is measured in watts (W).

<!-- class="theory-concept" -->
**Kirchhoff's Laws**

**Kirchhoff's Current Law (KCL)**: The algebraic sum of currents entering a node equals zero.

$$\sum_{k=1}^{n} I_k = 0$$

**Kirchhoff's Voltage Law (KVL)**: The algebraic sum of voltages around any closed loop equals zero.

$$\sum_{k=1}^{n} V_k = 0$$

These laws enable systematic analysis of arbitrarily complex circuits.

<!-- class="historical-note" -->
**Historical Development**

Gustav Kirchhoff formulated his circuit laws in 1845 while still a student at the University of Königsberg. These laws, derived from conservation of charge (KCL) and conservation of energy (KVL), became the foundation of circuit analysis. Georg Ohm published his law in 1827, though it was initially met with skepticism by the German physics community.

### 1.2 Series and Parallel Circuits

<!-- class="theory-concept" -->
**Series Resistance**

Resistors in series add directly:

$$R_{total} = R_1 + R_2 + \cdots + R_n$$

The same current flows through all series elements, but voltage divides proportionally.

**Voltage Divider Formula**:

$$V_{out} = V_{in} \cdot \frac{R_2}{R_1 + R_2}$$

Voltage dividers are fundamental to sensor interfacing and signal conditioning.

**Parallel Resistance**

Resistors in parallel combine as reciprocals:

$$\frac{1}{R_{total}} = \frac{1}{R_1} + \frac{1}{R_2} + \cdots + \frac{1}{R_n}$$

For two resistors: $R_{total} = \frac{R_1 R_2}{R_1 + R_2}$

Parallel circuits share the same voltage, but current divides.

**Quiz: Circuit Analysis**

Three resistors (10Ω, 20Ω, 30Ω) are connected in series across a 12V supply. What is the total resistance?

[( )] 10Ω
[( )] 20Ω
[(X)] 60Ω
[( )] 5.45Ω

In a voltage divider with R₁ = 1kΩ and R₂ = 9kΩ, with 10V input, what is the output voltage across R₂?

[( )] 1V
[( )] 5V
[(X)] 9V
[( )] 10V

### 1.3 Reactive Components: Capacitors and Inductors

<!-- class="theory-concept" -->
**Capacitors**

A capacitor stores charge and energy in an electric field:

$$Q = CV$$

where:
- Q is charge in coulombs
- C is capacitance in farads
- V is voltage

**Current-Voltage Relationship**:

$$I = C\frac{dV}{dt}$$

Capacitors oppose voltage changes. They pass AC signals but block DC.

**Energy Storage**:

$$E = \frac{1}{2}CV^2$$

**Applications in Robotics**:
- Power supply filtering and decoupling
- Signal coupling and DC blocking
- Timing circuits and oscillators

<!-- class="theory-concept" -->
**RC Time Constant**

When charging through a resistor R, a capacitor charges exponentially:

$$V(t) = V_{final}(1 - e^{-t/\tau})$$

where $\tau = RC$ is the time constant.

- At t = τ: 63.2% charged
- At t = 5τ: >99% charged (considered "fully charged")

**Inductors**

Inductors store energy in a magnetic field:

$$V = L\frac{dI}{dt}$$

where L is inductance in henries.

Inductors oppose current changes. They are essential in:
- Motor drive circuits (back-EMF suppression)
- Power supply filtering
- RF circuits and transformers

**Energy Storage**:

$$E = \frac{1}{2}LI^2$$

### 1.4 Thevenin and Norton Equivalents

<!-- class="theory-concept" -->
**Circuit Simplification**

Any linear circuit with two terminals can be replaced by:

**Thevenin Equivalent**: Voltage source $V_{th}$ in series with resistance $R_{th}$

**Norton Equivalent**: Current source $I_{N}$ in parallel with resistance $R_{N}$

Where:
- $V_{th}$ = open-circuit voltage
- $R_{th} = R_{N}$ = resistance seen from terminals with sources deactivated
- $I_{N} = V_{th}/R_{th}$

These equivalents simplify analysis and interface design in complex robotic systems.

---

## Module 2: Soldering and Assembly

### 2.1 Metallurgy of Soldering

<!-- class="theory-concept" -->
**The Soldering Process**

Soldering creates a metallurgical bond through intermetallic compound formation. The process requires:

1. **Surface Preparation**: Oxide removal from metal surfaces
2. **Flux Application**: Chemical agent that removes oxides and prevents re-oxidation
3. **Heat Application**: Sufficient temperature to melt solder and activate flux
4. **Wetting**: Solder spreads across the surface forming intermetallic bonds
5. **Cooling**: Solidification forms crystalline structure

**Solder Composition**:

Traditional: 63% Tin, 37% Lead (eutectic, melts at 183°C)
Modern (RoHS): 96.5% Tin, 3% Silver, 0.5% Copper (SAC305, melts at 217°C)

<!-- class="theory-concept" -->
**Quality Solder Joints**

A proper solder joint exhibits:
- **Shiny, smooth surface**: Indicates proper wetting
- **Concave fillet**: Forms a volcano or tent shape
- **Complete coverage**: Solder wets both pad and component lead
- **No cold joints**: Dull, grainy appearance indicates insufficient heat

**Common Defects**:
- Cold joint: Insufficient heat or movement during cooling
- Bridging: Unwanted solder connecting adjacent pins
- Insufficient solder: Weak mechanical and electrical connection
- Excess solder: Can cause bridging and poor heat dissipation

<!-- class="historical-note" -->
**Evolution of Soldering in Electronics**

Hand soldering dates to ancient metalworking (5000+ years). Electronics soldering emerged with the telegraph in the 1840s. The introduction of rosin flux in the early 1900s improved reliability. Wave soldering (1950s) and reflow soldering (1980s) automated production. The 2006 RoHS directive mandated lead-free solder in Europe, fundamentally changing electronics manufacturing.

### 2.2 Soldering Technique and Tools

<!-- class="theory-concept" -->
**Temperature Management**

**Soldering Iron Temperature**:
- Lead-based solder: 315-370°C (600-700°F)
- Lead-free solder: 370-400°C (700-750°F)

Higher temperatures risk:
- Component damage (especially semiconductors)
- Pad delamination from PCB
- Thermal stress and cracking

**Heat Transfer**:
- Use appropriate tip size (too large risks component damage, too small provides insufficient heat)
- Clean tip regularly with brass sponge or wet cellulose
- Tin the tip to improve thermal conductivity

<!-- class="theory-concept" -->
**Proper Soldering Sequence**

1. Secure component (use third hand or fixture)
2. Heat both pad and component lead simultaneously
3. Apply solder to the joint (not the tip) after 1-2 seconds
4. Allow solder to flow and wet surfaces
5. Remove solder, then remove iron
6. Allow joint to cool without movement (3-5 seconds)

### 2.3 From Breadboard to Perfboard

<!-- class="theory-concept" -->
**Perfboard Layout Strategy**

Perfboard (perforated board) provides a permanent alternative to breadboards:

**Layout Principles**:
1. **Component Grouping**: Place related components near each other
2. **Minimize Wire Crossings**: Reduces complexity and debugging difficulty
3. **Power Distribution**: Dedicate rows/columns for power rails
4. **Tall Components Last**: Solder low-profile components first
5. **Accessibility**: Keep test points and adjustments accessible

**Wire Selection**:
- 22-24 AWG solid core for point-to-point wiring
- 18-20 AWG for power distribution
- Stranded wire for connections to external components (strain relief)

**Quiz: Soldering Theory**

The eutectic point of 63/37 tin-lead solder means:

[( )] It melts at the lowest possible temperature
[(X)] It transitions directly between solid and liquid without a plastic phase
[( )] It has the highest tensile strength
[( )] It contains no flux

What indicates a cold solder joint?

[(X)] Dull, grainy appearance
[( )] Shiny, smooth surface
[( )] Concave fillet shape
[( )] Good wetting of surfaces

---

## Module 3: Diagnostic Instrumentation

### 3.1 The Digital Multimeter

<!-- class="theory-concept" -->
**Fundamental Measurements**

The multimeter is the primary diagnostic tool for electronics:

**Voltage Measurement (V)**:
- DC voltage: Polarity matters (red probe = positive)
- AC voltage: Measures RMS (root-mean-square) value
- High input impedance (typically 10MΩ) prevents circuit loading

**Current Measurement (A)**:
- Requires breaking circuit (series connection)
- Observe polarity and range limits
- Use inline fuse protection

**Resistance Measurement (Ω)**:
- Circuit must be de-energized
- Remove component from circuit for accurate reading
- Measures DC resistance, not impedance

**Continuity Test**:
- Audible indication of low resistance (<50Ω typically)
- Essential for verifying connections and finding shorts

<!-- class="theory-concept" -->
**Measurement Best Practices**

1. **Range Selection**: Start with highest range, then decrease
2. **Polarity**: Note conventions (red = positive, black = ground)
3. **Loading Effects**: High-impedance voltmeter minimizes circuit disturbance
4. **Safety**: Never measure resistance on energized circuits
5. **Accuracy**: Consider instrument accuracy (typically ±0.5% to ±2%)

### 3.2 The Oscilloscope

<!-- class="theory-concept" -->
**Visualizing Electrical Signals**

An oscilloscope displays voltage as a function of time, revealing:
- Waveform shape
- Frequency and period
- Amplitude (peak, peak-to-peak, RMS)
- Phase relationships
- Transients and noise

**Key Controls**:

**Volts/Division (V/div)**: Vertical scale
- Adjusts voltage sensitivity
- Typical ranges: 1mV/div to 10V/div

**Time/Division (t/div)**: Horizontal scale
- Adjusts time base
- Typical ranges: 1ns/div to 1s/div

**Trigger**: Synchronizes display with signal
- **Level**: Voltage threshold for triggering
- **Edge**: Rising or falling edge
- **Mode**: Auto, normal, single

<!-- class="theory-concept" -->
**Signal Measurement Techniques**

**Period (T)**: Time for one complete cycle
- Measured using horizontal cursors
- Frequency: $f = 1/T$

**Amplitude Measurements**:
- **Peak-to-Peak (Vpp)**: Total voltage swing
- **Peak (Vp)**: Maximum voltage from zero
- **RMS (Vrms)**: Effective voltage (for sine wave: $V_{rms} = \frac{V_p}{\sqrt{2}}$)

**Common Waveforms in Robotics**:
- PWM signals: Motor control, servo signals
- I2C/SPI: Digital communication protocols
- Encoder signals: Quadrature A/B channels
- Sensor outputs: Analog and pulse-width modulated

<!-- class="alternative-approach" -->
**Digital vs. Analog Oscilloscopes**

**Analog Oscilloscopes**: Use CRT to directly display signal
- Real-time display
- No sampling artifacts
- Limited capture and storage

**Digital Storage Oscilloscopes (DSO)**: Sample and digitize signal
- Store and analyze waveforms
- Advanced triggering and measurement
- Mathematical operations (FFT, integration)
- Standard in modern applications

### 3.3 Power Supplies and Function Generators

<!-- class="theory-concept" -->
**Bench Power Supply**

Provides stable, adjustable DC voltage for circuit testing:

**Key Specifications**:
- **Voltage Range**: Typical 0-30V
- **Current Limit**: Adjustable protection (0.01-3A typical)
- **Regulation**: Line and load regulation (<0.01%)
- **Ripple**: AC component on DC output (<1mV typical)

**Operating Modes**:
- **Constant Voltage (CV)**: Regulates voltage, current varies with load
- **Constant Current (CC)**: Limits current, voltage drops with load
- Use current limiting to protect circuits during development

**Function Generator**

Produces test signals with controlled characteristics:

**Waveforms**:
- Sine wave: Testing frequency response
- Square wave: Digital logic testing
- Triangle/Sawtooth: Ramp generation
- Pulse: PWM signal simulation

**Parameters**:
- Frequency (0.1Hz to 100MHz+)
- Amplitude (mVpp to 10Vpp)
- Offset (DC bias)
- Duty cycle (for pulse/square waves)

---

## Module 4: Power Supply Design

### 4.1 Linear Voltage Regulators

<!-- class="theory-concept" -->
**Regulation Principles**

A voltage regulator maintains constant output voltage despite variations in:
- Input voltage (line regulation)
- Output current (load regulation)
- Temperature

**LM7805 Three-Terminal Regulator**:

The 7805 provides regulated +5V output from higher input voltage:

```
Input (7-35V) → [7805] → Output (5V)
                   ↓
                  GND
```

**Key Specifications**:
- Output Voltage: 5V ± 0.2V
- Maximum Output Current: 1.5A (with heatsink)
- Dropout Voltage: 2V (minimum input = 7V)
- Line Regulation: 4mV typical
- Load Regulation: 15mV typical

**Power Dissipation**:

$$P_D = (V_{in} - V_{out}) \cdot I_{out}$$

Example: 12V input, 5V output, 0.5A load:
$$P_D = (12 - 5) \times 0.5 = 3.5W$$

This requires heatsinking to prevent thermal shutdown.

<!-- class="theory-concept" -->
**Thermal Management**

**Junction Temperature**: Maximum typically 125-150°C

**Thermal Resistance** θ relates temperature rise to power:

$$T_j = T_a + P_D \cdot \theta_{ja}$$

where:
- $T_j$ = junction temperature (°C)
- $T_a$ = ambient temperature (°C)
- $P_D$ = power dissipation (W)
- $\theta_{ja}$ = thermal resistance junction-to-ambient (°C/W)

Heatsinks reduce θ, allowing higher power dissipation.

### 4.2 Capacitor Selection for Power Supplies

<!-- class="theory-concept" -->
**Input and Output Capacitors**

**Input Capacitor** (typically 0.33-1μF):
- Prevents oscillation
- Compensates for long input leads
- Place close to regulator input pin

**Output Capacitor** (typically 10-100μF):
- Reduces output ripple
- Improves transient response
- Stabilizes feedback loop

**Electrolytic vs. Ceramic**:
- **Electrolytic**: High capacitance, polarized, higher ESR
- **Ceramic**: Low ESR, non-polarized, better high-frequency response
- Best practice: Parallel combination (e.g., 47μF electrolytic + 0.1μF ceramic)

### 4.3 Switching Regulators

<!-- class="theory-concept" -->
**Buck Converter Topology**

Switching regulators achieve high efficiency (>90%) by switching at high frequency:

**Basic Buck Converter**:
- Input voltage > Output voltage (step-down)
- Switch (MOSFET) alternates on/off at 100kHz-1MHz
- Inductor and capacitor filter output

**Advantages over Linear**:
- High efficiency (less heat)
- Wide input voltage range
- Smaller heatsinks

**Disadvantages**:
- Output ripple and noise
- EMI generation
- More complex design

<!-- class="alternative-approach" -->
**Boost and Buck-Boost Topologies**

**Boost Converter**: Steps voltage up (Vout > Vin)
- Used for battery-powered systems
- LED driver applications

**Buck-Boost Converter**: Output voltage can be higher or lower than input
- Handles wide input voltage variations
- Battery systems with varying charge state

**Quiz: Power Supply Design**

An LM7805 regulator dissipates 3W of power. Its junction-to-ambient thermal resistance is 50°C/W without heatsink. If ambient temperature is 25°C, what is the junction temperature?

[( )] 25°C
[( )] 75°C
[( )] 125°C
[(X)] 175°C

Why are switching regulators more efficient than linear regulators?

[( )] They use better materials
[(X)] They minimize resistive power loss by switching instead of dissipating
[( )] They operate at higher voltage
[( )] They require no external components

---

## Module 5: Circuit Protection and Safety

### 5.1 Fuses and Current Limiting

<!-- class="theory-concept" -->
**Overcurrent Protection**

Fuses protect circuits by opening when current exceeds rating:

**Fuse Characteristics**:
- **Current Rating**: Normal operating current (select 125-150% of maximum expected)
- **Voltage Rating**: Maximum voltage fuse can safely interrupt
- **I²t Rating**: Energy required to blow fuse
- **Response Time**: Fast-blow vs. slow-blow (time-delay)

**Fuse Selection**:
- **Fast-Blow**: Protects sensitive electronics (e.g., microcontrollers)
- **Slow-Blow**: Tolerates inrush currents (e.g., motors, transformers)

**Resetable Fuses (PTC)**:
- Polymer Positive Temperature Coefficient devices
- Self-resetting after cooling
- Lower current ratings (typically <5A)
- Slower response than traditional fuses

### 5.2 Reverse Polarity Protection

<!-- class="theory-concept" -->
**Diode Protection**

A series diode prevents damage from reversed power:

```
Battery (+) → [Diode] → Circuit (+)
Battery (-) → Circuit (-)
```

**Drawback**: 0.6-0.7V forward drop wastes power

**MOSFET Protection** (better):
- Use P-channel MOSFET as high-side switch
- Near-zero voltage drop when on
- Can include current limiting

### 5.3 Thermal Management

<!-- class="theory-concept" -->
**Wire Gauge Selection**

Wire ampacity (current-carrying capacity) depends on:
- Cross-sectional area (AWG gauge)
- Insulation temperature rating
- Ambient temperature
- Bundling and airflow

**American Wire Gauge (AWG) Guidelines**:
- 24 AWG: 0.5A max (signal wires)
- 22 AWG: 1A max (low-power circuits)
- 20 AWG: 1.5A max
- 18 AWG: 2.5A max (power distribution)
- 16 AWG: 5A max
- 14 AWG: 8A max (motor power)

**Voltage Drop Calculation**:

$$V_{drop} = I \cdot R_{wire} = I \cdot \rho \cdot \frac{L}{A}$$

where:
- ρ = resistivity of copper (1.68 × 10⁻⁸ Ω·m)
- L = wire length (m)
- A = cross-sectional area (m²)

Keep voltage drop <3% for power distribution, <1% for sensitive circuits.

---

## Summary

This course established the foundations of electronics for robotics:

1. **Circuit Theory**: Applied Kirchhoff's laws and Ohm's law to analyze DC and reactive circuits
2. **Soldering**: Understood metallurgical bonding and permanent circuit assembly techniques
3. **Instrumentation**: Learned to use multimeters, oscilloscopes, and power supplies for diagnostics
4. **Power Supplies**: Designed linear and switching regulators with proper thermal management
5. **Protection**: Implemented fuses, current limiting, and safe power distribution

**Key Takeaways**:
- Systematic circuit analysis enables predictable design
- Quality soldering creates reliable mechanical and electrical bonds
- Diagnostic instruments reveal circuit behavior beyond DC measurements
- Efficient power regulation requires thermal and electrical design
- Protection circuits prevent catastrophic failures

**Next Steps**: HARDWARE-202 examines microcontroller architectures and embedded programming to control these electronic systems.

---

## Optional Laboratory Exercises

### Lab 1: Perfboard Power Supply Assembly (2 hours)

<!-- class="optional-exercise" -->
**Objective**: Design and build a dual-output (5V and 12V) regulated power supply on perfboard.

**Components**:
- 7805 voltage regulator (5V)
- 7812 voltage regulator (12V)
- Input/output capacitors
- Fuses
- Terminal blocks
- LED indicators with current-limiting resistors

**Tasks**:
1. Design circuit schematic with input protection and filtering
2. Plan perfboard layout minimizing wire crossings
3. Solder all components with quality joints
4. Test outputs under no-load and loaded conditions (measure regulation)
5. Verify current limiting and fuse operation

<!-- class="exercise-tip" -->
**Tips**:
- Use 0.33μF input caps and 100μF output caps for each regulator
- Calculate required heatsinking for expected loads
- Add LED indicators (with 470Ω resistors) for visual status
- Test with gradually increasing loads up to 500mA

**Target Specifications**:
- 5V output: 4.9-5.1V under 0-500mA load
- 12V output: 11.8-12.2V under 0-300mA load
- Ripple <50mV peak-to-peak
- All solder joints pass visual inspection

### Lab 2: Oscilloscope Signal Analysis (1.5 hours)

<!-- class="optional-exercise" -->
**Objective**: Characterize various signals using oscilloscope measurements.

**Equipment**:
- Digital oscilloscope
- Function generator
- Arduino or signal source
- Ultrasonic sensor or other pulse-based sensor

**Tasks**:
1. **Calibration**: Verify oscilloscope using function generator reference
2. **Waveform Characterization**: Measure sine, square, and triangle waves
   - Frequency (compare to function generator setting)
   - Peak-to-peak amplitude
   - RMS voltage (verify $V_{rms} = V_p/\sqrt{2}$ for sine)
3. **PWM Analysis**: Generate PWM signal with Arduino, measure:
   - Frequency
   - Duty cycle
   - Rise/fall times
4. **Sensor Signals**: Capture ultrasonic sensor trigger and echo pulses
   - Measure pulse widths
   - Calculate distance from time-of-flight

<!-- class="exercise-advanced" -->
**Advanced Extensions**:
- Use math functions to compute FFT (frequency spectrum)
- Capture single-shot events with single trigger mode
- Analyze I2C or SPI communication timing
- Measure propagation delays in digital circuits

### Lab 3: Thermal Analysis of Voltage Regulators (1 hour)

<!-- class="optional-exercise" -->
**Objective**: Experimentally verify thermal calculations and evaluate heatsinking effectiveness.

**Equipment**:
- 7805 voltage regulator
- Variable power supply
- Electronic load or power resistor
- Infrared thermometer or thermocouple
- Various heatsinks

**Tasks**:
1. Calculate expected junction temperature for various input voltages and loads
2. Measure actual case temperature without heatsink
3. Repeat with different heatsinks
4. Plot temperature vs. power dissipation
5. Determine safe operating limits

<!-- class="exercise-tip" -->
**Safety Considerations**:
- Do not exceed 125°C junction temperature
- Allow adequate cooling time between tests
- Use thermal shutdown as safety feature, but don't rely on it
- Secure heatsinks properly (thermal compound improves contact)

**Analysis Questions**:
- How does measured θ compare to datasheet values?
- What power level triggers thermal shutdown?
- How much does heatsinking improve thermal performance?
- At what input voltage and current should you add a heatsink?

---

*Version 2.0.0 - Academic curriculum emphasizing theoretical foundations with optional practical exercises*
