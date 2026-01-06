<!--
author:   Dr. Elena Martinez
email:    elena.martinez@academy-of-robotic-sciences.github.io
version:  2.0.0

language: en
narrator: US English Female

comment:  Power Systems and Safety Engineering: Comprehensive treatment of battery technology, power electronics, voltage regulation, protection circuits, and fail-safe design for robotic systems.


mode:     Textbook

link:     https://raw.githubusercontent.com/Academy-of-Robotic-Sciences/courses/main/course-styles.css

import:   https://raw.githubusercontent.com/LiaTemplates/Pyodide/master/README.md
-->

# HARDWARE-204: Power Systems and Safety Engineering

> **Safe, reliable power distribution is fundamental to robot operational integrity.**

## Course Overview

| | |
|---|---|
| **Course Code** | HARDWARE-204 |
| **Duration** | 4 hours |
| **Level** | Intermediate |
| **Prerequisites** | HARDWARE-201 (Electronics Fundamentals) |
| **Theory-Practice** | 60% theory, 40% optional labs |

---

## Learning Objectives

Upon completion of this course, students will be able to:

### Theoretical Understanding

- Explain electrochemical principles of common battery technologies
- Analyze linear and switching voltage regulator topologies
- Understand overcurrent protection mechanisms and fuse selection
- Describe fail-safe design principles and emergency stop systems
- Apply thermal management theory to power system design

### Practical Skills

- Select appropriate battery technology for robot applications
- Design multi-voltage power distribution systems
- Implement protection circuits including fuses and current limiting
- Build emergency stop (E-Stop) circuits with proper safety interlocks
- Create wiring harnesses with appropriate gauge selection and cable management

---

## Module 1: Battery Technology and Management

### 1.1 Electrochemistry Fundamentals

<!-- class="theory-concept" -->
**Battery Operating Principles**

Batteries convert chemical energy to electrical energy through redox reactions:

**Anode** (negative terminal):
- Oxidation occurs (loses electrons)
- Higher potential energy

**Cathode** (positive terminal):
- Reduction occurs (gains electrons)
- Lower potential energy

**Electrolyte**: Ion conductor, electron insulator

**Cell Voltage**: Determined by electrode materials
- Lead-acid: 2.0V per cell
- NiMH: 1.2V per cell
- Li-ion/LiPo: 3.6-3.7V per cell

### 1.2 Battery Technologies for Robotics

<!-- class="theory-concept" -->
**Lead-Acid Batteries**

**Chemistry**: Pb + PbO₂ + H₂SO₄

**Characteristics**:
- Nominal voltage: 2.0V per cell (12V = 6 cells)
- Energy density: ~30-40 Wh/kg
- Cycle life: 200-300 cycles (deep discharge)
- Cost: Very low ($0.10-0.20/Wh)
- Self-discharge: ~5% per month

**Applications**:
- Large mobile robots
- Stationary robots with charging stations
- Budget-constrained applications

**Disadvantages**:
- Heavy
- Limited cycle life
- Sulfation if stored discharged

<!-- class="theory-concept" -->
**Nickel-Metal Hydride (NiMH)**

**Chemistry**: NiOOH + Metal Hydride

**Characteristics**:
- Nominal voltage: 1.2V per cell
- Energy density: 60-120 Wh/kg
- Cycle life: 500-1000 cycles
- Cost: Moderate ($0.30-0.50/Wh)
- Self-discharge: ~30% per month (improved in low self-discharge variants)

**Advantages**:
- Safer than lithium chemistry
- No memory effect (modern cells)
- Tolerates overcharging

**Applications**:
- Educational robots
- Consumer applications

<!-- class="theory-concept" -->
**Lithium-Polymer (LiPo) Batteries**

**Chemistry**: LiCoO₂ (or variants) + graphite

**Characteristics**:
- Nominal voltage: 3.7V per cell
- Fully charged: 4.2V per cell
- Energy density: 150-250 Wh/kg
- Cycle life: 300-500 cycles
- Cost: Higher ($0.50-1.00/Wh)
- Self-discharge: ~5% per month

**C-Rating**: Maximum safe discharge rate
- C = Capacity in Ah
- 20C rating on 2.2Ah battery → 44A max continuous discharge

**Safety Considerations**:
- **Overcharge**: >4.2V/cell → fire risk
- **Overdischarge**: <3.0V/cell → permanent damage
- **Physical damage**: Puncture → thermal runaway
- **High current**: Internal resistance heating

**Applications**:
- Drones and aerial robots
- High-performance mobile robots
- Applications requiring high power density

<!-- class="historical-note" -->
**Evolution of Battery Technology**

**1800**: Alessandro Volta invents the voltaic pile (first battery)
**1859**: Gaston Planté develops lead-acid battery
**1899**: Waldemar Jungner invents nickel-cadmium battery
**1970s**: Development of lithium batteries begins
**1991**: Sony commercializes first lithium-ion battery
**1999**: Lithium-polymer batteries emerge for consumer electronics
**2010s**: LiFePO₄ (lithium iron phosphate) offers improved safety for robotics

### 1.3 Battery Management Systems

<!-- class="theory-concept" -->
**State of Charge (SoC) Estimation**

**Voltage-Based (Simple)**:
Linear approximation between full and empty voltages

$$\text{SoC} \approx \frac{V_{current} - V_{empty}}{V_{full} - V_{empty}} \times 100\%$$

**Issues**:
- Voltage sags under load
- Inaccurate at extremes
- Temperature dependent

**Coulomb Counting (Better)**:
Integrate current over time:

$$\text{SoC}(t) = \text{SoC}(t_0) - \frac{1}{C}\int_{t_0}^t I(\tau) d\tau$$

where C is battery capacity (Ah)

**Requirements**:
- Current sensor (shunt resistor or Hall effect)
- Accurate initial SoC
- Compensate for efficiency and self-discharge

<!-- class="theory-concept" -->
**Cell Balancing**

Multi-cell batteries require balanced cells:

**Passive Balancing**:
- Dissipate excess energy from highest cell
- Use resistors to bleed charge
- Simple, wasteful

**Active Balancing**:
- Transfer energy between cells
- More efficient, complex
- Used in automotive and aerospace

**Why Balancing Matters**:
- Weakest cell limits pack capacity
- Overcharge protection trips on first full cell
- Imbalanced cells reduce cycle life

### 1.4 Safe Charging and Storage

<!-- class="theory-concept" -->
**LiPo Charging Protocol (CC-CV)**

**Constant Current (CC) Phase**:
- Charge at 1C (or manufacturer specification)
- Monitor cell voltages
- Switch to CV when cell reaches 4.2V

**Constant Voltage (CV) Phase**:
- Hold at 4.2V per cell
- Current decreases exponentially
- Stop when current < 0.05C (trickle)

**Never**:
- Charge > 4.2V per cell
- Charge < 0°C (lithium plating risk)
- Leave unattended during charging
- Charge damaged/swollen cells

**Storage**:
- Store at 3.8V per cell (40-60% SoC)
- Cool, dry environment
- Fireproof container (LiPo bag)
- Check voltage every 3-6 months

**Quiz: Battery Fundamentals**

A 3S LiPo battery has how many cells in series, and what is the fully charged voltage?

[( )] 3 cells, 11.1V
[(X)] 3 cells, 12.6V
[( )] 3 cells, 14.8V
[( )] 4 cells, 16.8V

The C-rating of a battery determines:

[( )] Capacity in amp-hours
[(X)] Maximum safe discharge current relative to capacity
[( )] Charging time
[( )] Number of cells in the pack

---

## Module 2: Voltage Regulation

### 2.1 Linear Regulator Theory

<!-- class="theory-concept" -->
**Series Pass Regulation**

Linear regulators use a variable resistor (pass element) to drop voltage:

**Operation**:
1. Error amplifier compares output to reference voltage
2. Adjusts pass transistor resistance
3. Maintains constant output voltage

**Dropout Voltage** ($V_{dropout}$):
Minimum input-output differential for regulation

$$V_{in, min} = V_{out} + V_{dropout}$$

Example: LM7805 has 2V dropout
- For 5V output: Input must be ≥7V

**Efficiency**:
$$\eta = \frac{V_{out}}{V_{in}} \times 100\%$$

Example: 12V → 5V
$$\eta = \frac{5}{12} = 41.7\%$$

**Power Dissipation**:
$$P_D = (V_{in} - V_{out}) \times I_{out}$$

This power is converted to heat, requiring thermal management.

<!-- class="theory-concept" -->
**Low-Dropout (LDO) Regulators**

**Advantages over standard linear**:
- Low dropout (0.1-0.5V typical)
- Efficient for small input-output differentials
- Low noise (good for analog circuits)

**Examples**:
- AMS1117 (1.2V dropout)
- MCP1700 (0.178V dropout)

**Applications in Robotics**:
- Sensor power supplies (low noise critical)
- Microcontroller power (3.3V from Li-ion 3.7V)
- Post-regulation after switching supply

### 2.2 Switching Regulator Topologies

<!-- class="theory-concept" -->
**Buck Converter (Step-Down)**

High efficiency voltage step-down:

**Components**:
- Switching MOSFET
- Inductor (stores energy)
- Diode (freewheeling)
- Capacitor (output filter)

**Operation**:
1. MOSFET on: Current increases through inductor
2. MOSFET off: Inductor maintains current through diode
3. Average output voltage controlled by duty cycle

**Voltage Relationship**:
$$V_{out} = D \times V_{in}$$

where D = duty cycle (ton / T period)

**Efficiency**: 85-95% typical

**Switching Frequency**:
- Higher frequency → smaller inductor/capacitor
- Typical: 100 kHz - 1 MHz
- Tradeoff: switching losses vs. component size

<!-- class="theory-concept" -->
**Boost Converter (Step-Up)**

Steps voltage up from input:

**Operation**:
1. MOSFET on: Energy stored in inductor
2. MOSFET off: Inductor voltage adds to input

**Voltage Relationship**:
$$V_{out} = \frac{V_{in}}{1 - D}$$

**Applications**:
- 5V USB to 12V motor supply
- Battery voltage to higher logic levels
- LED drivers

**Boost-Buck (SEPIC/Cuk)**:
- Can step up or down
- Single inductor (SEPIC) or dual (Cuk)
- Useful for variable input (battery discharge curve)

<!-- class="alternative-approach" -->
**Integrated Switching Regulators**

Modern ICs integrate control and power components:

**Advantages**:
- Simple design (few external components)
- Thermal shutdown and overcurrent protection
- Soft-start and enable control

**Examples**:
- LM2596 (buck, 3A, up to 40V input)
- LM2577 (boost, 3A switch)
- RECOM R-78 series (drop-in replacement for 78xx linear)

**Quiz: Voltage Regulation**

A buck converter with 12V input and 50% duty cycle produces what output voltage?

[( )] 12V
[(X)] 6V
[( )] 3V
[( )] 24V

Switching regulators are more efficient than linear regulators because:

[(X)] They minimize resistive power loss by switching rapidly
[( )] They use better quality components
[( )] They have lower dropout voltage
[( )] They generate less noise

---

## Module 3: Protection Circuits

### 3.1 Overcurrent Protection

<!-- class="theory-concept" -->
**Fuse Theory**

Fuses protect circuits by melting under overcurrent:

**I²t Rating**: Energy required to melt fuse element
$$\text{I}^2\text{t} = \int_0^{t_{melt}} I(t)^2 dt$$

**Fuse Selection Criteria**:

1. **Current Rating**: 125-150% of normal operating current
   - Accounts for startup surges
   - Temperature derating

2. **Voltage Rating**: Must exceed maximum circuit voltage
   - Determines arc suppression capability
   - Not the operating voltage, but max safe interruption

3. **Response Time**:
   - **Fast-Blow (F)**: Sensitive electronics, <0.1s typical
   - **Slow-Blow (T)**: Motor loads, tolerates startup surge
   - **Time-Delay**: Specified delay curve

4. **Breaking Capacity**: Maximum current fuse can safely interrupt

**Example Calculation**:
Robot with 12V, 2A nominal, 5A peak startup
- Select 3A slow-blow fuse
- 15V minimum voltage rating
- Ceramic body for high breaking capacity

<!-- class="theory-concept" -->
**Resettable Fuses (PolyFuses/PTCs)**

Polymer Positive Temperature Coefficient devices:

**Operation**:
- Low resistance when cool
- Overcurrent → heating → resistance increases
- Current limited, heat dissipates, resets

**Characteristics**:
- "Trip" current: Guaranteed to switch within 1 hour
- "Hold" current: Maximum without tripping
- Slower than traditional fuses (seconds vs. milliseconds)
- Lower current ratings (typically <5A)

**Applications**:
- USB ports
- Microcontroller power inputs
- Low-current sensor lines

### 3.2 Reverse Polarity Protection

<!-- class="theory-concept" -->
**Diode-Based Protection**

Series Schottky diode prevents reverse current:

**Advantages**:
- Simple, passive
- No control needed
- Fail-safe

**Disadvantages**:
- Forward voltage drop (0.3-0.7V)
- Power loss: $P = V_f \times I$
- For 5A: $P = 0.4V \times 5A = 2W$ wasted

<!-- class="theory-concept" -->
**MOSFET-Based Ideal Diode**

P-channel MOSFET as high-side switch:

**Operation**:
- Correct polarity: Gate-source voltage turns on MOSFET
- Reverse polarity: Body diode blocks, MOSFET off
- On-resistance (RDS(on)): 10-50 mΩ typical

**Power Loss**:
$$P = I^2 \times R_{DS(on)}$$

Example: 5A through 20mΩ MOSFET
$$P = 25 \times 0.020 = 0.5W$$

**Much better than diode solution!**

**Schematic**:
```
Battery+ ──┬─── P-MOSFET ───┬─── Load+
           │     (Body       │
        10kΩ     diode ↑)    │
           │                 │
           └─────────────────┘
Battery- ────────────────────── Load-
```

### 3.3 Emergency Stop Systems

<!-- class="theory-concept" -->
**E-Stop Design Philosophy**

Emergency stops must be **fail-safe**: Failure mode must be safe state.

**Normally-Closed (NC) Architecture**:
- E-Stop button is NC contact
- Pressed → opens circuit → motors stop
- Wire failure → motors stop (safe)

**Never use normally-open for E-stop!**

**Relay-Based E-Stop**:
```
+12V ────┬──── NC E-Stop Button ────┬──── Relay Coil ───┬
         │                          │                   │
         └────────────────────────────────── Motor Power │
                                            (through NC   │
                                             relay contact)
                                                          │
Ground ──────────────────────────────────────────────────┘
```

**Operation**:
- Normal: Button closed, relay energized, NC contact closed, motors powered
- E-Stop pressed: Button opens, relay de-energizes, NC contact opens, motors stop
- Wire break: Same as E-Stop pressed → safe

<!-- class="theory-concept" -->
**Redundancy and Diagnostics**

**Safety-Rated Systems** (industrial):
- Dual-channel architecture
- Monitor both channels for agreement
- Detect single-point failures

**Status Indication**:
- LED shows E-Stop state
- Serial message to controller
- Log E-Stop events for analysis

**Reset Procedure**:
- Require manual acknowledgment after E-Stop
- Prevent automatic restart
- Check system state before re-enabling

---

## Module 4: Thermal Management and Wire Selection

### 4.1 Heat Transfer Fundamentals

<!-- class="theory-concept" -->
**Thermal Resistance**

Analogous to electrical resistance:

$$\Delta T = P \times \theta$$

where:
- ΔT = temperature rise (°C)
- P = power dissipation (W)
- θ = thermal resistance (°C/W)

**Thermal Path**:
$$\theta_{ja} = \theta_{jc} + \theta_{cs} + \theta_{sa}$$

- θ_{jc}: Junction to case
- θ_{cs}: Case to heatsink (thermal compound reduces)
- θ_{sa}: Heatsink to ambient

**Safe Operating Temperature**:
$$T_j = T_a + P \times \theta_{ja} \leq T_{j,max}$$

**Example**: LM7805 regulating 12V → 5V at 1A
- $P_D = (12-5) \times 1 = 7W$
- θ_{jc} = 5°C/W, θ_{sa} = 10°C/W (with heatsink)
- θ_{ja} = 15°C/W
- $T_j = 25 + 7 \times 15 = 130°C$ (acceptable for 150°C max)

### 4.2 Wire Gauge Selection

<!-- class="theory-concept" -->
**American Wire Gauge (AWG) Standards**

Smaller AWG number → larger wire:

| AWG | Diameter (mm) | Area (mm²) | Resistance (Ω/km) | Max Current* |
|-----|---------------|------------|-------------------|--------------|
| 24  | 0.511         | 0.205      | 84.2              | 0.5 A        |
| 22  | 0.644         | 0.326      | 53.0              | 1.0 A        |
| 20  | 0.812         | 0.518      | 33.3              | 1.5 A        |
| 18  | 1.024         | 0.823      | 21.0              | 2.5 A        |
| 16  | 1.291         | 1.31       | 13.2              | 5.0 A        |
| 14  | 1.628         | 2.08       | 8.3               | 8.0 A        |
| 12  | 2.053         | 3.31       | 5.2               | 15 A         |

*Chassis wiring, 60°C insulation, typical values

<!-- class="theory-concept" -->
**Voltage Drop Calculation**

Acceptable voltage drops:
- Power distribution: <3% of supply voltage
- Sensitive circuits (analog sensors): <1%

**Resistance per Length**:
$$R = \rho \times \frac{L}{A}$$

For copper: ρ = 1.68 × 10⁻⁸ Ω·m

**Round-Trip Voltage Drop**:
$$V_{drop} = 2 \times I \times R_{wire}$$

(Factor of 2 for forward and return paths)

**Example**: 12V motor, 3A, 2m wire length, 18 AWG
- R = 21 Ω/km = 0.021 Ω/m
- $R_{total} = 2 \times 2m \times 0.021 = 0.084Ω$
- $V_{drop} = 3A \times 0.084Ω = 0.252V$
- Percent drop: 0.252 / 12 = 2.1% ✓ (acceptable)

### 4.3 Wiring Harness Design

<!-- class="theory-concept" -->
**Connector Selection**

**Criteria**:
1. **Current Rating**: Exceed maximum expected by 20%+
2. **Voltage Rating**: Exceed operating voltage
3. **Durability**: Mating cycles (100+ for removable panels)
4. **Environmental**: IP rating for water/dust
5. **Polarization**: Prevent incorrect connection

**Common Connector Types**:
- **JST**: Low current (1-3A), signal connections
- **XT60/XT90**: High current (60A/90A), battery power
- **Anderson Powerpole**: Modular, 15-45A, genderless
- **Dupont**: Breadboard-style, 1A max
- **Molex Mini-Fit Jr.**: Robust, 9-13A per pin

<!-- class="theory-concept" -->
**Cable Management Best Practices**:

1. **Strain Relief**: Prevent stress at solder joints and connectors
2. **Bundling**: Cable sleeves, zip ties (not too tight!)
3. **Routing**: Away from moving parts, sharp edges
4. **Labeling**: Wire labels at both ends
5. **Color Coding**:
   - Red: Positive power
   - Black: Ground/negative
   - Other colors: Signals (document)
6. **Service Loops**: Extra length for repairs
7. **Separation**: High-power (motor) away from low-level signals (sensors)

**Quiz: Protection and Wiring**

A fuse should be rated at what percentage of the normal operating current?

[( )] 50-75%
[( )] 100%
[(X)] 125-150%
[( )] 200-300%

For safety, emergency stop buttons should use:

[(X)] Normally-closed contacts
[( )] Normally-open contacts
[( )] Latching push buttons
[( )] Software-based detection

---

## Summary

This course established comprehensive power system design capabilities:

1. **Battery Technology**: Examined electrochemistry, characteristics, and management of lead-acid, NiMH, and LiPo batteries
2. **Voltage Regulation**: Analyzed linear and switching topologies with efficiency and thermal considerations
3. **Protection**: Designed overcurrent, reverse polarity, and emergency stop circuits
4. **Thermal/Wiring**: Applied heat transfer theory and wire gauge selection to practical harness design

**Key Takeaways**:
- Battery selection involves tradeoffs between energy density, safety, cost, and cycle life
- Switching regulators offer superior efficiency but increased complexity
- Fail-safe design principles are essential for safety-critical systems
- Thermal management and proper wire sizing prevent failures and hazards

**Next Steps**: HARDWARE-205 integrates all hardware concepts in a comprehensive robot assembly project.

---

## Optional Laboratory Exercises

### Lab 1: Multi-Voltage Power Distribution Board (2.5 hours)

<!-- class="optional-exercise" -->
**Objective**: Design and build a power board providing 12V, 5V, and 3.3V from a LiPo battery with protection.

**Components**:
- 3S LiPo battery (11.1V nominal)
- LM2596 buck converter module (12V output)
- LM7805 linear regulator (5V output)
- AMS1117-3.3 LDO regulator (3.3V output)
- Fuses (appropriate ratings)
- XT60 connector (battery input)
- Terminal blocks (outputs)
- Perfboard

**Tasks**:
1. Design schematic with protection and filtering
2. Calculate power dissipation for each regulator
3. Select appropriate fuse ratings
4. Build circuit on perfboard
5. Test outputs under load
6. Measure efficiency of each stage

<!-- class="exercise-tip" -->
**Design Specifications**:
- 12V output: 1A max (buck converter from battery)
- 5V output: 500mA max (linear regulator from 12V)
- 3.3V output: 300mA max (LDO from 5V)
- Fuse main input for total current
- LED indicators for each voltage rail
- Electrolytic caps (100μF) on each output

**Analysis Questions**:
- What is the overall efficiency at full load?
- How much power is dissipated as heat in the linear regulators?
- What heatsinking is required for the 7805?
- What happens if 5V output is shorted?

### Lab 2: Emergency Stop System (1.5 hours)

<!-- class="optional-exercise" -->
**Objective**: Build a fail-safe emergency stop circuit controlling motor power.

**Components**:
- Emergency stop button (NC contacts)
- 12V relay (SPDT, 10A contacts)
- Diode (flyback protection across relay coil)
- DC motor
- LED indicators (power, motor active)
- 12V power supply

**Tasks**:
1. Design E-Stop circuit using NC button and relay
2. Wire circuit with flyback diode across relay coil
3. Add LED indicating motor power state
4. Test normal operation
5. Verify E-Stop function
6. Test fault modes:
   - Disconnect wire to E-Stop button
   - Short E-Stop button
   - Disconnect power to relay
7. Document which faults result in safe state

<!-- class="exercise-advanced" -->
**Advanced Extensions**:
- Add dual-channel redundant E-Stop
- Implement "enable" button requiring hold to run
- Create reset circuit requiring manual acknowledgment
- Add emergency stop counter (EEPROM) and serial reporting
- Design key switch for maintenance mode override

**Safety Verification Checklist**:
- [ ] E-Stop pressed stops motor
- [ ] E-Stop wire disconnected stops motor
- [ ] Relay power loss stops motor
- [ ] Visual indication of E-Stop state
- [ ] Reset required after E-Stop
- [ ] No automatic restart possible

### Lab 3: Battery Monitoring System (2 hours)

<!-- class="optional-exercise" -->
**Objective**: Build a battery monitoring system with state-of-charge estimation and protection.

**Components**:
- 3S LiPo battery
- INA219 current/voltage sensor (I2C)
- STM32 or Arduino
- OLED display
- Buzzer (low voltage alarm)

**Tasks**:
1. Interface INA219 to measure battery voltage and current
2. Implement coulomb counting for SoC estimation:
   $$\text{SoC}(t) = \text{SoC}(t_0) - \frac{1}{C} \int I(\tau) d\tau$$
3. Monitor individual cell voltages (if BMS has balancer tap)
4. Display on OLED:
   - Total voltage
   - Current (+ for discharge, - for charge)
   - Power (W)
   - State of Charge (%)
   - Estimated time remaining
5. Implement protection:
   - Low voltage alarm (<3.3V per cell)
   - High current warning (>C-rating)
   - Cell imbalance warning (>0.1V difference)

**Coulomb Counting Implementation**:
```cpp
float capacity = 2.2;  // Ah
float soc = 100.0;     // Start at 100%

void loop() {
    float current = ina219.getCurrent_mA() / 1000.0;  // A
    float dt = loopTime / 3600.0;  // hours

    // Update SoC (negative current is discharge)
    soc -= (current / capacity) * dt * 100.0;

    // Clamp to 0-100%
    soc = constrain(soc, 0, 100);
}
```

<!-- class="exercise-tip" -->
**Calibration Tips**:
- Fully charge battery before starting (100% SoC)
- Measure actual capacity by full discharge test
- Compensate for charging efficiency (~85%)
- Reset SoC to 100% when voltage reaches 4.2V/cell during charge
- Consider temperature effects on capacity

---

*Version 2.0.0 - Academic curriculum emphasizing theoretical foundations with optional practical exercises*
