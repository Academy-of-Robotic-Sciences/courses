<!--
author:   Dr. Michael Chen
email:    michael.chen@academy-of-robotic-sciences.github.io
version:  2.0.0

language: en
narrator: US English Male

comment:  Control Theory and System Architecture: Comprehensive treatment of classical and modern control theory, stability analysis, system architecture design, and requirements engineering for robotic systems.


mode:     Textbook

link:     https://raw.githubusercontent.com/Academy-of-Robotic-Sciences/courses/main/course-styles.css

import:   https://raw.githubusercontent.com/LiaTemplates/Pyodide/master/README.md
-->

# SYSTEMS-201: Control Theory and System Architecture

> **Rigorous control theory and systematic architecture design transform reactive machines into predictable, stable systems.**

## Course Overview

| | |
|---|---|
| **Course Code** | SYSTEMS-201 |
| **Duration** | 6 hours |
| **Level** | Advanced |
| **Prerequisites** | Differential equations, linear algebra, basic control concepts |
| **Theory-Practice** | 60% theory, 40% optional labs |

---

## Learning Objectives

Upon completion of this course, students will be able to:

### Theoretical Understanding

- Derive and analyze transfer functions for linear time-invariant (LTI) systems
- Apply Laplace transforms to solve differential equations describing system dynamics
- Analyze stability using Routh-Hurwitz criterion and root locus techniques
- Design PID controllers using frequency domain and time domain methods
- Understand state-space representation and controllability/observability
- Apply Lyapunov stability theory to nonlinear systems
- Explain system architecture patterns and their trade-offs

### Practical Skills

- Translate stakeholder requirements into formal engineering specifications
- Design system architectures with well-defined interfaces
- Perform trade-off analysis for component selection
- Create professional architecture diagrams and technical specifications
- Tune PID controllers using Ziegler-Nichols and other empirical methods
- Simulate control systems using MATLAB/Simulink or Python

---

## Module 1: Foundations of Control Theory

### 1.1 Introduction to Feedback Control

<!-- class="theory-concept" -->
**Open-Loop vs. Closed-Loop Systems**

**Open-Loop Control**: Commands sent to actuators without feedback
- Simple implementation
- No compensation for disturbances or model uncertainty
- Example: Stepper motor position control without encoders

**Closed-Loop (Feedback) Control**: Output measured and compared to desired value
- Compensates for disturbances and uncertainty
- Improved accuracy and robustness
- Requires sensors and stability analysis
- Example: PID-controlled motor with encoder feedback

**Basic Feedback Structure**:

```
         +------+       +--------+       +-------+
r(t) --->| - +  |------>| G_c(s) |------>| G(s)  |-----> y(t)
    ^    +------+  e(t) +--------+  u(t) +-------+
    |      -                                |
    |                                       |
    +---------------------------------------+
```

Where:
- r(t) = reference (desired output)
- e(t) = error = r(t) - y(t)
- u(t) = control signal
- y(t) = system output
- G(s) = plant (system being controlled)
- G_c(s) = controller

<!-- class="historical-note" -->
**Historical Development of Control Theory**

Feedback control has ancient origins (water clocks with float regulators, ~250 BCE), but modern control theory emerged during the Industrial Revolution:

**1788**: James Watt's flyball governor for steam engines demonstrated automatic speed regulation, though analysis was purely empirical.

**1868**: James Clerk Maxwell published the first mathematical analysis of governor stability, establishing feedback theory.

**1922**: Nicolas Minorsky developed PID control for ship steering, introducing the three-term controller still dominant today.

**1932**: Harry Nyquist developed frequency domain stability criteria, enabling systematic analysis without solving differential equations.

**1940s**: Hendrik Bode introduced logarithmic plots (Bode diagrams) for frequency response analysis.

**1948**: Walter R. Evans invented the root locus method, providing graphical insight into closed-loop pole locations.

**1960**: Rudolf Kalman introduced state-space methods and optimal filtering, launching modern control theory.

### 1.2 Mathematical Modeling: Differential Equations

<!-- class="theory-concept" -->
**System Dynamics**

Physical systems are modeled by differential equations relating inputs to outputs.

**Example: RC Circuit**

For a resistor-capacitor circuit:

$$RC\frac{dv_c}{dt} + v_c = v_{in}$$

where:
- $v_c$ = capacitor voltage (output)
- $v_{in}$ = input voltage
- R = resistance (Ω)
- C = capacitance (F)
- τ = RC = time constant

**First-Order System General Form**:

$$\tau\frac{dy}{dt} + y = Ku$$

where:
- y = output
- u = input
- K = steady-state gain
- τ = time constant

**Step Response**: For input u(t) = 1 (unit step):

$$y(t) = K(1 - e^{-t/\tau})$$

At t = τ: y = 63.2% of final value
At t = 5τ: y ≈ 99% of final value (considered settled)

<!-- class="theory-concept" -->
**Second-Order Systems**

Most mechanical systems exhibit second-order dynamics:

$$\frac{d^2y}{dt^2} + 2\zeta\omega_n\frac{dy}{dt} + \omega_n^2 y = \omega_n^2 u$$

where:
- $\omega_n$ = natural frequency (rad/s)
- ζ = damping ratio (dimensionless)

**Damping Classification**:
- ζ > 1: Overdamped (slow, no oscillation)
- ζ = 1: Critically damped (fastest response without overshoot)
- 0 < ζ < 1: Underdamped (oscillatory response)
- ζ = 0: Undamped (continuous oscillation)

**Performance Metrics**:
- Rise time $t_r$: Time to reach final value
- Settling time $t_s$: Time to stay within ±2% of final value
- Overshoot: $M_p = e^{-\pi\zeta/\sqrt{1-\zeta^2}}$ (for ζ < 1)
- Peak time $t_p = \pi/(\omega_n\sqrt{1-\zeta^2})$

**Quiz: System Dynamics**

A first-order system has time constant τ = 0.5s. After what time does it reach 95% of its final value?

[( )] 0.5s
[( )] 1.0s
[(X)] 1.5s
[( )] 2.5s

For a second-order system, which damping ratio provides the fastest response without overshoot?

[( )] ζ = 0
[( )] ζ = 0.5
[(X)] ζ = 1.0
[( )] ζ = 2.0

### 1.3 Laplace Transform and Transfer Functions

<!-- class="theory-concept" -->
**The Laplace Transform**

The Laplace transform converts differential equations to algebraic equations:

$$\mathcal{L}\{f(t)\} = F(s) = \int_0^\infty f(t)e^{-st}dt$$

where s = σ + jω is the complex frequency variable.

**Key Laplace Transforms**:

| Time Domain f(t) | Laplace Domain F(s) |
|------------------|---------------------|
| δ(t) (impulse) | 1 |
| 1 (step) | 1/s |
| t (ramp) | 1/s² |
| e^(-at) | 1/(s+a) |
| sin(ωt) | ω/(s²+ω²) |
| cos(ωt) | s/(s²+ω²) |

**Transform Properties**:
- Linearity: $\mathcal{L}\{af(t) + bg(t)\} = aF(s) + bG(s)$
- Differentiation: $\mathcal{L}\{\frac{df}{dt}\} = sF(s) - f(0)$
- Integration: $\mathcal{L}\{\int f(t)dt\} = \frac{F(s)}{s}$

<!-- class="theory-concept" -->
**Transfer Functions**

The transfer function G(s) is the Laplace transform of the impulse response:

$$G(s) = \frac{Y(s)}{U(s)}$$

where Y(s) and U(s) are Laplace transforms of output and input with zero initial conditions.

**First-Order Transfer Function**:

$$G(s) = \frac{K}{\tau s + 1}$$

**Second-Order Transfer Function**:

$$G(s) = \frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}$$

**DC Motor Example**:

Transfer function from voltage to angular velocity:

$$G(s) = \frac{\Omega(s)}{V(s)} = \frac{K_m}{\tau_m s + 1}$$

where:
- $K_m$ = motor gain (rad/s/V)
- $\tau_m$ = mechanical time constant

---

## Module 2: Classical Control: PID

### 2.1 Proportional-Integral-Derivative Control

<!-- class="theory-concept" -->
**The PID Controller**

The PID controller is the most widely used feedback controller:

$$u(t) = K_p e(t) + K_i \int_0^t e(\tau)d\tau + K_d \frac{de(t)}{dt}$$

where:
- $K_p$ = proportional gain
- $K_i$ = integral gain
- $K_d$ = derivative gain
- e(t) = error = r(t) - y(t)

**Transfer Function Form**:

$$G_c(s) = K_p + \frac{K_i}{s} + K_d s$$

**Alternative Representation**:

$$G_c(s) = K_p\left(1 + \frac{1}{T_i s} + T_d s\right)$$

where:
- $T_i = K_p/K_i$ = integral time constant
- $T_d = K_d/K_p$ = derivative time constant

<!-- class="theory-concept" -->
**Component Roles**

**Proportional (P)**: Responds to current error
- Reduces steady-state error
- Fast response
- Alone, leaves steady-state error for type 0 systems
- $u_p = K_p e(t)$

**Integral (I)**: Responds to accumulated error
- Eliminates steady-state error
- Compensates for constant disturbances
- Can cause overshoot and oscillation
- $u_i = K_i \int_0^t e(\tau)d\tau$

**Derivative (D)**: Responds to rate of change of error
- Provides damping
- Anticipates future error
- Sensitive to measurement noise
- Never used alone
- $u_d = K_d \frac{de(t)}{dt}$

<!-- class="historical-note" -->
**Evolution of PID Control**

**1922**: Nicolas Minorsky developed three-term control for automatic ship steering. His analysis of helmsman behavior revealed proportional, integral, and derivative actions.

**1930s**: Pneumatic PID controllers appeared in process industries (Taylor Instruments).

**1940s**: Electronic PID controllers emerged with operational amplifiers.

**1942**: Ziegler and Nichols published empirical tuning rules that remain widely used.

**1950s-1960s**: Digital implementation of PID in early computers.

**1980s-present**: Microcontroller-based PID became ubiquitous. Modern refinements include anti-windup, gain scheduling, and autotuning.

Despite 100+ years and many advanced alternatives, PID remains dominant due to simplicity, robustness, and sufficient performance for most applications.

### 2.2 PID Tuning Methods

<!-- class="theory-concept" -->
**Ziegler-Nichols Tuning**

**Method 1: Reaction Curve (Open-Loop)**

Apply step input, measure response:
- L = delay time
- T = time constant (approximated from slope)
- R = reaction rate = ΔOutput/(ΔInput × T)

**Tuning Rules**:

| Controller | $K_p$ | $T_i$ | $T_d$ |
|------------|-------|-------|-------|
| P | T/(RL) | - | - |
| PI | 0.9T/(RL) | L/0.3 | - |
| PID | 1.2T/(RL) | 2L | 0.5L |

**Method 2: Ultimate Gain (Closed-Loop)**

1. Remove I and D terms (P-only control)
2. Increase $K_p$ until sustained oscillation occurs
3. Record ultimate gain $K_u$ and oscillation period $P_u$

**Tuning Rules**:

| Controller | $K_p$ | $T_i$ | $T_d$ |
|------------|-------|-------|-------|
| P | 0.5$K_u$ | - | - |
| PI | 0.45$K_u$ | $P_u$/1.2 | - |
| PID | 0.6$K_u$ | $P_u$/2 | $P_u$/8 |

<!-- class="alternative-approach" -->
**Modern Tuning Methods**

**Cohen-Coon Method**: Better for systems with significant dead time

**IMC (Internal Model Control)**: Based on desired closed-loop time constant

**Relay Autotuning**: Automated version of Ziegler-Nichols ultimate gain method

**Optimization-Based**: Minimize cost function (ISE, IAE, ITAE)
- ISE (Integral of Squared Error): $\int_0^\infty e^2(t)dt$
- IAE (Integral of Absolute Error): $\int_0^\infty |e(t)|dt$
- ITAE (Integral of Time × Absolute Error): $\int_0^\infty t|e(t)|dt$

**Gain Scheduling**: Switch PID parameters based on operating point

### 2.3 Implementation Considerations

<!-- class="theory-concept" -->
**Digital PID Implementation**

Discrete-time PID (sample period T):

$$u[k] = K_p e[k] + K_i T \sum_{j=0}^k e[j] + K_d \frac{e[k] - e[k-1]}{T}$$

**Velocity Form** (computes change in control):

$$\Delta u[k] = K_p \Delta e[k] + K_i T e[k] + K_d \frac{e[k] - 2e[k-1] + e[k-2]}{T}$$

Advantages: No "bump" when switching to automatic, natural anti-windup.

<!-- class="theory-concept" -->
**Practical Refinements**

**Integral Windup**: Occurs when actuator saturates but integrator continues accumulating error.

**Anti-Windup Solutions**:
1. **Conditional Integration**: Stop integration when saturated
2. **Back-Calculation**: Reduce integral term when output saturates
3. **Clamping**: Limit integral term to prevent saturation

**Derivative Kick**: Step change in setpoint causes spike in derivative term.

**Solutions**:
1. Derivative on measurement (not error): $D(y)$ instead of $D(e)$
2. Setpoint filtering (ramp changes instead of steps)

**Measurement Noise**: Derivative term amplifies high-frequency noise.

**Solutions**:
1. Low-pass filter on derivative term
2. Use filtered derivative: $\frac{K_d s}{\tau_f s + 1}$
3. Reduce $K_d$ or eliminate derivative

**Quiz: PID Control**

In Ziegler-Nichols ultimate gain method, if $K_u$ = 2.0 and $P_u$ = 4.0s, what is $K_p$ for PID control?

[( )] 0.9
[( )] 1.0
[(X)] 1.2
[( )] 2.0

Which PID term eliminates steady-state error for step inputs?

[( )] Proportional
[(X)] Integral
[( )] Derivative
[( )] None, all three required

---

## Module 3: Stability Analysis

### 3.1 Poles, Zeros, and Stability

<!-- class="theory-concept" -->
**Poles and Zeros**

For transfer function:

$$G(s) = \frac{b_m s^m + \cdots + b_1 s + b_0}{a_n s^n + \cdots + a_1 s + a_0} = K\frac{(s-z_1)(s-z_2)\cdots(s-z_m)}{(s-p_1)(s-p_2)\cdots(s-p_n)}$$

**Poles** $p_i$: Roots of denominator (characteristic equation = 0)
**Zeros** $z_i$: Roots of numerator

**Pole Locations and Stability**:

A system is **stable** if all poles have negative real parts (left half of s-plane).

- **Left Half-Plane (Re(s) < 0)**: Stable, decaying response
- **Imaginary Axis (Re(s) = 0)**: Marginally stable, sustained oscillation
- **Right Half-Plane (Re(s) > 0)**: Unstable, growing response

**Closed-Loop Poles**:

For feedback system with plant G(s) and controller G_c(s):

$$T(s) = \frac{G_c(s)G(s)}{1 + G_c(s)G(s)H(s)}$$

Poles are roots of characteristic equation:

$$1 + G_c(s)G(s)H(s) = 0$$

<!-- class="theory-concept" -->
**Routh-Hurwitz Stability Criterion**

Determines stability without solving for roots explicitly.

For characteristic equation:

$$a_n s^n + a_{n-1} s^{n-1} + \cdots + a_1 s + a_0 = 0$$

Construct Routh array:

```
s^n    a_n      a_{n-2}    a_{n-4}    ...
s^{n-1} a_{n-1}  a_{n-3}    a_{n-5}    ...
s^{n-2} b_1      b_2        b_3        ...
...
s^1    ...
s^0    ...
```

where:
$$b_1 = \frac{a_{n-1}a_{n-2} - a_n a_{n-3}}{a_{n-1}}$$

**Stability Condition**: All elements in first column must have same sign. Number of sign changes = number of right half-plane poles.

**Example**: $s^3 + 6s^2 + 11s + 6 = 0$

```
s^3    1       11
s^2    6       6
s^1    10      0
s^0    6
```

All positive → stable.

### 3.2 Root Locus Method

<!-- class="theory-concept" -->
**Root Locus Construction**

The root locus shows how closed-loop poles move as gain K varies.

For characteristic equation: $1 + KG(s)H(s) = 0$

**Construction Rules** (Evans, 1948):

1. **Starting Points (K=0)**: Locus starts at open-loop poles
2. **Ending Points (K→∞)**: Locus ends at open-loop zeros or infinity
3. **Number of Branches**: n = number of open-loop poles
4. **Real Axis Segments**: Locus exists where total number of poles+zeros to the right is odd
5. **Asymptotes**:
   - Number: n - m (poles minus zeros)
   - Angles: $\frac{(2k+1)\pi}{n-m}$ for k = 0, 1, ..., n-m-1
   - Centroid: $\sigma_a = \frac{\sum \text{poles} - \sum \text{zeros}}{n - m}$
6. **Breakaway/Break-in Points**: Solve $\frac{dK}{ds} = 0$

**Design Using Root Locus**:
- Select K to place dominant poles at desired location
- Desired characteristics: damping ratio ζ, natural frequency $\omega_n$
- Lines of constant ζ are radial lines from origin: $\theta = \cos^{-1}(\zeta)$

### 3.3 Frequency Domain Analysis

<!-- class="theory-concept" -->
**Bode Plots**

Frequency response G(jω) plotted as:
- Magnitude: $20\log_{10}|G(j\omega)|$ (dB) vs. log(ω)
- Phase: $\angle G(j\omega)$ (degrees) vs. log(ω)

**First-Order System**: $G(s) = \frac{K}{\tau s + 1}$

Magnitude:
- Low frequency (ω << 1/τ): 0 dB (flat)
- High frequency (ω >> 1/τ): -20 dB/decade slope
- Corner frequency ω = 1/τ: -3 dB

Phase:
- Low frequency: 0°
- Corner frequency: -45°
- High frequency: -90°

**Gain Margin (GM)**: Amount of gain increase before instability
- Measured at phase crossover frequency (phase = -180°)
- $GM = \frac{1}{|G(j\omega_{pc})|}$ (linear) or $GM_{dB} = -20\log_{10}|G(j\omega_{pc})|$
- Typical requirement: GM > 6 dB

**Phase Margin (PM)**: Amount of phase lag before instability
- Measured at gain crossover frequency ($|G(j\omega_{gc})| = 1$)
- $PM = 180° + \angle G(j\omega_{gc})$
- Typical requirement: PM > 45°

Larger margins → more robust but slower response.

---

## Module 4: State-Space Methods

### 4.1 State-Space Representation

<!-- class="theory-concept" -->
**State Variables and State Equations**

Alternative to transfer functions, enables multi-input/multi-output (MIMO) analysis:

$$\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}$$
$$\mathbf{y} = \mathbf{C}\mathbf{x} + \mathbf{D}\mathbf{u}$$

where:
- $\mathbf{x}$ = state vector (n × 1)
- $\mathbf{u}$ = input vector (m × 1)
- $\mathbf{y}$ = output vector (p × 1)
- $\mathbf{A}$ = system matrix (n × n)
- $\mathbf{B}$ = input matrix (n × m)
- $\mathbf{C}$ = output matrix (p × n)
- $\mathbf{D}$ = feedthrough matrix (p × m)

**Example: Mass-Spring-Damper**

$$m\ddot{x} + c\dot{x} + kx = f$$

Define states: $x_1 = x$ (position), $x_2 = \dot{x}$ (velocity)

$$\begin{bmatrix} \dot{x}_1 \\ \dot{x}_2 \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ -k/m & -c/m \end{bmatrix} \begin{bmatrix} x_1 \\ x_2 \end{bmatrix} + \begin{bmatrix} 0 \\ 1/m \end{bmatrix} f$$

$$y = \begin{bmatrix} 1 & 0 \end{bmatrix} \begin{bmatrix} x_1 \\ x_2 \end{bmatrix}$$

<!-- class="theory-concept" -->
**Controllability and Observability**

**Controllability**: Can we move the system from any initial state to any final state?

Controllability matrix:
$$\mathcal{C} = \begin{bmatrix} \mathbf{B} & \mathbf{AB} & \mathbf{A}^2\mathbf{B} & \cdots & \mathbf{A}^{n-1}\mathbf{B} \end{bmatrix}$$

System is controllable if rank($\mathcal{C}$) = n.

**Observability**: Can we determine the internal state from output measurements?

Observability matrix:
$$\mathcal{O} = \begin{bmatrix} \mathbf{C} \\ \mathbf{CA} \\ \mathbf{CA}^2 \\ \vdots \\ \mathbf{CA}^{n-1} \end{bmatrix}$$

System is observable if rank($\mathcal{O}$) = n.

### 4.2 Lyapunov Stability Theory

<!-- class="theory-concept" -->
**Stability of Nonlinear Systems**

For nonlinear system: $\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x})$

**Lyapunov's Direct Method**: If we can find a positive definite "energy-like" function V(x) such that:
1. V(0) = 0 and V(x) > 0 for x ≠ 0
2. $\dot{V}(\mathbf{x}) \leq 0$ along trajectories

Then the system is stable.

If $\dot{V}(\mathbf{x}) < 0$ (strictly), the system is asymptotically stable.

**Linear Systems**: For $\dot{\mathbf{x}} = \mathbf{A}\mathbf{x}$, use quadratic Lyapunov function:

$$V(\mathbf{x}) = \mathbf{x}^T\mathbf{P}\mathbf{x}$$

System is stable if there exists positive definite P satisfying:

$$\mathbf{A}^T\mathbf{P} + \mathbf{P}\mathbf{A} = -\mathbf{Q}$$

for some positive definite Q (Lyapunov equation).

**Quiz: State-Space**

A system has state matrix A with eigenvalues at -1, -2, and +3. The system is:

[( )] Stable
[( )] Marginally stable
[(X)] Unstable
[( )] Cannot determine

If a system's controllability matrix has rank less than n, what can we conclude?

[( )] The system is unstable
[(X)] Some states cannot be controlled by the input
[( )] The system is not observable
[( )] The system is nonlinear

---

## Module 5: System Architecture Design

### 5.1 Requirements Engineering

<!-- class="theory-concept" -->
**Functional vs. Non-Functional Requirements**

**Functional Requirements**: What the system must do
- "The robot shall navigate to waypoint within ±10cm"
- "The system shall detect obstacles at 5m range"
- "The gripper shall apply 5-20N force"

**Non-Functional Requirements**: How well the system must perform
- Performance: "Response time < 100ms"
- Reliability: "MTBF > 1000 hours"
- Safety: "Emergency stop response < 50ms"
- Usability: "Setup time < 5 minutes"
- Cost: "Bill of materials < $10,000"

<!-- class="theory-concept" -->
**Requirements Elicitation**

**Stakeholder Analysis**:
1. Identify stakeholders (users, operators, maintainers, regulators)
2. Understand needs and constraints
3. Resolve conflicting requirements

**Use Cases**: Describe interactions between users and system
- Actor: Who uses the system
- Goal: What they want to accomplish
- Scenario: Step-by-step interaction

**Requirements Specification Format**:
- Unique identifier (REQ-001)
- Priority (must-have, should-have, nice-to-have)
- Verification method (test, analysis, inspection)
- Traceability to higher-level requirements

### 5.2 Architecture Patterns

<!-- class="theory-concept" -->
**Layered Architecture**

**Sense-Plan-Act Hierarchy**:

```
┌─────────────────┐
│  Planning Layer │  (High-level reasoning, slow)
├─────────────────┤
│ Executive Layer │  (Task sequencing, medium)
├─────────────────┤
│  Control Layer  │  (Real-time control, fast)
└─────────────────┘
```

**Advantages**:
- Clear separation of concerns
- Easier debugging and testing
- Natural decomposition

**Disadvantages**:
- Communication latency between layers
- Difficult to handle unexpected events at low levels

<!-- class="alternative-approach" -->
**Reactive (Behavior-Based) Architecture**

**Subsumption Architecture** (Brooks, 1986):
- Multiple parallel behaviors
- Higher-level behaviors "subsume" (suppress or override) lower levels
- No central planning
- Fast, robust to sensor noise

**Example**: Mobile robot
- Layer 0: Avoid obstacles (highest priority)
- Layer 1: Wander randomly
- Layer 2: Explore systematically
- Layer 3: Goal seeking (lowest priority)

**Advantages**:
- Fast response
- Robust, degrades gracefully
- Emergent complex behavior

**Disadvantages**:
- Hard to predict and verify
- Difficult to coordinate complex tasks

<!-- class="theory-concept" -->
**Hybrid (Three-Layer) Architecture**

Combines deliberative and reactive approaches:

```
┌──────────────────────┐
│ Deliberative Layer   │  Planning, slow (1-10s)
├──────────────────────┤
│ Sequencing Layer     │  Task execution (0.1-1s)
├──────────────────────┤
│ Reactive Layer       │  Sensor-motor (1-100ms)
└──────────────────────┘
```

Examples: 3T (Three Tiered), CLARAty (NASA), ROS Navigation Stack

### 5.3 Interface Design

<!-- class="theory-concept" -->
**Interface Specification**

Well-defined interfaces enable modular development:

**API (Application Programming Interface)**:
- Function signatures
- Data types and structures
- Error handling and exceptions
- Threading model and concurrency

**Message Interfaces** (ROS, middleware):
- Topic names and namespaces
- Message type definitions
- Quality of Service (QoS) policies
- Coordinate frames

**Hardware Interfaces**:
- Communication protocol (I2C, SPI, UART, Ethernet)
- Connector types and pinouts
- Voltage levels and current limits
- Timing diagrams

<!-- class="theory-concept" -->
**Interface Control Documents (ICD)**

Formal specification of interfaces between subsystems:

**Contents**:
1. Scope and purpose
2. Applicable documents and standards
3. Interface description
4. Data formats and protocols
5. Timing and performance requirements
6. Error handling
7. Verification requirements

**Benefits**:
- Enables parallel development
- Reduces integration risk
- Provides verification criteria
- Serves as reference documentation

### 5.4 Trade-Off Analysis

<!-- class="theory-concept" -->
**Decision Matrices**

Systematic evaluation of alternatives:

**Example: Sensor Selection**

| Option | Cost | Range | Accuracy | Power | Weight | Score |
|--------|------|-------|----------|-------|--------|-------|
| Weight | 10% | 20% | 30% | 25% | 15% | - |
| LiDAR A | 3 | 9 | 8 | 6 | 7 | 7.25 |
| LiDAR B | 7 | 7 | 9 | 5 | 6 | 6.95 |
| Stereo | 8 | 5 | 6 | 8 | 8 | 6.75 |

Rating scale: 1-10 (higher better)
Score = Σ(weight × rating)

**Sensitivity Analysis**: How do conclusions change if weights change?

<!-- class="alternative-approach" -->
**Pareto Optimization**

For multi-objective optimization, find Pareto frontier: solutions where improving one objective requires worsening another.

**Example**: Battery selection
- Objective 1: Maximize capacity (runtime)
- Objective 2: Minimize weight

Pareto-optimal solutions define trade-off curve. Final selection depends on relative importance.

---

## Summary

This course established foundations of control theory and system architecture:

1. **Control Fundamentals**: Analyzed feedback systems using differential equations and transfer functions
2. **PID Control**: Designed and tuned three-term controllers using classical methods
3. **Stability Analysis**: Applied Routh-Hurwitz, root locus, and frequency domain techniques
4. **State-Space Methods**: Analyzed MIMO systems, controllability, and Lyapunov stability
5. **Architecture Design**: Translated requirements into system architectures with defined interfaces

**Key Takeaways**:
- Feedback enables robustness despite uncertainty and disturbances
- PID remains dominant due to simplicity and effectiveness
- Stability must be verified analytically before implementation
- System architecture patterns provide proven solutions
- Requirements engineering prevents costly redesign

**Next Steps**: SYSTEMS-202 applies these concepts to integrate complex distributed systems using ROS2 and modern middleware.

---

## Optional Laboratory Exercises

### Lab 1: PID Control Simulation (2 hours)

<!-- class="optional-exercise" -->
**Objective**: Design, simulate, and tune a PID controller for a second-order system.

**System**: DC motor with inertia
$$G(s) = \frac{\Omega(s)}{V(s)} = \frac{10}{s(s+5)}$$

**Tasks**:
1. Simulate open-loop step response (MATLAB/Python)
2. Design P, PI, and PID controllers using Ziegler-Nichols
3. Compare performance (rise time, overshoot, settling time, steady-state error)
4. Add disturbance torque, evaluate disturbance rejection
5. Investigate effects of measurement noise on derivative term

<!-- class="exercise-tip" -->
**Python Implementation Example**:

```python
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

# System: G(s) = 10/(s(s+5))
num = [10]
den = [1, 5, 0]
sys = signal.TransferFunction(num, den)

# Step response
t, y = signal.step(sys)
plt.plot(t, y)
plt.xlabel('Time (s)')
plt.ylabel('Output')
plt.title('Open-Loop Step Response')
plt.grid(True)
plt.show()

# PID controller
Kp, Ki, Kd = 2.0, 1.0, 0.5  # Tune these values
num_c = [Kd, Kp, Ki]
den_c = [1, 0]
controller = signal.TransferFunction(num_c, den_c)

# Closed-loop system
sys_cl = signal.feedback(controller * sys)
t, y = signal.step(sys_cl)
```

**Target Performance**:
- Overshoot < 20%
- Settling time < 2s
- Zero steady-state error
- Gain margin > 6 dB
- Phase margin > 45°

### Lab 2: Root Locus Design (1.5 hours)

<!-- class="optional-exercise" -->
**Objective**: Use root locus to design a compensator for desired performance.

**System**:
$$G(s) = \frac{1}{s(s+2)(s+5)}$$

**Requirements**:
- Damping ratio ζ ≥ 0.5
- Settling time $t_s$ ≤ 2s (2% criterion)

**Tasks**:
1. Plot root locus for proportional gain only
2. Determine if P-control can meet requirements
3. Design lead compensator: $G_c(s) = K\frac{s+z}{s+p}$ where p > z
4. Plot root locus with compensator
5. Select K to place dominant poles at desired location
6. Verify performance with step response simulation

<!-- class="exercise-advanced" -->
**Advanced Extensions**:
- Design PID using root locus (PID = two zeros, one pole at origin)
- Investigate effect of additional pole from derivative filter
- Design notch filter to cancel problematic zeros
- Compare frequency domain performance (Bode plots)

### Lab 3: System Architecture Design Project (2.5 hours)

<!-- class="optional-exercise" -->
**Objective**: Design complete system architecture for a complex robotic application.

**Scenario**: Autonomous UV-C Disinfection Robot for Hospital Rooms

**Given Constraints**:
- Room size: 5m × 5m
- Disinfection requirement: 99% surface coverage
- Battery life: 2 hours continuous operation
- Budget: <$15,000
- Safety: Must detect humans and stop immediately

**Tasks**:

1. **Requirements Analysis** (30 min):
   - List 20+ functional requirements
   - List 10+ non-functional requirements
   - Prioritize requirements (must/should/could have)

2. **Component Selection** (45 min):
   - Compute platform: Raspberry Pi vs. Jetson vs. Industrial PC
   - Sensors: LiDAR, cameras, ultrasonic, IMU
   - UV-C lamp: Power, coverage, safety
   - Battery: Capacity, voltage, weight
   - Create decision matrix for each major component

3. **Architecture Design** (60 min):
   - Create system block diagram (draw.io or similar)
   - Define all major software modules
   - Specify ROS2 topics/services between modules
   - Choose architecture pattern (layered, reactive, hybrid)

4. **Interface Specification** (15 min):
   - Define 5+ key interfaces
   - Message types, coordinate frames, QoS policies
   - Error handling strategy

<!-- class="exercise-tip" -->
**Deliverables**:
- Requirements document (table format)
- Component selection matrices with justifications
- System architecture diagram (PDF or image)
- Interface Control Document (ICD) outline

**Evaluation Criteria**:
- Completeness of requirements
- Justification of component choices
- Clarity of architecture diagram
- Feasibility of design

---

*Version 2.0.0 - Academic curriculum emphasizing theoretical foundations with optional practical exercises*
