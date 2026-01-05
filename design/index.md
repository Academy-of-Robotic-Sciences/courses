<!--
author:   Dr. Maria Torres
email:    maria.torres@robotcampus.dev
version:  2.0.0

language: en
narrator: US English Female

comment:  Design Track Overview: Introduction to computer-aided design, mechanical engineering principles, kinematics, and manufacturing processes for robotic systems.

icon:     https://robotcampus.dev/logos/design-track.png

mode:     Textbook

link:     https://robotcampus.dev/styles/course-styles.css

-->

# Design Track: Robotic Mechanical Engineering

> **From fundamental geometric principles to advanced mechatronic systems, this track explores the theoretical foundations and practical methodologies of robot design.**

## Track Overview

| | |
|---|---|
| **Track** | Robotic Design Engineering |
| **Duration** | 24 hours (5 courses + capstone) |
| **Level** | Intermediate |
| **Prerequisites** | Linear algebra, basic physics, spatial reasoning |
| **Theory-Practice** | 60% theory, 40% optional labs |

---

## Track Objectives

This specialization track provides comprehensive training in the mechanical design, kinematic analysis, and manufacturing processes essential for modern robotics engineering. Students will develop both theoretical understanding and practical skills in computer-aided design, structural analysis, and prototyping methodologies.

### Theoretical Competencies

Upon completion, students will be able to:

- Explain the mathematical foundations of parametric solid modeling and constraint-based design
- Derive kinematic equations for serial and parallel manipulators using Denavit-Hartenberg notation
- Apply stress-strain analysis and failure theories to evaluate mechanical component strength
- Understand the historical evolution of CAD systems and additive manufacturing technologies
- Analyze manufacturing processes and their implications for design constraints

### Practical Skills

Students will gain experience with:

- Professional CAD software (Fusion 360) for parametric solid modeling
- Finite Element Analysis (FEA) for structural optimization
- Additive manufacturing workflow from digital design to physical fabrication
- Assembly modeling with kinematic constraints and motion simulation
- Design iteration and rapid prototyping methodologies

---

## Curriculum Structure

### DESIGN-201: Parametric Solid Modeling Fundamentals (4 hours)

<!-- class="theory-concept" -->
**Core Topics**:
- History of CAD systems: From drafting boards to parametric modeling
- Geometric constraint solving and variational geometry
- Boolean operations and boundary representation (B-rep)
- Sketch plane theory and constraint satisfaction
- Mathematical foundations of extrusion and revolution operations

**Optional Lab**: Design and export a robot gripper finger component

### DESIGN-202: Additive Manufacturing Theory and Practice (4 hours)

<!-- class="theory-concept" -->
**Core Topics**:
- Physics of material extrusion processes
- Slicing algorithms and toolpath generation
- Thermodynamics of polymer deposition and layer bonding
- Material properties: PLA, PETG, ABS comparison
- Process parameter optimization: layer height, infill patterns, supports

**Optional Lab**: 3D print and post-process a mechanical component

### DESIGN-203: Structural Mechanics and Analysis (4 hours)

<!-- class="theory-concept" -->
**Core Topics**:
- Stress-strain relationships and material constitutive models
- Finite Element Method (FEM) mathematical foundations
- Von Mises stress criterion and factor of safety analysis
- Failure modes: tensile, compressive, shear, buckling, fatigue
- Design optimization theory and topology optimization

**Optional Lab**: FEA simulation and structural redesign

### DESIGN-204: Kinematics and Parametric Assemblies (4 hours)

<!-- class="theory-concept" -->
**Core Topics**:
- Denavit-Hartenberg kinematic representation
- Homogeneous transformations and coordinate frames
- Joint types: revolute, prismatic, spherical, cylindrical
- Degrees of freedom analysis (Grübler's equation)
- Parametric design systems and design intent capture

**Optional Lab**: Multi-part gripper assembly with motion simulation

### DESIGN-205: Capstone Design Project (8 hours)

<!-- class="theory-concept" -->
**Comprehensive Application**:

The capstone project integrates all theoretical and practical knowledge from the track. Students complete a full design cycle from requirements analysis through fabrication and testing.

**Project Phases**:
1. **Requirements Analysis**: Functional decomposition and constraint specification
2. **Conceptual Design**: Multiple solution generation and evaluation
3. **Detailed Design**: CAD modeling with parametric control
4. **Analysis**: FEA validation of structural requirements
5. **Manufacturing**: Additive fabrication and assembly
6. **Testing**: Functional validation and performance assessment

**Deliverables**: Working prototype with supporting documentation

---

## Theoretical Foundations

### Historical Context

<!-- class="historical-note" -->
**Evolution of Computer-Aided Design**

The development of CAD systems revolutionized mechanical engineering:

- **1960s**: Early computer graphics systems (Sketchpad, Ivan Sutherland, 1963)
- **1970s**: First commercial CAD systems for 2D drafting
- **1980s**: Solid modeling with CSG and B-rep representations
- **1990s**: Parametric modeling (Pro/ENGINEER, 1988; SolidWorks, 1995)
- **2000s**: Direct modeling and synchronous technology
- **2010s**: Cloud-based CAD and generative design
- **2020s**: AI-assisted design and real-time collaboration

**Additive Manufacturing Timeline**:

- **1984**: Stereolithography (SLA) invented by Chuck Hull
- **1988**: Fused Deposition Modeling (FDM) patented by Scott Crump
- **1990s**: Industrial adoption for rapid prototyping
- **2000s**: Selective Laser Sintering (SLS) for metal parts
- **2009**: FDM patents expire, RepRap open-source movement
- **2010s**: Consumer 3D printer revolution
- **2020s**: Metal printing, bioprinting, construction-scale applications

### Mathematical Foundations

<!-- class="theory-concept" -->
**Parametric Curves and Surfaces**

CAD systems represent geometry using mathematical formulations:

**Bézier Curves** (degree n):
$$\mathbf{P}(t) = \sum_{i=0}^{n} B_{i,n}(t) \mathbf{P}_i, \quad t \in [0,1]$$

where $B_{i,n}(t) = \binom{n}{i} t^i (1-t)^{n-i}$ are Bernstein polynomials.

**B-Spline Surfaces**:
$$\mathbf{S}(u,v) = \sum_{i=0}^{m} \sum_{j=0}^{n} N_{i,p}(u) N_{j,q}(v) \mathbf{P}_{i,j}$$

where $N_{i,p}$ are B-spline basis functions.

**Denavit-Hartenberg Parameters**:

For robot kinematic chains, transformation from frame i-1 to frame i:

$$T_i = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

**Stress-Strain Relationships** (Hooke's Law for linear elasticity):

$$\boldsymbol{\sigma} = \mathbf{C} : \boldsymbol{\epsilon}$$

where:
- **σ** is the stress tensor
- **ε** is the strain tensor
- **C** is the fourth-order elasticity tensor

For isotropic materials:
$$\sigma = E \epsilon \quad \text{(uniaxial)}$$

where E is Young's modulus.

---

## Design Methodology

<!-- class="theory-concept" -->
**Systematic Design Process**

The track follows a rigorous engineering design methodology:

1. **Problem Definition**: Establish requirements, constraints, and success criteria
2. **Conceptual Design**: Generate multiple solutions, evaluate trade-offs
3. **Embodiment Design**: Select materials, establish dimensions, create preliminary CAD
4. **Detail Design**: Complete geometric definition, tolerance specification
5. **Analysis**: Validate design through simulation (FEA, kinematics)
6. **Prototype**: Fabricate physical model for testing
7. **Test**: Validate against requirements, identify failure modes
8. **Iterate**: Refine design based on empirical results

This iterative process mirrors professional engineering practice.

---

## Software and Tools

### Primary Software
- **CAD Platform**: Autodesk Fusion 360 (educational license)
- **Slicer Software**: Ultimaker Cura
- **Analysis**: Fusion 360 integrated FEA

### Hardware Resources
- Prusa i3 MK3S+ FDM 3D printers
- Precision measurement tools (calipers, micrometers)
- SO-101 robot arm (test platform)

---

## Assessment Philosophy

<!-- class="theory-concept" -->
**Competency-Based Evaluation**

Assessment focuses on demonstrating understanding through both theoretical mastery and practical application:

**Theory Components** (60%):
- Integrated quizzes testing conceptual understanding
- Mathematical problem-solving
- Design decision rationale
- Analysis interpretation

**Practice Components** (40%):
- Optional laboratory exercises
- Design artifacts (CAD models, simulation results)
- Physical prototypes
- Documentation quality

The capstone project synthesizes all competencies into a single comprehensive deliverable.

---

## Learning Pathway

### Prerequisites
Students should have:
- Strong spatial reasoning and 3D visualization skills
- Basic understanding of physics (statics, material properties)
- Familiarity with linear algebra and trigonometry
- Comfort with technical software tools

### Recommended Sequence
1. Complete RC-101 (Foundation Sprint) for robotics context
2. DESIGN-201: Build CAD fundamentals
3. DESIGN-202: Understand manufacturing constraints
4. DESIGN-203: Learn analysis and optimization
5. DESIGN-204: Master complex assemblies
6. DESIGN-205: Demonstrate comprehensive capability

### Post-Track Opportunities

<!-- class="alternative-approach" -->
**Advanced Specializations**:

- **Generative Design**: AI-assisted topology optimization
- **Advanced Manufacturing**: Multi-material printing, CNC machining
- **Mechanism Design**: Linkages, gear trains, cam systems
- **Systems Engineering**: Integration with electronics and software
- **Research Projects**: Novel end-effector designs, bio-inspired mechanisms

**Cross-Disciplinary Integration**:
- Hardware Track: Design enclosures for electronic systems
- Software Track: CAD API programming, parametric automation
- AI Track: Design for robot perception (camera mounting, sensor integration)

---

## Course Preparation Quiz

Test your readiness for the Design Track:

**What is the primary advantage of parametric modeling over direct modeling?**

[( )] Parametric modeling produces better-looking designs
[(X)] Parametric modeling captures design intent and enables rapid iteration
[( )] Parametric modeling requires less computational resources
[( )] Parametric modeling is easier to learn

**In stress analysis, what does the term "factor of safety" represent?**

[( )] The percentage of the part that is structurally sound
[(X)] The ratio of material strength to applied stress
[( )] The probability that a part will fail
[( )] The number of tests required before production

**Which manufacturing process builds parts layer-by-layer from digital models?**

[( )] CNC machining
[( )] Injection molding
[(X)] Additive manufacturing
[( )] Casting

**The Denavit-Hartenberg convention is used to describe:**

[( )] CAD file formats
[(X)] Kinematic transformations between robot joints
[( )] Material properties
[( )] 3D printing parameters

---

## Summary

The Design Track provides comprehensive training in the mechanical engineering aspects of robotics, from fundamental CAD skills through advanced kinematic analysis and structural optimization. Students develop both theoretical understanding and practical capabilities, preparing them for professional work in mechatronic system design.

**Next Steps**: Begin with DESIGN-201 to establish foundational CAD skills, or review prerequisite mathematics if needed.

---

*Version 2.0.0 - Academic curriculum emphasizing theoretical foundations with optional practical exercises*
