<!--
author:   Dr. Maria Torres
email:    maria.torres@academy-of-robotic-sciences.github.io
version:  2.0.0

language: en
narrator: US English Female

comment:  Capstone Design Project: Comprehensive integration of CAD, analysis, optimization, kinematics, and manufacturing for a complete robot end-effector design cycle.


mode:     Textbook

link:     https://raw.githubusercontent.com/Academy-of-Robotic-Sciences/courses/main/course-styles.css

-->

# DESIGN-205: Capstone Design Project

> **The capstone project synthesizes theoretical knowledge and practical skills into a complete engineering design cycle from requirements through validation.**

## Project Overview

| | |
|---|---|
| **Course Code** | DESIGN-205 |
| **Duration** | 8 hours |
| **Level** | Capstone |
| **Prerequisites** | DESIGN-201 through DESIGN-204 |
| **Theory-Practice** | 40% methodology, 60% guided project |

---

## Learning Objectives

Upon completion of this capstone, students will be able to:

### Comprehensive Integration

- Apply systematic engineering design methodology to complex problems
- Integrate CAD modeling, FEA simulation, and kinematic analysis
- Navigate trade-offs between conflicting design objectives
- Document design decisions with technical justification
- Validate designs through simulation and physical testing
- Iterate designs based on empirical results

### Professional Skills

- Work independently on open-ended design challenges
- Manage time and resources for multi-phase projects
- Communicate technical concepts through documentation
- Present design solutions to technical audiences
- Evaluate risk and make informed engineering judgments

---

## Module 1: Engineering Design Methodology

### 1.1 Systematic Design Process

<!-- class="theory-concept" -->
**Structured Approach to Complex Problems**

The design process follows a systematic framework adapted from Pahl & Beitz, Ulrich & Eppinger, and modern mechatronic design methodologies.

**Phase 1: Problem Definition and Requirements Analysis**

**Functional Requirements** (WHAT the design must achieve):
- Primary function specification
- Performance metrics (quantitative)
- Operating environment constraints
- Interface requirements

**Non-Functional Requirements** (HOW well it must perform):
- Safety and reliability
- Manufacturability
- Cost constraints
- Aesthetic considerations

**Requirements Format**:
- **Demands**: Must be satisfied (hard constraints)
- **Wishes**: Desirable but negotiable (soft constraints)

**Example: Robot Gripper Requirements**

| ID | Type | Requirement | Specification |
|----|------|-------------|---------------|
| F1 | Demand | Grasp cylindrical objects | Diameter: 20-50mm |
| F2 | Demand | Maintain grip during motion | Grip force: ≥10N |
| F3 | Demand | Interface with SO-101 | Mounting: M4 bolt pattern |
| NF1 | Demand | Safety factor | SF ≥ 2.0 |
| NF2 | Demand | Fabrication method | FDM 3D printing (PLA/PETG) |
| NF3 | Wish | Mass | < 50g |
| NF4 | Wish | Aesthetic | Organic, bioinspired form |

<!-- class="historical-note" -->
**Evolution of Design Methodologies**

- **1960s**: Systematic design methods emerge (Pahl & Beitz, Germany)
- **1970s**: Value engineering, design for manufacturing (DFM)
- **1980s**: Concurrent engineering, design for assembly (DFA)
- **1990s**: Design for X (DFX), QFD (Quality Function Deployment)
- **2000s**: TRIZ (Theory of Inventive Problem Solving)
- **2010s**: Design thinking, human-centered design
- **2020s**: Generative design, AI-assisted optimization

### 1.2 Conceptual Design

<!-- class="theory-concept" -->
**Solution Space Exploration**

**Functional Decomposition**:
Break complex function into sub-functions:

```
Main Function: Grasp Object
├── Sub-function 1: Generate closing motion
│   ├── Linear actuator
│   ├── Rotary actuator + linkage
│   └── Shape memory alloy
├── Sub-function 2: Contact object
│   ├── Rigid fingers
│   ├── Compliant surfaces
│   └── Adaptive gripper
└── Sub-function 3: Maintain grip
    ├── Passive (friction, geometry)
    ├── Active (continuous actuation)
    └── Locking mechanism
```

**Morphological Matrix**:
Combine sub-solutions into complete concepts:

| Sub-function | Option A | Option B | Option C |
|--------------|----------|----------|----------|
| Actuation | Servo motor | Pneumatic | SMA wire |
| Transmission | Linkage | Gear | Direct |
| Finger type | Parallel jaw | Adaptive | Underactuated |
| Contact surface | Rigid | Rubber pad | Soft gripper |

Each column combination = one complete concept.

**Concept Evaluation** (Pugh Matrix):
- Establish evaluation criteria (weight, complexity, reliability, cost)
- Score each concept (1-5 or relative to baseline)
- Weight criteria by importance
- Calculate weighted sum

**Concept Selection**:
Choose 1-2 concepts for detailed design based on:
- Technical feasibility
- Requirement satisfaction
- Risk assessment
- Available resources

### 1.3 Embodiment Design

<!-- class="theory-concept" -->
**From Concept to Preliminary CAD**

**Tasks**:
1. **Material Selection**: Based on loading, environment, cost
2. **Preliminary Sizing**: Rough dimensions from analytical calculations
3. **Layout Design**: Spatial arrangement of components
4. **Interface Definition**: Connections, fasteners, tolerances

**Design for Manufacturing** (DFM for FDM):
- Minimize support structures (orient parts favorably)
- Maintain minimum wall thickness (≥ 0.8mm for 0.4mm nozzle)
- Avoid enclosed cavities (powder removal impossible)
- Design self-supporting features (45° overhang rule)
- Consider assembly direction (reduce assembly steps)

**Design for Assembly** (DFA):
- Minimize part count (combine features where possible)
- Standard fasteners (M3, M4 screws common)
- Clearance for tools (screwdriver access)
- Alignment features (pins, chamfers for easy insertion)

---

## Module 2: Detailed Design and Analysis

### 2.1 Parametric CAD Modeling

<!-- class="theory-concept" -->
**Intelligent Geometry Definition**

**Parameter Hierarchy**:
```
Global Parameters (System-level)
├── gripper_width_max = 80mm
├── gripper_width_min = 20mm
├── finger_length = 100mm
└── material_thickness = 4mm

Derived Parameters (Calculated)
├── jaw_opening = gripper_width_max - gripper_width_min
├── link_length = f(jaw_opening, geometry)
└── mass_budget = density × volume
```

**Design Intent Capture**:
- Use equations: `hole_spacing = mounting_width - 2 × edge_distance`
- Symmetry constraints: Mirror features automatically
- Adaptive patterns: Hole count = f(finger_length)

**Best Practices**:
- Name parameters descriptively (`finger_width`, not `d1`)
- Add units in description (`mm`, `deg`, `N`)
- Group related parameters (geometry, material, performance)
- Document assumptions in comments

### 2.2 Finite Element Analysis

<!-- class="theory-concept" -->
**Simulation-Driven Design**

**Load Cases**:
Define realistic loading scenarios:
1. **Maximum grip force**: Object at minimum diameter, maximum friction
2. **Impact loading**: Sudden contact (dynamic load factor ≈ 2×)
3. **Worst-case orientation**: Gravity + grip force combined

**Boundary Conditions**:
- **Constraints**: Fixed mounting interface
- **Loads**: Distributed contact force on grip surface
- **Contacts**: Bonded (fingers to base), frictionless (object to finger)

**Material Models**:
For 3D printed parts, use orthotropic model if available:
- $E_x = E_y \approx 3.5$ GPa (PLA, in-plane)
- $E_z \approx 0.75 E_x$ (cross-layer)
- $G_{xy}, G_{xz}, G_{yz}$ from E and ν

Or conservative isotropic with reduced E.

**Mesh Convergence**:
1. Start with coarse global mesh (5mm elements)
2. Refine high-stress regions (contact areas, fillets)
3. Check convergence: max stress changes < 5% with 50% finer mesh

**Validation Criteria**:
- Von Mises stress: σ_VM < σ_y / SF
- Safety factor: SF ≥ 2.0 throughout
- Maximum displacement: δ_max < tolerance (e.g., < 0.5mm)

### 2.3 Kinematic Analysis

<!-- class="theory-concept" -->
**Motion Validation**

**Assembly Motion Study**:
1. Define joint limits based on physical stops
2. Animate full range of motion (minimum to maximum opening)
3. Check for:
   - Self-collision (finger-to-finger)
   - Interference with base or mounting
   - Clearance for object insertion/removal
4. Verify symmetric motion (both fingers move equally)

**Workspace Analysis**:
- Graspable object space: Sweep fingers through motion range
- Approach angle constraints: Limited by geometry
- Force closure analysis: Can grip be maintained in all orientations?

**Mechanical Advantage**:
If using linkage transmission:
$$MA = \frac{F_{\text{grip}}}{F_{\text{actuator}}} = \frac{r_{\text{input}}}{r_{\text{output}}}$$

Higher MA → more grip force, but less motion range.

---

## Module 3: Optimization and Iteration

### 3.1 Design Iteration Methodology

<!-- class="theory-concept" -->
**Systematic Improvement**

**Iteration Loop**:
```
1. Identify weakness (high stress, excessive mass, etc.)
2. Hypothesize solution (add material, change geometry, etc.)
3. Implement change in CAD
4. Analyze (FEA, kinematics, mass properties)
5. Evaluate against requirements
6. Document result
7. Repeat if not satisfactory
```

**Common Improvements**:
- **Stress reduction**: Add fillets, increase thickness locally, ribs
- **Mass reduction**: Remove material from low-stress regions, hollow sections
- **Stiffness increase**: Gussets, I-beam profiles, triangulation
- **Interference resolution**: Chamfers, clearance cuts, reorientation

**Version Control**:
- Save design states (V1, V2, V3, etc.)
- Document changes in each version
- Compare performance metrics across versions

### 3.2 Multi-Objective Optimization

<!-- class="alternative-approach" -->
**Balancing Competing Goals**

**Objectives** (typically conflicting):
- Minimize mass
- Maximize strength (safety factor)
- Minimize cost (material, print time)
- Maximize workspace

**Pareto Frontier**:
No single "optimal" solution—trade-off curve.

**Example**:
- Design A: 30g, SF=2.5 (strong but heavy)
- Design B: 20g, SF=2.1 (lighter but marginal safety)
- Design C: 25g, SF=2.3 (balanced compromise)

Choice depends on application priority (weight-critical vs. reliability-critical).

**Optimization Strategies**:
1. **Manual**: Iterative refinement by designer
2. **Parametric sweep**: Vary key parameter, plot performance
3. **Topology optimization**: Algorithmic material distribution
4. **Multi-objective genetic algorithm**: Automated search (advanced)

---

## Module 4: Manufacturing and Validation

### 4.1 Fabrication Planning

<!-- class="theory-concept" -->
**From CAD to Physical Part**

**Pre-Print Checklist**:
- [ ] All dimensions verified against requirements
- [ ] Tolerances adjusted for FDM (± 0.2mm typical)
- [ ] STL exported with fine resolution (0.01mm chord tolerance)
- [ ] Mesh verified (watertight, manifold, correct normals)
- [ ] Print orientation optimized (minimize supports, align strength)

**Slicing Strategy**:
- **Layer height**: 0.2mm (balance quality/time) or 0.15mm (critical surfaces)
- **Infill**: 30-50% (functional parts), gyroid or honeycomb pattern
- **Perimeters**: 3-4 walls for strength
- **Supports**: Tree supports for easy removal, 10-15% density

**Print Time Budget**:
- Estimate: 2-4 hours per gripper finger (100mm × 20mm × 5mm)
- Account for: Base plate, mounting hardware, test pieces
- Total: 6-8 hours printing time

**Material Selection**:
- **PLA**: Easy print, low warp, sufficient for testing
- **PETG**: Higher strength, better layer adhesion, more reliable
- **ABS**: High-temp applications (if enclosure available)

### 4.2 Assembly and Integration

<!-- class="theory-concept" -->
**Component Integration**

**Assembly Sequence**:
1. Print all components
2. Post-process (remove supports, sand contact surfaces)
3. Test-fit before final assembly
4. Install fasteners (M3/M4 screws, nuts/inserts)
5. Verify joint motion (smooth, no binding)
6. Mount to SO-101 interface

**Tolerance Stack-Up**:
Accumulated dimensional variation:
$$\Delta_{\text{total}} = \sqrt{\Delta_1^2 + \Delta_2^2 + \ldots + \Delta_n^2}$$

For critical fits, design clearance > Δ_total.

### 4.3 Testing and Validation

<!-- class="theory-concept" -->
**Empirical Verification**

**Functional Tests**:
1. **Range of Motion**: Minimum to maximum opening measurement
2. **Grip Force**: Load cell or spring scale measurement
3. **Object Grasping**: Success rate across object size range
4. **Reliability**: 100 grasp/release cycles without failure
5. **Safety**: No sharp edges, secure fasteners

**Comparison to Simulation**:
- Measure actual vs. predicted deformation under load
- Compare observed failure mode to FEA prediction
- Calculate error: typical 10-30% difference (acceptable if conservative)

**Acceptance Criteria**:
All demands from requirements must be met.
- [ ] Grasps specified object size range
- [ ] Maintains grip force ≥ requirement
- [ ] No structural failure (SF ≥ 2.0 validated)
- [ ] Interfaces correctly with SO-101
- [ ] Fabricated within constraints (FDM, material budget)

**Documentation**:
- Photos of physical prototype
- Test data (measurements, observations)
- Comparison table (requirements vs. actual performance)
- Failure mode analysis (if any)

---

## Module 5: Professional Documentation

### 5.1 Technical Report Structure

<!-- class="theory-concept" -->
**Communicating Engineering Work**

**Report Outline**:

1. **Executive Summary** (1 page)
   - Problem statement
   - Solution approach
   - Key results
   - Conclusion

2. **Introduction**
   - Background and motivation
   - Objectives
   - Scope and limitations

3. **Requirements Analysis**
   - Functional requirements table
   - Non-functional requirements
   - Interface specifications

4. **Conceptual Design**
   - Functional decomposition
   - Concept generation (morphological matrix)
   - Concept evaluation (Pugh chart)
   - Selected concept rationale

5. **Detailed Design**
   - CAD models (isometric views, dimensioned drawings)
   - Material selection justification
   - Parametric design table
   - Assembly exploded view

6. **Analysis and Optimization**
   - FEA setup and results (stress plots, SF values)
   - Kinematic analysis (workspace, motion study)
   - Optimization iterations (version comparison)
   - Final design validation

7. **Manufacturing**
   - Fabrication process (slicing settings, print parameters)
   - Assembly procedure
   - Quality control measures

8. **Testing and Validation**
   - Test setup and procedure
   - Results (data tables, graphs)
   - Comparison to requirements
   - Discussion of discrepancies

9. **Conclusion**
   - Summary of achievements
   - Lessons learned
   - Future improvements
   - Recommendations

10. **Appendices**
    - Bill of materials
    - Complete parameter table
    - Detailed FEA mesh information
    - Raw test data

**Figures and Tables**:
- All figures numbered and captioned
- Referenced in text ("See Figure 3...")
- High-resolution CAD screenshots
- Clear, readable fonts (≥ 10pt)

### 5.2 Design Presentation

<!-- class="theory-concept" -->
**Oral Communication**

**Presentation Structure** (10-15 minutes):

1. **Problem** (2 min)
   - What challenge are we solving?
   - Why is it important?
   - Key requirements

2. **Solution** (3 min)
   - Concept overview
   - Key design features
   - How it works (animation/video)

3. **Analysis** (3 min)
   - FEA results (one stress plot, interpretation)
   - Kinematic validation (motion study clip)
   - Performance metrics

4. **Results** (3 min)
   - Physical prototype (photos or bring real part)
   - Test results vs. requirements
   - Success demonstration (video of gripper in action)

5. **Conclusion** (2 min)
   - Achievements
   - Challenges overcome
   - Future work

**Tips**:
- Use visuals (images, videos, animations)
- Minimize text on slides (bullet points, not paragraphs)
- Practice timing
- Prepare for questions (know your design deeply)

---

## Capstone Project: Custom Robot Gripper

### Project Brief

**Challenge**: Design, analyze, fabricate, and validate a custom gripper for the SO-101 robot arm capable of grasping a specified object.

**Assigned Object** (instructor provides one of):
- Cylindrical: PVC pipe, 25mm diameter
- Rectangular: Foam block, 30mm × 40mm × 50mm
- Spherical: Tennis ball, 65mm diameter
- Irregular: Plastic bottle, complex geometry

**Timeline** (8 hours):
- Hour 1: Requirements, concepts, selection
- Hours 2-4: Detailed CAD, analysis, optimization
- Hours 5-7: Fabrication (slicing, printing, assembly)
- Hour 8: Testing, validation, documentation/presentation

**Deliverables**:
1. **Technical Report** (8-12 pages, template provided)
2. **CAD Files** (Fusion 360 archive, STL exports)
3. **Physical Prototype** (assembled, ready to demonstrate)
4. **Presentation** (10-slide deck, 10-minute talk)

---

## Optional Laboratory Exercise

### Guided Capstone Project (8 hours)

<!-- class="optional-exercise" -->
**Comprehensive Design Cycle**

**Phase 1: Planning and Conceptual Design** (60 minutes)

1. **Requirement Development**:
   - Measure assigned object (calipers, scale)
   - Define functional requirements (grasp range, force)
   - Specify constraints (SO-101 interface, FDM manufacturing, material budget)
   - Create requirements table

2. **Concept Generation**:
   - Sketch 3 different gripper concepts
   - Consider: parallel jaw, adaptive, claw, suction, etc.
   - Create morphological matrix

3. **Concept Evaluation**:
   - Pugh chart with weighted criteria
   - Select primary concept
   - Document rationale

<!-- class="exercise-tip" -->
**Tip**: Don't overthink concepts. Simple, manufacturable designs often outperform complex ones.

**Phase 2: Detailed CAD Modeling** (120 minutes)

1. **Parametric Model**:
   - Create parameter table (gripper_width, finger_length, etc.)
   - Model base plate with SO-101 mounting holes (M4, 50mm spacing)
   - Model gripper fingers (use parameters throughout)
   - Add grip surface features (ridges, compliant pads)

2. **Assembly**:
   - Create assembly with base + 2 fingers
   - Add revolute joints
   - Create motion link for symmetric operation
   - Animate full range of motion

3. **Validation**:
   - Check interference (collision detection)
   - Measure grasp range (minimum and maximum opening)
   - Verify object fits within workspace

<!-- class="exercise-advanced" -->
**Advanced**: Add four-bar linkage for mechanical advantage.

**Phase 3: Analysis** (90 minutes)

1. **FEA Simulation**:
   - Define material (PLA: E=3.5 GPa, σ_y=50 MPa)
   - Apply constraints (fixed mounting interface)
   - Apply load (10N grip force on finger contact surface)
   - Mesh (2mm global, 0.5mm local refinement)
   - Solve and extract maximum von Mises stress, SF, displacement

2. **Optimization Iteration**:
   - If SF < 2.0: Add material (increase thickness, fillets, ribs)
   - If SF >> 2.0: Remove material (hollow sections, lightening holes)
   - Re-analyze after each change
   - Target: SF = 2.0-2.5, mass < 50g

3. **Documentation**:
   - Screenshot FEA setup and results
   - Create table: Version, Mass, Max Stress, SF, Changes

**Phase 4: Fabrication** (180 minutes)

1. **Export and Slice** (30 min):
   - Export STL files (base, finger-left, finger-right)
   - Import to Cura
   - Optimize print orientation
   - Configure: 0.2mm layers, 40% infill (gyroid), 3 perimeters
   - Add supports (tree, 15%)
   - Generate G-code

2. **3D Printing** (120 min):
   - Start prints (monitor first layer carefully)
   - Print time estimation and queue management
   - While printing: Prepare assembly tools, review test procedure

3. **Post-Processing** (30 min):
   - Remove supports (pliers, flush cutters)
   - Light sanding on contact surfaces
   - Install threaded inserts (if used) or prepare fasteners
   - Test-fit components before final assembly

**Phase 5: Testing and Validation** (60 minutes)

1. **Assembly**:
   - Assemble gripper (install fasteners, verify joint motion)
   - Mount to SO-101 interface

2. **Functional Testing**:
   - Manually actuate gripper
   - Measure actual grip force (spring scale)
   - Attempt to grasp assigned object
   - Test: Can it lift object? Maintain grip during motion?
   - Record success/failure, observations

3. **Documentation**:
   - Photos: CAD model, FEA results, physical prototype, object grasping
   - Measurements: Actual vs. predicted dimensions, forces
   - Create results table

<!-- class="exercise-tip" -->
**Common Issues**:
- **Fingers don't move smoothly**: Sand joint surfaces, check for warping
- **Grip force insufficient**: Increase friction (add texture, rubber pad)
- **Part broke during test**: FEA likely underestimated stress (check mesh, boundary conditions)

**Phase 6: Presentation Preparation** (30 minutes)

1. Create slide deck:
   - Title slide
   - Problem statement (object photo, requirements)
   - Design concept (CAD render)
   - Analysis results (FEA stress plot, key metrics)
   - Physical prototype (photos)
   - Testing results (success demonstration photo/video)
   - Conclusion (achievements, lessons learned)

2. Practice presentation (time yourself, aim for 10 minutes)

3. Prepare for questions:
   - Why did you choose this concept?
   - How did you validate your analysis?
   - What would you improve with more time?

---

## Assessment Rubric

The capstone project is evaluated across multiple dimensions:

### Technical Excellence (40%)

- **Requirements**: Complete, clear, measurable (10%)
- **Analysis**: Appropriate FEA setup, mesh convergence, correct interpretation (15%)
- **Design Quality**: Manufacturability, parametric modeling, optimization (15%)

### Prototype Performance (30%)

- **Functionality**: Successfully grasps assigned object (20%)
- **Quality**: Clean assembly, no structural failures (10%)

### Documentation (20%)

- **Report**: Complete sections, clear figures, professional formatting (10%)
- **Presentation**: Clear communication, effective visuals (10%)

### Process (10%)

- **Methodology**: Systematic approach, design iterations documented
- **Time Management**: Efficient use of 8-hour period

**Passing Criteria**: ≥ 70% total score and functional prototype demonstration.

---

## Summary

This capstone project synthesized all Design Track competencies:

1. **CAD Modeling**: Parametric solid modeling and assembly (DESIGN-201)
2. **Manufacturing**: Additive fabrication planning and execution (DESIGN-202)
3. **Structural Analysis**: FEA simulation and optimization (DESIGN-203)
4. **Kinematics**: Mechanism design and motion validation (DESIGN-204)
5. **Integration**: Systematic design methodology and professional documentation

**Key Takeaways**:
- Engineering design is iterative, not linear
- Analysis informs design decisions (simulation-driven development)
- Physical validation is essential (models are approximations)
- Documentation and communication are as important as technical work

**Congratulations**: Upon successful completion, students demonstrate comprehensive capability in mechatronic system design from concept through validated prototype.

---

*Version 2.0.0 - Academic curriculum emphasizing theoretical foundations with optional practical exercises*
