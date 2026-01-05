<!--
author:   Dr. Maria Torres
email:    maria.torres@robotcampus.dev
version:  2.0.0

language: en
narrator: US English Female

comment:  Additive Manufacturing Theory and Practice: Comprehensive treatment of material extrusion physics, slicing algorithms, thermodynamics, process optimization, and quality control for FDM 3D printing.

icon:     https://robotcampus.dev/logos/design-202.png

mode:     Textbook

link:     https://robotcampus.dev/styles/course-styles.css

-->

# DESIGN-202: Additive Manufacturing Theory and Practice

> **Additive manufacturing transforms digital geometry into physical matter through precise, layer-by-layer material deposition.**

## Course Overview

| | |
|---|---|
| **Course Code** | DESIGN-202 |
| **Duration** | 4 hours |
| **Level** | Intermediate |
| **Prerequisites** | DESIGN-201, basic thermodynamics |
| **Theory-Practice** | 60% theory, 40% optional labs |

---

## Learning Objectives

Upon completion of this course, students will be able to:

### Theoretical Understanding

- Explain the historical development of additive manufacturing technologies
- Understand the physics of material extrusion and layer bonding
- Describe slicing algorithms and toolpath generation strategies
- Analyze thermal dynamics during polymer deposition
- Compare different additive manufacturing processes and their applications
- Evaluate material properties and their impact on part performance

### Practical Skills

- Configure slicer software with appropriate process parameters
- Diagnose and resolve common print failures through systematic analysis
- Select optimal materials for specific mechanical requirements
- Perform post-processing operations for surface finish improvement
- Validate printed parts against CAD specifications

---

## Module 1: Additive Manufacturing History and Taxonomy

### 1.1 Evolution of Additive Manufacturing

<!-- class="historical-note" -->
**From Rapid Prototyping to Production Technology**

**Pre-AM Era (Pre-1980s)**:
- Subtractive manufacturing dominated: milling, turning, grinding
- Prototyping required expensive tooling and long lead times
- Design iterations were costly and time-consuming

**Invention Era (1980s)**:

**Stereolithography (SLA)** - Chuck Hull, 1984:
- First commercial AM process
- UV laser cures liquid photopolymer resin layer-by-layer
- Founded 3D Systems Corporation (1986)
- Introduced STL file format (still standard today)

**Selective Laser Sintering (SLS)** - Carl Deckard, 1986:
- Laser fuses powder materials (polymers, metals, ceramics)
- No support structures needed (powder provides support)
- University of Texas at Austin research

**Fused Deposition Modeling (FDM)** - Scott Crump, 1988:
- Extrude thermoplastic filament through heated nozzle
- Founded Stratasys Corporation
- Most widely adopted process today

**Industrial Adoption (1990s-2000s)**:
- "Rapid Prototyping" for design validation
- Aerospace and medical applications
- High equipment costs ($100,000+)
- Limited materials and poor surface finish

**Democratization (2009-Present)**:

**RepRap Project** (Adrian Bowyer, 2005):
- Self-replicating 3D printer (can print its own parts)
- Open-source design
- Inspired hobbyist movement

**Patent Expirations (2009)**:
- FDM patents expired
- Consumer printers emerged ($500-$2000)
- MakerBot, Ultimaker, Prusa Research founded
- "Maker Movement" accelerated adoption

**Current State**:
- Metal printing for aerospace (GE turbine blades, SpaceX engines)
- Bioprinting for tissue engineering
- Construction-scale concrete printing
- Polymer production parts (Adidas shoes, dental aligners)

### 1.2 Additive Manufacturing Taxonomy

<!-- class="theory-concept" -->
**Seven Process Categories (ISO/ASTM 52900)**

1. **Vat Photopolymerization**:
   - SLA (Stereolithography), DLP (Digital Light Processing)
   - UV light cures liquid resin
   - High resolution (~25 μm layers)
   - Smooth surface finish
   - Applications: Jewelry, dental, microfluidics

2. **Material Extrusion**:
   - FDM/FFF (Fused Filament Fabrication)
   - Focus of this course
   - Thermoplastic filament melted and deposited
   - Most accessible and versatile

3. **Powder Bed Fusion**:
   - SLS (polymers), SLM/DMLS (metals)
   - Laser or electron beam fuses powder
   - No supports needed
   - Aerospace, medical implants

4. **Binder Jetting**:
   - Liquid binder selectively bonds powder
   - Fast, full-color capability
   - Post-processing required (sintering)

5. **Material Jetting**:
   - Inkjet-like droplet deposition
   - Multi-material, full-color
   - PolyJet (Stratasys)
   - High cost, limited materials

6. **Directed Energy Deposition**:
   - Metal powder/wire fed into laser/electron beam
   - Large-scale metal parts
   - Repair and hybrid manufacturing

7. **Sheet Lamination**:
   - Paper or metal sheets bonded layer-by-layer
   - LOM (Laminated Object Manufacturing)
   - Low cost, limited applications

### 1.3 FDM Process Overview

<!-- class="theory-concept" -->
**Material Extrusion Fundamentals**

The FDM process consists of four primary subsystems:

1. **Material Supply**:
   - Thermoplastic filament (typically 1.75mm or 2.85mm diameter)
   - Stored on spool, fed through guide tube
   - Drive gears push filament into hotend

2. **Extrusion System (Hotend)**:
   - **Heater block**: Melts filament (180-260°C depending on material)
   - **Nozzle**: Restricts flow, defines bead width (typically 0.4mm)
   - **Heat break**: Thermal barrier prevents premature melting

3. **Motion System**:
   - **Cartesian**: X-Y gantry, Z-axis bed (most common)
   - **CoreXY**: Belt-driven parallel kinematics (faster)
   - **Delta**: Parallel arms, circular build volume (tall prints)
   - Stepper motors with precise positioning (~10 μm resolution)

4. **Build Platform**:
   - **Heated bed**: Maintains first layer adhesion, reduces warping
   - **Surface**: Glass, PEI, textured steel
   - **Leveling**: Critical for consistent first layer

**Quiz: AM History**

What was the first commercial additive manufacturing technology?

[(X)] Stereolithography (SLA)
[( )] Fused Deposition Modeling (FDM)
[( )] Selective Laser Sintering (SLS)
[( )] Binder Jetting

The RepRap project's key contribution was:

[( )] Inventing FDM printing
[(X)] Open-source, self-replicating printer design
[( )] Metal 3D printing
[( )] First consumer 3D printer

---

## Module 2: Physics of Material Extrusion

### 2.1 Thermodynamics of Polymer Deposition

<!-- class="theory-concept" -->
**Thermal Behavior of Thermoplastics**

**Glass Transition Temperature (Tg)**:
- Temperature where polymer transitions from rigid to rubbery state
- Below Tg: Polymer is brittle, glassy
- Above Tg: Polymer chains gain mobility
- Examples: PLA Tg ≈ 60°C, ABS Tg ≈ 105°C

**Melting Temperature (Tm)**:
- For semi-crystalline polymers (PLA, Nylon)
- Crystalline regions melt, material flows
- Amorphous polymers (ABS) have no distinct Tm, gradual softening

**Printing Temperature Window**:
$$T_{\text{print}} > T_m + \Delta T_{\text{margin}}$$

where ΔT_margin ≈ 20-40°C ensures adequate flow.

**Heat Transfer Model**:

Heat flow from nozzle to filament:
$$Q = k A \frac{dT}{dx}$$

where:
- Q = heat transfer rate (W)
- k = thermal conductivity of polymer (~0.2 W/m·K)
- A = contact area
- dT/dx = temperature gradient

**Residence Time**: Duration filament spends in hotend
- Too short → incomplete melting, weak bonding
- Too long → thermal degradation, discoloration

### 2.2 Layer Bonding Mechanism

<!-- class="theory-concept" -->
**Interfacial Adhesion**

Layer bonding occurs through:

1. **Thermal Diffusion**:
   - Polymer chains from adjacent layers intermingle
   - Requires temperature above Tg
   - **Reptation theory**: Chain diffusion across interface

2. **Contact Pressure**:
   - Nozzle exerts force on deposited bead
   - Increases contact area
   - Drives air out of interface

3. **Cooling Time**:
   - If previous layer cools below Tg before next layer deposits → poor bonding
   - **Interlayer bond strength** ≈ 80-95% of bulk material strength

**Mathematical Model** (Bellehumeur et al., 2004):

Bond strength ratio:
$$\frac{\sigma_{\text{bond}}}{\sigma_{\text{bulk}}} \approx \sqrt{\frac{t_{\text{contact}}}{t_{\text{reptation}}}}$$

where:
- t_contact = time at bonding temperature
- t_reptation = characteristic polymer chain relaxation time

**Anisotropy**: FDM parts are stronger in XY plane than Z direction due to layer interfaces.

### 2.3 Material Flow and Extrusion

<!-- class="theory-concept" -->
**Rheology of Molten Polymers**

Molten thermoplastics exhibit **non-Newtonian** flow:

**Power-Law Model**:
$$\eta = K \dot{\gamma}^{n-1}$$

where:
- η = viscosity
- K = consistency index
- γ̇ = shear rate
- n = power-law index (n < 1 for shear-thinning behavior)

**Volumetric Flow Rate**:
$$Q = v_{\text{nozzle}} \cdot A_{\text{nozzle}}$$

where:
- v_nozzle = nozzle travel speed
- A_nozzle = cross-sectional area of extruded bead

**Extrusion Multiplier**:
Calibrates actual vs. expected extrusion:
$$E = \frac{Q_{\text{actual}}}{Q_{\text{commanded}}}$$

Typical values: E ≈ 0.90 - 1.05

### 2.4 Warping and Thermal Stress

<!-- class="theory-concept" -->
**Volumetric Shrinkage**

Most polymers contract upon cooling:

**Coefficient of Thermal Expansion (CTE)**:
$$\Delta L = \alpha L_0 \Delta T$$

where:
- α = CTE (typically 50-100 × 10⁻⁶ /°C for thermoplastics)
- L₀ = original length
- ΔT = temperature change

**Example**: 100mm PLA part cooling from 200°C to 25°C:
$$\Delta L = (70 \times 10^{-6})(100)(175) = 1.225 \text{ mm}$$

**Warping Mechanism**:
1. Bottom layer adheres to heated bed
2. Upper layers cool and contract
3. Differential contraction induces stress
4. Corners lift from bed if adhesion force < residual stress

**Mitigation Strategies**:
- Heated bed (maintain elevated temperature)
- Adhesion aids (glue stick, hairspray)
- Enclosures (reduce ambient cooling rate)
- Material selection (PLA warps less than ABS)

**Quiz: Thermal Physics**

Why do FDM parts have anisotropic strength?

[( )] Different materials are used in different directions
[(X)] Layer interfaces are weaker than bulk material
[( )] The printer moves faster in one direction
[( )] Gravity affects layer bonding

What causes warping in 3D prints?

[( )] Excessive filament extrusion
[( )] Incorrect nozzle diameter
[(X)] Differential thermal contraction between layers
[( )] Too much bed adhesion

---

## Module 3: Slicing and Toolpath Generation

### 3.1 STL File Processing

<!-- class="theory-concept" -->
**From Continuous Geometry to Discrete Layers**

**Input: STL Triangle Mesh**
- Vertices: (x, y, z) coordinates
- Faces: Triplets of vertex indices with normal vectors
- Approximation of smooth surfaces

**Slicing Algorithm**:

1. **Intersection Computation**:
   For each layer height h_i:
   - Define plane: z = h_i
   - Find all triangle-plane intersections
   - Each intersection yields a line segment

2. **Contour Construction**:
   - Connect line segments into closed polygons
   - **Outer contours**: Part boundary
   - **Inner contours**: Holes or voids

3. **Topology Handling**:
   - Identify islands (disconnected regions)
   - Detect bridges and overhangs
   - Determine support requirements

**Computational Complexity**: O(n·m) where n = triangles, m = layers

**Adaptive Slicing**:
- Variable layer height based on geometry
- Thin layers for curved surfaces (high detail)
- Thick layers for flat regions (speed)
- Reduces print time while maintaining quality

### 3.2 Toolpath Generation Strategies

<!-- class="theory-concept" -->
**Infill Patterns and Their Properties**

**Perimeters (Shells)**:
- Trace part outline
- Multiple perimeters (typically 2-4) for strength
- **Innermost-first** vs **outermost-first** ordering

**Infill Patterns**:

1. **Rectilinear (Lines)**:
   - Simple, fast
   - 45° or 90° alternating between layers
   - Moderate strength, good speed

2. **Grid**:
   - Perpendicular lines in same layer
   - Higher strength than rectilinear
   - More material usage

3. **Honeycomb (Hexagonal)**:
   - Biomimetic pattern (bees chose this for good reason)
   - Best strength-to-weight ratio
   - Slowest to print

4. **Gyroid**:
   - Mathematical triply periodic minimal surface
   - Isotropic strength (equal in all directions)
   - No long straight lines (reduces vibration)
   - Formula: $\sin(x)\cos(y) + \sin(y)\cos(z) + \sin(z)\cos(x) = 0$

5. **Adaptive**:
   - Dense infill near perimeters and top/bottom
   - Sparse infill in interior
   - Optimizes material efficiency

**Infill Density**:
- 0%: Hollow (vase mode)
- 10-20%: Lightweight models
- 30-50%: Functional parts
- 100%: Maximum strength (slow, expensive)

### 3.3 Support Structure Generation

<!-- class="theory-concept" -->
**Overhang Analysis and Support Placement**

**Overhang Angle Threshold**:
- Typically 45° from vertical
- Steeper angles require support
- Based on material bridging capability

**Support Types**:

1. **Normal Supports**:
   - Same material as part
   - Difficult to remove
   - Good mechanical strength during print

2. **Tree Supports**:
   - Branch-like structures
   - Touch part at fewer points
   - Easier removal, better surface finish
   - Inspired by natural tree branching

3. **Soluble Supports**:
   - Different material (PVA, HIPS)
   - Dissolved in water or limonene
   - Requires dual-extrusion printer
   - Best surface quality

**Support Interface**:
- Thin layer between support and part
- Easier separation
- Roof interface (top) and floor interface (bottom)

**Support Density**:
- 10-20% typical
- Denser support → better overhang quality
- Sparser support → easier removal

### 3.4 G-code Generation

<!-- class="theory-concept" -->
**Machine Instructions**

Slicer outputs **G-code**: Human-readable machine instructions

**Common G-code Commands**:

```gcode
G28           ; Home all axes
G1 X100 Y100  ; Linear move to (100, 100)
G1 Z0.2 F1200 ; Move to Z=0.2 at 1200 mm/min
M104 S200     ; Set hotend temperature to 200°C
M140 S60      ; Set bed temperature to 60°C
M109 S200     ; Wait for hotend to reach 200°C
G92 E0        ; Reset extruder position
G1 X110 Y110 E5 F1800  ; Extrude while moving
```

**G-code Structure for Layer**:
```
; Layer 10, Z=2.0mm
G1 Z2.0 F600        ; Lift to layer height
G1 X50 Y50 F4800    ; Move to start (non-printing)
G1 X150 E5.2 F1200  ; Print perimeter
...                 ; Continue perimeter
G1 X60 Y60          ; Move to infill start
G1 X140 E10.5       ; Print infill line
```

**Optimization Considerations**:
- Minimize non-printing moves (travel)
- Avoid crossing perimeters (stringing)
- Optimize print sequence for cooling
- Retraction for long travels (reduce oozing)

**Quiz: Slicing**

What is the purpose of adaptive layer height?

[(X)] Reduce print time while maintaining detail in curved areas
[( )] Make the print stronger
[( )] Save filament material
[( )] Improve bed adhesion

Which infill pattern provides the most isotropic strength?

[( )] Rectilinear
[( )] Honeycomb
[(X)] Gyroid
[( )] Grid

---

## Module 4: Material Properties and Selection

### 4.1 Common FDM Materials

<!-- class="theory-concept" -->
**Thermoplastic Filament Comparison**

**PLA (Polylactic Acid)**:
- **Chemistry**: Biodegradable, derived from corn starch or sugarcane
- **Tg**: 60°C, **Tm**: 150-160°C
- **Print temp**: 190-220°C
- **Bed temp**: 50-60°C (optional)
- **Properties**:
  - Tensile strength: 50-70 MPa
  - Elastic modulus: 2-4 GPa
  - Elongation at break: 3-10%
  - Brittle, low impact resistance
- **Pros**: Easy to print, minimal warping, low odor
- **Cons**: Heat-sensitive, brittle, UV degradation
- **Applications**: Prototypes, decorative items, low-stress parts

**PETG (Polyethylene Terephthalate Glycol)**:
- **Chemistry**: Modified PET (water bottles)
- **Tg**: 80°C
- **Print temp**: 220-250°C
- **Bed temp**: 70-80°C
- **Properties**:
  - Tensile strength: 50-60 MPa
  - Elastic modulus: 2-2.5 GPa
  - Elongation at break: 50-150%
  - Ductile, good impact resistance
- **Pros**: Strong, flexible, chemical resistant, food-safe
- **Cons**: Stringing, slower cooling, hygroscopic
- **Applications**: Functional parts, containers, mechanical components

**ABS (Acrylonitrile Butadiene Styrene)**:
- **Chemistry**: Petroleum-based terpolymer
- **Tg**: 105°C
- **Print temp**: 220-250°C
- **Bed temp**: 90-110°C (critical)
- **Properties**:
  - Tensile strength: 40-50 MPa
  - Elastic modulus: 2-3 GPa
  - High impact resistance
  - Good heat resistance
- **Pros**: Strong, machinable, acetone-smoothable
- **Cons**: Warping, toxic fumes (styrene), requires enclosure
- **Applications**: Functional parts, tooling, high-temp applications

**Advanced Materials**:
- **Nylon (PA)**: High strength, flexibility, wear resistance
- **TPU (Thermoplastic Polyurethane)**: Flexible, rubber-like
- **Polycarbonate (PC)**: Very high strength and heat resistance
- **Composites**: Carbon fiber, glass fiber filled for rigidity

### 4.2 Material Testing Standards

<!-- class="theory-concept" -->
**Characterizing Mechanical Properties**

**ASTM D638**: Tensile Testing
- Dog-bone specimen geometry
- Measure force vs. elongation
- Calculate:
  - **Ultimate tensile strength** (σ_UTS)
  - **Elastic modulus** (E)
  - **Elongation at break** (ε_f)

**ASTM D256**: Impact Testing (Izod)
- Pendulum strikes notched specimen
- Measures energy absorbed during fracture
- Units: J/m (energy per unit thickness)

**Orientation Dependence**:
For FDM parts, test in multiple orientations:
- **XY (flat)**: Strongest, continuous fiber paths
- **XZ/YZ (on-edge)**: Moderate, layer interfaces at angle
- **Z (vertical)**: Weakest, layer interfaces perpendicular to load

Typical strength ratios:
$$\frac{\sigma_{XY}}{\sigma_Z} \approx 1.2-1.5$$

### 4.3 Environmental Considerations

<!-- class="alternative-approach" -->
**Sustainability and Safety**

**Biodegradability**:
- **PLA**: Industrially compostable (>60°C, 6 months)
- Not home-compostable (too cool)
- Microplastic concern in natural environments

**Recycling**:
- Most filament is virgin material
- Filament recyclers exist (grind failed prints → new filament)
- Challenges: Contamination, material mixing, property degradation

**VOC Emissions**:
- **ABS**: Significant styrene emission (irritant, potential carcinogen)
- **PLA**: Minimal emissions, lactide odor (generally safe)
- **Recommendation**: Ventilation for all printing, especially ABS

**Energy Consumption**:
- Typical print: 50-200 Wh per part
- Lower than injection molding for small quantities
- Higher than molding for mass production

---

## Module 5: Process Optimization and Quality Control

### 5.1 Critical Print Parameters

<!-- class="theory-concept" -->
**Parameter Interdependencies**

**Layer Height (h)**:
- Range: 0.1-0.4 mm (typically 0.2 mm)
- **Finer layers**: Better surface finish, longer print time
- **Thicker layers**: Faster, stronger Z-axis bonds, rougher surface
- Rule of thumb: h ≤ 80% of nozzle diameter

**Print Speed (v)**:
- Range: 30-100 mm/s
- **Faster**: Shorter print time, possible quality issues
- **Slower**: Better quality, improved layer bonding
- Limited by:
  - **Flow rate**: Hotend must melt filament fast enough
  - **Acceleration**: Motion system mechanical limits
  - **Cooling**: Time for layer to solidify

**Extrusion Temperature (T)**:
- Material-dependent
- **Higher T**: Better flow, stronger bonding, more stringing
- **Lower T**: Sharper details, less oozing, possible under-extrusion

**Cooling (Fan Speed)**:
- **More cooling**: Sharper overhangs, less warping, weaker Z-bonding
- **Less cooling**: Stronger parts, worse overhangs, more drooping
- Material-specific: PLA needs lots, ABS minimal

**Optimization Strategy**:
1. Start with manufacturer defaults
2. Print calibration parts (temperature tower, flow cube)
3. Adjust one parameter at a time
4. Document results systematically

### 5.2 Common Failure Modes

<!-- class="theory-concept" -->
**Diagnostic Troubleshooting**

**First Layer Adhesion Failure**:
- **Symptoms**: Part detaches from bed, corners lift
- **Causes**: Bed not level, nozzle too high, bed too cool, dirty bed
- **Solutions**: Re-level bed, decrease Z-offset, increase bed temp, clean with IPA

**Warping**:
- **Symptoms**: Corners curl upward
- **Causes**: Thermal contraction, poor adhesion, drafts
- **Solutions**: Heated bed, enclosure, brim/raft, lower print temp

**Stringing/Oozing**:
- **Symptoms**: Thin plastic threads between parts
- **Causes**: Nozzle drips during travel moves
- **Solutions**: Increase retraction distance/speed, lower temperature, faster travel

**Layer Shifting**:
- **Symptoms**: Layers misaligned horizontally
- **Causes**: Belt slipping, motor skipping, excessive speed
- **Solutions**: Tighten belts, reduce acceleration, check motor current

**Under-Extrusion**:
- **Symptoms**: Gaps in layers, weak parts
- **Causes**: Clogged nozzle, low extrusion multiplier, temperature too low
- **Solutions**: Clean/replace nozzle, increase flow %, increase temp

**Over-Extrusion**:
- **Symptoms**: Blobs, rough surface, dimensional inaccuracy
- **Causes**: Extrusion multiplier too high
- **Solutions**: Decrease flow %, calibrate E-steps

### 5.3 Post-Processing Techniques

<!-- class="theory-concept" -->
**Surface Finish Improvement**

**Support Removal**:
- **Tools**: Pliers, flush cutters, dental picks
- **Technique**: Remove bulk supports first, then interface layer
- **Sanding**: Smooth support contact points

**Mechanical Finishing**:
1. **Sanding**: Progressively finer grits (120 → 400 → 800 → 1200)
2. **Filing**: Remove large imperfections
3. **Tumbling**: Bulk parts in rotary tumbler with abrasive media

**Chemical Smoothing**:
- **ABS**: Acetone vapor bath (dissolves surface, creates gloss)
- **PLA**: Chloroform (hazardous, not recommended), sanding preferred
- **Safety**: Ventilation, gloves, proper disposal

**Filling and Painting**:
1. Apply filler primer (fills layer lines)
2. Sand smooth
3. Paint with acrylic or spray paint
4. Clear coat for protection

**Annealing** (Heat Treatment):
- Heat PLA to 70-80°C for 30-60 minutes
- Increases crystallinity → higher strength and heat resistance
- Risk: Part deformation, warping
- Use support fixture to maintain dimensions

---

## Summary

This course established comprehensive understanding of additive manufacturing theory and practice:

1. **Historical Context**: Traced AM development from industrial prototyping to consumer accessibility
2. **Physics**: Examined thermal dynamics, layer bonding, and material flow
3. **Slicing**: Studied toolpath generation, infill strategies, and support placement
4. **Materials**: Compared thermoplastic properties and selection criteria
5. **Process Control**: Analyzed parameter optimization and failure diagnosis

**Key Takeaways**:
- FDM builds parts through controlled thermal deposition and bonding
- Slicing transforms continuous geometry into discrete machine instructions
- Material selection balances printability, mechanical properties, and application requirements
- Systematic parameter tuning and failure analysis enable reliable production

**Next Steps**: DESIGN-203 explores structural mechanics and finite element analysis for optimizing part strength.

---

## Optional Laboratory Exercises

### Lab 1: Slicing Parameter Study (90 minutes)

<!-- class="optional-exercise" -->
**Objective**: Systematically investigate the effect of slicing parameters on print quality, strength, and time.

**Test Part**: Standardized tensile specimen (ISO 527-2 Type 1A geometry, scaled 50%)

**Tasks**:
1. Slice the same part with varying parameters:
   - Layer height: 0.1mm, 0.2mm, 0.3mm
   - Infill density: 20%, 50%, 80%
   - Infill pattern: Rectilinear, honeycomb, gyroid
2. For each combination, record:
   - Estimated print time
   - Filament usage (mass)
   - Actual print time
3. Print selected representative samples
4. Perform qualitative strength test (manual bending)
5. Measure surface roughness with calipers (layer visibility)

<!-- class="exercise-tip" -->
**Analysis Questions**:
- How does layer height affect surface quality vs. print time?
- Which infill pattern provides best strength-to-weight ratio?
- What is the relationship between infill density and part stiffness?

### Lab 2: Failure Mode Diagnosis and Correction (60 minutes)

<!-- class="optional-exercise" -->
**Objective**: Develop troubleshooting skills by intentionally creating and resolving print failures.

**Tasks**:
1. **First Layer Failure**: Set nozzle 0.5mm too high → observe poor adhesion → correct Z-offset
2. **Warping**: Print ABS without heated bed → observe corner lifting → add heated bed + brim
3. **Stringing**: Print bridging test with temperature too high → observe strings → tune retraction
4. **Under-Extrusion**: Reduce flow to 80% → observe gaps → recalibrate extrusion multiplier

**Deliverable**: Troubleshooting guide with photos documenting each failure mode and solution.

<!-- class="exercise-advanced" -->
**Advanced Extension**:
- Create a decision tree for systematic failure diagnosis
- Measure dimensional accuracy of prints with calipers, compare to CAD
- Investigate effect of print orientation on surface finish and strength

### Lab 3: Material Property Comparison (120 minutes)

<!-- class="optional-exercise" -->
**Objective**: Empirically compare mechanical properties of PLA, PETG, and ABS through standardized testing.

**Requirements**:
- Print identical test specimens in each material
- Use consistent print settings (same layer height, infill, etc.)
- Orientation: XY (flat) for maximum strength

**Tasks**:
1. **Tensile Testing** (if equipment available):
   - Use universal testing machine or manual pull test
   - Record maximum force and elongation at break
   - Calculate tensile strength and elastic modulus

2. **Impact Testing** (simplified):
   - Drop standard mass from varying heights onto cantilevered specimen
   - Record height at which specimen fractures
   - Compare impact energy absorption

3. **Heat Resistance**:
   - Place specimens in oven at increasing temperatures
   - Record temperature at which significant deformation occurs
   - Compare to published Tg values

4. **Environmental Testing**:
   - Submerge specimens in water, isopropyl alcohol, acetone
   - Observe swelling, dissolution, or embrittlement
   - Document chemical resistance

<!-- class="exercise-tip" -->
**Safety Notes**:
- ABS printing requires ventilation
- Acetone is flammable and toxic
- Wear safety glasses during impact testing

**Deliverable**: Comprehensive material comparison report with test data, photos, and material selection recommendations for different applications.

---

*Version 2.0.0 - Academic curriculum emphasizing theoretical foundations with optional practical exercises*
