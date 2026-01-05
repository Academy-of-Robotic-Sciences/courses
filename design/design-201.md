<!--
author:   Dr. Maria Torres
email:    maria.torres@robotcampus.dev
version:  2.0.0

language: en
narrator: US English Female

comment:  Parametric Solid Modeling Fundamentals: Comprehensive treatment of CAD theory, geometric constraint solving, sketch-based modeling, and feature-based design methodologies.

icon:     https://robotcampus.dev/logos/design-201.png

mode:     Textbook

link:     https://robotcampus.dev/styles/course-styles.css

-->

# DESIGN-201: Parametric Solid Modeling Fundamentals

> **Parametric modeling transforms design from geometric description to computational representation of intent.**

## Course Overview

| | |
|---|---|
| **Course Code** | DESIGN-201 |
| **Duration** | 4 hours |
| **Level** | Intermediate |
| **Prerequisites** | Spatial reasoning, basic geometry, trigonometry |
| **Theory-Practice** | 60% theory, 40% optional labs |

---

## Learning Objectives

Upon completion of this course, students will be able to:

### Theoretical Understanding

- Explain the evolution of CAD systems from 2D drafting to parametric solid modeling
- Understand the mathematical foundations of geometric constraint solving
- Describe boundary representation (B-rep) and constructive solid geometry (CSG)
- Analyze the relationship between sketches, constraints, and design intent
- Compare direct modeling and parametric modeling paradigms

### Practical Skills

- Navigate professional CAD software interfaces (Fusion 360)
- Create fully-constrained 2D sketches using geometric and dimensional constraints
- Apply feature-based modeling operations (extrude, revolve, fillet)
- Export models in standard formats (STL, STEP) for manufacturing
- Evaluate model quality and manufacturability

---

## Module 1: Historical Evolution of CAD Systems

### 1.1 From Manual Drafting to Computer Graphics

<!-- class="historical-note" -->
**The Pre-Computer Era**

Before the 1960s, all engineering design was documented through manual drafting:

- **Technical drawing instruments**: T-squares, compasses, French curves
- **Standardized conventions**: Orthographic projection, sectioning, dimensioning
- **Paper-based workflow**: Blueprints (cyanotype process), tracings, revisions
- **Limitations**: Time-intensive, error-prone, difficult to modify, challenging to share

**The Birth of Computer Graphics** (1960s)

**Sketchpad** (Ivan Sutherland, MIT, 1963):
- First interactive computer graphics system
- Light pen interface on vector display
- Introduced **constraint-based drawing**: parallel lines, perpendicularity
- Demonstrated that computers could understand geometric relationships
- Laid foundation for all modern CAD systems

Key insight: Design relationships (constraints) are more fundamental than coordinates.

### 1.2 Evolution of CAD Representations

<!-- class="theory-concept" -->
**2D CAD Era (1970s-1980s)**

Early commercial CAD systems automated drafting:

- **AutoCAD** (1982): Replaced drafting boards with digital equivalents
- Geometry stored as lines, arcs, circles with explicit coordinates
- No understanding of 3D geometry or part relationships
- Primary benefit: Easy modification and reproduction

**3D Wireframe and Surface Modeling (1980s)**

- Represented objects as edge networks or parametric surfaces
- No concept of solid volume
- Ambiguous representations (impossible to determine "inside" vs "outside")
- Used primarily for visualization, not analysis

**Solid Modeling Revolution (Late 1980s)**

Two fundamental representations emerged:

1. **Constructive Solid Geometry (CSG)**:
   - Objects defined by Boolean operations on primitives
   - Tree structure: sphere ∪ cylinder - box
   - Compact representation
   - Limited modeling flexibility

2. **Boundary Representation (B-rep)**:
   - Objects defined by bounding surfaces
   - Explicit representation of vertices, edges, faces
   - Enables complex geometry
   - Foundation of modern CAD

### 1.3 Parametric Modeling Paradigm

<!-- class="historical-note" -->
**The Parametric Revolution**

**Pro/ENGINEER** (PTC, 1988):
- First commercial parametric solid modeler
- Introduced **feature-based modeling**: design history captured as sequence of operations
- **Parametric relations**: Dimensions linked by equations
- **Design intent**: Constraints preserve relationships during modification

**Key Innovation**: Models become programs, not just geometry.

**SolidWorks** (1995):
- Made parametric modeling accessible with intuitive interface
- Popularized sketch-based workflow
- Established modern CAD paradigm

<!-- class="theory-concept" -->
**Parametric vs. Direct Modeling**

**Parametric Modeling**:
- Design history preserved
- Dimensions and constraints define geometry
- Changes propagate through feature tree
- Captures design intent
- Ideal for iterative design

**Direct Modeling**:
- Modify geometry directly (push/pull faces)
- No history, no constraints
- Faster for simple modifications
- Less predictable for complex changes
- Useful for imported geometry

Modern systems (like Fusion 360) integrate both paradigms.

**Quiz: CAD History**

What was the key innovation of Sketchpad (1963)?

[( )] It was the first computer program for engineering
[( )] It could draw circles and lines automatically
[(X)] It introduced constraint-based geometric relationships
[( )] It invented 3D graphics

Which representation is used by modern CAD systems?

[( )] Wireframe
[( )] CSG only
[(X)] Boundary representation (B-rep)
[( )] 2D drafting

---

## Module 2: Mathematical Foundations

### 2.1 Geometric Constraint Solving

<!-- class="theory-concept" -->
**The Constraint Satisfaction Problem**

A 2D sketch consists of:
- **Geometric entities**: Points, lines, arcs, circles, splines
- **Geometric constraints**: Parallel, perpendicular, tangent, coincident, concentric
- **Dimensional constraints**: Distances, angles, radii

**Problem**: Given a set of constraints, find entity positions that satisfy all constraints simultaneously.

**Mathematical Formulation**:

For n parameters (coordinates) $\mathbf{p} = (p_1, p_2, \ldots, p_n)$ and m constraints $C_i(\mathbf{p}) = 0$:

$$\begin{aligned}
C_1(p_1, p_2, \ldots, p_n) &= 0 \\
C_2(p_1, p_2, \ldots, p_n) &= 0 \\
&\vdots \\
C_m(p_1, p_2, \ldots, p_n) &= 0
\end{aligned}$$

This is a **system of nonlinear equations**.

**Example**: Two lines L₁ and L₂ constrained perpendicular:

If L₁ has direction vector $(a_1, b_1)$ and L₂ has direction $(a_2, b_2)$:

$$C_{\perp}(\mathbf{p}) = a_1 \cdot a_2 + b_1 \cdot b_2 = 0$$

**Solution Methods**:

1. **Numerical optimization** (Newton-Raphson):
   - Iteratively refine positions to minimize constraint violations
   - Fast convergence when initial guess is good
   - May find local minima

2. **Symbolic methods**:
   - Solve constraint equations algebraically
   - Guaranteed correct solution when it exists
   - Computationally expensive for complex systems

3. **Graph-based decomposition**:
   - Analyze constraint graph structure
   - Solve small subsystems independently
   - Most common in modern CAD

### 2.2 Degrees of Freedom Analysis

<!-- class="theory-concept" -->
**Under-Constrained, Fully-Constrained, Over-Constrained**

A sketch's constraint state determines its behavior:

**Degrees of Freedom (DOF)**:
$$\text{DOF} = n_{\text{params}} - n_{\text{constraints}}$$

- **Under-constrained** (DOF > 0): Sketch can move/change
- **Fully-constrained** (DOF = 0): Unique solution, sketch rigid
- **Over-constrained** (DOF < 0): Conflicting constraints, no solution

**Example**: Rectangle with 4 corners

- **Parameters**: 8 coordinates (4 points × 2D)
- **Implicit constraints**: 4 horizontal/vertical + 4 equal lengths + 1 corner position = 9 constraints
- **Explicit constraints needed**: 2 dimensions (width, height) + 1 position (x,y of origin) = 3
- **Total constraints**: 12
- **DOF**: 8 - 12 = -4 (appears over-constrained, but redundancy is intentional)

Modern CAD systems automatically detect and visualize constraint status.

### 2.3 Parametric Curves and Surfaces

<!-- class="theory-concept" -->
**Mathematical Representation of Curves**

CAD systems use parametric curves for sketches and edges:

**Line Segment**:
$$\mathbf{L}(t) = \mathbf{P}_0 + t(\mathbf{P}_1 - \mathbf{P}_0), \quad t \in [0,1]$$

**Circular Arc**:
$$\mathbf{C}(t) = \mathbf{C}_{\text{center}} + R(\cos\theta(t), \sin\theta(t)), \quad \theta(t) = \theta_0 + t(\theta_1 - \theta_0)$$

**Bézier Curves** (smooth freeform curves):

For control points $\mathbf{P}_0, \mathbf{P}_1, \ldots, \mathbf{P}_n$:

$$\mathbf{B}(t) = \sum_{i=0}^{n} \binom{n}{i} t^i (1-t)^{n-i} \mathbf{P}_i$$

Properties:
- Passes through first and last control points
- Intermediate points influence shape but curve doesn't pass through them
- Entire curve lies within convex hull of control points

**B-Splines** (industry standard):

$$\mathbf{S}(t) = \sum_{i=0}^{n} N_{i,p}(t) \mathbf{P}_i$$

where $N_{i,p}$ are B-spline basis functions.

Advantages:
- **Local control**: Moving one control point affects limited curve region
- **Continuity**: Smooth joins between curve segments
- **NURBS** (Non-Uniform Rational B-Splines): Can exactly represent conics (circles, ellipses)

### 2.4 Boundary Representation (B-rep)

<!-- class="theory-concept" -->
**Topological and Geometric Data**

A solid model separates topology (connectivity) from geometry (shape):

**Topological Elements**:
- **Vertices** (V): Points in 3D space
- **Edges** (E): Curves connecting vertices
- **Faces** (F): Surfaces bounded by edge loops
- **Shells**: Connected sets of faces
- **Solids**: Bounded volumes

**Euler-Poincaré Formula** (for simple polyhedra):
$$V - E + F = 2$$

For complex solids with holes:
$$V - E + F - H = 2(S - G)$$

where:
- H = number of holes through object
- S = number of shells
- G = genus (number of handles)

**Geometric Elements**:
- Vertex → 3D point $(x, y, z)$
- Edge → Parametric curve $\mathbf{C}(t)$
- Face → Parametric surface $\mathbf{S}(u, v)$

**Orientation**: Each face has an outward-pointing normal, defining "inside" vs "outside".

**Quiz: Mathematical Foundations**

A sketch with 10 parameters and 8 constraints has how many degrees of freedom?

[( )] 8
[( )] 10
[(X)] 2
[( )] 18

What does a Bézier curve pass through?

[(X)] Its first and last control points
[( )] All of its control points
[( )] None of its control points
[( )] Only its midpoint

---

## Module 3: Sketch-Based Modeling Workflow

### 3.1 The Sketch Plane

<!-- class="theory-concept" -->
**2D Sketches in 3D Space**

Modern CAD workflow begins with 2D sketches:

1. **Select a sketch plane**:
   - World coordinate planes (XY, XZ, YZ)
   - Existing planar face
   - Offset plane

2. **Create 2D geometry** in sketch plane coordinate system

3. **Apply constraints** to achieve design intent

4. **Generate 3D features** from sketch (extrude, revolve, etc.)

**Coordinate Systems**:
- **World coordinates**: Global XYZ reference
- **Sketch coordinates**: Local 2D system (typically UV or XY within plane)
- **Transformation**: 3D rotation matrix maps sketch to world space

### 3.2 Geometric Constraints

<!-- class="theory-concept" -->
**Common Constraint Types**

**Positional Constraints**:
- **Coincident**: Two points occupy same location
- **Midpoint**: Point lies at center of line segment
- **Concentric**: Two circles/arcs share same center

**Orientation Constraints**:
- **Horizontal/Vertical**: Line parallel to sketch axes
- **Parallel**: Two lines have same direction
- **Perpendicular**: Two lines at 90°
- **Tangent**: Curve smoothly touches another curve

**Dimensional Constraints**:
- **Distance**: Length between two points
- **Angle**: Rotation between two lines
- **Radius/Diameter**: Size of arc or circle

**Design Intent Example**:

For a mounting bracket:
- "This hole is always centered on the plate" → Midpoint constraints
- "These two holes are always the same distance apart" → Dimension + equality
- "The bracket width is twice its height" → Parametric equation: W = 2H

### 3.3 Constraint Solving Strategies

<!-- class="alternative-approach" -->
**Best Practices for Robust Sketches**

1. **Start with overall shape**, then add details
2. **Use geometric constraints before dimensional constraints**
   - Geometric constraints reduce DOF faster
   - Fewer numerical parameters to solve
3. **Avoid over-constraining**
   - Redundant constraints cause solver conflicts
   - CAD will indicate conflicts (orange/red highlighting)
4. **Anchor the sketch**
   - Fix at least one point to prevent floating
   - Typically constrain origin or key reference point
5. **Use construction geometry**
   - Reference lines for symmetry, center lines
   - Not included in final profile

**Common Pitfalls**:
- Assuming constraints that weren't explicitly applied
- Creating "accidental" constraints (nearly horizontal interpreted as horizontal)
- Chaining too many dependent dimensions

---

## Module 4: Feature-Based Solid Modeling

### 4.1 Extrusion Operations

<!-- class="theory-concept" -->
**Linear Extension of 2D Profiles**

**Extrude** operation sweeps a 2D profile along a direction:

$$\text{Solid} = \{\mathbf{p} + t\mathbf{d} \mid \mathbf{p} \in \text{Profile}, t \in [0, h]\}$$

where:
- **Profile**: Closed 2D sketch region
- **d**: Extrusion direction (typically perpendicular to sketch plane)
- **h**: Extrusion height/distance

**Variations**:
- **Symmetric extrude**: Extend equally in both directions (±h/2)
- **To object**: Extrude until intersecting another face
- **Through all**: Extrude through entire existing geometry

**Boolean Operations**:
- **New body**: Create separate solid
- **Join**: Union with existing solid (∪)
- **Cut**: Subtract from existing solid (-)
- **Intersect**: Keep only overlap (∩)

### 4.2 Revolution Operations

<!-- class="theory-concept" -->
**Rotational Symmetry**

**Revolve** operation rotates a 2D profile around an axis:

$$\text{Solid} = \{R_\theta(\mathbf{p}) \mid \mathbf{p} \in \text{Profile}, \theta \in [0, \theta_{\max}]\}$$

where $R_\theta$ is rotation matrix around the axis.

**Applications**:
- Cylindrical parts: shafts, pins, bushings
- Conical parts: funnels, nozzles
- Spherical parts: ball joints, domes

**Full vs. Partial Revolution**:
- Full revolution: θ_max = 360°
- Partial revolution: Create sectors, helical features

### 4.3 Additional Features

<!-- class="theory-concept" -->
**Modifying Geometry**

**Fillet**: Rounds sharp edges with radius R
- **Mathematical**: Blend surface tangent to adjacent faces
- **Purpose**: Reduce stress concentrations, improve aesthetics, safer handling
- **Variable radius**: Different R along edge length

**Chamfer**: Bevels edges at angle or distance
- **Types**: Distance-distance, distance-angle
- **Purpose**: Ease assembly, deburring, aesthetic

**Shell**: Hollows solid, leaving thin walls
- **Thickness**: Uniform or variable wall thickness
- **Open faces**: Remove selected faces (create open box)
- **Applications**: Injection-molded parts, weight reduction

**Hole**: Specialized feature for fasteners
- **Types**: Simple, counterbore, countersink, tapped (threaded)
- **Standards**: Metric/imperial sizing for screws
- **Placement**: On planar or cylindrical faces

**Pattern**: Replicate features
- **Rectangular**: Rows and columns
- **Circular**: Around an axis (e.g., bolt circle)
- **Path**: Along curve

### 4.4 Feature Tree and History

<!-- class="theory-concept" -->
**Parametric Feature History**

CAD maintains ordered sequence of features:

```
Sketch 1 (Base profile)
→ Extrude 1 (Create base)
→ Sketch 2 (Hole location)
→ Extrude 2 (Cut hole)
→ Fillet 1 (Round corners)
→ Pattern 1 (Replicate hole)
```

**Design Intent Preservation**:
- Edit earlier features → later features update automatically
- Reorder features (if dependencies allow)
- Suppress features (temporarily disable)

**Parent-Child Relationships**:
- Each feature can reference previous features
- Circular references not allowed
- Deleting parent feature may break children

**Best Practice**: Organize features logically, minimize dependencies.

**Quiz: Feature Operations**

Which operation is most appropriate for creating a circular shaft?

[( )] Extrude a circle
[(X)] Revolve a rectangle
[( )] Pattern a cylinder
[( )] Shell a sphere

What is the primary purpose of a fillet?

[( )] Make the part look better
[(X)] Reduce stress concentration at sharp corners
[( )] Increase material usage
[( )] Simplify manufacturing

---

## Module 5: File Formats and Interoperability

### 5.1 CAD File Formats

<!-- class="theory-concept" -->
**Native vs. Exchange Formats**

**Native Formats** (proprietary):
- **Fusion 360**: .f3d (cloud-based)
- **SolidWorks**: .sldprt, .sldasm
- **Inventor**: .ipt, .iam
- Preserve full parametric history
- Best for working within same CAD system

**Neutral Exchange Formats**:

**STEP (.step, .stp)** - ISO 10303:
- **Industry standard** for CAD exchange
- Preserves B-rep geometry accurately
- Includes product structure, metadata
- Loses parametric history
- Best for CAD-to-CAD transfer

**IGES (.igs, .iges)**:
- Older exchange format
- Surface and wireframe focus
- Less reliable than STEP

**Parasolid (.x_t, .x_b)**:
- Kernel format used by many CAD systems
- High fidelity geometry transfer

### 5.2 Manufacturing Formats

<!-- class="theory-concept" -->
**Formats for Fabrication**

**STL (Stereolithography)** - .stl:
- **Tesselated surface**: Triangle mesh approximation
- No B-rep, no curves—only flat triangles
- Standard for 3D printing
- ASCII or binary variants

**Structure**:
```
solid name
  facet normal nx ny nz
    outer loop
      vertex v1x v1y v1z
      vertex v2x v2y v2z
      vertex v3x v3y v3z
    endloop
  endfacet
  ...
endsolid
```

**Resolution**: Finer triangulation → larger file, better accuracy

**Limitations**:
- No color, material, or internal structure
- Approximation introduces small errors
- Units not explicitly stored (assume mm)

**3MF (3D Manufacturing Format)**:
- Modern replacement for STL
- Supports color, materials, lattices
- Compressed (smaller files)
- Growing adoption

### 5.3 Export Best Practices

<!-- class="alternative-approach" -->
**Preparing Models for Manufacturing**

**For 3D Printing (STL)**:
1. Set appropriate mesh resolution (balance quality/file size)
2. Verify **watertight** mesh (no gaps or holes)
3. Check for **inverted normals** (faces pointing inward)
4. Ensure **manifold geometry** (each edge shared by exactly 2 faces)
5. Orient part for optimal print orientation

**For CNC Machining (STEP)**:
1. Include all necessary geometry
2. Verify scale and units
3. Remove construction geometry
4. Check tolerance requirements

**For Simulation/Analysis**:
1. Simplify small features that don't affect results
2. Remove fillets/chamfers if appropriate (reduce mesh size)
3. Idealize geometry for analysis purposes

---

## Module 6: Design for Manufacturing Considerations

### 6.1 Additive Manufacturing Constraints

<!-- class="theory-concept" -->
**Design Rules for FDM 3D Printing**

**Overhangs**:
- **Maximum angle**: ~45° from vertical without support
- **Reason**: Molten plastic needs underlying material
- **Solution**: Add support structures or reorient part

**Minimum Wall Thickness**:
- **Typical**: 0.8-1.2 mm (2-3 nozzle widths for 0.4mm nozzle)
- **Thinner walls**: May not print reliably

**Bridging**:
- **Maximum unsupported horizontal span**: ~10-20 mm
- Depends on material, temperature, speed

**Hole Orientation**:
- **Vertical holes**: Print accurately
- **Horizontal holes**: May sag, print undersize due to layer compression

**Tolerances**:
- **Typical FDM accuracy**: ±0.2 mm
- **Functional fits**: Add clearance for sliding/press fits
- **Example**: For 8mm shaft, design 8.3mm hole for loose fit

### 6.2 Material Selection Impact

<!-- class="theory-concept" -->
**Common 3D Printing Materials**

**PLA (Polylactic Acid)**:
- **Ease of use**: Easy to print, minimal warping
- **Strength**: Moderate, brittle
- **Temperature**: Low heat resistance (~60°C)
- **Applications**: Prototypes, non-functional models

**PETG (Polyethylene Terephthalate Glycol)**:
- **Strength**: Higher impact resistance than PLA
- **Flexibility**: Less brittle
- **Temperature**: Better heat resistance (~80°C)
- **Chemical resistance**: Good
- **Applications**: Functional parts, mechanical components

**ABS (Acrylonitrile Butadiene Styrene)**:
- **Strength**: High impact resistance
- **Temperature**: Good heat resistance (~100°C)
- **Challenges**: Requires heated bed, prone to warping, toxic fumes
- **Applications**: Engineering parts, high-temp applications

Material choice affects:
- Print settings (temperature, speed)
- Support requirements
- Post-processing methods
- Final part properties

---

## Summary

This course established the theoretical and practical foundations of parametric solid modeling:

1. **Historical Evolution**: Traced CAD development from Sketchpad to modern parametric systems
2. **Mathematical Foundations**: Examined constraint solving, B-rep, and parametric curves
3. **Sketch-Based Workflow**: Studied constraint application and design intent capture
4. **Feature Operations**: Analyzed extrude, revolve, and modification features
5. **File Formats**: Understood native and exchange formats for different workflows

**Key Takeaways**:
- Parametric modeling captures design intent through constraints and relationships
- Boundary representation enables accurate solid geometry with topological structure
- Feature history provides flexible, reusable design definitions
- Manufacturing constraints must inform geometry creation

**Next Steps**: DESIGN-202 explores additive manufacturing theory and practice, translating digital models into physical objects.

---

## Optional Laboratory Exercises

### Lab 1: Constraint-Based Sketch (45 minutes)

<!-- class="optional-exercise" -->
**Objective**: Master geometric and dimensional constraints through systematic sketch creation.

**Tasks**:
1. Create a sketch for a mounting bracket with:
   - Rectangular base: 50mm × 30mm
   - Two mounting holes: Ø6mm, positioned 10mm from edges
   - Central slot: 8mm × 20mm
2. Apply constraints to fully constrain the sketch
3. Verify zero degrees of freedom (fully constrained state)
4. Create a parameter table (Width, Height, HoleDia, EdgeDist)
5. Link all dimensions to parameters
6. Demonstrate design flexibility by changing parameters

<!-- class="exercise-tip" -->
**Tips**:
- Start with construction lines for symmetry
- Use geometric constraints before dimensional
- Name parameters descriptively
- Verify constraint status (blue = fully constrained)

### Lab 2: Feature-Based Part Model (90 minutes)

<!-- class="optional-exercise" -->
**Objective**: Design a complete robot gripper finger using feature-based modeling.

**Requirements**:
- **Base body**: 60mm length, 15mm width, 5mm thickness
- **Mounting holes**: Two Ø3mm holes, 50mm apart, countersunk for M3 screws
- **Grip surface**: Serrated pattern (5 ridges, 2mm height, 8mm spacing)
- **Rounded edges**: 2mm fillet on all external edges
- **Material efficiency**: Maximum 15g of PLA (density 1.24 g/cm³)

**Tasks**:
1. Create base sketch with parametric dimensions
2. Extrude base body
3. Create hole pattern (sketch + extrude cut)
4. Model grip ridges (sketch + pattern)
5. Apply fillets
6. Verify mass constraint (use CAD analysis tools)
7. Export as STL for 3D printing

<!-- class="exercise-advanced" -->
**Advanced Extensions**:
- Make the finger parametric: adjustable length and width
- Create a mirrored left/right hand version
- Design a two-part finger with snap-fit assembly
- Optimize ridge geometry for maximum friction

### Lab 3: Design Iteration Workflow (60 minutes)

<!-- class="optional-exercise" -->
**Objective**: Experience parametric design flexibility through rapid iteration.

**Scenario**: Design a mounting boss for different screw sizes.

**Tasks**:
1. Create parametric model of cylindrical boss:
   - Parameter: ScrewSize (M3, M4, M5)
   - Boss diameter: 2.5 × ScrewSize
   - Boss height: 1.5 × ScrewSize
   - Hole diameter: ScrewSize + 0.3mm (clearance)
2. Create design variants using configuration table
3. Generate STL exports for all variants
4. Document design intent in parameter names/comments

<!-- class="exercise-tip" -->
**Analysis Questions**:
- How does parametric modeling accelerate design changes?
- What are limitations of pure parametric approach?
- When would direct modeling be more appropriate?

---

*Version 2.0.0 - Academic curriculum emphasizing theoretical foundations with optional practical exercises*
