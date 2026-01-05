<!--
author:   Dr. Maria Torres
email:    maria.torres@robotcampus.dev
version:  2.0.0

language: en
narrator: US English Female

comment:  Structural Mechanics and Analysis: Comprehensive treatment of stress-strain theory, finite element analysis, failure criteria, and design optimization for mechanical components.

icon:     https://robotcampus.dev/logos/design-203.png

mode:     Textbook

link:     https://robotcampus.dev/styles/course-styles.css

-->

# DESIGN-203: Structural Mechanics and Analysis

> **Structural analysis transforms intuitive design into quantitatively validated engineering through mathematical modeling and simulation.**

## Course Overview

| | |
|---|---|
| **Course Code** | DESIGN-203 |
| **Duration** | 4 hours |
| **Level** | Intermediate |
| **Prerequisites** | DESIGN-201, DESIGN-202, statics, calculus |
| **Theory-Practice** | 60% theory, 40% optional labs |

---

## Learning Objectives

Upon completion of this course, students will be able to:

### Theoretical Understanding

- Explain fundamental concepts of stress, strain, and material constitutive relationships
- Understand the mathematical foundations of the Finite Element Method (FEM)
- Analyze failure theories: von Mises, Tresca, maximum principal stress
- Describe stress concentration factors and their design implications
- Compare analytical and numerical approaches to structural analysis
- Evaluate design optimization strategies: topology, shape, and size optimization

### Practical Skills

- Perform hand calculations for simple stress analysis problems
- Set up and run FEA simulations in Fusion 360
- Interpret FEA results: stress distributions, safety factors, deformation
- Identify weak points and optimize designs for strength
- Apply design rules to minimize stress concentrations
- Validate simulation results through physical testing

---

## Module 1: Fundamentals of Stress and Strain

### 1.1 Stress Tensor and Equilibrium

<!-- class="theory-concept" -->
**Defining Internal Forces**

When a solid body is loaded, internal forces develop to maintain equilibrium. **Stress** is the intensity of internal force per unit area.

**Cauchy Stress Tensor** (3D):
$$\boldsymbol{\sigma} = \begin{bmatrix}
\sigma_{xx} & \tau_{xy} & \tau_{xz} \\
\tau_{yx} & \sigma_{yy} & \tau_{yz} \\
\tau_{zx} & \tau_{zy} & \sigma_{zz}
\end{bmatrix}$$

where:
- **σ_ii**: Normal stresses (perpendicular to face)
- **τ_ij**: Shear stresses (parallel to face)
- Units: Pa (N/m²), MPa common in engineering

**Stress Types**:

1. **Tensile Stress** (positive normal stress): Pulls apart
2. **Compressive Stress** (negative normal stress): Pushes together
3. **Shear Stress**: Causes sliding/twisting

**Equilibrium Equations**:
For static equilibrium in 3D:
$$\frac{\partial \sigma_{xx}}{\partial x} + \frac{\partial \tau_{yx}}{\partial y} + \frac{\partial \tau_{zx}}{\partial z} + f_x = 0$$

and similarly for y and z directions, where **f** are body forces (gravity, acceleration).

<!-- class="historical-note" -->
**Development of Continuum Mechanics**

- **1638**: Galileo studied beam bending, material strength
- **1773**: Coulomb analyzed failure in beams and columns
- **1821**: Navier derived beam equations
- **1822**: Cauchy formulated stress tensor concept
- **1826**: Cauchy developed linear elasticity theory
- **1856**: Saint-Venant contributed torsion and bending solutions
- **1950s**: Digital computers enabled numerical stress analysis

### 1.2 Strain and Deformation

<!-- class="theory-concept" -->
**Measuring Material Response**

**Strain** quantifies deformation relative to original dimensions.

**Normal Strain** (elongation/compression):
$$\epsilon_{xx} = \frac{\Delta L}{L_0}$$

where ΔL is change in length, L₀ is original length.

**Shear Strain** (angular distortion):
$$\gamma_{xy} = \frac{\Delta x}{h}$$

**Strain Tensor** (small deformations):
$$\boldsymbol{\epsilon} = \begin{bmatrix}
\epsilon_{xx} & \frac{1}{2}\gamma_{xy} & \frac{1}{2}\gamma_{xz} \\
\frac{1}{2}\gamma_{yx} & \epsilon_{yy} & \frac{1}{2}\gamma_{yz} \\
\frac{1}{2}\gamma_{zx} & \frac{1}{2}\gamma_{zy} & \epsilon_{zz}
\end{bmatrix}$$

**Strain-Displacement Relations**:
$$\epsilon_{xx} = \frac{\partial u}{\partial x}, \quad \gamma_{xy} = \frac{\partial u}{\partial y} + \frac{\partial v}{\partial x}$$

where **u**, **v**, **w** are displacements in x, y, z directions.

### 1.3 Constitutive Relations: Hooke's Law

<!-- class="theory-concept" -->
**Relating Stress to Strain**

For **linear elastic, isotropic materials**:

**Uniaxial Stress-Strain**:
$$\sigma = E \epsilon$$

where **E** is Young's modulus (stiffness).

**3D Generalized Hooke's Law**:
$$\epsilon_{xx} = \frac{1}{E}[\sigma_{xx} - \nu(\sigma_{yy} + \sigma_{zz})]$$
$$\gamma_{xy} = \frac{\tau_{xy}}{G}$$

where:
- **ν**: Poisson's ratio (lateral contraction ratio, typically 0.2-0.4)
- **G**: Shear modulus, $G = \frac{E}{2(1+\nu)}$

**Material Properties** (typical values):

| Material | E (GPa) | ν | Density (g/cm³) |
|----------|---------|---|-----------------|
| Steel | 200 | 0.30 | 7.85 |
| Aluminum | 70 | 0.33 | 2.70 |
| PLA (3D print) | 3-4 | 0.36 | 1.24 |
| PETG | 2.2 | 0.38 | 1.27 |
| ABS | 2-3 | 0.35 | 1.05 |

**Anisotropy in FDM Parts**:
3D printed parts are **transversely isotropic**:
- E_xy ≠ E_z (different stiffness in layer vs. cross-layer direction)
- Typically E_z ≈ 0.7-0.85 × E_xy

**Quiz: Stress-Strain Fundamentals**

What is the physical meaning of Young's modulus?

[(X)] Material stiffness (resistance to deformation)
[( )] Material strength (resistance to failure)
[( )] Material density
[( )] Maximum stress before yielding

A material with Poisson's ratio ν = 0.5 is:

[( )] Very stiff
[( )] Very compressible
[(X)] Incompressible (like rubber)
[( )] Anisotropic

---

## Module 2: Analytical Stress Analysis

### 2.1 Axial Loading and Bending

<!-- class="theory-concept" -->
**Simple Loading Cases**

**Axial Tension/Compression**:
For prismatic bar with force F and cross-sectional area A:
$$\sigma = \frac{F}{A}$$
$$\delta = \frac{FL}{EA}$$

**Beam Bending** (Euler-Bernoulli theory):
For beam subjected to moment M:
$$\sigma(y) = -\frac{M y}{I}$$

where:
- **I**: Second moment of area (depends on cross-section shape)
- **y**: Distance from neutral axis
- Maximum stress at **y = ±c** (outermost fibers)

**Section Modulus**:
$$S = \frac{I}{c}$$
$$\sigma_{\max} = \frac{M}{S}$$

**Common Cross-Sections**:
- Rectangular (b × h): $I = \frac{bh^3}{12}$
- Circular (diameter d): $I = \frac{\pi d^4}{64}$
- Hollow tube: $I = \frac{\pi (d_o^4 - d_i^4)}{64}$

**Design Principle**: Maximize I (stiffness) while minimizing A (weight) → hollow tubes are efficient.

### 2.2 Torsion and Shear

<!-- class="theory-concept" -->
**Twisting Loads**

**Torsion in Circular Shafts**:
For torque T applied to shaft:
$$\tau(r) = \frac{Tr}{J}$$

where:
- **J**: Polar moment of inertia
- **r**: Radial distance from center
- Maximum shear at outer surface

For solid circular shaft (diameter d):
$$J = \frac{\pi d^4}{32}$$
$$\tau_{\max} = \frac{16T}{\pi d^3}$$

**Shear in Beams**:
Transverse shear stress from shear force V:
$$\tau = \frac{VQ}{Ib}$$

where Q is first moment of area.

### 2.3 Stress Concentrations

<!-- class="theory-concept" -->
**Geometric Discontinuities**

Sharp geometric changes create local stress amplification.

**Stress Concentration Factor** (K_t):
$$\sigma_{\max} = K_t \sigma_{\text{nominal}}$$

**Common Stress Raisers**:
1. **Holes**: K_t ≈ 3 for circular hole in tension
2. **Fillets**: K_t depends on radius/diameter ratio
3. **Notches**: Sharp corners have very high K_t
4. **Threads**: K_t ≈ 3-4 for threaded fasteners
5. **Abrupt section changes**: Sharp transitions

**Mitigation Strategies**:
- **Increase fillet radius**: Smoother transition reduces K_t
- **Gradual tapers**: Avoid sudden diameter changes
- **Material removal**: Drill "relief holes" at sharp corners
- **Orientation**: Align features with principal stress directions

<!-- class="alternative-approach" -->
**Peterson's Stress Concentration Charts**

For fillet radius r connecting sections with width ratio D/d:
$$K_t = f\left(\frac{r}{d}, \frac{D}{d}\right)$$

Charts provide K_t values based on geometry:
- Larger r → lower K_t (smoother is better)
- Sharper transitions → higher K_t

**Quiz: Analytical Methods**

For a beam in bending, where does maximum stress occur?

[( )] At the neutral axis
[(X)] At the outermost fibers
[( )] At the supports
[( )] Uniformly distributed

Why are hollow tubes structurally efficient?

[(X)] High second moment of area with low cross-sectional area
[( )] They are lighter
[( )] They are easier to manufacture
[( )] They have higher strength

---

## Module 3: Failure Theories

### 3.1 Yield Criteria for Ductile Materials

<!-- class="theory-concept" -->
**Predicting Material Failure**

For multiaxial stress states, when does material yield (begin plastic deformation)?

**Principal Stresses**:
Eigenvalues of stress tensor: σ₁ ≥ σ₂ ≥ σ₃

**von Mises Criterion** (most widely used):
Material yields when von Mises stress reaches yield strength:
$$\sigma_{VM} = \sqrt{\frac{1}{2}[(\sigma_1-\sigma_2)^2 + (\sigma_2-\sigma_3)^2 + (\sigma_3-\sigma_1)^2]}$$

Yield condition: σ_VM ≥ σ_y

**Physical Interpretation**: Distortion energy theory - yield occurs when distortional strain energy reaches critical value.

**Tresca Criterion** (maximum shear stress):
$$\tau_{\max} = \frac{\sigma_1 - \sigma_3}{2} \geq \frac{\sigma_y}{2}$$

More conservative than von Mises, simpler to calculate.

**Comparison**:
- **von Mises**: Better experimental correlation for ductile metals
- **Tresca**: More conservative, simpler
- Both predict same result for uniaxial and pure shear

### 3.2 Failure of Brittle Materials

<!-- class="theory-concept" -->
**Fracture-Based Criteria**

Brittle materials (ceramics, cast iron, some plastics) fracture without yielding.

**Maximum Principal Stress Theory** (Rankine):
Failure when:
$$\max(\sigma_1, |\sigma_3|) \geq \sigma_{\text{UTS}}$$

where σ_UTS is ultimate tensile strength.

**Mohr-Coulomb Theory**:
Accounts for different tensile and compressive strengths:
$$\frac{\sigma_1}{\sigma_t} - \frac{\sigma_3}{\sigma_c} \geq 1$$

where σ_t = tensile strength, σ_c = compressive strength.

### 3.3 Factor of Safety

<!-- class="theory-concept" -->
**Design Margins**

**Factor of Safety (FOS)** or **Safety Factor (SF)**:
$$SF = \frac{\text{Material Strength}}{\text{Applied Stress}}$$

For von Mises:
$$SF = \frac{\sigma_y}{\sigma_{VM}}$$

**Typical Values**:
- **SF > 1**: Safe (design goal)
- **SF = 1**: At yield limit
- **SF < 1**: Failure predicted
- **Aerospace**: SF = 1.5-2.0 (weight-critical)
- **General machinery**: SF = 2.0-3.0
- **Civil structures**: SF = 3.0-5.0 (safety-critical)
- **Unknown loading**: SF = 4.0-6.0

**Considerations**:
- Uncertainty in loading
- Material property variation
- Manufacturing defects
- Fatigue and corrosion
- Consequences of failure

---

## Module 4: Finite Element Analysis

### 4.1 FEA Mathematical Foundations

<!-- class="theory-concept" -->
**Discretization and Approximation**

FEA divides complex geometry into simple elements, approximates solution.

**Basic Concept**:
1. **Discretization**: Mesh of elements (tetrahedra, hexahedra)
2. **Approximation**: Displacement field within element:
   $$u(x,y,z) = N_1(x,y,z) u_1 + N_2(x,y,z) u_2 + \ldots$$
   where N_i are shape functions, u_i are nodal displacements
3. **Element Equations**: From virtual work principle:
   $$[K^e] \{u^e\} = \{F^e\}$$
   where K^e is element stiffness matrix
4. **Assembly**: Combine element equations into global system:
   $$[K_{global}] \{U\} = \{F\}$$
5. **Solve**: Linear system for nodal displacements
6. **Post-Process**: Calculate stresses from displacements

**Element Stiffness Matrix**:
$$K_{ij}^e = \int_{\Omega^e} [B]^T [D] [B] \, dV$$

where:
- **[B]**: Strain-displacement matrix
- **[D]**: Material constitutive matrix (from Hooke's law)

<!-- class="historical-note" -->
**FEA Development Timeline**

- **1943**: Courant's variational methods for structural analysis
- **1956**: Turner et al. developed direct stiffness method
- **1960**: "Finite Element" term coined by Clough
- **1965**: Commercial software emerged (NASTRAN for NASA)
- **1970s**: Nonlinear and dynamic analysis capabilities
- **1980s**: CAD integration, graphical pre/post-processing
- **2000s**: Multi-physics coupling, optimization integration
- **2010s**: Cloud-based simulation, real-time analysis

### 4.2 Meshing Strategies

<!-- class="theory-concept" -->
**Discretization Quality**

**Element Types**:
- **Tetrahedra (Tet4, Tet10)**: 4 or 10 nodes, automatic meshing
- **Hexahedra (Hex8, Hex20)**: 8 or 20 nodes, higher accuracy, manual effort
- **Higher-order elements**: Quadratic shape functions, curved edges

**Mesh Quality Metrics**:
1. **Element size**: Smaller → more accurate, more computation
2. **Aspect ratio**: Length/width, should be < 5:1
3. **Skewness**: Element distortion, should be < 0.8
4. **Jacobian**: Measure of element warping

**Mesh Refinement**:
- **Global**: Uniform element size reduction
- **Local**: Fine mesh in high-stress regions
- **Adaptive**: Automatic refinement based on error estimates

**Convergence Study**:
1. Run analysis with coarse mesh
2. Refine mesh (reduce element size)
3. Compare results (stress, displacement)
4. Repeat until results change < 5%
5. Use converged mesh for final analysis

### 4.3 Boundary Conditions and Loading

<!-- class="theory-concept" -->
**Defining the Problem**

**Constraints** (prevent rigid body motion):
- **Fixed**: All degrees of freedom constrained (u = v = w = 0)
- **Pinned**: Translations constrained, rotations free
- **Symmetry**: Displacement normal to plane = 0
- **Frictionless support**: Normal displacement = 0

**Loads**:
- **Force**: Point force (N), distributed force (N/m²)
- **Pressure**: Normal stress on surface (Pa)
- **Moment/Torque**: Rotational loading (N·m)
- **Body force**: Gravity, acceleration (N/m³)
- **Thermal**: Temperature change inducing thermal stress

**Common Errors**:
- **Under-constrained**: Rigid body motion → singular stiffness matrix
- **Over-constrained**: Artificial stresses from conflicting constraints
- **Unrealistic loads**: Point loads on fine mesh → stress singularities

### 4.4 Interpreting Results

<!-- class="theory-concept" -->
**Critical Analysis**

**Stress Visualization**:
- **von Mises stress**: Scalar field, indicates yielding
- **Principal stresses**: Maximum/minimum normal stresses
- **Shear stress**: Magnitude and direction

**Color Maps**:
- Blue → low stress (safe)
- Green/yellow → moderate
- Red → high stress (approaching limits)
- Scale is critical: Check maximum values

**Common Issues**:
1. **Singularities**: Infinite stress at point loads, sharp corners
   - Solution: Average over region, use local stresses away from singularity
2. **Mesh dependency**: Coarse mesh underestimates stress
   - Solution: Mesh refinement, convergence study
3. **Constraint artifacts**: Artificial stress concentrations at fixed points
   - Solution: Use realistic constraints

**Validation**:
- Hand calculations for simple geometries
- Comparison with analytical solutions
- Physical testing of prototypes
- Experience and engineering judgment

**Quiz: FEA Concepts**

What happens if a mesh is too coarse?

[(X)] The analysis underestimates stress and displacement
[( )] The analysis takes too long to run
[( )] The results are more accurate
[( )] Nothing, mesh size doesn't matter

A stress singularity at a sharp corner means:

[( )] The part will definitely fail there
[(X)] The stress is artificially infinite due to idealized geometry
[( )] The mesh is too fine
[( )] The material model is wrong

---

## Module 5: Design Optimization

### 5.1 Optimization Problem Formulation

<!-- class="theory-concept" -->
**Mathematical Framework**

**General Optimization**:
$$\min_{x} \quad f(x)$$
$$\text{subject to:} \quad g_i(x) \leq 0, \quad h_j(x) = 0$$

where:
- **x**: Design variables (dimensions, shape, topology)
- **f(x)**: Objective function (minimize mass, cost, deflection)
- **g_i**: Inequality constraints (stress < yield, size limits)
- **h_j**: Equality constraints (volume = constant)

**Example**: Minimize beam mass while maintaining strength
- Variables: width w, height h
- Objective: minimize A = w·h (proportional to mass)
- Constraint: σ_max < σ_allowable

### 5.2 Topology Optimization

<!-- class="theory-concept" -->
**Material Distribution Optimization**

Finds optimal material layout within design space.

**SIMP Method** (Solid Isotropic Material with Penalization):
- Assign density ρ ∈ [0, 1] to each element
- Interpolate stiffness: E(ρ) = ρ^p E_0 (p ≈ 3 for penalization)
- Optimize: Maximize stiffness subject to volume constraint
- Result: Binary distribution (ρ → 0 or 1)

**Workflow**:
1. Define design space (where material can be placed)
2. Define non-design space (fixed geometry)
3. Specify loads and constraints
4. Set volume fraction target (e.g., 30% of design space)
5. Run optimization
6. Interpret organic result
7. Recreate manufacturable geometry

**Applications**:
- Aerospace brackets (30-50% weight reduction)
- Automotive parts
- Additive manufacturing (enables complex geometries)

<!-- class="alternative-approach" -->
**Generative Design**

Extends topology optimization with AI/ML:
- Explores thousands of design alternatives
- Multi-objective optimization (strength, cost, manufacturability)
- Considers manufacturing constraints (FDM, CNC, casting)
- Available in Fusion 360, Autodesk Netfabb

### 5.3 Shape and Size Optimization

<!-- class="theory-concept" -->
**Parametric Refinement**

**Size Optimization**:
- Variables: Dimensions (thickness, diameter, etc.)
- Fixed topology/shape
- Gradient-based methods (efficient)

**Shape Optimization**:
- Variables: Boundary geometry (spline control points)
- Topology fixed
- More complex, requires remeshing

**Multi-Objective Optimization**:
Pareto frontier: Trade-off between competing objectives
- Strength vs. weight
- Stiffness vs. cost
- Performance vs. manufacturability

---

## Summary

This course established comprehensive understanding of structural mechanics and analysis:

1. **Stress-Strain Theory**: Examined fundamental relationships and material behavior
2. **Analytical Methods**: Studied classical solutions for beams, shafts, and stress concentrations
3. **Failure Theories**: Analyzed von Mises, Tresca, and brittle failure criteria
4. **Finite Element Analysis**: Understood mathematical foundations and practical application
5. **Design Optimization**: Explored topology, shape, and size optimization strategies

**Key Takeaways**:
- Stress analysis enables quantitative validation of designs before fabrication
- FEA provides numerical solutions for complex geometries beyond analytical methods
- Failure theories predict when materials yield or fracture under multiaxial stress
- Optimization techniques systematically improve designs for performance metrics

**Next Steps**: DESIGN-204 explores kinematic analysis and parametric assemblies for mechanism design.

---

## Optional Laboratory Exercises

### Lab 1: Analytical vs. FEA Comparison (90 minutes)

<!-- class="optional-exercise" -->
**Objective**: Validate FEA results against analytical beam theory.

**Test Case**: Cantilever beam (fixed at one end, force at free end)
- Geometry: L = 100mm, b = 10mm, h = 5mm
- Material: PLA (E = 3.5 GPa, σ_y = 50 MPa)
- Load: F = 20 N at free end

**Tasks**:
1. **Hand Calculation**:
   - Calculate maximum stress: $\sigma_{\max} = \frac{M c}{I}$ where M = F·L
   - Calculate deflection: $\delta = \frac{FL^3}{3EI}$
   - Calculate safety factor: SF = σ_y / σ_max

2. **FEA Simulation**:
   - Model beam in Fusion 360
   - Apply fixed constraint at one end
   - Apply 20N force at free end
   - Mesh with 2mm elements
   - Run static stress analysis
   - Extract maximum stress and deflection

3. **Comparison**:
   - Calculate percent error: $\text{Error} = \frac{|\text{FEA} - \text{Analytical}|}{\text{Analytical}} \times 100\%$
   - Should be < 5% for well-meshed model

<!-- class="exercise-tip" -->
**Tip**: Maximum stress will be at the fixed end, top/bottom surface.

### Lab 2: Stress Concentration Study (120 minutes)

<!-- class="optional-exercise" -->
**Objective**: Investigate effect of fillet radius on stress concentration.

**Design**: Flat bar with central hole, subjected to tension
- Bar: 50mm × 20mm × 3mm
- Hole diameter: 8mm
- Applied stress: σ_nominal = 10 MPa

**Tasks**:
1. Create parametric model with fillet radius R as parameter
2. Run FEA for R = 0mm, 0.5mm, 1mm, 2mm, 4mm
3. For each case, record σ_max at hole edge
4. Calculate K_t = σ_max / σ_nominal
5. Plot K_t vs. R
6. Compare with Peterson's stress concentration factors

<!-- class="exercise-advanced" -->
**Advanced Extension**:
- Add physical 3D printed specimens
- Perform tension test to failure
- Observe crack initiation location
- Correlate with FEA predictions

### Lab 3: Design Optimization Project (150 minutes)

<!-- class="optional-exercise" -->
**Objective**: Optimize robot gripper finger for minimum mass with strength constraint.

**Requirements**:
- Applied force: 50N compressive load
- Safety factor: SF ≥ 2.0
- Material: PETG (σ_y = 50 MPa, E = 2.2 GPa)

**Tasks**:
1. **Baseline Design**:
   - Create initial solid finger design from DESIGN-201
   - Run FEA to determine initial mass and SF

2. **Manual Optimization**:
   - Identify low-stress regions (candidates for material removal)
   - Add lightening holes, ribs, or hollow sections
   - Maintain mounting interface geometry
   - Iterate: Remove material → Analyze → Check SF
   - Target: 40% mass reduction while maintaining SF ≥ 2.0

3. **Topology Optimization** (if available):
   - Set up generative design study
   - Define preserve regions (mounting points, contact surfaces)
   - Set objectives: Minimize mass, SF ≥ 2.0
   - Run optimization
   - Interpret organic result, create manufacturable geometry

4. **Comparison**:
   - Compare manual vs. topology optimized designs
   - Evaluate: mass, strength, manufacturability, aesthetics

<!-- class="exercise-tip" -->
**Design Principles**:
- Place material where stress is highest
- Hollow sections increase I/A ratio (efficient bending resistance)
- Gradual transitions prevent stress concentrations
- Consider print orientation and support requirements

**Deliverable**: Design report with CAD models, FEA results, mass comparison, and recommendation for final design.

---

*Version 2.0.0 - Academic curriculum emphasizing theoretical foundations with optional practical exercises*
