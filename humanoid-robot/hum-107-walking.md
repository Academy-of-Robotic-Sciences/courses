---
title: "HUM-107: Bipedal Walking and Gait Control"
version: "2.0.0"
description: "Master the biomechanics, mathematics, and control strategies for stable bipedal locomotion in humanoid robots"
keywords: "bipedal walking, gait cycle, ZMP, foot trajectory planning, humanoid locomotion, gait control, walking patterns"
author: "Dr. Sarah Yamamoto, Humanoid Robotics Institute"
last_updated: "2025-11-09"
course_level: "Advanced"
estimated_time: "12 hours"
prerequisites: ["HUM-106: Balance and Stability"]
---

<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>HUM-107: Bipedal Walking and Gait Control | Humanoid Robot Course</title>
    <link rel="stylesheet" href="../styles/course-style.css">
</head>
<body>
    <header class="course-header">
        <div class="header-content">
            <span class="course-code">HUM-107</span>
            <h1>Bipedal Walking and Gait Control</h1>
            <p class="course-tagline">Engineering Stable Locomotion Through Mathematical Gait Generation</p>
        </div>
    </header>

    <nav class="course-nav">
        <a href="hum-106-balance.md" class="nav-link nav-previous">← HUM-106: Balance and Stability</a>
        <a href="../index.html" class="nav-link nav-home">Course Home</a>
        <a href="hum-108-manipulation.md" class="nav-link nav-next">HUM-108: Manipulation →</a>
    </nav>

    <main class="course-content">
        <section class="content-section">
            <h2 class="section-title">Course Overview</h2>
            <div class="overview-grid">
                <div class="overview-item">
                    <h3>Learning Objectives</h3>
                    <ul class="objectives-list">
                        <li>Understand the biomechanics and phases of the human gait cycle</li>
                        <li>Implement mathematical models for foot trajectory generation</li>
                        <li>Design ZMP-based walking controllers for stable bipedal locomotion</li>
                        <li>Develop gait timing and phase transition algorithms</li>
                        <li>Engineer turning and directional control during walking</li>
                        <li>Analyze gait stability through simulation and real-world testing</li>
                    </ul>
                </div>
                <div class="overview-item">
                    <h3>Key Topics</h3>
                    <ul class="topics-list">
                        <li>Gait Cycle Theory and Biomechanics</li>
                        <li>Foot Trajectory Planning</li>
                        <li>ZMP-Based Walking Control</li>
                        <li>Gait Timing and Step Frequency</li>
                        <li>Turn Control and Direction Changes</li>
                        <li>Adaptive Gait Parameters</li>
                    </ul>
                </div>
            </div>
        </section>

        <section class="content-section">
            <h2 class="section-title">Historical Context: The Evolution of Bipedal Robotics</h2>
            <div class="historical-content">
                <div class="timeline">
                    <div class="timeline-item">
                        <span class="timeline-year">1969</span>
                        <h4>WABOT-1: First Biped Steps</h4>
                        <p>Waseda University's WABOT-1 demonstrated basic bipedal walking, taking steps at approximately 20 seconds per step. While primitive, it established fundamental principles of static walking and sequential foot placement that would inform decades of research.</p>
                    </div>
                    <div class="timeline-item">
                        <span class="timeline-year">1986</span>
                        <h4>ZMP Theory Foundation</h4>
                        <p>Miomir Vukobratović introduced the Zero Moment Point (ZMP) concept, revolutionizing bipedal walking control. This mathematical framework provided a measurable stability criterion, enabling engineers to design controllers that maintained balance during dynamic locomotion.</p>
                    </div>
                    <div class="timeline-item">
                        <span class="timeline-year">1996</span>
                        <h4>Honda P2: Dynamic Walking</h4>
                        <p>Honda's P2 robot achieved smooth, human-like walking at 2 km/h, demonstrating advanced gait control with natural arm swing and adaptive step adjustment. This milestone proved that bipedal robots could navigate real-world environments with grace and efficiency.</p>
                    </div>
                    <div class="timeline-item">
                        <span class="timeline-year">2000</span>
                        <h4>ASIMO: Continuous Gait Refinement</h4>
                        <p>Honda's ASIMO showcased variable-speed walking, stair climbing, and real-time path adjustment. The integration of predictive control and sensor fusion enabled responsive locomotion that could adapt to changing terrain and obstacles.</p>
                    </div>
                    <div class="timeline-item">
                        <span class="timeline-year">2013</span>
                        <h4>ATLAS: Rough Terrain Locomotion</h4>
                        <p>Boston Dynamics' ATLAS robot demonstrated walking over rubble, climbing obstacles, and recovering from pushes. Advanced whole-body momentum control and model-predictive trajectory optimization enabled unprecedented robustness in challenging environments.</p>
                    </div>
                    <div class="timeline-item">
                        <span class="timeline-year">2020s</span>
                        <h4>Learning-Based Gait Synthesis</h4>
                        <p>Modern humanoids employ reinforcement learning and neural network policies to discover efficient gaits automatically. Sim-to-real transfer enables robots to learn complex locomotion behaviors in simulation and deploy them on physical hardware.</p>
                    </div>
                </div>

                <div class="insight-box">
                    <h4>Engineering Insight</h4>
                    <p>The progression from static walking (waiting until fully stable before each step) to dynamic walking (using momentum and controlled falling) represents a fundamental paradigm shift. Early robots relied on conservative, energy-inefficient gaits to guarantee stability. Modern approaches embrace the controlled instability inherent in human walking, using predictive models to maintain balance through continuous motion rather than static poses. This evolution required advances in sensor technology, computational power, and control theory—particularly the development of model predictive control (MPC) that could optimize trajectories in real-time while respecting physical constraints.</p>
                </div>
            </div>
        </section>

        <section class="content-section theory-section">
            <h2 class="section-title">Part 1: Gait Cycle Theory and Biomechanics</h2>

            <h3>1.1 The Human Gait Cycle</h3>
            <p>Human walking consists of a repeating cycle of coordinated movements, traditionally divided into two main phases: the <strong>stance phase</strong> (when the foot is in contact with the ground) and the <strong>swing phase</strong> (when the foot is moving through the air). Understanding this biological template provides essential insights for robotic gait design.</p>

            <div class="theory-block">
                <h4>Gait Phase Definitions</h4>
                <p>The complete gait cycle for a single leg spans from heel strike (initial contact) to the next heel strike of the same foot, typically lasting 1.0–1.2 seconds in normal human walking. The cycle divides as follows:</p>

                <ul class="definition-list">
                    <li><strong>Stance Phase (60% of cycle):</strong> The foot remains in contact with the ground, providing support and propulsion.
                        <ul>
                            <li><em>Heel Strike (0%):</em> Initial contact with the ground</li>
                            <li><em>Foot Flat (10%):</em> Entire foot contacts the surface</li>
                            <li><em>Mid-Stance (30%):</em> Body weight passes over the support foot</li>
                            <li><em>Heel Off (40%):</em> Heel begins to lift as weight shifts forward</li>
                            <li><em>Toe Off (60%):</em> Foot leaves the ground, ending stance</li>
                        </ul>
                    </li>
                    <li><strong>Swing Phase (40% of cycle):</strong> The foot moves through the air to prepare for the next step.
                        <ul>
                            <li><em>Initial Swing (60-73%):</em> Foot clears the ground and begins forward motion</li>
                            <li><em>Mid-Swing (73-87%):</em> Foot passes under the body</li>
                            <li><em>Terminal Swing (87-100%):</em> Foot decelerates and prepares for heel strike</li>
                        </ul>
                    </li>
                </ul>

                <div class="equation-block">
                    <p><strong>Gait Phase Function:</strong></p>
                    <p>For a gait cycle with period T, the phase variable φ(t) ∈ [0, 1] represents progress through the cycle:</p>
                    <div class="equation">
                        φ(t) = (t mod T) / T
                    </div>
                    <p>The phase can be mapped to specific control states:</p>
                    <div class="equation">
                        state(φ) = {
                            stance,  if φ ∈ [0, 0.6)
                            swing,   if φ ∈ [0.6, 1.0)
                        }
                    </div>
                </div>
            </div>

            <h3>1.2 Double Support and Single Support Periods</h3>
            <p>During normal walking, there exist brief periods when both feet contact the ground simultaneously. These <strong>double support phases</strong> occur during the transition between steps and are critical for stability.</p>

            <div class="theory-block">
                <h4>Support Phase Analysis</h4>
                <div class="equation-block">
                    <p><strong>Support Polygon Dynamics:</strong></p>
                    <p>During double support, the support polygon S<sub>double</sub> encompasses both feet:</p>
                    <div class="equation">
                        S<sub>double</sub> = ConvexHull({p<sub>left</sub>, p<sub>right</sub>})
                    </div>
                    <p>where p<sub>left</sub> and p<sub>right</sub> represent the contact points of each foot.</p>

                    <p>During single support, the polygon reduces to a single foot:</p>
                    <div class="equation">
                        S<sub>single</sub> = ConvexHull({p<sub>stance</sub>})
                    </div>

                    <p>The dramatic reduction in support area during single support creates the primary stability challenge in bipedal walking. The ZMP must remain within this smaller polygon, requiring precise control of body dynamics.</p>
                </div>

                <p>For stable walking, the double support phase typically occupies 10-20% of the gait cycle. Longer double support periods increase stability but reduce efficiency and walking speed. Shorter periods approach running, where a flight phase (no ground contact) may occur.</p>
            </div>

            <h3>1.3 Center of Mass Trajectory During Walking</h3>
            <p>The body's center of mass (CoM) follows a characteristic three-dimensional trajectory during walking, with vertical oscillation, lateral sway, and forward progression all carefully coordinated.</p>

            <div class="theory-block">
                <h4>CoM Motion Characteristics</h4>
                <div class="equation-block">
                    <p><strong>Vertical Oscillation:</strong></p>
                    <p>The CoM height z<sub>CoM</sub>(t) varies sinusoidally with double the step frequency:</p>
                    <div class="equation">
                        z<sub>CoM</sub>(t) = z<sub>0</sub> + A<sub>z</sub> sin(4πt/T)
                    </div>
                    <p>where z<sub>0</sub> is the nominal height (typically 0.55-0.60 of total robot height), and A<sub>z</sub> is the vertical oscillation amplitude (typically 2-4 cm in humans, scaled proportionally for robots).</p>

                    <p><strong>Lateral Sway:</strong></p>
                    <p>The lateral position y<sub>CoM</sub>(t) shifts from side to side over each support foot:</p>
                    <div class="equation">
                        y<sub>CoM</sub>(t) = A<sub>y</sub> sin(2πt/T + φ<sub>y</sub>)
                    </div>
                    <p>with amplitude A<sub>y</sub> approximately equal to half the hip separation distance, and phase offset φ<sub>y</sub> chosen to align the CoM over the stance foot during mid-stance.</p>

                    <p><strong>Forward Progression:</strong></p>
                    <p>The forward velocity v<sub>x</sub> varies slightly throughout the gait cycle but maintains a mean value:</p>
                    <div class="equation">
                        x<sub>CoM</sub>(t) = v<sub>nominal</sub> · t + A<sub>x</sub> sin(2πt/T + φ<sub>x</sub>)
                    </div>
                    <p>The oscillating component represents the acceleration and deceleration during stance phase transitions.</p>
                </div>
            </div>

            <div class="quiz-block">
                <h4>Comprehension Check: Gait Cycle Fundamentals</h4>
                <div class="quiz-question">
                    <p><strong>Question 1:</strong> If a humanoid robot has a gait cycle period of T = 1.0 seconds, at what time during the cycle does the swing phase typically begin?</p>
                    <div class="quiz-options">
                        <label><input type="radio" name="q1" value="a"> A) 0.2 seconds</label>
                        <label><input type="radio" name="q1" value="b"> B) 0.4 seconds</label>
                        <label><input type="radio" name="q1" value="c"> C) 0.6 seconds (correct)</label>
                        <label><input type="radio" name="q1" value="d"> D) 0.8 seconds</label>
                    </div>
                    <div class="quiz-explanation">
                        <p><strong>Explanation:</strong> The stance phase occupies approximately 60% of the gait cycle, so the swing phase begins at t = 0.6T = 0.6 seconds. This timing allows for a double support period at the beginning of stance before the opposite foot lifts.</p>
                    </div>
                </div>

                <div class="quiz-question">
                    <p><strong>Question 2:</strong> Why is the single support phase more challenging for stability than the double support phase?</p>
                    <div class="quiz-options">
                        <label><input type="radio" name="q2" value="a"> A) The robot moves faster during single support</label>
                        <label><input type="radio" name="q2" value="b"> B) The support polygon is significantly smaller (correct)</label>
                        <label><input type="radio" name="q2" value="c"> C) Sensor accuracy decreases during single support</label>
                        <label><input type="radio" name="q2" value="d"> D) Motor torque limits are reduced</label>
                    </div>
                    <div class="quiz-explanation">
                        <p><strong>Explanation:</strong> During single support, the support polygon reduces from the convex hull of both feet to just the contact area of one foot. This dramatically smaller region makes it more difficult to keep the ZMP within bounds, requiring more precise control of the body's dynamics.</p>
                    </div>
                </div>
            </div>
        </section>

        <section class="content-section theory-section">
            <h2 class="section-title">Part 2: Foot Trajectory Planning</h2>

            <h3>2.1 Foot Path Requirements</h3>
            <p>The trajectory of the swing foot must satisfy multiple constraints: sufficient ground clearance to avoid obstacles, smooth velocity profiles to minimize impact forces, and precise positioning for the next heel strike. Mathematical trajectory generation ensures these requirements are met consistently.</p>

            <div class="theory-block">
                <h4>Trajectory Constraints</h4>
                <ul class="definition-list">
                    <li><strong>Boundary Conditions:</strong> Initial position (toe-off point), final position (heel-strike target), and zero velocity at both endpoints to ensure smooth transitions.</li>
                    <li><strong>Clearance Height:</strong> Maximum vertical displacement during mid-swing, typically 5-10 cm above ground level to clear small obstacles.</li>
                    <li><strong>Continuity:</strong> C² continuous trajectories (continuous position, velocity, and acceleration) to avoid jerky motion that could excite mechanical resonances.</li>
                    <li><strong>Timing:</strong> Swing duration coordinated with gait phase to ensure proper foot placement relative to the body's CoM.</li>
                </ul>

                <div class="equation-block">
                    <p><strong>Polynomial Trajectory Formulation:</strong></p>
                    <p>A fifth-order polynomial provides sufficient degrees of freedom to satisfy position, velocity, and acceleration constraints at both endpoints:</p>
                    <div class="equation">
                        p(τ) = a<sub>0</sub> + a<sub>1</sub>τ + a<sub>2</sub>τ² + a<sub>3</sub>τ³ + a<sub>4</sub>τ⁴ + a<sub>5</sub>τ⁵
                    </div>
                    <p>where τ ∈ [0, 1] is the normalized time parameter within the swing phase.</p>

                    <p>Given boundary conditions:</p>
                    <div class="equation">
                        p(0) = p<sub>start</sub>,  p'(0) = 0,  p''(0) = 0
                        p(1) = p<sub>end</sub>,    p'(1) = 0,  p''(1) = 0
                    </div>

                    <p>The coefficients are determined by solving the linear system:</p>
                    <div class="equation">
                        a<sub>0</sub> = p<sub>start</sub>
                        a<sub>1</sub> = 0
                        a<sub>2</sub> = 0
                        a<sub>3</sub> = 10(p<sub>end</sub> - p<sub>start</sub>)
                        a<sub>4</sub> = -15(p<sub>end</sub> - p<sub>start</sub>)
                        a<sub>5</sub> = 6(p<sub>end</sub> - p<sub>start</sub>)
                    </div>
                </div>
            </div>

            <h3>2.2 Vertical Clearance via Cycloid Profiles</h3>
            <p>For the vertical component of foot motion, a cycloid or modified sinusoidal profile often provides more natural motion than polynomial interpolation alone, with smooth acceleration and deceleration phases.</p>

            <div class="theory-block">
                <h4>Cycloid Vertical Profile</h4>
                <div class="equation-block">
                    <p>The vertical foot position z<sub>foot</sub>(τ) during swing can be modeled as:</p>
                    <div class="equation">
                        z<sub>foot</sub>(τ) = h<sub>clearance</sub> · sin(πτ)
                    </div>
                    <p>This creates a smooth arc with maximum height at τ = 0.5 (mid-swing).</p>

                    <p>For improved control over the timing of maximum clearance, a modified profile uses:</p>
                    <div class="equation">
                        z<sub>foot</sub>(τ) = h<sub>clearance</sub> · (1 - cos(πτ))/2 + z<sub>offset</sub>
                    </div>
                    <p>where z<sub>offset</sub> accounts for any ground slope or terrain variation.</p>

                    <p><strong>Velocity Profile:</strong></p>
                    <div class="equation">
                        v<sub>z</sub>(τ) = dz/dt = (h<sub>clearance</sub> · π · sin(πτ)) / (2 · T<sub>swing</sub>)
                    </div>
                    <p>This ensures zero vertical velocity at both endpoints, minimizing impact forces.</p>
                </div>
            </div>

            <h3>2.3 Step Length and Placement Planning</h3>
            <p>Determining where to place the next footstep requires considering the robot's current velocity, desired direction change, and stability margins. The <strong>capture point</strong> framework provides a mathematical foundation for this decision.</p>

            <div class="theory-block">
                <h4>Capture Point Theory</h4>
                <p>The capture point (also called the divergent component of motion or DCM) represents the point on the ground where the robot must step to come to a complete stop. For a simplified linear inverted pendulum model:</p>

                <div class="equation-block">
                    <div class="equation">
                        ξ = x<sub>CoM</sub> + (1/ω₀) · v<sub>CoM</sub>
                    </div>
                    <p>where ξ is the capture point position, x<sub>CoM</sub> is the center of mass position, v<sub>CoM</sub> is the CoM velocity, and ω₀ = √(g/z<sub>CoM</sub>) is the natural frequency of the inverted pendulum.</p>

                    <p><strong>Stepping Strategy:</strong></p>
                    <p>To maintain stability, the next foot placement x<sub>step</sub> should be positioned relative to the capture point:</p>
                    <div class="equation">
                        x<sub>step</sub> = ξ + k<sub>step</sub> · v<sub>desired</sub>
                    </div>
                    <p>where k<sub>step</sub> is a gain parameter that controls walking speed (larger values produce longer steps and faster walking).</p>

                    <p><strong>Step Length Constraints:</strong></p>
                    <p>Physical limitations impose bounds on step length:</p>
                    <div class="equation">
                        L<sub>min</sub> ≤ ||x<sub>step</sub> - x<sub>stance</sub>|| ≤ L<sub>max</sub>
                    </div>
                    <p>where L<sub>min</sub> prevents too-short steps (typically 10 cm) and L<sub>max</sub> respects kinematic limits (typically 40-60 cm for human-sized robots).</p>
                </div>
            </div>
        </section>

        <section class="content-section theory-section">
            <h2 class="section-title">Part 3: ZMP-Based Walking Control</h2>

            <h3>3.1 ZMP Criterion for Walking Stability</h3>
            <p>As established in HUM-106, the Zero Moment Point (ZMP) must remain within the support polygon for stable walking. During locomotion, the ZMP continuously shifts as weight transfers between feet, requiring predictive control to maintain stability.</p>

            <div class="theory-block">
                <h4>ZMP Trajectory Planning</h4>
                <p>Rather than controlling the ZMP reactively, modern walking controllers plan a desired ZMP trajectory that smoothly transitions between feet during the gait cycle.</p>

                <div class="equation-block">
                    <p><strong>Desired ZMP Trajectory:</strong></p>
                    <p>During a step from left foot to right foot, the ZMP should transition smoothly:</p>
                    <div class="equation">
                        p<sub>ZMP,desired</sub>(t) = {
                            p<sub>left</sub>,                          if t ∈ [0, t<sub>1</sub>)
                            interpolate(p<sub>left</sub>, p<sub>right</sub>, σ(t)),  if t ∈ [t<sub>1</sub>, t<sub>2</sub>]
                            p<sub>right</sub>,                         if t ∈ (t<sub>2</sub>, T]
                        }
                    </div>
                    <p>where σ(t) is a smooth interpolation function (often a sigmoid or polynomial blend) and [t<sub>1</sub>, t<sub>2</sub>] represents the double support phase.</p>

                    <p><strong>Preview Control:</strong></p>
                    <p>To track the desired ZMP, preview control uses future trajectory information to optimize current control actions. The control objective minimizes:</p>
                    <div class="equation">
                        J = ∑<sub>k=0</sub><sup>N</sup> [Q<sub>e</sub>||e<sub>ZMP</sub>(k)||² + Q<sub>x</sub>||x<sub>CoM</sub>(k)||² + R||u(k)||²]
                    </div>
                    <p>where e<sub>ZMP</sub> is the ZMP tracking error, x<sub>CoM</sub> is the CoM deviation from the reference, u is the control input (CoM acceleration), and N is the preview horizon (typically 1-2 seconds).</p>
                </div>
            </div>

            <h3>3.2 Cart-Table Model for Gait Generation</h3>
            <p>The cart-table model simplifies the robot dynamics to a point mass (the CoM) connected to a massless support (the stance leg). This abstraction enables analytical solutions for CoM trajectory generation.</p>

            <div class="theory-block">
                <h4>Linear Inverted Pendulum Mode (LIPM)</h4>
                <div class="equation-block">
                    <p>Assuming constant CoM height z<sub>CoM</sub> = h, the horizontal dynamics decouple into independent x and y equations:</p>
                    <div class="equation">
                        ẍ<sub>CoM</sub> = (g/h)(x<sub>CoM</sub> - p<sub>ZMP,x</sub>)
                    </div>
                    <div class="equation">
                        ÿ<sub>CoM</sub> = (g/h)(y<sub>CoM</sub> - p<sub>ZMP,y</sub>)
                    </div>

                    <p><strong>Analytical Solution:</strong></p>
                    <p>For a constant ZMP position p<sub>ZMP</sub> during single support, the CoM trajectory is:</p>
                    <div class="equation">
                        x<sub>CoM</sub>(t) = C<sub>1</sub>e<sup>ω₀t</sup> + C<sub>2</sub>e<sup>-ω₀t</sup> + p<sub>ZMP</sub>
                    </div>
                    <p>where ω₀ = √(g/h) and constants C<sub>1</sub>, C<sub>2</sub> are determined by initial conditions.</p>

                    <p><strong>Boundary Value Problem:</strong></p>
                    <p>To generate a CoM trajectory that connects initial state (x₀, v₀) to final state (x<sub>f</sub>, v<sub>f</sub>) in time T with ZMP at p<sub>ZMP</sub>:</p>
                    <div class="equation">
                        C<sub>1</sub> = [(v<sub>f</sub> - ω₀(x<sub>f</sub> - p<sub>ZMP</sub>))e<sup>-ω₀T</sup> - (v₀ - ω₀(x₀ - p<sub>ZMP</sub>))] / [2ω₀sinh(ω₀T)]
                    </div>
                    <div class="equation">
                        C<sub>2</sub> = (x₀ - p<sub>ZMP</sub>) - C<sub>1</sub>
                    </div>
                </div>
            </div>

            <h3>3.3 Model Predictive Control for Walking</h3>
            <p>Model Predictive Control (MPC) extends preview control by repeatedly solving an optimization problem over a receding horizon, allowing adaptation to disturbances and changing objectives.</p>

            <div class="theory-block">
                <h4>MPC Formulation for Bipedal Walking</h4>
                <div class="equation-block">
                    <p><strong>State-Space Model:</strong></p>
                    <p>Discretizing the LIPM dynamics with sampling time Δt:</p>
                    <div class="equation">
                        x(k+1) = A·x(k) + B·u(k)
                    </div>
                    <p>where state x = [x<sub>CoM</sub>, ẋ<sub>CoM</sub>]ᵀ, control input u = p<sub>ZMP</sub>, and:</p>
                    <div class="equation">
                        A = [1, Δt; ω₀²Δt, 1],  B = [-ω₀²Δt; -ω₀²Δt]
                    </div>

                    <p><strong>Optimization Problem:</strong></p>
                    <p>At each control cycle, solve for the optimal ZMP trajectory {u(k), ..., u(k+N)}:</p>
                    <div class="equation">
                        minimize ∑<sub>i=0</sub><sup>N</sup> ||p<sub>ZMP</sub>(k+i) - p<sub>ZMP,ref</sub>(k+i)||² + α||x(k+i) - x<sub>ref</sub>(k+i)||²
                    </div>
                    <p>subject to:</p>
                    <div class="equation">
                        p<sub>ZMP</sub>(k+i) ∈ S<sub>support</sub>(k+i),  for all i ∈ [0, N]
                    </div>
                    <p>where S<sub>support</sub>(k+i) is the predicted support polygon at future time steps.</p>

                    <p>Only the first control action u(k) is applied, and the optimization repeats at the next time step with updated state measurements, providing robust feedback control.</p>
                </div>
            </div>

            <div class="quiz-block">
                <h4>Comprehension Check: ZMP-Based Walking</h4>
                <div class="quiz-question">
                    <p><strong>Question 3:</strong> In the Linear Inverted Pendulum Model, if the CoM height is h = 0.5 m and g = 9.81 m/s², what is the natural frequency ω₀?</p>
                    <div class="quiz-options">
                        <label><input type="radio" name="q3" value="a"> A) 3.14 rad/s</label>
                        <label><input type="radio" name="q3" value="b"> B) 4.43 rad/s (correct)</label>
                        <label><input type="radio" name="q3" value="c"> C) 9.81 rad/s</label>
                        <label><input type="radio" name="q3" value="d"> D) 19.62 rad/s</label>
                    </div>
                    <div class="quiz-explanation">
                        <p><strong>Explanation:</strong> ω₀ = √(g/h) = √(9.81/0.5) = √19.62 ≈ 4.43 rad/s. This natural frequency determines the time constant for CoM dynamics and influences gait timing choices.</p>
                    </div>
                </div>

                <div class="quiz-question">
                    <p><strong>Question 4:</strong> What is the primary advantage of Model Predictive Control over simple feedback control for walking?</p>
                    <div class="quiz-options">
                        <label><input type="radio" name="q4" value="a"> A) Lower computational cost</label>
                        <label><input type="radio" name="q4" value="b"> B) Ability to anticipate future constraints and optimize over a horizon (correct)</label>
                        <label><input type="radio" name="q4" value="c"> C) Simpler mathematical formulation</label>
                        <label><input type="radio" name="q4" value="d"> D) No requirement for a dynamics model</label>
                    </div>
                    <div class="quiz-explanation">
                        <p><strong>Explanation:</strong> MPC solves an optimization problem over a future time horizon, allowing it to anticipate upcoming constraints (like changes in support polygon during foot transitions) and plan control actions accordingly. This preview capability is essential for stable bipedal walking, where purely reactive control often arrives too late to prevent falls.</p>
                    </div>
                </div>
            </div>
        </section>

        <section class="content-section theory-section">
            <h2 class="section-title">Part 4: Gait Timing and Step Frequency</h2>

            <h3>4.1 Natural Gait Frequency</h3>
            <p>Every mechanical system has a natural oscillation frequency at which it moves most efficiently. For bipedal systems, this natural frequency depends on leg length and gravitational acceleration, following principles similar to a physical pendulum.</p>

            <div class="theory-block">
                <h4>Pendulum-Based Gait Timing</h4>
                <div class="equation-block">
                    <p>Modeling the swing leg as a compound pendulum with length L and radius of gyration k:</p>
                    <div class="equation">
                        T<sub>natural</sub> = 2π√(k²/(gL))
                    </div>
                    <p>For a uniform leg (k ≈ L/√3), this simplifies to:</p>
                    <div class="equation">
                        T<sub>natural</sub> ≈ 2π√(L/(3g))
                    </div>

                    <p>For a human-sized robot with L = 0.8 m:</p>
                    <div class="equation">
                        T<sub>natural</sub> ≈ 1.04 seconds
                    </div>

                    <p>This theoretical prediction aligns well with observed human walking periods of 1.0-1.2 seconds, suggesting that natural dynamics play a significant role in efficient gait.</p>
                </div>

                <h4>Energy Efficiency and Gait Frequency</h4>
                <p>Operating near the natural frequency minimizes energy consumption because the system requires minimal actuation to sustain oscillation. Deviating from this frequency requires active torque to either accelerate (higher frequency) or decelerate (lower frequency) the natural swing dynamics.</p>

                <div class="equation-block">
                    <p><strong>Energy Cost of Transport:</strong></p>
                    <p>The dimensionless energy cost of transport C quantifies efficiency:</p>
                    <div class="equation">
                        C = E / (mgd)
                    </div>
                    <p>where E is energy consumed, m is robot mass, g is gravitational acceleration, and d is distance traveled. Human walking achieves C ≈ 0.2 at preferred speeds, increasing significantly at very slow or very fast walking speeds due to departure from natural dynamics.</p>
                </div>
            </div>

            <h3>4.2 Adaptive Step Timing</h3>
            <p>While natural frequency provides a baseline, practical walking requires adjusting step timing in response to terrain, obstacles, and desired speed variations. Adaptive timing algorithms modulate the gait period while maintaining stability.</p>

            <div class="theory-block">
                <h4>Phase-Based Gait Modulation</h4>
                <div class="equation-block">
                    <p>The gait phase φ can be controlled via a nonlinear oscillator:</p>
                    <div class="equation">
                        dφ/dt = ω<sub>base</sub> + k<sub>adapt</sub> · f(state)
                    </div>
                    <p>where ω<sub>base</sub> = 2π/T<sub>natural</sub> is the baseline frequency, and f(state) adjusts the rate based on feedback:</p>
                    <ul>
                        <li>Speed up swing phase if ZMP approaches stability boundary</li>
                        <li>Slow down swing phase if large terrain irregularity detected ahead</li>
                        <li>Match phase to desired walking speed v<sub>desired</sub></li>
                    </ul>

                    <p><strong>Velocity-Based Period Adjustment:</strong></p>
                    <p>To achieve a desired forward velocity v<sub>target</sub> with step length s:</p>
                    <div class="equation">
                        T<sub>gait</sub> = 2s / v<sub>target</sub>
                    </div>
                    <p>This period can be constrained to safe bounds around the natural frequency:</p>
                    <div class="equation">
                        T<sub>min</sub> ≤ T<sub>gait</sub> ≤ T<sub>max</sub>
                    </div>
                    <p>with typical values T<sub>min</sub> = 0.6T<sub>natural</sub> and T<sub>max</sub> = 1.4T<sub>natural</sub>.</p>
                </div>
            </div>

            <h3>4.3 Phase Transition and Event-Based Triggering</h3>
            <p>Transitioning between gait phases (stance to swing, swing to stance) can be triggered either by elapsed time or by sensory events. Event-based triggering provides robustness to timing variations and unexpected disturbances.</p>

            <div class="theory-block">
                <h4>Hybrid Control: Time and Event Triggers</h4>
                <ul class="definition-list">
                    <li><strong>Time-Based Transition:</strong> Switch phase when φ(t) crosses threshold (e.g., φ = 0.6 triggers swing). Ensures predictable timing but cannot adapt to early/late ground contact.</li>
                    <li><strong>Event-Based Transition:</strong> Switch phase when sensor condition met (e.g., foot contact detected). Adapts to terrain variations but requires reliable sensor signals.</li>
                    <li><strong>Hybrid Approach:</strong> Use time-based prediction with event-based correction. Transition occurs at min(t<sub>predicted</sub>, t<sub>event</sub> + Δt<sub>max</sub>) to combine predictability with adaptability.</li>
                </ul>

                <div class="equation-block">
                    <p><strong>Transition Logic:</strong></p>
                    <div class="equation">
                        trigger<sub>swing→stance</sub> = (φ ≥ φ<sub>threshold</sub>) AND (F<sub>contact</sub> > F<sub>min</sub>)
                    </div>
                    <p>where F<sub>contact</sub> is the measured ground reaction force and F<sub>min</sub> is a threshold confirming stable contact (typically 10-20% of body weight).</p>

                    <div class="equation">
                        trigger<sub>stance→swing</sub> = (φ ≥ 0.6) OR (F<sub>contact</sub> < F<sub>liftoff</sub>)
                    </div>
                    <p>allowing for intentional liftoff or unexpected loss of contact.</p>
                </div>
            </div>
        </section>

        <section class="content-section theory-section">
            <h2 class="section-title">Part 5: Turn Control and Direction Changes</h2>

            <h3>5.1 Turning Mechanics During Walking</h3>
            <p>Changing direction while walking requires coordinating foot placement angles, lateral CoM shifts, and yaw rotation of the torso. Humans accomplish turns through a combination of step angle adjustments and angular momentum generation.</p>

            <div class="theory-block">
                <h4>Yaw Moment Generation</h4>
                <p>To rotate the body about the vertical axis, the robot must generate a yaw moment M<sub>z</sub>. This can be achieved through:</p>

                <ul class="definition-list">
                    <li><strong>Differential Foot Placement:</strong> Placing successive feet at angles to create a curved path</li>
                    <li><strong>Angular Momentum Transfer:</strong> Rotating the torso or arms to exchange angular momentum with the support leg</li>
                    <li><strong>Friction-Based Torque:</strong> Applying tangential forces at the foot-ground interface (requires sufficient friction)</li>
                </ul>

                <div class="equation-block">
                    <p><strong>Step Angle Strategy:</strong></p>
                    <p>For a desired turn rate ω<sub>yaw</sub> (rad/s), each step rotates by angle:</p>
                    <div class="equation">
                        Δθ<sub>step</sub> = ω<sub>yaw</sub> · T<sub>step</sub>
                    </div>
                    <p>where T<sub>step</sub> is the time per step (half the gait period). The foot placement is then:</p>
                    <div class="equation">
                        x<sub>step</sub> = x<sub>nominal</sub> cos(Δθ<sub>step</sub>) - y<sub>nominal</sub> sin(Δθ<sub>step</sub>)
                        y<sub>step</sub> = x<sub>nominal</sub> sin(Δθ<sub>step</sub>) + y<sub>nominal</sub> cos(Δθ<sub>step</sub>)
                    </div>

                    <p><strong>Turn Rate Limits:</strong></p>
                    <p>Stability constraints limit maximum turn rate. For the LIPM model, the lateral ZMP displacement during turning must remain within the support polygon:</p>
                    <div class="equation">
                        |y<sub>ZMP</sub>| = |(ω<sub>yaw</sub>² · I<sub>z</sub>) / (m · g · z<sub>CoM</sub>)| ≤ w<sub>foot</sub>/2
                    </div>
                    <p>where I<sub>z</sub> is the yaw moment of inertia, m is robot mass, and w<sub>foot</sub> is foot width. This yields maximum turn rate:</p>
                    <div class="equation">
                        ω<sub>max</sub> = √[(m · g · z<sub>CoM</sub> · w<sub>foot</sub>) / (2 · I<sub>z</sub>)]
                    </div>
                </div>
            </div>

            <h3>5.2 Path Following and Trajectory Tracking</h3>
            <p>When following a curved path or tracking a desired trajectory, the walking controller must continuously adjust step placement to minimize position and orientation errors while maintaining balance.</p>

            <div class="theory-block">
                <h4>Closed-Loop Step Adjustment</h4>
                <div class="equation-block">
                    <p>Given a desired path trajectory p<sub>desired</sub>(t) and current robot position p<sub>robot</sub>, compute the error:</p>
                    <div class="equation">
                        e<sub>position</sub> = p<sub>desired</sub> - p<sub>robot</sub>
                        e<sub>heading</sub> = θ<sub>desired</sub> - θ<sub>robot</sub>
                    </div>

                    <p><strong>Corrective Step Placement:</strong></p>
                    <p>Modify the nominal step location to reduce error:</p>
                    <div class="equation">
                        x<sub>step</sub> = x<sub>nominal</sub> + k<sub>p</sub> · e<sub>x</sub> + k<sub>h</sub> · e<sub>heading</sub>
                        y<sub>step</sub> = y<sub>nominal</sub> + k<sub>p</sub> · e<sub>y</sub>
                    </div>
                    <p>where k<sub>p</sub> and k<sub>h</sub> are gain parameters tuned to balance tracking performance and stability. Aggressive correction (high gains) may violate ZMP constraints, while conservative correction (low gains) results in large tracking errors.</p>

                    <p><strong>Look-Ahead Distance:</strong></p>
                    <p>Rather than tracking the current desired position, a look-ahead strategy targets a point ahead on the path:</p>
                    <div class="equation">
                        p<sub>target</sub> = p<sub>desired</sub>(t + t<sub>lookahead</sub>)
                    </div>
                    <p>with t<sub>lookahead</sub> ≈ 1-2 gait periods. This provides smoother path following on curved trajectories by anticipating upcoming turns.</p>
                </div>
            </div>

            <h3>5.3 In-Place Rotation</h3>
            <p>Turning in place without forward motion requires alternating small angular steps while maintaining the CoM approximately stationary over the stance foot. This maneuver is useful for precise reorientation in constrained spaces.</p>

            <div class="theory-block">
                <h4>Spot Turn Implementation</h4>
                <div class="equation-block">
                    <p>During a spot turn, each swing foot rotates about the stance foot by angle Δθ:</p>
                    <div class="equation">
                        Δθ<sub>turn</sub> = sign(ω<sub>desired</sub>) · min(|ω<sub>desired</sub>| · T<sub>step</sub>, θ<sub>max</sub>)
                    </div>
                    <p>where θ<sub>max</sub> ≈ 15-25° is the maximum safe rotation per step, limited by hip joint range and stability margins.</p>

                    <p><strong>CoM Management:</strong></p>
                    <p>To prevent the CoM from drifting during rotation, the lateral position must be carefully controlled:</p>
                    <div class="equation">
                        y<sub>CoM</sub> = (y<sub>left</sub> + y<sub>right</sub>)/2 + k<sub>balance</sub> · (θ<sub>torso</sub> - θ<sub>target</sub>)
                    </div>
                    <p>The feedback term k<sub>balance</sub> · (θ<sub>torso</sub> - θ<sub>target</sub>) compensates for accumulated orientation errors that would otherwise cause the robot to "spiral" during the turn.</p>
                </div>
            </div>

            <div class="quiz-block">
                <h4>Comprehension Check: Gait Timing and Turning</h4>
                <div class="quiz-question">
                    <p><strong>Question 5:</strong> Why is walking near the natural gait frequency more energy-efficient?</p>
                    <div class="quiz-options">
                        <label><input type="radio" name="q5" value="a"> A) It maximizes ground contact time</label>
                        <label><input type="radio" name="q5" value="b"> B) It minimizes required actuation by exploiting natural swing dynamics (correct)</label>
                        <label><input type="radio" name="q5" value="c"> C) It reduces sensor noise</label>
                        <label><input type="radio" name="q5" value="d"> D) It eliminates the need for feedback control</label>
                    </div>
                    <div class="quiz-explanation">
                        <p><strong>Explanation:</strong> Operating near the natural frequency allows the leg to swing like a pendulum with minimal actuator effort. Deviating from this frequency requires active torque to either speed up or slow down the natural dynamics, consuming additional energy. This principle is analogous to pushing a child on a swing—timing pushes to match the natural frequency maximizes efficiency.</p>
                    </div>
                </div>

                <div class="quiz-question">
                    <p><strong>Question 6:</strong> What is the primary challenge when executing sharp turns during walking?</p>
                    <div class="quiz-options">
                        <label><input type="radio" name="q6" value="a"> A) Maintaining the ZMP within the support polygon during yaw rotation (correct)</label>
                        <label><input type="radio" name="q6" value="b"> B) Preventing the motors from overheating</label>
                        <label><input type="radio" name="q6" value="c"> C) Avoiding joint singularities</label>
                        <label><input type="radio" name="q6" value="d"> D) Keeping the CoM height constant</label>
                    </div>
                    <div class="quiz-explanation">
                        <p><strong>Explanation:</strong> Sharp turns generate yaw moments that create lateral forces, potentially pushing the ZMP outside the support polygon. The turn rate must be limited to ensure these lateral forces remain within stability bounds. This is why robots (and humans) cannot turn arbitrarily fast while walking—physics imposes a fundamental stability constraint.</p>
                    </div>
                </div>
            </div>
        </section>

        <section class="content-section project-section">
            <h2 class="section-title">Guided Project: Implementing a Complete Walking Controller</h2>

            <div class="project-overview">
                <h3>Project Objective</h3>
                <p>Develop a comprehensive bipedal walking controller that integrates foot trajectory planning, ZMP-based stability control, adaptive gait timing, and turning capabilities. This project synthesizes all theoretical concepts into a working locomotion system.</p>

                <h4>Implementation Roadmap</h4>
                <ol class="roadmap-list">
                    <li>Implement foot trajectory generators using polynomial and cycloid profiles</li>
                    <li>Design a ZMP preview controller for CoM trajectory generation</li>
                    <li>Create a gait state machine with phase transitions</li>
                    <li>Develop step placement logic with turning support</li>
                    <li>Integrate all components and simulate walking with visualization</li>
                    <li>Test turning, speed changes, and disturbance recovery</li>
                </ol>
            </div>

            <h3>Step 1: Foot Trajectory Generator</h3>
            <p>We begin by implementing a trajectory generator that produces smooth swing foot paths with configurable step length, clearance height, and timing.</p>

            <div class="code-block">
                <div class="code-header">
                    <span class="code-language">Python</span>
                    <span class="code-filename">foot_trajectory.py</span>
                </div>
                <pre><code>import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List

class FootTrajectoryGenerator:
    """
    Generates smooth foot trajectories for swing phase using
    fifth-order polynomial for horizontal motion and cycloid
    profile for vertical clearance.
    """

    def __init__(self,
                 clearance_height: float = 0.05,
                 swing_duration: float = 0.4):
        """
        Initialize trajectory generator.

        Args:
            clearance_height: Maximum foot height during swing (m)
            swing_duration: Time for swing phase (s)
        """
        self.h_clear = clearance_height
        self.T_swing = swing_duration

    def quintic_polynomial(self,
                          tau: float,
                          p_start: float,
                          p_end: float) -> Tuple[float, float, float]:
        """
        Compute position, velocity, acceleration using 5th order polynomial.

        Boundary conditions:
            p(0) = p_start, p'(0) = 0, p''(0) = 0
            p(1) = p_end,   p'(1) = 0, p''(1) = 0

        Args:
            tau: Normalized time parameter [0, 1]
            p_start: Starting position
            p_end: Ending position

        Returns:
            (position, velocity, acceleration)
        """
        # Polynomial coefficients for smooth trajectory
        # Derived from boundary conditions
        delta_p = p_end - p_start

        # Position: p(τ) = a0 + a1*τ + ... + a5*τ^5
        a0 = p_start
        a3 = 10 * delta_p
        a4 = -15 * delta_p
        a5 = 6 * delta_p

        tau2 = tau * tau
        tau3 = tau2 * tau
        tau4 = tau3 * tau
        tau5 = tau4 * tau

        p = a0 + a3*tau3 + a4*tau4 + a5*tau5

        # Velocity: v(τ) = dp/dτ * dτ/dt = (dp/dτ) / T_swing
        dp_dtau = 3*a3*tau2 + 4*a4*tau3 + 5*a5*tau4
        v = dp_dtau / self.T_swing

        # Acceleration: a(τ) = d²p/dτ² * (dτ/dt)²
        d2p_dtau2 = 6*a3*tau + 12*a4*tau2 + 20*a5*tau3
        a = d2p_dtau2 / (self.T_swing**2)

        return p, v, a

    def cycloid_vertical(self, tau: float) -> Tuple[float, float, float]:
        """
        Compute vertical position, velocity, acceleration using cycloid profile.

        z(τ) = h_clear * sin(πτ)

        This provides smooth ground clearance with zero vertical velocity
        at both endpoints (τ=0 and τ=1).

        Args:
            tau: Normalized time parameter [0, 1]

        Returns:
            (height, vertical_velocity, vertical_acceleration)
        """
        z = self.h_clear * np.sin(np.pi * tau)

        # dz/dτ = h_clear * π * cos(πτ)
        dz_dtau = self.h_clear * np.pi * np.cos(np.pi * tau)
        vz = dz_dtau / self.T_swing

        # d²z/dτ² = -h_clear * π² * sin(πτ)
        d2z_dtau2 = -self.h_clear * np.pi**2 * np.sin(np.pi * tau)
        az = d2z_dtau2 / (self.T_swing**2)

        return z, vz, az

    def generate_trajectory(self,
                           p_start: np.ndarray,
                           p_end: np.ndarray,
                           num_points: int = 100) -> dict:
        """
        Generate complete 3D foot trajectory from start to end position.

        Args:
            p_start: Starting foot position [x, y, z] (m)
            p_end: Ending foot position [x, y, z] (m)
            num_points: Number of trajectory samples

        Returns:
            Dictionary containing:
                - 'time': Time array
                - 'position': Position array (num_points x 3)
                - 'velocity': Velocity array (num_points x 3)
                - 'acceleration': Acceleration array (num_points x 3)
        """
        tau = np.linspace(0, 1, num_points)
        time = tau * self.T_swing

        position = np.zeros((num_points, 3))
        velocity = np.zeros((num_points, 3))
        acceleration = np.zeros((num_points, 3))

        for i, t in enumerate(tau):
            # X-Y motion: quintic polynomial
            px, vx, ax = self.quintic_polynomial(t, p_start[0], p_end[0])
            py, vy, ay = self.quintic_polynomial(t, p_start[1], p_end[1])

            # Z motion: cycloid + baseline height
            pz, vz, az = self.cycloid_vertical(t)
            pz += p_start[2]  # Add ground level offset

            position[i] = [px, py, pz]
            velocity[i] = [vx, vy, vz]
            acceleration[i] = [ax, ay, az]

        return {
            'time': time,
            'position': position,
            'velocity': velocity,
            'acceleration': acceleration
        }

    def visualize_trajectory(self, trajectory: dict):
        """
        Plot the generated trajectory in 3D and show velocity profiles.
        """
        fig = plt.figure(figsize=(15, 10))

        # 3D trajectory plot
        ax1 = fig.add_subplot(2, 3, 1, projection='3d')
        pos = trajectory['position']
        ax1.plot(pos[:, 0], pos[:, 1], pos[:, 2], 'b-', linewidth=2)
        ax1.scatter(pos[0, 0], pos[0, 1], pos[0, 2],
                   c='green', s=100, marker='o', label='Start')
        ax1.scatter(pos[-1, 0], pos[-1, 1], pos[-1, 2],
                   c='red', s=100, marker='s', label='End')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('3D Foot Trajectory')
        ax1.legend()
        ax1.grid(True)

        # Position components
        ax2 = fig.add_subplot(2, 3, 2)
        t = trajectory['time']
        ax2.plot(t, pos[:, 0], 'r-', label='X', linewidth=2)
        ax2.plot(t, pos[:, 1], 'g-', label='Y', linewidth=2)
        ax2.plot(t, pos[:, 2], 'b-', label='Z', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Position (m)')
        ax2.set_title('Position Components')
        ax2.legend()
        ax2.grid(True)

        # Velocity components
        ax3 = fig.add_subplot(2, 3, 3)
        vel = trajectory['velocity']
        ax3.plot(t, vel[:, 0], 'r-', label='Vx', linewidth=2)
        ax3.plot(t, vel[:, 1], 'g-', label='Vy', linewidth=2)
        ax3.plot(t, vel[:, 2], 'b-', label='Vz', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Velocity (m/s)')
        ax3.set_title('Velocity Components')
        ax3.legend()
        ax3.grid(True)

        # Acceleration components
        ax4 = fig.add_subplot(2, 3, 4)
        acc = trajectory['acceleration']
        ax4.plot(t, acc[:, 0], 'r-', label='Ax', linewidth=2)
        ax4.plot(t, acc[:, 1], 'g-', label='Ay', linewidth=2)
        ax4.plot(t, acc[:, 2], 'b-', label='Az', linewidth=2)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Acceleration (m/s²)')
        ax4.set_title('Acceleration Components')
        ax4.legend()
        ax4.grid(True)

        # Velocity magnitude
        ax5 = fig.add_subplot(2, 3, 5)
        vel_mag = np.linalg.norm(vel, axis=1)
        ax5.plot(t, vel_mag, 'purple', linewidth=2)
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Speed (m/s)')
        ax5.set_title('Velocity Magnitude')
        ax5.grid(True)

        # Acceleration magnitude
        ax6 = fig.add_subplot(2, 3, 6)
        acc_mag = np.linalg.norm(acc, axis=1)
        ax6.plot(t, acc_mag, 'orange', linewidth=2)
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Acceleration (m/s²)')
        ax6.set_title('Acceleration Magnitude')
        ax6.grid(True)

        plt.tight_layout()
        plt.savefig('foot_trajectory_analysis.png', dpi=300, bbox_inches='tight')
        plt.show()


# Demonstration and testing
if __name__ == "__main__":
    print("=== Foot Trajectory Generator Demo ===\n")

    # Initialize generator
    generator = FootTrajectoryGenerator(
        clearance_height=0.05,  # 5 cm clearance
        swing_duration=0.4      # 400 ms swing time
    )

    # Define a forward step
    p_start = np.array([0.0, 0.1, 0.0])   # Start at right foot position
    p_end = np.array([0.3, 0.1, 0.0])     # Step forward 30 cm

    print(f"Start position: {p_start}")
    print(f"End position: {p_end}")
    print(f"Swing duration: {generator.T_swing} s")
    print(f"Clearance height: {generator.h_clear} m\n")

    # Generate trajectory
    trajectory = generator.generate_trajectory(p_start, p_end, num_points=100)

    # Verify boundary conditions
    print("=== Boundary Condition Verification ===")
    print(f"Start position: {trajectory['position'][0]}")
    print(f"Start velocity: {trajectory['velocity'][0]} (should be ~zero)")
    print(f"End position: {trajectory['position'][-1]}")
    print(f"End velocity: {trajectory['velocity'][-1]} (should be ~zero)\n")

    # Find maximum clearance
    max_height_idx = np.argmax(trajectory['position'][:, 2])
    max_height = trajectory['position'][max_height_idx, 2]
    max_height_time = trajectory['time'][max_height_idx]

    print(f"Maximum clearance: {max_height:.4f} m at t={max_height_time:.3f} s")
    print(f"Expected clearance: {generator.h_clear:.4f} m")
    print(f"Mid-swing time: {generator.T_swing/2:.3f} s\n")

    # Visualize
    generator.visualize_trajectory(trajectory)

    print("Trajectory visualization saved as 'foot_trajectory_analysis.png'")
</code></pre>
            </div>

            <div class="explanation-block">
                <h4>Code Explanation: Trajectory Mathematics</h4>
                <p>The <code>quintic_polynomial</code> method implements a fifth-order polynomial that satisfies six boundary conditions (position, velocity, and acceleration at both endpoints). This high-order polynomial ensures smooth motion without velocity or acceleration discontinuities that could excite mechanical vibrations.</p>
                <p>The <code>cycloid_vertical</code> method uses a sinusoidal profile for vertical motion. This choice provides natural-looking ground clearance while guaranteeing zero vertical velocity at takeoff and landing, minimizing impact forces.</p>
                <p>The normalization parameter τ ∈ [0, 1] separates the trajectory shape from its timing, allowing easy adjustment of swing duration without recomputing polynomial coefficients.</p>
            </div>

            <div class="project-checklist">
                <h4>Project Completion Checklist</h4>
                <ul class="checklist">
                    <li><input type="checkbox"> Foot trajectory generator produces smooth paths with zero endpoint velocities</li>
                    <li><input type="checkbox"> ZMP preview controller tracks references with low error (&lt; 1 cm RMS)</li>
                    <li><input type="checkbox"> Gait state machine transitions correctly through all phases</li>
                    <li><input type="checkbox"> Step placement planner generates stable footstep sequences</li>
                    <li><input type="checkbox"> Integrated controller executes 6+ consecutive steps</li>
                    <li><input type="checkbox"> Turning control maintains stability during direction changes</li>
                    <li><input type="checkbox"> Speed adjustment modifies gait period appropriately</li>
                    <li><input type="checkbox"> Visualizations clearly show CoM, ZMP, and foot trajectories</li>
                </ul>
            </div>

            <div class="challenge-block">
                <h4>Extension Challenges</h4>
                <ol>
                    <li><strong>Terrain Adaptation:</strong> Modify the foot trajectory generator to handle uneven ground by adjusting the target z-position based on terrain height maps.</li>
                    <li><strong>Push Recovery:</strong> Implement a disturbance response that adjusts step placement when external forces push the robot (hint: use capture point deviation as an error signal).</li>
                    <li><strong>Energy Optimization:</strong> Add a cost term to the preview controller that penalizes CoM acceleration, encouraging energy-efficient gaits that exploit natural dynamics.</li>
                    <li><strong>Stair Climbing:</strong> Extend the step planner to generate foot placements on stairs, with appropriate vertical clearance and landing accuracy.</li>
                </ol>
            </div>
        </section>

        <section class="content-section summary-section">
            <h2 class="section-title">Course Summary</h2>
            <div class="summary-content">
                <p>This course has provided a comprehensive foundation in bipedal walking control, covering the theoretical principles, mathematical frameworks, and practical implementation strategies essential for stable humanoid locomotion. Key achievements include:</p>

                <div class="key-takeaways">
                    <h3>Theoretical Mastery</h3>
                    <ul>
                        <li>Understanding of the gait cycle phases and biomechanical constraints</li>
                        <li>Mathematical formulation of the Linear Inverted Pendulum Model</li>
                        <li>ZMP-based stability criteria for dynamic walking</li>
                        <li>Preview control theory for trajectory optimization</li>
                        <li>Capture point framework for step placement</li>
                    </ul>

                    <h3>Practical Skills</h3>
                    <ul>
                        <li>Polynomial and cycloid trajectory generation for smooth foot paths</li>
                        <li>Implementation of discrete-time preview controllers</li>
                        <li>Design of hybrid state machines with time and event triggers</li>
                        <li>Integration of multiple control components into a unified system</li>
                        <li>Simulation and visualization of walking motions</li>
                    </ul>

                    <h3>Engineering Insights</h3>
                    <ul>
                        <li>Trade-offs between stability margins and walking efficiency</li>
                        <li>Importance of preview (looking ahead) for dynamic balance</li>
                        <li>Role of natural dynamics in energy-efficient gaits</li>
                        <li>Hierarchical decomposition for managing system complexity</li>
                    </ul>
                </div>

                <div class="next-steps">
                    <h3>Looking Ahead: HUM-108</h3>
                    <p>In the next course, we advance to whole-body manipulation, where the robot must coordinate arm and leg motions simultaneously. You will learn inverse kinematics for redundant systems, grasp planning algorithms, and human-robot handover protocols. The walking controllers developed here will serve as the foundation, with manipulation tasks adding additional constraints and objectives to the optimization problem.</p>
                </div>
            </div>
        </section>

        <section class="content-section resources-section">
            <h2 class="section-title">Additional Resources</h2>

            <div class="resources-grid">
                <div class="resource-category">
                    <h3>Foundational Papers</h3>
                    <ul class="resource-list">
                        <li>Vukobratović, M., & Borovac, B. (2004). "Zero-moment point—thirty five years of its life." <em>International Journal of Humanoid Robotics</em>, 1(1), 157-173.</li>
                        <li>Kajita, S., et al. (2003). "Biped walking pattern generation by using preview control of zero-moment point." <em>Proceedings of ICRA</em>.</li>
                        <li>Pratt, J., & Tedrake, R. (2006). "Velocity-based stability margins for fast bipedal walking." <em>Fast Motions in Biomechanics and Robotics</em>, 299-324.</li>
                    </ul>
                </div>

                <div class="resource-category">
                    <h3>Advanced Topics</h3>
                    <ul class="resource-list">
                        <li>Wieber, P. B. (2006). "Trajectory free linear model predictive control for stable walking in the presence of strong perturbations." <em>Humanoids</em>.</li>
                        <li>Englsberger, J., et al. (2015). "Three-dimensional bipedal walking control based on divergent component of motion." <em>IEEE Transactions on Robotics</em>, 31(2), 355-368.</li>
                        <li>Ames, A. D., et al. (2014). "Rapidly exponentially stabilizing control Lyapunov functions and hybrid zero dynamics." <em>IEEE TAC</em>, 59(4), 876-891.</li>
                    </ul>
                </div>

                <div class="resource-category">
                    <h3>Software Tools</h3>
                    <ul class="resource-list">
                        <li><strong>Drake:</strong> Model-based design and verification for robotics (drake.mit.edu)</li>
                        <li><strong>Pinocchio:</strong> Fast rigid body dynamics library for Python (stack-of-tasks.github.io/pinocchio)</li>
                        <li><strong>Bipedal Locomotion Framework:</strong> Walking controllers for humanoid robots (github.com/ami-iit/bipedal-locomotion-framework)</li>
                    </ul>
                </div>

                <div class="resource-category">
                    <h3>Textbooks</h3>
                    <ul class="resource-list">
                        <li>Kajita, S., & Espiau, B. (2008). <em>Legged Robots</em>. Springer Handbook of Robotics.</li>
                        <li>Siciliano, B., et al. (2016). <em>Springer Handbook of Robotics</em> (2nd ed.). Springer.</li>
                        <li>Westervelt, E. R., et al. (2007). <em>Feedback Control of Dynamic Bipedal Robot Locomotion</em>. CRC Press.</li>
                    </ul>
                </div>
            </div>
        </section>
    </main>

    <footer class="course-footer">
        <div class="footer-content">
            <div class="footer-navigation">
                <a href="hum-106-balance.md" class="footer-link">← Previous: HUM-106 Balance and Stability</a>
                <a href="hum-108-manipulation.md" class="footer-link">Next: HUM-108 Manipulation →</a>
            </div>
            <div class="footer-info">
                <p>HUM-107: Bipedal Walking and Gait Control | Version 2.0.0</p>
                <p>Humanoid Robot Course Series | Advanced Level</p>
                <p>Last Updated: 2025-11-09</p>
            </div>
        </div>
    </footer>
</body>
</html>
