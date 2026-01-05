<!--
author: Robot Campus Curriculum Team
email: academic@robotcampus.io
version: 2.0.0
language: en
narrator: US English Female

comment: Comprehensive guide to LiaScript for Robot Campus curriculum development - academic textbook format with theory-practice integration

link: https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/css/bootstrap.min.css
link: https://raw.githubusercontent.com/robot-campus/robot-courses/main/assets/styles/academic.css

mode: Textbook
-->

# LiaScript Reference Guide for Robot Campus Curriculum

> Comprehensive reference for creating interactive educational content in academic format

## Introduction

This guide provides comprehensive documentation for using LiaScript in Robot Campus curriculum development. LiaScript is an extended Markdown format enabling creation of interactive educational materials with embedded quizzes, executable code, animations, and multimedia content.

**Version 2.0.0 Standards**: All curriculum content should follow academic textbook format with appropriate theory-practice balance (60/40 for design/hardware, 70/30 for software/AI/systems). This guide demonstrates LiaScript features suitable for rigorous academic content.

---

## Document Structure

### Headers

Headers create hierarchical document structure:

``` markdown
# Level 1 Header - Course Title
## Level 2 Header - Major Section
### Level 3 Header - Subsection
#### Level 4 Header - Topic
##### Level 5 Header - Subtopic
###### Level 6 Header - Detail Level
```

**Academic Usage**: Use consistent header hierarchy to organize theoretical content, methodologies, and applications.

---

## Text Formatting

### Basic Text Emphasis

``` markdown
_italic text_ or *italic text*
__bold text__ or **bold text**
___bold italic___ or ***bold italic***

~strikethrough~
~~underlined~~
~~~strikethrough and underlined~~~

^superscript^

`inline code or technical terms`
```

**Academic Usage**: Use bold for key terms, italic for emphasis, inline code for technical terminology and commands.

### Block Quotations

``` markdown
Simple paragraphs are separated by blank lines.

> Block quotations are created with leading `>`
> characters. Useful for definitions, theorems,
> or important concepts.

Multiple paragraphs in quotations require
blank line separation.
```

**Academic Usage**: Use quotations for formal definitions, theorem statements, or notable observations.

---

## Lists

### Unordered Lists

``` markdown
* List items with `*`, `+`, or `-`

  Lists can contain multiple paragraphs
  with proper indentation.

* Nested lists:
  - Level 2 item
  - Level 2 item
    + Level 3 item
    + Level 3 item
```

### Ordered Lists

``` markdown
1. Ordered lists use numbers
   with proper indentation for
   continuation paragraphs.

2. Second item with nested content:

   a. Sub-item A
   b. Sub-item B
      + Further nesting
```

**Academic Usage**: Use ordered lists for sequential procedures, algorithms, or step-by-step derivations. Use unordered lists for properties, characteristics, or non-sequential collections.

---

## Horizontal Rules

``` markdown
Text above horizontal rule.

---

Text below horizontal rule.
Minimum three consecutive hyphens required.

--------------------------------------
Longer rules are equivalent.
```

**Academic Usage**: Separate major sections, distinguish examples from theory, or mark transitions between topics.

---

## References and Links

### Hyperlinks

``` markdown
* Plain URL: https://LiaScript.github.io

* Formatted link:
  - External: [LiaScript Documentation](https://liascript.github.io/)
  - With title: [LiaScript](https://liascript.github.io/ "Interactive Learning")

  - Internal section: [Text Formatting](#text-formatting)
  - With title: [Formatting](#text-formatting "Go to formatting section")
  - By slide number: [Slide 5](#5)
  - With title: [Fifth Slide](#5 "Jump to slide 5")
```

### Images

``` markdown
External images:

![Alternative text](https://example.com/image.jpg)
![Alternative text](https://example.com/image.jpg "Image title")

Project-internal images:

![Diagram](../assets/images/diagram.png)
![Diagram](../assets/images/diagram.png "System architecture")
```

**Academic Usage**: Include diagrams, plots, and figures with descriptive alternative text. Always provide figure captions and references in text.

### Audio Content

``` markdown
External audio:

?[Audio description](https://example.com/audio.mp3)
?[Lecture recording](https://example.com/lecture.mp3 "Introduction to kinematics")

Project-internal audio:

?[Pronunciation guide](../assets/audio/pronunciation.mp3)
```

**Academic Usage**: Provide pronunciation guides for technical terms, supplementary lectures, or accessibility features.

### Video Content

``` markdown
External video:

!?[Video description](https://youtube.com/watch?v=example)
!?[Demonstration](https://example.com/video.mp4 "Robot assembly demonstration")

Project-internal video:

!?[Laboratory procedure](../assets/video/lab_procedure.mp4)
```

**Academic Usage**: Embed demonstrations, laboratory procedures, or supplementary lectures.

---

## Mathematical Notation

LiaScript uses [KaTeX](https://katex.org/docs/supported.html) for mathematical typesetting.

### Inline Mathematics

``` markdown
Inline equations: The kinematic equation $ f(a,b,c) = (a^2+b^2+c^2)^3 $ appears in text.
```

### Display Mathematics

``` markdown
Block equations:

$$
\sum_{i=1}^\infty\frac{1}{n^2} = \frac{\pi^2}{6}
$$

Multiple equations:

$$
\begin{aligned}
F &= ma \\
\tau &= I\alpha \\
E &= mc^2
\end{aligned}
$$
```

**Academic Usage**: Use mathematical notation for all equations, derivations, and formal expressions. Number important equations for reference.

---

## Animations and Progressive Disclosure

### Block Animations

``` markdown
             {{1}}
This content appears at animation step 1.

{{2-3}} This content appears at step 2
and disappears at step 3.

             {{4}}
Final content appears at step 4.
```

### Inline Animations

``` markdown
             {{1}}
Theoretical concepts may include {2-3}{_progressive_}
disclosure of {2}{complex terminology} to aid
comprehension. Animation order is controlled by {3}{numbers}.
```

### Multi-Block Animations

``` markdown
             {{1}}
*********************************
Multiple blocks can be grouped
using asterisk delimiters.

Blocks may contain {2}{inline animations}
and multiple paragraphs.

...
*********************************
```

**Academic Usage**: Use animations to reveal derivation steps, build complex diagrams progressively, or guide attention through multi-step procedures.

---

## Text-to-Speech Narration

### Basic Narration

``` markdown
          --{{1}}--
This paragraph will be read aloud with default voice
at animation step 1, supporting accessibility.

    --{{2 UK English Male}}--
This text uses specified voice and language.
```

### Hidden Narration

``` markdown
<!-- --{{1}}--
This narration is read aloud but does not
appear in textbook mode for print versions.
-->
```

### Available Voices

| Female | Male |
|--------|------|
| UK English Female | UK English Male |
| US English Female | US English Male |
| Deutsch Female | Deutsch Male |
| French Female | French Male |
| Spanish Female | Spanish Male |

**See full voice list**: [ResponsiveVoice](https://responsivevoice.org)

**Academic Usage**: Provide narration for accessibility, pronunciation guidance, or to emphasize key concepts.

---

## Interactive Quizzes

### Text Input Quiz

``` markdown
What is the primary middleware framework used in modern robotics?

[[ROS2]]
```

### Single Choice Quiz

``` markdown
Which kinematic approach solves for joint angles given end-effector position?

[( )] Forward kinematics
[(X)] Inverse kinematics
[( )] Differential kinematics
[( )] Velocity kinematics
```

### Multiple Choice Quiz

``` markdown
Select all sensors commonly used for robot localization:

[[ ]] Camera
[[X]] IMU
[[X]] Encoder
[[ ]] Temperature sensor
[[X]] Lidar
```

### Matrix Quiz

``` markdown
Classify each algorithm by its primary application:

[[Path Planning] [Localization] [Control]]
[    [X]            [ ]            [ ]     ]  A* algorithm
[    [ ]            [X]            [ ]     ]  Kalman filter
[    [ ]            [ ]            [X]     ]  PID controller
[    [X]            [ ]            [ ]     ]  RRT algorithm
```

### Selection Quiz

``` markdown
Select the correct definition:

[[ Kinematics studies motion without considering forces
|  Dynamics studies motion without considering forces
|  ( Kinematics studies motion considering forces )
|  Statics studies motion without considering forces
]]
```

### Quiz Hints

``` markdown
What is the dimension of SE(3)?

[[6]]
[[?]] Consider both rotational and translational degrees of freedom
[[?]] Rotational: 3 DOF, Translational: 3 DOF
```

### Extended Solutions

``` markdown
Calculate the determinant of the rotation matrix.

[[1]]
[[?]] Rotation matrices are orthogonal
[[?]] For orthogonal matrices, det(R) = ±1
[[?]] Proper rotation matrices have det(R) = 1
**************************************************

## Solution Explanation

Rotation matrices belong to the special orthogonal group SO(3),
which requires:

1. Orthogonality: $R^T R = I$
2. Proper rotation: $\det(R) = 1$

Therefore, all valid rotation matrices have determinant equal to 1.

$$
\det(R) = 1
$$

**************************************************
```

### Custom Validation

``` markdown
Enter the robotics middleware framework name:

[[ROS2]]
<script>
  // @input is replaced with user input
  let input_string = "@input";
  "ros2" == input_string.trim().toLowerCase();
</script>
```

### Generic Quiz

``` markdown
[[!]]
<script>
  // Custom validation logic
  Math.random() > 0.1
</script>
```

**Academic Usage**: Use quizzes to reinforce learning, check comprehension, and provide immediate feedback. Include hints and detailed solutions for self-study.

---

## Styling and CSS

### Block-Level Styling

``` markdown
<!-- class="theorem" -->
**Theorem 1**: For any rotation matrix $R \in SO(3)$,
the determinant satisfies $\det(R) = 1$.

<!-- style="color: #2c3e50; border-left: 4px solid #3498db; padding-left: 1em" -->
**Important Concept**: Forward kinematics maps joint
space to task space using the transformation chain.
```

### Inline Styling

``` markdown
The forward kinematics equation<!-- class="highlight-equation" -->
relates joint angles to end-effector pose.

![System architecture](diagram.png)<!--
class="figure"
style="width: 100%; max-width: 800px"
title="Figure 1: Robot control architecture"
-->
```

**Academic Usage**: Use semantic CSS classes (theorem, definition, example, figure) defined in external stylesheet. Avoid inline styling for maintainability.

---

## HTML Integration

``` markdown
<div class="example">
<h3>Example 1: Forward Kinematics</h3>

Standard _Markdown_ and **LiaScript** syntax
work inside HTML blocks.

| Joint | Angle | Link Length |
|-------|-------|-------------|
| 1     | θ₁    | L₁          |
| 2     | θ₂    | L₂          |

</div>

<lia-keep>
  Prevents LiaScript processing within block:

  <div>
  This **Markdown** will not be processed.
  </div>
</lia-keep>
```

**Academic Usage**: Use HTML for complex layouts, specialized formatting, or integration with external libraries. Use sparingly to maintain Markdown simplicity.

---

## ASCII Art and Diagrams

### Data Plots

``` markdown
                                   Force vs. Displacement
    10 | DOTS
       |                 ***                   (* Measured)
    F  |               *     *                 (- Theoretical)
    o  | - - - - - - -*       *- - - - - - -
    r  |             *         *
    c  |            *           *
    e  | + + + + + *+ + + + + + *+ + + + + +
       |         *                 *
       | *  * *                       * *  *
     0 +----------------------------------------
       0          Displacement              10
```

### Structural Diagrams

Use code blocks with `ascii` tag or 10+ backticks:

```` markdown
``` ascii
        +-------------+       .--------------.
+------#|  Sensors    |------*| Controller   +-------.
 \      +-------------+       '-o------------'       |
  \                            /                     |
   ^                          /                ______|______
    \                        v                |   Actuator  |
     +--- Feedback ---<--- Loop ----O---------|             |
                                              |_____________|
```
````

**Academic Usage**: Include block diagrams, control loops, system architectures, and mathematical plots. ASCII art renders as scalable vector graphics.

---

## Tables and Data Visualization

### Basic Tables

``` markdown
| Header | Left-Aligned | Right-Aligned | Centered |
|--------|:-------------|-------------:|:--------:|
| Row 1  | _Italic_     |         0.10 |  Value   |
| Row 2  | **Bold**     |         0.12 | Another  |
| Row 3  | ^super^      |          ... |    ...   |
```

### Automatic Visualization

Tables are automatically visualized based on data structure:

#### Line Charts

``` markdown
|   x |  y₁ |  y₂ |  y₃ |
| ---:| ---:| ---:| ---:|
|   1 |   1 |   1 |  15 |
|   2 |   2 |   4 |  15 |
|   3 |   3 |   9 |  15 |
|   4 |   4 |  16 |  15 |
|   5 |   5 |  25 |  15 |
```

#### Bar Charts

``` markdown
| Category | Value A | Value B | Value C |
|----------|--------:|--------:|--------:|
| Item 1   |      28 |      15 |      95 |
| Item 2   |      85 |      30 |      50 |
| Item 3   |      20 |      45 |      10 |
```

#### Pie Charts

``` markdown
| Category | Value |
|----------|------:|
| Theory   |    70 |
| Practice |    30 |
```

### Table Configuration

``` markdown
<!--
data-type="linechart"
data-title="Experimental Results"
data-xlabel="Time (s)"
data-ylabel="Position (m)"
data-show
-->
| Time | Position | Velocity |
|-----:|---------:|---------:|
|    0 |      0.0 |      0.0 |
|    1 |      0.5 |      1.0 |
|    2 |      2.0 |      2.0 |
```

**Configuration Options:**
- `data-type`: Override automatic detection (map, boxplot, barchart, linechart, scatterplot, piechart, radar, parallel, heatmap, graph, sankey, none)
- `data-show`: Display visualization immediately
- `data-transpose`: Swap rows and columns
- `data-title`: Custom chart title
- `data-xlabel`: X-axis label
- `data-ylabel`: Y-axis label

**Academic Usage**: Present experimental data, comparative results, statistical distributions, and trends. Include proper labels and captions.

---

## Executable Code

### Basic Code Blocks

```` markdown
``` python
# Python code with syntax highlighting
def forward_kinematics(theta, link_lengths):
    """Calculate end-effector position"""
    x = sum(L * cos(t) for L, t in zip(link_lengths, theta))
    y = sum(L * sin(t) for L, t in zip(link_lengths, theta))
    return x, y
```
````

### Executable Code

```` markdown
``` python
print("Hello, Robotics!")
for i in range(3):
    print(f"Step {i}")
```
<script>@input</script>
````

**Note**: `@input` is replaced with code content before execution.

### Multi-File Projects

```` markdown
``` javascript     -main.js
let data = require('./data.json');

if(data.online) {
  console.log(data.name + " is online");
} else {
  console.log(data.name + " is offline");
}
```
``` json    +data.json
{
  "name": "Robot-01",
  "online": true,
  "position": [1.5, 2.3, 0.8]
}
```
<script>
  let data = @input(1);
  eval(`@input(0)`);
</script>
````

**File naming**: Prefix with `+` (expanded) or `-` (collapsed) for initial display state.

### Default Output

```` markdown
``` python
for i in range(5):
    print(f"Iteration {i}")
```
``` text  @output
Iteration 0
Iteration 1
Iteration 2
Iteration 3
Iteration 4
```
<script>@input</script>
````

### Code Configuration

```` markdown
<!--
data-readOnly="false"
data-showGutter="true"
data-fontSize="14pt"
data-theme="monokai"
data-firstLineNumber="1"
data-tabSize="4"
-->
``` cpp
// Configured C++ code block
#include <iostream>

int main() {
    std::cout << "Robot control system" << std::endl;
    return 0;
}
```
<script>@input</script>
````

**Configuration Options:**
- `data-readOnly`: Allow/prevent editing
- `data-showGutter`: Show/hide line numbers
- `data-fontSize`: Font size with unit
- `data-theme`: Ace editor theme
- `data-firstLineNumber`: Starting line number
- `data-tabSize`: Tab width in spaces
- `data-marker`: Highlight code regions

**Academic Usage**: Provide executable examples for algorithms, simulations, and demonstrations. Include expected output and explanatory comments.

---

## Macros and Templates

### Single-Line Macros

``` markdown
<!--
author: Dr. Jane Smith

@institution: Department of Robotics
  University of Technology

  Established 2020
-->

# Course Title

Instructor: @author

@institution
```

### Block Macros

``` markdown
<!--
@theorem
**Theorem**: Formal statement here.

**Proof**: Detailed proof follows.

Mathematical derivation...

$$
\therefore \text{Conclusion}
$$

∎
@end
-->

## Section Title

@theorem
```

### Commented Macros

``` markdown
<!--
@@draft: This macro is commented out
  and will not be processed

@@ Multiple single-line comments
@@ can be created this way

@@@block-comment
This entire block macro
is commented out
@end
-->
```

### Parameterized Macros

``` markdown
<!--
@highlight: <span class="highlight" style="color: @0">@1</span>

@definition:
**Definition (@0)**: @1
@end
-->

@highlight(blue, important concept)

@definition(Kinematics, The study of motion without considering forces)
```

**Parameter Syntax:**
- `@0`, `@1`, ..., `@n`: Parameter placeholders
- Parameters separated by commas
- Use backticks for parameters with commas: `@macro(param1, `complex, param`)`
- Block parameters: Use code block as last parameter

```` markdown
<!--
@example:
<div class="example">
<h4>Example @0</h4>

@1
</div>
@end
-->

@example(1, ``` python
# Python example code
print("Hello")
```)
````

### Important Built-In Macros

#### Import Macros

``` markdown
<!--
script: https://cdn.example.com/library.js
        https://another-script.js

link:   https://cdn.example.com/styles.css
        https://another-style.css

import: https://raw.githubusercontent.com/repo/course.md
        https://another-course.md
-->
```

**Note**: When importing LiaScript documents, only header macros are imported.

#### Course Metadata

``` markdown
<!--
version:  2.0.0

author:   Dr. Robotics Instructor

email:    instructor@robotcampus.io

comment:  Comprehensive robotics curriculum covering
          theoretical foundations and practical applications.

          Version 2.0.0 - Academic textbook format

logo:     https://robotcampus.io/logo.png

language: en

narrator: US English Female

mode:     Textbook

date:     2025-11-09

@onload
console.log("Course loaded successfully");
@end

attribute: Course materials licensed under CC BY-NC-SA 4.0
attribute: Based on research by [Author et al.](https://doi.org/...)

translation: Deutsch  translations/de.md
translation: Español  translations/es.md
-->

# Course Title
```

#### Utility Macros

``` markdown
@uid - Generates unique identifier

@section - Current section number
```

#### Debugging Macros

``` markdown
<!--
@complex_macro:
  Step 1: @0
  Step 2: @1
  Result: @highlight(@0)
@end
-->

Prefix with @ to see expansion without rendering:

@@complex_macro(First, Second)
```

**Academic Usage**: Create reusable templates for theorems, definitions, examples, proofs, and figures. Maintain consistency across curriculum.

---

## Best Practices for Academic Content

### 1. Theory-Practice Balance
- **Software/AI/Systems tracks**: 70% theory, 30% practice
- **Design/Hardware tracks**: 60% theory, 40% practice
- Use animations and progressive disclosure for complex derivations
- Include worked examples after theoretical presentation

### 2. Structured Learning
- Begin each module with learning objectives
- Use consistent header hierarchy
- Include formative quizzes throughout content
- Conclude with summative assessment or project

### 3. Mathematical Rigor
- Use proper mathematical notation (KaTeX)
- Number important equations for reference
- Provide complete derivations with explanations
- Include units in all physical quantities

### 4. Code Quality
- Provide complete, executable examples
- Include comprehensive comments
- Show expected output
- Demonstrate best practices

### 5. Accessibility
- Include alternative text for images
- Provide text-to-speech narration for key concepts
- Use semantic HTML and CSS classes
- Ensure content works in textbook mode (print)

### 6. Version Control
- Use external CSS (avoid inline styles)
- Store assets in organized directories
- Document macro usage
- Maintain changelog in version comments

---

## External Stylesheet Integration

Link external CSS for consistent styling:

``` markdown
<!--
link: https://raw.githubusercontent.com/robot-campus/robot-courses/main/assets/styles/academic.css
-->
```

**Recommended CSS Classes:**
- `.theorem` - Theorem statements
- `.definition` - Formal definitions
- `.example` - Worked examples
- `.proof` - Mathematical proofs
- `.important` - Key concepts
- `.figure` - Images and diagrams
- `.course-meta` - Course metadata (duration, format)
- `.schedule-info` - Schedule information

---

## Version 2.0.0 Academic Standards

All curriculum content should adhere to:

1. **Academic tone**: Formal, precise language
2. **Theoretical rigor**: Mathematical foundations and derivations
3. **Systematic organization**: Logical progression from fundamentals to applications
4. **Balanced pedagogy**: Appropriate theory/practice ratio for discipline
5. **Assessment integration**: Quizzes and projects aligned with learning objectives
6. **Professional presentation**: Consistent formatting and styling
7. **Accessibility**: Multiple modalities (text, visual, audio)
8. **Reproducibility**: Complete examples with expected results

---

## Additional Resources

### LiaScript Documentation
- **Official Guide**: [https://liascript.github.io/course/](https://liascript.github.io/course/)
- **GitHub Repository**: [https://github.com/LiaScript/LiaScript](https://github.com/LiaScript/LiaScript)
- **Live Editor**: [https://liascript.github.io/](https://liascript.github.io/)

### KaTeX Mathematics
- **Documentation**: [https://katex.org/docs/supported.html](https://katex.org/docs/supported.html)
- **Reference**: Complete list of supported functions and symbols

### Curriculum Development
- Robot Campus Style Guide
- Course Template Repository
- Example Courses (RC-101 through RC-106)

---

## Conclusion

LiaScript provides powerful capabilities for creating interactive, accessible, and engaging educational content in academic format. By following these guidelines and the Version 2.0.0 standards, curriculum developers can create rigorous, professional robotics courses that effectively integrate theory and practice.

For questions or contributions, contact: academic@robotcampus.io

---

*Robot Campus - Excellence in Robotics Education*
