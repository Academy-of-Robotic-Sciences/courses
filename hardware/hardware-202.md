<!--
author:   Dr. Elena Martinez
email:    elena.martinez@robotcampus.dev
version:  2.0.0

language: en
narrator: US English Female

comment:  Embedded Systems Programming: Comprehensive treatment of microcontroller architectures, interrupt-driven programming, hardware abstraction, and real-time embedded systems for robotics.

icon:     https://robotcampus.dev/logos/hardware-202.png

mode:     Textbook

link:     https://robotcampus.dev/styles/course-styles.css

import:   https://raw.githubusercontent.com/LiaTemplates/Pyodide/master/README.md
-->

# HARDWARE-202: Embedded Systems Programming

> **Microcontrollers serve as the computational substrate for real-time robot control systems.**

## Course Overview

| | |
|---|---|
| **Course Code** | HARDWARE-202 |
| **Duration** | 4 hours |
| **Level** | Intermediate |
| **Prerequisites** | HARDWARE-201, programming fundamentals (RC-103 or equivalent) |
| **Theory-Practice** | 60% theory, 40% optional labs |

---

## Learning Objectives

Upon completion of this course, students will be able to:

### Theoretical Understanding

- Explain the Von Neumann and Harvard architectures in microcontroller design
- Understand interrupt-driven programming and real-time execution constraints
- Analyze timer/counter peripherals and hardware PWM generation
- Describe memory hierarchies and embedded system optimization techniques
- Explain hardware abstraction layers and cross-platform development

### Practical Skills

- Program Arduino (AVR) microcontrollers using advanced timing and interrupt techniques
- Develop firmware for ARM Cortex-M processors (STM32) using professional toolchains
- Implement hardware abstraction for portable embedded code
- Utilize hardware timers for precise timing and PWM control
- Debug embedded systems using serial communication and logic analyzers

---

## Module 1: Microcontroller Architecture

### 1.1 The Von Neumann and Harvard Architectures

<!-- class="theory-concept" -->
**Computer Architecture Fundamentals**

Modern microcontrollers employ variants of two fundamental architectures:

**Von Neumann Architecture**:
- Single memory space for both instructions and data
- Single bus connects CPU to memory
- Simpler design, but potential bottleneck (Von Neumann bottleneck)
- Example: Most general-purpose processors

**Harvard Architecture**:
- Separate memory spaces for instructions (program memory) and data
- Separate buses enable simultaneous access
- Higher performance, more complex design
- Example: AVR (Arduino), PIC, most DSPs

**Modified Harvard**: Most modern microcontrollers use a hybrid approach with instruction cache and unified addressing.

<!-- class="historical-note" -->
**Evolution of Microcontrollers**

The microcontroller emerged in the 1970s as a system-on-chip solution:

**1971**: Intel 4004, the first microprocessor (4-bit, 740 kHz)
**1975**: MOS Technology 6502, revolutionized embedded systems (used in Apple II, Commodore 64)
**1976**: Intel 8048, first true microcontroller with on-chip ROM and RAM
**1985**: Atmel AVR architecture, pioneered in-system programming
**1993**: ARM7TDMI, beginning of ARM's dominance in embedded systems
**2004**: ARM Cortex-M series, optimized for microcontroller applications
**2005**: Arduino platform, democratized embedded development
**2012**: Raspberry Pi Pico (RP2040), introduced Programmable I/O (PIO) concept

### 1.2 CPU Core and Instruction Set

<!-- class="theory-concept" -->
**RISC vs. CISC**

**Reduced Instruction Set Computer (RISC)**:
- Small, fixed-size instructions (typically 16 or 32 bits)
- Load/store architecture (only load/store access memory)
- Many general-purpose registers
- Simple, fast instruction execution
- Example: ARM Cortex-M, AVR, RISC-V

**Complex Instruction Set Computer (CISC)**:
- Variable-length instructions
- Memory operands in arithmetic instructions
- Fewer registers, more addressing modes
- Example: x86, 68HC11

Most modern microcontrollers use RISC for power efficiency and predictable timing.

<!-- class="theory-concept" -->
**AVR (Arduino Uno/Nano) Architecture**

**ATmega328P Specifications**:
- 8-bit AVR RISC core
- 16 MHz clock (default Arduino)
- 32 general-purpose registers
- 32 KB Flash (program memory)
- 2 KB SRAM (data memory)
- 1 KB EEPROM (non-volatile data)
- Harvard architecture with separate program/data buses

**Instruction Execution**:
- Most instructions execute in single clock cycle
- At 16 MHz: 16 MIPS (million instructions per second)
- Predictable timing critical for real-time control

**Quiz: Architecture Fundamentals**

What is the primary advantage of Harvard architecture over Von Neumann?

[( )] Lower cost
[(X)] Simultaneous instruction fetch and data access
[( )] Simpler programming model
[( )] Larger memory capacity

### 1.3 ARM Cortex-M Architecture

<!-- class="theory-concept" -->
**32-Bit ARM Cortex-M Family**

The ARM Cortex-M series dominates modern embedded systems:

**Cortex-M0/M0+**: Ultra-low power, simple applications
- 32-bit RISC, Thumb instruction set
- Up to 48 MHz typical
- Minimal peripherals

**Cortex-M3**: General-purpose microcontroller
- 32-bit RISC, Thumb-2 instruction set
- Up to 120 MHz
- Hardware division, saturating arithmetic
- Nested Vectored Interrupt Controller (NVIC)
- Example: STM32F1 series

**Cortex-M4**: DSP and floating-point applications
- All M3 features plus:
- Single-precision FPU (optional)
- DSP instructions (SIMD)
- Example: STM32F4 series, used in drones and advanced robotics

**Cortex-M7**: High-performance embedded
- Superscalar, out-of-order execution
- Double-precision FPU
- Cache memory
- Up to 500+ MHz

<!-- class="theory-concept" -->
**STM32 "Blue Pill" (STM32F103C8T6)**

**Specifications**:
- ARM Cortex-M3 core
- 72 MHz maximum frequency
- 64 KB Flash (program memory)
- 20 KB SRAM
- 3× USART, 2× SPI, 2× I2C
- 2× 12-bit ADC
- 7× timers
- 37 GPIO pins

**Performance Comparison** (vs. Arduino Uno):
- ~4.5× faster clock
- ~32× more RAM
- True 32-bit operations (vs. 8-bit)
- Hardware floating-point capable

### 1.4 Memory Organization

<!-- class="theory-concept" -->
**Memory Hierarchy in Embedded Systems**

**Flash Memory** (Non-volatile):
- Stores program code (firmware)
- Persists without power
- Limited write cycles (10,000-100,000 typical)
- Slow write, fast read

**SRAM** (Volatile):
- Stores runtime variables and stack
- Fast read/write
- Lost when power removed
- Limited size (KB range in microcontrollers)

**EEPROM** (Non-volatile):
- Stores configuration and calibration data
- Byte-addressable (vs. Flash page/block erase)
- Very limited write cycles (100,000-1,000,000)

**Memory-Mapped I/O**:
- Peripherals accessed as memory addresses
- Unified addressing simplifies programming
- Example: Writing to 0x40021010 might set GPIO pin on STM32

**Stack and Heap**:
- **Stack**: Automatic variables, function calls (LIFO)
- **Heap**: Dynamic allocation (malloc/new)
- Embedded systems typically avoid heap (unpredictable timing)

---

## Module 2: Timing and Interrupts

### 2.1 The Limitations of delay()

<!-- class="theory-concept" -->
**Blocking vs. Non-Blocking Code**

The Arduino `delay()` function halts execution:

```cpp
digitalWrite(LED_PIN, HIGH);
delay(1000);  // CPU does nothing for 1 second
digitalWrite(LED_PIN, LOW);
```

**Problems**:
- CPU cannot respond to sensors during delay
- Cannot perform multiple tasks
- Wastes energy
- Unacceptable for real-time systems

**Real-Time Requirement**: Robotic control loops typically require:
- 10-100 Hz update rates for motion control
- <1ms response to critical events (e.g., collision detection)
- Simultaneous sensor reading, computation, and actuation

### 2.2 Non-Blocking Timing with millis()

<!-- class="theory-concept" -->
**The millis() Function**

`millis()` returns elapsed time since program start (milliseconds):

```cpp
unsigned long previousMillis = 0;
const long interval = 1000;

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        // Execute periodic task
        toggleLED();
    }

    // Other tasks can run here
    readSensors();
    updateMotors();
}
```

**Key Concepts**:
- State machine approach
- Non-blocking execution
- Multiple timers for different tasks
- Foundation of real-time embedded programming

**Rollover Handling**: `millis()` overflows after ~49.7 days. Subtraction method handles this correctly:
```cpp
// This works correctly even after rollover
if ((currentMillis - previousMillis) >= interval)
```

<!-- class="theory-concept" -->
**Timer Implementation**

`millis()` is implemented using a hardware timer interrupt:

```cpp
// Simplified ATmega328P implementation
volatile unsigned long timer0_millis = 0;

ISR(TIMER0_COMPA_vect) {
    // Interrupt fires every 1 ms
    timer0_millis++;
}

unsigned long millis() {
    unsigned long m;
    uint8_t oldSREG = SREG;
    cli();  // Disable interrupts
    m = timer0_millis;
    SREG = oldSREG;  // Restore interrupts
    return m;
}
```

### 2.3 Hardware Interrupts

<!-- class="theory-concept" -->
**Interrupt-Driven Programming**

An interrupt is an asynchronous event that temporarily suspends normal execution:

**Interrupt Sources**:
- External pins (button press, encoder pulse)
- Timers (periodic events)
- Communication peripherals (UART data received)
- ADC conversion complete

**Interrupt Service Routine (ISR)**:
```cpp
volatile bool buttonPressed = false;

void setup() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN),
                    buttonISR, FALLING);
}

void buttonISR() {
    // Executed immediately when button pressed
    buttonPressed = true;
}

void loop() {
    if (buttonPressed) {
        buttonPressed = false;
        handleButtonPress();
    }
}
```

**ISR Best Practices**:
1. **Keep ISRs short**: No delays, minimal computation
2. **Use volatile variables**: Shared between ISR and main code
3. **Avoid blocking operations**: No Serial.print(), delay()
4. **Minimize shared data**: Use atomic operations or disable interrupts
5. **Set flags, process in main loop**: ISR signals, main loop acts

<!-- class="theory-concept" -->
**Interrupt Priorities and Nesting**

**AVR (Arduino)**: Single interrupt priority, non-nested by default
- Interrupts disabled during ISR execution
- Pending interrupts served in fixed priority order

**ARM Cortex-M (STM32)**: Nested Vectored Interrupt Controller (NVIC)
- 16 priority levels (configurable)
- Higher priority interrupts can preempt lower priority
- Enables complex real-time scheduling

**Quiz: Interrupts and Timing**

Why should interrupt service routines be kept short?

[( )] To save memory
[(X)] To minimize latency for other interrupts and main loop execution
[( )] To reduce power consumption
[( )] To simplify debugging

What happens if millis() overflows after 49.7 days?

[(X)] Subtraction-based comparison continues to work correctly
[( )] The microcontroller resets
[( )] Time comparisons fail permanently
[( )] millis() returns to zero and comparisons break

### 2.4 Hardware Timers and PWM

<!-- class="theory-concept" -->
**Timer/Counter Peripherals**

Microcontrollers contain dedicated timer hardware:

**ATmega328P Timers**:
- Timer0: 8-bit, used by millis()
- Timer1: 16-bit, high resolution
- Timer2: 8-bit, asynchronous capable

**Timer Modes**:
1. **Normal Mode**: Count up, overflow interrupt
2. **CTC (Clear Timer on Compare)**: Reset at match value
3. **PWM Mode**: Generate pulse-width modulated signals
4. **Input Capture**: Measure external signal timing

<!-- class="theory-concept" -->
**PWM (Pulse-Width Modulation)**

PWM encodes analog values as digital pulse widths:

**Duty Cycle**: Fraction of period signal is HIGH
$$\text{Duty Cycle} = \frac{T_{on}}{T_{period}} \times 100\%$$

**Average Voltage**:
$$V_{avg} = V_{high} \times \text{Duty Cycle}$$

**Applications**:
- Motor speed control
- LED brightness
- Servo position
- Power regulation

**Arduino analogWrite()**:
```cpp
analogWrite(pin, value);  // value: 0-255
// Generates PWM at ~490 Hz (pins 3,9,10,11) or ~980 Hz (5,6)
// Duty cycle: value/255
```

**Hardware PWM Advantages** over software PWM:
- Precise timing (not affected by software delays)
- No CPU overhead
- Stable frequency and duty cycle

---

## Module 3: Cross-Platform Development

### 3.1 Hardware Abstraction Layers

<!-- class="theory-concept" -->
**Portability in Embedded Systems**

Hardware Abstraction Layer (HAL) separates platform-specific code from application logic:

```cpp
// hal.h - Interface (platform-independent)
class HardwareAbstraction {
public:
    virtual void digitalWrite(int pin, bool value) = 0;
    virtual int analogRead(int pin) = 0;
    virtual void delayMs(int ms) = 0;
};

// hal_arduino.cpp - Arduino implementation
class ArduinoHAL : public HardwareAbstraction {
public:
    void digitalWrite(int pin, bool value) override {
        ::digitalWrite(pin, value ? HIGH : LOW);
    }
    // ... other methods
};

// hal_stm32.cpp - STM32 implementation
class STM32HAL : public HardwareAbstraction {
public:
    void digitalWrite(int pin, bool value) override {
        HAL_GPIO_WritePin(getPort(pin), getPin(pin),
                         value ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    // ... other methods
};
```

**Benefits**:
- Code reuse across platforms
- Simplified testing (mock HAL)
- Easier platform migration
- Clear separation of concerns

<!-- class="alternative-approach" -->
**Compile-Time vs. Runtime Abstraction**

**Compile-Time (Preprocessor)**:
```cpp
#ifdef ARDUINO
    #define WRITE_PIN(pin, val) digitalWrite(pin, val)
#elif defined(STM32)
    #define WRITE_PIN(pin, val) HAL_GPIO_WritePin(pin, val)
#endif
```
- Zero runtime overhead
- More complex build system

**Runtime (Virtual Functions)**:
- Cleaner code
- Small performance penalty (acceptable for most robotics)

### 3.2 Professional Development Environments

<!-- class="theory-concept" -->
**PlatformIO**

PlatformIO provides professional embedded development:

**Key Features**:
- Unified IDE (VSCode integration)
- Multi-platform support (200+ boards)
- Library management (dependency resolution)
- Unit testing framework
- Advanced debugging (GDB integration)
- Build configurations

**platformio.ini Example**:
```ini
[env:bluepill_f103c8]
platform = ststm32
board = bluepill_f103c8
framework = arduino
upload_protocol = stlink

[env:uno]
platform = atmelavr
board = uno
framework = arduino

[env:pico]
platform = raspberrypi
board = pico
framework = arduino
```

**Advantages over Arduino IDE**:
- IntelliSense (code completion)
- Advanced refactoring
- Version control integration
- Scalable to large projects

### 3.3 Servo Control Across Platforms

<!-- class="theory-concept" -->
**Servo Motor Protocol**

Standard hobby servos use PWM with specific timing:

**Pulse Parameters**:
- Period: 20 ms (50 Hz)
- Pulse width: 1-2 ms
  - 1.0 ms: 0° (minimum position)
  - 1.5 ms: 90° (center)
  - 2.0 ms: 180° (maximum position)

**Angle Calculation**:
$$T_{pulse} = 1.0\text{ ms} + \left(\frac{\theta}{180°}\right) \times 1.0\text{ ms}$$

**Arduino Servo Library**:
```cpp
#include <Servo.h>

Servo myServo;

void setup() {
    myServo.attach(9);  // Use hardware PWM pin
}

void loop() {
    myServo.write(90);  // Set to 90 degrees
}
```

**STM32 Implementation** (using HAL):
```cpp
// Timer configured for 50 Hz PWM
void setServoAngle(uint8_t angle) {
    // Map 0-180° to 1000-2000 μs pulse width
    uint16_t pulseWidth = 1000 + (angle * 1000) / 180;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulseWidth);
}
```

---

## Module 4: Advanced Embedded Concepts

### 4.1 Real-Time Operating Systems (RTOS)

<!-- class="theory-concept" -->
**Need for RTOS**

Complex robotics require managing multiple concurrent tasks:

**Challenges**:
- Multiple periodic tasks at different rates
- Priority-based execution
- Inter-task communication
- Resource sharing (mutexes, semaphores)

**FreeRTOS** - Most popular embedded RTOS:

**Task Creation**:
```cpp
void motorControlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100 Hz

    while(1) {
        updateMotorControl();  // 100 Hz control loop
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void sensorReadTask(void *pvParameters) {
    while(1) {
        readSensors();  // 50 Hz sensor reading
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void setup() {
    xTaskCreate(motorControlTask, "Motor", 256, NULL, 2, NULL);
    xTaskCreate(sensorReadTask, "Sensor", 256, NULL, 1, NULL);
    vTaskStartScheduler();  // Never returns
}
```

**RTOS Concepts**:
- **Tasks**: Independent threads of execution
- **Scheduler**: Determines which task runs
- **Priority**: Higher priority tasks preempt lower
- **Semaphores**: Synchronization primitives
- **Queues**: Inter-task communication

### 4.2 Low-Power Modes

<!-- class="theory-concept" -->
**Power Management**

Battery-powered robots require power optimization:

**AVR Sleep Modes**:
1. **Idle**: CPU stopped, peripherals running (~50% reduction)
2. **Power-Down**: All clocks stopped, wake on interrupt (~99% reduction)
3. **Power-Save**: Asynchronous timer running

**ARM Cortex-M Sleep Modes**:
1. **Sleep**: Core clock stopped, peripherals active
2. **Deep Sleep**: Most clocks stopped, SRAM retained
3. **Standby**: Lowest power, only RTC and wakeup pins active

**Example**:
```cpp
#include <avr/sleep.h>

void enterSleep() {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();  // Sleep here
    sleep_disable();  // Execution resumes here after interrupt
}
```

**Power-Saving Strategies**:
- Sleep between sensor readings
- Reduce clock frequency
- Disable unused peripherals
- Use hardware acceleration (PWM, ADC)

### 4.3 Raspberry Pi Pico and PIO

<!-- class="theory-concept" -->
**Programmable I/O (PIO)**

The RP2040 (Pico) introduces unique Programmable I/O blocks:

**PIO Architecture**:
- 2 PIO blocks, 4 state machines each
- Custom hardware peripherals in software
- Deterministic timing (independent of CPU)

**Applications**:
- Custom protocols (WS2812 LEDs, DVI output)
- High-speed parallel I/O
- Perfect servo control without jitter

**PIO Servo Example** (assembly-like):
```
.program servo
    pull block          ; Get pulse width from FIFO
    mov x, osr          ; Copy to X register
    set pins, 1         ; Set pin HIGH
pulseloop:
    jmp x-- pulseloop   ; Delay for X cycles
    set pins, 0         ; Set pin LOW
```

**Advantages**:
- Zero CPU overhead
- Nanosecond precision
- Flexible, programmable hardware

<!-- class="historical-note" -->
**The PIO Innovation**

The Raspberry Pi Foundation's RP2040 (2021) introduced PIO as a solution to the proliferation of specialized peripherals. Rather than dedicated hardware for every protocol, PIO provides programmable state machines that can implement custom peripherals, reducing silicon area and increasing flexibility.

**Quiz: Advanced Concepts**

What is the primary advantage of using an RTOS over a simple super-loop?

[( )] Faster execution
[(X)] Priority-based preemptive scheduling
[( )] Lower memory usage
[( )] Simpler code

The RP2040's PIO state machines operate:

[(X)] Independently of the CPU cores
[( )] Only when the CPU is idle
[( )] At the same speed as the CPU
[( )] Only for I2C and SPI

---

## Summary

This course established the foundations of embedded systems programming:

1. **Architecture**: Examined Von Neumann vs. Harvard, RISC vs. CISC, and modern microcontroller designs
2. **Timing**: Progressed from blocking delays to non-blocking code and interrupt-driven systems
3. **Cross-Platform**: Developed hardware abstraction strategies for portable embedded code
4. **Advanced Topics**: Introduced RTOS, power management, and programmable hardware (PIO)

**Key Takeaways**:
- Microcontroller architecture determines performance and capabilities
- Non-blocking, interrupt-driven code enables real-time responsiveness
- Hardware abstraction enables code reuse across platforms
- Professional toolchains (PlatformIO) scale to complex projects
- Advanced concepts (RTOS, PIO) address sophisticated robotics requirements

**Next Steps**: HARDWARE-203 examines sensor integration, communication protocols, and signal processing.

---

## Optional Laboratory Exercises

### Lab 1: Non-Blocking Multi-Task Controller (2 hours)

<!-- class="optional-exercise" -->
**Objective**: Implement a non-blocking control system managing multiple tasks at different rates.

**Tasks**:
1. Implement three concurrent tasks using millis():
   - LED blink at 1 Hz
   - Servo sweep at 0.5 Hz (0-180-0 degrees)
   - Sensor reading at 10 Hz (print to serial)
2. Use state machines for each task
3. Verify no task blocks others

<!-- class="exercise-tip" -->
**Tips**:
- Create separate `previousMillis` variable for each task
- Use enum for state machine states (INCREASING, DECREASING for servo)
- Print sensor readings only when changed (threshold comparison)
- Monitor serial output to verify all tasks running concurrently

**Target Performance**:
- LED blinks precisely at 1 Hz
- Servo sweeps smoothly without pauses
- Sensor readings update at 10 Hz
- All tasks run concurrently without interference

### Lab 2: Interrupt-Based Encoder Reading (2 hours)

<!-- class="optional-exercise" -->
**Objective**: Implement quadrature encoder reading using hardware interrupts.

**Components**:
- Quadrature encoder (e.g., motor encoder)
- Arduino or STM32

**Tasks**:
1. Connect encoder A and B channels to interrupt pins
2. Implement ISRs to increment/decrement position counter
3. Determine direction from phase relationship
4. Display position and velocity on serial monitor
5. Manually rotate encoder and verify accurate counting

**Quadrature Decoding Logic**:
```cpp
volatile long encoderPosition = 0;

void encoderISR_A() {
    if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
        encoderPosition++;
    } else {
        encoderPosition--;
    }
}

void encoderISR_B() {
    if (digitalRead(ENCODER_A) != digitalRead(ENCODER_B)) {
        encoderPosition++;
    } else {
        encoderPosition--;
    }
}
```

<!-- class="exercise-advanced" -->
**Advanced Extensions**:
- Calculate velocity from position change over time
- Implement acceleration calculation (second derivative)
- Add filtering to reject noise
- Port code to STM32 using HAL interrupts

### Lab 3: Cross-Platform Servo Tester (3 hours)

<!-- class="optional-exercise" -->
**Objective**: Build a universal servo tester that compiles for Arduino, STM32, and Raspberry Pi Pico.

**Components**:
- Potentiometer (angle input)
- LCD or OLED display
- Servo motor
- Three microcontroller platforms

**Tasks**:
1. Design HAL interface with methods:
   - `analogRead(pin)`
   - `setPWM(pin, pulseWidth_us)`
   - `displayText(x, y, text)`
2. Implement HAL for each platform
3. Write platform-independent application code
4. Configure PlatformIO for multi-platform builds
5. Compile and test on all three platforms

<!-- class="exercise-tip" -->
**PlatformIO Structure**:
```
src/
  main.cpp          # Platform-independent logic
  hal/
    hal.h           # Abstract interface
    hal_arduino.cpp # Arduino implementation
    hal_stm32.cpp   # STM32 implementation
    hal_pico.cpp    # Pico implementation
platformio.ini      # Multi-environment config
```

**Analysis Questions**:
- How much code is shared vs. platform-specific?
- What are the performance differences between platforms?
- Which platform is easiest to develop for? Why?
- How does PIO-based servo control differ from timer-based?

---

*Version 2.0.0 - Academic curriculum emphasizing theoretical foundations with optional practical exercises*
