# PID Motor Controller Simulator

#### Video Demo: <https://youtu.be/yJTb15iICQ0>

#### Description:

This project is a command-line C program that simulates a DC motor being controlled by a PID (Proportional-Integral-Derivative) controller, entirely in software, with no physical hardware required. It models both the physical behavior of a motor and the control algorithm that drives it, then logs the results to a CSV file so the response can be visualized and analyzed. The goal was to take a real, industry-standard control algorithm — the same kind used in robotics, HVAC systems, drones, and industrial automation — and implement it from first principles in C, while handling the practical engineering problems that a naive implementation would miss.

A PID controller works by continuously measuring the difference between a desired target (the "setpoint") and the actual measured value (the "input" or "feedback"), and computing a control output meant to drive that difference toward zero. The proportional term reacts to the current error, the integral term reacts to the accumulated error over time (eliminating steady-state offset that proportional control alone cannot fix), and the derivative term reacts to how quickly the error is changing (damping oscillation and overshoot). Getting all three terms working together correctly, without the several well-known pitfalls that beginner implementations fall into, is the real substance of this project.

The motor itself is modeled as a simple first-order linear system, `dv/dt = (K*u - v) / tau`, where `v` is velocity, `u` is the control input, `K` is the motor's gain, and `tau` is its time constant. This is integrated numerically using Euler's method once per simulated time step. It is not a full electromagnetic model of a real motor (that would require modeling armature resistance, inductance, and back-EMF), but it is a standard simplification used throughout control-systems education, and it is expressive enough to demonstrate genuine PID behavior, including realistic lag, overshoot, and disturbance response.

**File-by-file breakdown:**

`motor.h` / `motor.c` implement the "plant" — the motor's physical model. This file has no knowledge of PID, setpoints, or control theory; it only knows "given a control input, how does velocity change over one time step." Keeping this logic completely separate from the controller makes it independently testable and mirrors how real embedded control software is structured, where the physical system and the controller are two decoupled pieces of code.

`pid.h` / `pid.c` implement the PID controller itself as a struct-based, hardware-agnostic module. Beyond the basic P, I, and D terms, this includes several features that distinguish it from a beginner-level implementation: anti-windup (clamping the integral term so it cannot grow unboundedly while the actuator is saturated, which would otherwise cause large overshoot), derivative-on-measurement (computing the derivative of the actual feedback signal rather than the error, to avoid a large, physically meaningless "derivative kick" whenever the setpoint changes suddenly), output saturation (clamping the final control signal to simulate a real actuator's voltage limits), bumpless MANUAL/AUTOMATIC mode switching (so that if an operator takes manual control of the motor and later hands control back to the PID controller, the integral term is seeded with the last real output, preventing a sudden jump), and configurable control direction (DIRECT or REVERSE, for plants where increasing the output decreases the measured value, such as a cooling valve).

`simulate.c` contains `main()` and is the program's entry point. It creates a `Motor` and a `PIDController`, defines the simulation scenario (a setpoint of 100, a 10-second run, and a sudden load disturbance injected halfway through to test disturbance rejection), runs the time-stepped simulation loop, and writes every step's time, setpoint, velocity, error, and control output to `output.csv`.

`test_manual_auto.c` is a second, separate executable that specifically exercises and verifies the MANUAL/AUTOMATIC bumpless-transfer feature and the DIRECT/REVERSE direction feature, printing internal controller state to confirm both work as designed.

`types.h` defines a single typedef, `real_n` (currently `double`), used throughout every file instead of hardcoding a numeric type. This was a deliberate design choice: if this project is later ported to a microcontroller with limited memory and no hardware floating-point unit, the entire codebase's numeric precision can be changed in exactly one place, rather than hunting through every file.

`plot.py` is an optional Python script using matplotlib that reads `output.csv` and produces a graph of velocity versus setpoint over time, alongside the control signal, making the step response and disturbance recovery easy to see visually rather than reading raw numbers.


**Key design decisions:** every module was deliberately kept ignorant of the others' internals — the motor knows nothing about control theory, and the controller knows nothing about motors — communicating only through plain numeric values (error in, control signal out). This separation, along with the bug-fixes and safety features described above (anti-windup, derivative-on-measurement, bumpless transfer), was chosen specifically because these are the same real-world problems and patterns encountered when implementing PID control on actual embedded systems, which is directly relevant to my mechatronics engineering studies.

