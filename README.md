# 🚀 DC Motor PID Control Simulator

Welcome to my CS50x final project! 
This project is a C-based simulation of a Proportional-Integral-Derivative (PID) control system regulating the speed of a Direct Current (DC) motor. I designed this project to merge my passion for mechatronics engineering with my programming skills.

## ✨ Features

* **Realistic Simulation:** Incorporates a first-order DC motor mathematical model based on motor gain and time constant[cite: 3].
* **Advanced PID Controller:** The controller is programmed to support advanced features such as:
  * Control mode switching (Auto/Manual) with Bumpless Transfer for smooth transitions[cite: 4, 5].
  * Direction control switching (Direct/Reverse) depending on system requirements[cite: 4, 5].
  * Integral Anti-windup protection (clamps) to prevent massive error accumulation and actuator saturation[cite: 4, 5].
* **Disturbance Testing:** Includes the ability to simulate a sudden load disturbance (e.g., friction or an obstacle) to test how quickly the controller responds and corrects the error[cite: 3, 8].
* **Data Visualization:** A companion Python script is provided to read the simulation outputs and plot them graphically, making it easier to analyze the controller's performance.

## 🛠️ Technologies Used
* **C Language** for programming the core controller and simulation logic.
* **Makefile** to streamline the build and compilation process[cite: 1].
* **Python** (with the `matplotlib` library) for data visualization[cite: 6].

## 🚀 How to Run the Project

1. **Build the Project:**
   Ensure you have a GCC compiler installed, open your terminal, and run:
   ```bash
   gcc simulate.c pid.c motor.c -o simulate
2. **Visualize the Results:**
   python3 plot.py

