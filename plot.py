#!/usr/bin/env python3
"""
Quick visualization of the PID motor simulator output.
Usage: python3 plot.py
Requires: pip install matplotlib
"""

import csv
import matplotlib.pyplot as plt

time, setpoint, velocity, error, control = [], [], [], [], []

with open("output.csv", "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        time.append(float(row["time"]))
        setpoint.append(float(row["setpoint"]))
        velocity.append(float(row["velocity"]))
        error.append(float(row["error"]))
        control.append(float(row["control"]))

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

ax1.plot(time, setpoint, "k--", label="Setpoint")
ax1.plot(time, velocity, "b-", label="Motor Velocity")
ax1.set_ylabel("Velocity")
ax1.set_title("PID Motor Controller Response")
ax1.legend()
ax1.grid(True)

ax2.plot(time, control, "r-", label="Control Output (u)")
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Control Signal")
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.savefig("response.png", dpi=150)
print("Plot saved to response.png")
plt.show()
