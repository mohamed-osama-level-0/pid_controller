#include <stdio.h>
#include "motor.h"
#include "pid.h"

int main(void)
{
    // ---- Motor setup ----
    // K = 10.0 (velocity per volt), tau = 0.5s (moderately sluggish motor)
    // With output clamped to +-12V below, max reachable steady-state velocity
    // is K*12 = 120, so our setpoint of 100 is comfortably achievable.
    Motor motor = motor_init(10.0, 0.5);
    
    // ---- PID setup ----
    // Starting gains - feel free to tune these and re-run
    PIDController pid = pid_init(2.0, 0.5, 0.1);

    // Anti-windup: clamp integral term; clamp output to simulate max voltage (+-12V)
    pid_set_limits(&pid, -20.0, 20.0, -12.0, 12.0);

    // ---- Simulation parameters ----
    real_n setpoint = 100.0; // target velocity (arbitrary units, e.g. RPM)
    real_n dt = 0.01;        // 10 ms time ste
    int steps = 1000;        // 10 seconds of simulated time

    // Apply a load disturbance partway through, to test disturbance rejection
    int disturbance_step = 500;   // at t = 5.0s
    real_n disturbance_amount = -30.0; // sudden velocity drop (e.g. added load)

    FILE *fp = fopen("output.csv", "w");
    if (fp == NULL) {
        printf("Error: could not open output.csv for writing\n");
        return 1;
    }

    fprintf(fp, "time,setpoint,velocity,error,control\n");

    for (int i = 0; i < steps; i++) {
        real_n t = i * dt;

        if (i == disturbance_step) {
            motor_apply_disturbance(&motor, disturbance_amount);
        }

        real_n error = setpoint - motor.velocity;
        real_n u = pid_compute(&pid, setpoint, motor.velocity, dt);
        motor_update(&motor, u, dt);

        fprintf(fp, "%.3f,%.2f,%.4f,%.4f,%.4f\n",
                t, setpoint, motor.velocity, error, u);
    }

    fclose(fp);
    printf("Simulation complete. Results written to output.csv\n");

    return 0;

}