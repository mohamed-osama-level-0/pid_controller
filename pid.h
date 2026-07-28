#ifndef PID_H
#define PID_H

#include "types.h"

typedef struct {
    real_n Kp, Ki, Kd;

    real_n integral;      // accumulated error over time
    real_n prev_input;    // error from the previous step (for derivative term)

    int initialized;

    real_n integral_min;  // anti-windup clamp (lower bound)
    real_n integral_max;  // anti-windup clamp (upper bound)

    real_n output_min;    // clamp on the final control output (actuator limits)
    real_n output_max;
} PIDController;

// Create a PID controller with given gains.
// integral/output clamps default to a wide range; override with pid_set_limits().
PIDController pid_init(real_n Kp, real_n Ki, real_n Kd);

// Optionally set anti-windup and output saturation limits
void pid_set_limits(PIDController *pid,
                     real_n integral_min, real_n integral_max,
                     real_n output_min, real_n output_max);

// Compute the control output for a given error and time step
real_n pid_compute(PIDController *pid, real_n setpoint, real_n input, real_n dt);

// small helper to keep a value within [max, min]
static real_n SetMax_MinLimits(real_n value, real_n min, real_n max);

#endif