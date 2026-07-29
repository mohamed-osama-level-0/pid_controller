#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include "types.h"

// تعريفات الوضع والاتجاه
#define MANUAL 0
#define AUTOMATIC 1

#define DIRECT 0
#define REVERSE 1

typedef struct {
    real_n Kp, Ki, Kd;

    real_n integral;      // ITerm: accumulated error over time
    real_n prev_input;    // error from the previous step (for derivative term)

    int initialized;

    real_n integral_min;  // anti-windup clamp (lower bound)
    real_n integral_max;  // anti-windup clamp (upper bound)

    real_n output_min;    // clamp on the final control output (actuator limits)
    real_n output_max;
    
    bool inAuto;          //control method(auto/manual)
    int direction;        //direction(direct or reversed) based on the system
} PIDController;

// Create a PID controller with given gains.
PIDController pid_init(real_n Kp, real_n Ki, real_n Kd);

// Optionally set anti-windup and output saturation limits
void pid_set_limits(PIDController *pid,
                     real_n integral_min, real_n integral_max,
                     real_n output_min, real_n output_max);

// Compute the control output for a given error and time step
real_n pid_compute(PIDController *pid, real_n setpoint, real_n input, real_n dt);

// determain control direction
void pid_set_direction(PIDController *pid, int direction);

// changing control mode
void pid_set_mode(PIDController *pid, int mode, real_n current_input, real_n current_output);

#endif