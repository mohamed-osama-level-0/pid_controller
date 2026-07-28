#ifndef MOTOR_H
#define MOTOR_H

#include "types.h"

// Simple first-order DC motor model:
//   dv/dt = (K*u - v) / tau
// K   = motor gain   (steady-state velocity per unit input)
// tau = time constant (how fast the motor reacts to input changes)
typedef struct {
    real_n K;
    real_n tau;
    real_n velocity;   // current state: motor's velocity
} Motor;

// Create a motor with given gain and time constant, velocity starts at 0
Motor motor_init(real_n K, real_n tau);

// Advance the motor's velocity by one time step, given control input u
void motor_update(Motor *m, real_n u, real_n dt);

// Apply a sudden load disturbance (e.g., friction, obstacle) by directly
// knocking the velocity down/up. Useful for testing disturbance rejection.
void motor_apply_disturbance(Motor *m, real_n delta_v);

#endif