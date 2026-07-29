#include "pid.h"

// Helper function
static real_n SetMax_MinLimits(real_n value, real_n min, real_n max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

PIDController pid_init(real_n Kp, real_n Ki, real_n Kd) {
    PIDController pid;
    pid.Kp = Kp;
    pid.Ki = Ki;
    pid.Kd = Kd;

    pid.integral = 0.0;
    pid.prev_input = 0.0;
    pid.initialized = 0;

    pid.integral_min = -1e6;
    pid.integral_max = 1e6;
    pid.output_min = -1e6;
    pid.output_max = 1e6;

    // defulte
    pid.inAuto = true;       // auto
    pid.direction = DIRECT;  // direct

    return pid;
}

void pid_set_limits(PIDController *pid,
                     real_n integral_min, real_n integral_max,
                     real_n output_min, real_n output_max) {
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid->output_min = output_min;
    pid->output_max = output_max;
}

//Direction changing function
void pid_set_direction(PIDController *pid, int direction) {
    // if direction changed, change the sing of the gains
    if (pid->direction != direction) {
        pid->Kp = -(pid->Kp);
        pid->Ki = -(pid->Ki);
        pid->Kd = -(pid->Kd);
        pid->direction = direction;
    }
}

// smooth tarnstion between auto/manual
void pid_set_mode(PIDController *pid, int mode, real_n current_input, real_n current_output) {
    bool newAuto = (mode == AUTOMATIC);
    
    // when switch from manual to auto (Bumpless Transfer)
    if (newAuto && !pid->inAuto) {
        pid->prev_input = current_input;
        pid->integral = current_output; // ITerm = Output
        
        // to prevent wind-up problem
        pid->integral = SetMax_MinLimits(pid->integral, pid->integral_min, pid->integral_max);
    }
    pid->inAuto = newAuto;
}

real_n pid_compute(PIDController *pid, real_n setpoint, real_n input, real_n dt) {
    
    // if the program in manual mode, then pid will not be calculated
    if (!pid->inAuto) {
        return 0.0; 
    }

    real_n error = setpoint - input;

    // Proportional term
    real_n p_term = pid->Kp * error;

    // Integral term 
    pid->integral += (pid->Ki * error * dt);
    pid->integral = SetMax_MinLimits(pid->integral, pid->integral_min, pid->integral_max);
    real_n i_term = pid->integral;

    // Derivative term 
    real_n d_term = 0.0;
    if (!pid->initialized) {
        pid->initialized = 1;
    } else {
        real_n dInput = input - pid->prev_input;
        // dinput to limit the derivative kick
        d_term = -pid->Kd * (dInput / dt); 
    }
    pid->prev_input = input;

    // Output
    real_n output = p_term + i_term + d_term;
    output = SetMax_MinLimits(output, pid->output_min, pid->output_max);

    return output;
}
