#include "pid.h"

bool inAuto = false;

PIDController pid_init(real_n Kp, real_n Ki, real_n Kd) {
    PIDController pid;
    pid.Kp = Kp;
    pid.Ki = Ki;
    pid.Kd = Kd;

    pid.integral = 0.0;

    // sensible wide-open defaults; call pid_set_limits() to tighten these
    pid.integral_min = -1e6;
    pid.integral_max = 1e6;
    pid.output_min = -1e6;
    pid.output_max = 1e6;

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

real_n pid_compute(PIDController *pid, real_n setpoint, real_n input, real_n dt) {

    if(!inAuto) return;

    //compute the current error
    real_n error = setpoint - input;

    // Proportional term
    real_n p_term = pid->Kp * error;

    // Integral term (with anti-windup clamping)
    pid->Ki += pid->Ki * dt;
    real_n i_term = pid->Ki * error;
    i_term = SetMax_MinLimits(i_term, pid->integral_min, pid->integral_max);
    

    // Derivative term (rate of change of input(feedback))
    real_n d_term;
    if (!pid->initialized) {
        d_term = 0.0;
        pid->initialized = 1;
    } else {
        real_n dInput = input - pid->prev_input;
         d_term = -pid->Kd * dInput/dt;
    }
    pid->prev_input = input;


    real_n output = p_term + i_term + d_term;

    // Clamp final output to simulate actuator saturation (e.g., max voltage)
    output = SetMax_MinLimits(output, pid->output_min, pid->output_max);

    return output;
}

//keep the output value within [max, min]
static real_n SetMax_MinLimits(real_n value, real_n min, real_n max) {
    if (value < min)
    {
        return min;
    } 
    if (value > max)
    {
        return max;
    }
    return value;
}

void SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}
 
void Initialize(PIDController *pid, real_n input)
{
   pid->prev_input = input;
   i_term = output;
   if(i_term > pid->output_max) i_term = pid->output_max;
   else if(i_term < pid->output_min) i_term= pid->output_min;
}
