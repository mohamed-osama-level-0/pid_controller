#include "motor.h"

Motor motor_init(real_n K, real_n tau) {
    Motor m;
    m.K = K;
    m.tau = tau;
    m.velocity = 0.0;
    return m;
}

void motor_update(Motor *m, real_n u, real_n dt) {
    // Euler integration of dv/dt = (K*u - v) / tau
    real_n dv = (m->K * u - m->velocity) / m->tau;
    m->velocity += dv * dt;
}

void motor_apply_disturbance(Motor *m, real_n delta_v) {
    m->velocity += delta_v;
}

