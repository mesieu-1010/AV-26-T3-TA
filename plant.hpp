#pragma once
// Your model of the actuator, reconstructed from the decoded CSVs. This is
// the Part B deliverable, alongside your written notes.
//
// Implement step(): given a commanded velocity and a timestep, return the
// measured output angle. The placeholder below is a bare integrator with
// gain 1 -- NOT the real actuator. Replace it with what the data shows
// (dynamics, gain, any nonlinearity, any lag), or the harness proves nothing.

#include <cmath>

struct Plant {
    // add whatever state your model needs (velocity, motor-side angle, ...)
    double angle = 0.0;
    double velocity = 0.0;
    double motorAngle = 0.0;

    // Constants for applying first-dynamic orders and backlash nonlinearity
    static constexpr double K = 1.3;
    static constexpr double tau = 0.075;
    static constexpr double BACKLASH = 5.0;

    // u_cmd : commanded velocity, deg/s
    // dt    : timestep, seconds
    // return: measured output angle, deg
    double step(double u_cmd, double dt) {
        // 1. First-order dynamics
        double targetVelocity = K * u_cmd;
        velocity += ((targetVelocity - velocity) / tau) * dt;

        // 2. Update motorAngle 
        motorAngle += velocity * dt;
        double halfGap = BACKLASH / 2.0;
        
        // 3. Modify the angle 
        if (motorAngle > angle + halfGap) {
            angle = motorAngle - halfGap;
        } else if (motorAngle < angle - halfGap) {
            angle = motorAngle + halfGap;
        }

        return std::round(angle / 0.1) * 0.1;  // the sensor reads to 0.1 deg
    }

    void reset() { 
        velocity = 0.0;
        angle = 0.0;
        motorAngle = 0.0; 
    }
};
