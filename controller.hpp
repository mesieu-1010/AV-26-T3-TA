#pragma once
// Implement Controller so that, given only the target angle, the last
// measured angle, and the timestep, it drives the system to the target --
// despite whatever nonlinearity you identified from the CSVs.
//
// This is the file you submit. You can add private members, helper methods,
// filters, whatever your design needs. We will never run your internals.


#include "controller_interface.hpp"

class Controller : public IController {
private:
    // PID tuning parameters
    double kp = 1.0;
    double ki = 0.0;
    double kd = 0.0;

    // PID state
    double previousError = 0.0;
    double integral = 0.0;

    // Used to prevent a derivative spike on the first update
    bool firstUpdate = true;

    // Anti-windup limit
    static constexpr double integralLimit = 20.0;

public:
    double update(double target, double measured, double dt) override {
        // 1. Current error
        double error = target - measured;

        // 2. Proportional term
        double proportional = kp * error;

        // 3. Integral term
        integral += error * dt;

        // Anti-windup: prevent integral from growing indefinitely
        if (integral > integralLimit) {
            integral = integralLimit;
        } else if (integral < -integralLimit) {
            integral = -integralLimit;
        }

        double integralTerm = ki * integral;

        // 4. Derivative term
        double derivative = 0.0;

        if (!firstUpdate && dt > 0.0) {
            derivative = (error - previousError) / dt;
        }

        double derivativeTerm = kd * derivative;

        // 5. PID controller output
        double uCommanded =
            proportional
            + integralTerm
            + derivativeTerm;

        // 6. Save current error for next timestep
        previousError = error;
        firstUpdate = false;

        return uCommanded;
    }

    void reset() override {
        previousError = 0.0;
        integral = 0.0;
        firstUpdate = true;
    }
};
