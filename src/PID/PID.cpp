#include "PID.h"

PIDController::PIDController(double kp, double ki, double kd, double setpoint, double output_min, double output_max)
    : _kp(kp), _ki(ki), _kd(kd), _setpoint(setpoint), _output_min(output_min), _output_max(output_max),
      _last_input(0), _integral(0), _output(0), _last_time(millis()) {}

double PIDController::compute(double input) {
    unsigned long now = millis();
    double dt = (now - _last_time) / 1000.0;

    double error = _setpoint - input;

    _integral += error * dt;

    double derivative = (input - _last_input) / dt;
    _last_input = input;

    _output = _kp * error + _ki * _integral + _kd * derivative;
    _output = constrain(_output, _output_min, _output_max);
    
    _last_time = now;
    return _output;
}

void PIDController::setTunings(double kp, double ki, double kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::setSetpoint(double setpoint) {
    _setpoint = setpoint;
}

void PIDController::setOutputLimits(double output_min, double output_max) {
    _output_min = output_min;
    _output_max = output_max;
}