#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PIDController {
  public:
    PIDController(double kp, double ki, double kd, double setpoint, double outputmin, double output_max);
    double compute(double input);
    void setTunings(double kp, double ki, double kd);
    void setSetpoint(double setpoint);
    void setOutputLimits(double output_min, double output_max);

  private:
    double _kp;
    double _ki;
    double _kd;
    double _setpoint;
    double _output_min;
    double _output_max;
    double _last_input;
    double _integral;
    double _output;
    unsigned long _last_time;
};

#endif