#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

class PidController {
public:
    PidController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), prev_err_(0.0), integ_(0.0) {}

    double compute(double setpoint, double measured, double dt)
    {
        double err = setpoint - measured;
        integ_ += err * dt;
        double deriv = (err - prev_err_) / dt;
        prev_err_ = err;
        return kp_ * err + ki_ * integ_ + kd_ * deriv;
    }

private:
    double kp_, ki_, kd_;
    double prev_err_, integ_;
};

#endif  // PID_CONTROLLER_HPP_
