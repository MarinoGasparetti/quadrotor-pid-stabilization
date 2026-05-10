#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
public:
    struct Params {
        double kp;
        double ki;
        double kd;
        double min_output;
        double max_output;
    };

    PIDController(const Params& params) : params_(params), integral_(0), last_error_(0) {}

    double compute(double setpoint, double current_value, double dt) {
        double error = setpoint - current_value;

        double p_out = params_.kp * error;

        integral_ += error * dt;
        double i_out = params_.ki * integral_;

        double derivative = 0;
        if (dt > 1e-6) {
            derivative = (error - last_error_) / dt;
        }
        double d_out = params_.kd * derivative;

        double output = p_out + i_out + d_out;

        if (output > params_.max_output) output = params_.max_output;
        else if (output < params_.min_output) output = params_.min_output;

        last_error_ = error;
        return output;
    }

    void reset() {
        integral_ = 0;
        last_error_ = 0;
    }

private:
    Params params_;
    double integral_;
    double last_error_;
};

#endif // PID_CONTROLLER_HPP
