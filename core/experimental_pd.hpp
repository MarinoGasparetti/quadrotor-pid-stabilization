#ifndef EXPERIMENTAL_PD_HPP
#define EXPERIMENTAL_PD_HPP

#include <cmath>
#include <algorithm>

class DualOpposingPD {
public:
    struct PDParams {
        double kp;
        double kd;
    };

    DualOpposingPD(PDParams a, PDParams b)
        : params_a_(a), params_b_(b), last_error_(0) {}

    double compute(double setpoint, double current, double dt) {
        if (dt <= 0) return 0;

        double error = setpoint - current;
        double derivative = (error - last_error_) / dt;

        double out_a = (params_a_.kp * error) + (params_a_.kd * derivative);
        double out_b = (params_b_.kp * error) + (params_b_.kd * derivative);

        double error_threshold = 1.0;
        double dominance = std::tanh(std::fabs(error) / error_threshold);

        double final_output = (dominance > 0.5) ? out_a : out_b;

        last_error_ = error;
        return final_output;
    }

private:
    PDParams params_a_;
    PDParams params_b_;
    double last_error_;
};

#endif // EXPERIMENTAL_PD_HPP
